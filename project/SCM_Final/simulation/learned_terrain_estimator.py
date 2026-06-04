#!/usr/bin/env python3
"""LearnedTerrainEstimator
========================

Sliding-window MLP terrain estimator built around the paper-retained
proprioceptive window features. The estimator supports both:

* ``n``-only regression of the Bekker sinkage exponent
* joint ``(n, phi)`` regression of sinkage exponent and friction angle

Why this estimator
------------------
The 1-D Bekker manifold (clay→dirt→sand) only spans a single direction in
the (Kphi, Kc, c, phi, k, n) parameter space.  Real SCM responses across
the three soils are not perfectly aligned with that direction, so any
physics-only filter that trades a single ``n`` against the full SCM
response will systematically over- or under-fit one of the three terrains.
A discriminative regressor avoids that projection bias entirely: it learns
which kinematic statistics (top-speed deficit, max lateral acceleration,
wheel-slip mean, …) discriminate the soils on the *vehicle* and outputs an
estimate of ``n`` directly.

Inference is intentionally cheap (single MLP forward pass on a handful of
hand-crafted statistics) and uses only signals available on the real
vehicle: longitudinal speed, lateral velocity (from IMU/GNSS), yaw rate,
body-frame accelerations, wheel encoder speeds, steering sensor angle, and
the commanded throttle.  No oracle tire forces are read at inference
time, in line with the project rule that the architecture must transfer
to hardware.
"""

from __future__ import annotations

import json
import math
import pickle
from collections import deque
from pathlib import Path
from typing import Dict, Optional, Tuple

import numpy as np
import torch

from param_consistency import TERRAIN_PRESETS, terrain_preset_to_internal

# ``train_terrain_window_mlp`` was moved out of simulation/ into nn_training/
# during the 2026-05-16 cleanup; resolve its path explicitly so the runtime
# import works regardless of how this module is launched.
import sys as _sys
_NN_TRAINING_DIR = Path(__file__).resolve().parents[1] / "nn_training"
if str(_NN_TRAINING_DIR) not in _sys.path:
    _sys.path.insert(0, str(_NN_TRAINING_DIR))
from train_terrain_window_mlp import (  # noqa: E402
    FEATURE_NAMES, N_FEATURES, TerrainWindowMLP, compute_window_features,
    WHEEL_RADIUS,
)


_PRESET_INTERNAL = {
    name: terrain_preset_to_internal(preset)
    for name, preset in TERRAIN_PRESETS.items()
}
_PRESET_SEQUENCE = tuple(
    sorted(((name, params) for name, params in _PRESET_INTERNAL.items()),
           key=lambda item: float(item[1]["n"]))
)
_N_BOUNDS = (
    float(_PRESET_SEQUENCE[0][1]["n"]),
    float(_PRESET_SEQUENCE[-1][1]["n"]),
)
# Wider bounds for the regressor's *output*.  The v2 model was trained on a
# diverse-soil dataset that covers n∈[0.45, 1.20] (interpolated/extrapolated
# along the preset manifold) — clamping the prediction to the original 3
# preset n's would force visible saturation at the boundary.  We still use
# the narrower _N_BOUNDS for terrain-parameter interpolation so the
# downstream Bekker mapping never extrapolates the cohesion / friction
# coefficients off the manifold.
_PRED_BOUNDS = (0.40, 1.30)
_PHI_BOUNDS = (
    min(float(params["phi"]) for params in _PRESET_INTERNAL.values()),
    max(float(params["phi"]) for params in _PRESET_INTERNAL.values()),
)


def _terrain_params_for_n(n_val: float) -> Dict[str, float]:
    """Interpolate the 6 soil parameters along the preset n-manifold (mirrors
    the mapping used by the UKF estimator so downstream MPC code stays
    consistent)."""
    n_val = float(np.clip(n_val, _N_BOUNDS[0], _N_BOUNDS[1]))
    for idx, (_, params_hi) in enumerate(_PRESET_SEQUENCE):
        if n_val <= float(params_hi["n"]):
            if idx == 0:
                return dict(params_hi)
            _, params_lo = _PRESET_SEQUENCE[idx - 1]
            n_lo = float(params_lo["n"])
            n_hi = float(params_hi["n"])
            if n_hi <= n_lo:
                return dict(params_hi)
            ratio = (n_val - n_lo) / (n_hi - n_lo)
            return {key: float(params_lo[key] + ratio *
                               (params_hi[key] - params_lo[key]))
                    for key in ("Kphi", "Kc", "n", "c", "phi", "k")}
    return dict(_PRESET_SEQUENCE[-1][1])


def _closest_preset_name(n_val: float, phi_val: Optional[float]) -> str:
    best_name = "unknown"
    best_score = float("inf")
    for name, preset in _PRESET_INTERNAL.items():
        dn = abs(float(preset["n"]) - float(n_val)) / max(_N_BOUNDS[1] - _N_BOUNDS[0], 1e-6)
        if phi_val is None:
            score = dn
        else:
            dphi = abs(float(preset["phi"]) - float(phi_val)) / max(_PHI_BOUNDS[1] - _PHI_BOUNDS[0], 1e-6)
            score = dn + dphi
        if score < best_score:
            best_name = name
            best_score = score
    return best_name


class LearnedTerrainEstimator:
    """Sliding-window MLP terrain estimator for the retained paper path."""

    def __init__(
        self,
        model_dir: Optional[str] = None,
        initial_terrain: Optional[Dict[str, float]] = None,
        *,
        update_interval: int = 1,
        verbose: bool = False,
        smoothing_alpha: float = 0.02,
        # API-compat kwargs (ignored).
        window_size: int = 50,
        min_excitation: float = 0.0,
        lr: float = 0.0,
        n_steps: int = 0,
    ):
        if model_dir is None:
            raise ValueError("LearnedTerrainEstimator needs --learned-model-dir")
        mdir = Path(model_dir)
        if not (mdir / "weights.pt").exists():
            raise FileNotFoundError(f"weights.pt not found in {mdir}")

        with open(mdir / "scaler.pkl", "rb") as f:
            sc = pickle.load(f)
        with open(mdir / "config.json") as f:
            cfg = json.load(f)

        self._x_mean = np.asarray(sc["x_mean"], dtype=np.float64)
        self._x_std  = np.asarray(sc["x_std"],  dtype=np.float64)
        self._win_seconds = float(cfg["win_seconds"])
        hidden = int(cfg.get("hidden", sc.get("hidden", 64)))
        output_names = list(cfg.get("output_names", ["n"]))
        if not output_names:
            output_names = ["n"]

        state_dict = torch.load(mdir / "weights.pt", map_location="cpu")
        final_key = next(k for k in state_dict if k.endswith("net.6.weight"))
        n_out = int(state_dict[final_key].shape[0])
        if len(output_names) != n_out:
            if n_out == 1:
                output_names = ["n"]
            elif n_out == 2:
                output_names = ["n", "phi"]
            else:
                raise ValueError(
                    f"Unsupported learned estimator output dim {n_out} in {mdir}"
                )
        if "n" not in output_names:
            raise ValueError("LearnedTerrainEstimator requires an output named 'n'")
        self._output_names = tuple(output_names)
        self._phi_index = self._output_names.index("phi") if "phi" in self._output_names else None
        self._y_mean = np.asarray(sc.get("y_mean", np.zeros(n_out)), dtype=np.float64).reshape(-1)
        self._y_std = np.asarray(sc.get("y_std", np.ones(n_out)), dtype=np.float64).reshape(-1)
        if self._y_mean.size != n_out:
            self._y_mean = np.zeros(n_out, dtype=np.float64)
        if self._y_std.size != n_out:
            self._y_std = np.ones(n_out, dtype=np.float64)
        output_bounds = cfg.get("output_bounds", {})
        phi_bounds = output_bounds.get("phi") if isinstance(output_bounds, dict) else None
        if phi_bounds and len(phi_bounds) == 2:
            lo, hi = float(phi_bounds[0]), float(phi_bounds[1])
            pad = max(0.5, 0.05 * (hi - lo))
            self._phi_bounds = (lo - pad, hi + pad)
        else:
            self._phi_bounds = _PHI_BOUNDS

        self._device = "cpu"   # tiny model, CPU is fine and avoids GPU
                               # contention with the chrono process
        self._model = TerrainWindowMLP(N_FEATURES, hidden=hidden, n_out=n_out).to(self._device)
        self._model.load_state_dict(state_dict)
        self._model.eval()

        # Pre-extract weight matrices for a fast numpy forward pass — torch
        # has ~30 µs of overhead per call which adds up at 25 Hz with the
        # rest of the UKF / sim pipeline.
        self._np_layers: list[tuple[np.ndarray, np.ndarray, bool]] = []
        layers = list(self._model.net)
        linear_layers = [l for l in layers if isinstance(l, torch.nn.Linear)]
        for i, lin in enumerate(linear_layers):
            W = lin.weight.detach().cpu().numpy().astype(np.float64)
            b = lin.bias.detach().cpu().numpy().astype(np.float64)
            is_last = (i == len(linear_layers) - 1)
            self._np_layers.append((W, b, not is_last))   # ReLU between layers

        # Initial n / smoothed n.
        if initial_terrain and "n" in initial_terrain:
            init_n = float(initial_terrain["n"])
        else:
            init_n = float(_PRESET_INTERNAL["clay"]["n"])
        init_n = float(np.clip(init_n, _N_BOUNDS[0], _N_BOUNDS[1]))
        self._n_raw = init_n
        self._n_smooth = init_n
        self._phi_raw: Optional[float] = None
        self._phi_smooth: Optional[float] = (
            float(initial_terrain["phi"]) if initial_terrain and "phi" in initial_terrain else None
        )
        self._n_smooth_alpha = float(cfg.get("smoothing_alpha", smoothing_alpha))
        self._update_interval = max(int(update_interval), 1)
        self._verbose = bool(verbose)

        # Sliding buffer of (t, u, v, omega, ax, ay, w_fl, w_fr, w_rl, w_rr,
        # delta, throttle).  Length is bounded by 2× the configured window
        # plus warmup so we never grow unbounded.
        self._buf: deque = deque(maxlen=int(50 * self._win_seconds * 2 + 50))
        self._last_throttle = 0.0
        self._buffer_ready = False

        # Bookkeeping
        self._obs_count = 0
        self._total_obs = 0
        self._estimated_params = _terrain_params_for_n(init_n)
        if self._phi_smooth is not None:
            self._estimated_params["phi"] = float(self._phi_smooth)
        self._terrain_name = "init"
        self._confidence = 0.0
        self._mu_ema = 0.15

        # Residual-variance proxy for estimator uncertainty: EMA of squared
        # (n_raw - n_smooth) deviations. Acts as a model-free stand-in for the
        # ensemble disagreement that the abstract describes; it captures the
        # same "how noisy is the current estimate" signal at near-zero cost.
        self._n_resid_var_ema = 0.0
        self._n_resid_var_alpha = 0.05  # ~20-sample horizon at 5 Hz updates
        # Slope d_phi / d_n derived from the canonical clay/dirt/sand presets:
        # phi spans ~7° across n ∈ [0.5, 1.5], so a 1.0 swing in n maps to
        # ~7° of phi uncertainty. Same slope used for the abstract's
        # phi_uncertainty signal so the gate is a calibrated quantity, not a
        # raw RMS.
        self._dphi_dn_deg = self._compute_dphi_dn_deg()

        # omega-dot estimator (API compat with downstream code)
        self._omega_hist: deque = deque(maxlen=7)
        self._omega_time: deque = deque(maxlen=7)

    # ── numpy forward pass (≈ 5 µs vs 30 µs for torch) ───────────────
    def _nn_forward(self, x: np.ndarray) -> np.ndarray:
        h = x
        for W, b, act in self._np_layers:
            h = h @ W.T + b
            if act:
                h = np.maximum(h, 0.0)
        return np.asarray(h, dtype=np.float64).reshape(-1)

    # ── public API mirror ─────────────────────────────────────────────
    def estimate_omega_dot(self, omega: float, t: float) -> Optional[float]:
        self._omega_hist.append(omega)
        self._omega_time.append(t)
        if len(self._omega_hist) < 5:
            return None
        omegas = list(self._omega_hist)
        times  = list(self._omega_time)
        dt = (times[-1] - times[0]) / (len(times) - 1)
        if dt < 1e-6:
            return None
        return float((-2*omegas[-5] - omegas[-4]
                      + omegas[-2] + 2*omegas[-1]) / (10.0 * dt))

    def set_throttle(self, throttle: float) -> None:
        self._last_throttle = float(throttle)

    def observe(
        self,
        kappa: float,
        alpha_f: float,
        alpha_r: float,
        u: float,
        Fz_f: float,
        Fz_r: float,
        sr: float,
        ay_imu: float,
        omega_dot: float,
        *,
        omega: float = 0.0,
        pred_Fy_f: float = 0.0,
        pred_Fy_r: float = 0.0,
        v_ref: float = 0.0,
        v_lateral: float = 0.0,
        x_pos: float = 0.0,
        y_pos: float = 0.0,
        psi: float = 0.0,
        ax_cmd: float = 0.0,
        sim_time: float = 0.0,
        kappa_f: Optional[float] = None,
        kappa_r: Optional[float] = None,
        wheel_ops: Optional[Dict[str, float]] = None,
        wheel_omegas: Optional[Tuple[float, float, float, float]] = None,
        ax_imu: float = 0.0,
        az_imu: float = 0.0,
        roll_rate: float = 0.0,
        pitch_rate: float = 0.0,
        throttle_cmd: Optional[float] = None,
    ) -> bool:
        """Push a new observation; if enough samples in the window, run the
        regressor and update ``n``."""
        if throttle_cmd is not None:
            self._last_throttle = float(throttle_cmd)

        # Reconstruct steering wheel angle from bicycle slip-angle convention
        # used by the openloop runner (kept consistent with UKF estimator).
        u_safe = max(abs(u), 0.5)
        delta = float(alpha_f) + math.atan2(v_lateral + 1.593 * omega, u_safe)

        # Wheel speeds — fall back to (u/R) if not provided so the buffer
        # still gets populated with physically reasonable values when the
        # caller hasn't wired them in.
        if wheel_omegas is not None:
            w_fl, w_fr, w_rl, w_rr = (float(x) for x in wheel_omegas)
        else:
            w_fl = w_fr = w_rl = w_rr = float(u) / WHEEL_RADIUS

        self._buf.append((
            float(sim_time), float(u), float(v_lateral), float(omega),
            float(ax_imu), float(ay_imu),
            w_fl, w_fr, w_rl, w_rr, delta,
            float(self._last_throttle),
            # Vertical-dynamics channels appended 2026-05 (Buzhardt 2024).
            float(az_imu), float(roll_rate), float(pitch_rate),
        ))

        # Need a full window before we can run the regressor.
        if len(self._buf) < 8:
            return False
        t_oldest = self._buf[0][0]
        t_newest = self._buf[-1][0]
        if (t_newest - t_oldest) < self._win_seconds:
            return False
        self._buffer_ready = True

        # Slice the most-recent ``win_seconds`` worth of samples.
        t_cut = t_newest - self._win_seconds
        rows = [r for r in self._buf if r[0] >= t_cut]
        if len(rows) < 8:
            return False
        arr = np.asarray(rows, dtype=np.float64)
        # Buffer layout: [t, u, v, omega, ax, ay, w_fl, w_fr, w_rl, w_rr,
        #                 delta, throttle, az, roll_rate, pitch_rate]
        # `dyn` columns expected by compute_window_features:
        # [u, v, omega, ax, ay, w_fl, w_fr, w_rl, w_rr, delta,
        #  az, omega_x, omega_y]
        if arr.shape[1] >= 15:
            dyn = np.column_stack([arr[:, 1:11], arr[:, 12:15]])
        else:
            # Legacy buffer without vertical channels — pad zeros so the
            # feature extractor's new columns evaluate to 0.
            dyn = np.column_stack([arr[:, 1:11], np.zeros((arr.shape[0], 3))])
        thr = arr[:, 11]
        feat = compute_window_features(dyn, thr)

        x_s = (feat - self._x_mean) / self._x_std
        pred = self._nn_forward(x_s)
        pred = pred * self._y_std + self._y_mean
        pred_map = {
            name: float(pred[idx])
            for idx, name in enumerate(self._output_names)
        }
        n_pred = float(np.clip(pred_map["n"], _PRED_BOUNDS[0], _PRED_BOUNDS[1]))
        self._n_raw = n_pred
        # Track raw/smooth disagreement before the smoother absorbs the new
        # sample, so high-noise periods inflate the residual EMA promptly.
        n_resid = n_pred - self._n_smooth
        self._n_resid_var_ema += self._n_resid_var_alpha * (
            n_resid * n_resid - self._n_resid_var_ema
        )
        self._n_smooth += self._n_smooth_alpha * (n_pred - self._n_smooth)

        phi_pred = None
        if self._phi_index is not None:
            phi_pred = float(np.clip(pred_map["phi"], self._phi_bounds[0], self._phi_bounds[1]))
            self._phi_raw = phi_pred
            if self._phi_smooth is None:
                self._phi_smooth = phi_pred
            else:
                self._phi_smooth += self._n_smooth_alpha * (phi_pred - self._phi_smooth)

        self._obs_count += 1
        self._total_obs += 1

        mu_cent = abs(u * omega) / 9.81
        self._mu_ema += 0.05 * (mu_cent - self._mu_ema)

        if self._verbose and self._obs_count % 10 == 0:
            phi_txt = ""
            if phi_pred is not None and self._phi_smooth is not None:
                phi_txt = f" phi_raw={phi_pred:.2f} phi_sm={self._phi_smooth:.2f}"
            print(f"    [LRN] u={u:.2f} ay={ay_imu:+.2f} omega={omega:+.2f} "
                  f"slip_mean={feat[FEATURE_NAMES.index('wheel_slip_mean')]:.3f} "
                  f"-> n_raw={n_pred:.3f} n_sm={self._n_smooth:.3f}{phi_txt}")
        return True

    # ── accessors expected by the controller / validation scripts ─────
    def should_update(self) -> bool:
        return self._obs_count >= self._update_interval

    def estimate(self) -> Tuple[Dict[str, float], float]:
        self._obs_count = 0
        n_est = float(self._n_smooth)
        self._estimated_params = _terrain_params_for_n(n_est)
        phi_est = self._phi_smooth
        if phi_est is not None:
            self._estimated_params["phi"] = float(phi_est)
        self._terrain_name = _closest_preset_name(n_est, phi_est)
        dists = [abs(float(preset["n"]) - n_est) for preset in TERRAIN_PRESETS.values()]
        self._confidence = float(np.clip(1.0 - min(dists) / 0.4, 0.0, 1.0))
        return self._estimated_params, self._confidence

    def get_terrain_mpc_params(self) -> Dict[str, float]:
        return dict(self._estimated_params)

    def get_bekker_n(self) -> float:
        return float(self._n_smooth)

    def get_friction_angle_deg(self) -> float:
        if self._phi_smooth is None:
            return float(self._estimated_params["phi"])
        return float(self._phi_smooth)

    @property
    def output_names(self) -> Tuple[str, ...]:
        """Names of the regressed outputs, e.g. ``("n",)`` or
        ``("n", "phi")``."""
        return self._output_names

    @property
    def is_joint_estimator(self) -> bool:
        return self._phi_index is not None

    @staticmethod
    def _compute_dphi_dn_deg() -> float:
        ns = [float(p["n"]) for p in _PRESET_INTERNAL.values()]
        phis_deg = [float(p["phi"]) for p in _PRESET_INTERNAL.values()]
        if len(ns) < 2:
            return 7.0
        # Least-squares |d phi / d n| over the preset (n, phi-deg) cloud.
        # ``_PRESET_INTERNAL`` already stores phi in degrees (see
        # param_consistency.terrain_preset_to_internal), so the slope is
        # already in degrees-per-unit-n — no math.degrees() conversion.
        n_arr = np.asarray(ns, dtype=float)
        phi_arr = np.asarray(phis_deg, dtype=float)
        n_mean = n_arr.mean()
        denom = float(np.sum((n_arr - n_mean) ** 2))
        if denom < 1e-9:
            return 7.0
        slope_deg_per_n = float(
            np.sum((n_arr - n_mean) * (phi_arr - phi_arr.mean())) / denom
        )
        return abs(slope_deg_per_n)

    def get_n_uncertainty(self) -> float:
        """Estimator-disagreement proxy: sqrt(EMA of (n_raw - n_smooth)^2).

        Acts as the model-free stand-in for ensemble disagreement; ≈0 when
        the smoother absorbs samples without drift, grows under high-noise
        or regime-shift periods.
        """
        return float(math.sqrt(max(self._n_resid_var_ema, 0.0)))

    def get_phi_uncertainty_deg(self) -> float:
        """phi uncertainty in degrees, derived from sigma_n via the preset
        n -> phi slope. Wired into the MPPI shield's friction-cone gate."""
        return float(self.get_n_uncertainty() * self._dphi_dn_deg)

    @property
    def mu_estimate(self) -> float: return self._mu_ema
    @property
    def mu_peak(self) -> float: return self._mu_ema
    @property
    def loss(self) -> float: return 0.0
    @property
    def confidence(self) -> float: return self._confidence
    @property
    def total_observations(self) -> int: return self._total_obs

    def describe(self) -> str:
        phi_txt = ""
        if self._phi_smooth is not None:
            phi_raw = self._phi_raw if self._phi_raw is not None else self._phi_smooth
            phi_txt = f" phi_raw={phi_raw:.2f} phi_sm={self._phi_smooth:.2f}"
        return (
            f"n_raw={self._n_raw:.3f} n_sm={self._n_smooth:.3f}{phi_txt} "
            f"({self._terrain_name}, conf={self._confidence:.2f})"
        )


class BlendedLearnedTerrainEstimator:
    """Smoothly blend low- and high-regime learned n estimators.

    ``blend.json`` supplies the two child checkpoint paths and the logistic
    gate parameters.  The low-regime estimator drives the gate so clay-like
    windows stay on the benchmark-matched checkpoint instead of being pulled
    high by a broad-LHS expert that is stronger on sand-like windows.
    """

    def __init__(
        self,
        model_dir: str,
        initial_terrain: Optional[Dict[str, float]] = None,
        *,
        update_interval: int = 1,
        verbose: bool = False,
        smoothing_alpha: float = 0.02,
        **kwargs,
    ):
        mdir = Path(model_dir)
        with open(mdir / "blend.json") as f:
            cfg = json.load(f)

        def child_dir(key: str) -> str:
            path = Path(cfg[key]).expanduser()
            if not path.is_absolute():
                path = (mdir / path).resolve()
            return str(path)

        child_smoothing_alpha = float(cfg.get("smoothing_alpha", smoothing_alpha))
        self._low = LearnedTerrainEstimator(
            model_dir=child_dir("low_model_dir"),
            initial_terrain=initial_terrain,
            update_interval=update_interval,
            verbose=verbose,
            smoothing_alpha=child_smoothing_alpha,
            **kwargs,
        )
        self._high = LearnedTerrainEstimator(
            model_dir=child_dir("high_model_dir"),
            initial_terrain=initial_terrain,
            update_interval=update_interval,
            verbose=verbose,
            smoothing_alpha=child_smoothing_alpha,
            **kwargs,
        )
        if self._low.is_joint_estimator or self._high.is_joint_estimator:
            raise ValueError("Blended learned terrain checkpoints must be n-only")

        self._gate_center = float(cfg.get("gate_center", 0.66))
        self._gate_width = max(float(cfg.get("gate_width", 0.02)), 1e-6)
        self._update_interval = max(int(update_interval), 1)
        self._obs_count = 0
        self._total_obs = 0
        self._estimated_params = dict(self._low.get_terrain_mpc_params())
        self._terrain_name = "init"
        self._confidence = 0.0
        self._gate_high = 0.0
        self._n_est = float(self._low.get_bekker_n())

    def _high_weight(self, low_n: float) -> float:
        z = float(np.clip(
            (float(low_n) - self._gate_center) / self._gate_width,
            -60.0,
            60.0,
        ))
        return float(1.0 / (1.0 + math.exp(-z)))

    def estimate_omega_dot(self, omega: float, t: float) -> Optional[float]:
        return self._low.estimate_omega_dot(omega, t)

    def set_throttle(self, throttle: float) -> None:
        self._low.set_throttle(throttle)
        self._high.set_throttle(throttle)

    def observe(self, *args, **kwargs) -> bool:
        low_updated = self._low.observe(*args, **kwargs)
        high_updated = self._high.observe(*args, **kwargs)
        if low_updated or high_updated:
            self._obs_count += 1
            self._total_obs += 1
        return low_updated or high_updated

    def should_update(self) -> bool:
        return self._obs_count >= self._update_interval

    def estimate(self) -> Tuple[Dict[str, float], float]:
        if self._low.should_update():
            self._low.estimate()
        if self._high.should_update():
            self._high.estimate()
        self._obs_count = 0

        low_n = self._low.get_bekker_n()
        high_n = self._high.get_bekker_n()
        self._gate_high = self._high_weight(low_n)
        self._n_est = float((1.0 - self._gate_high) * low_n
                            + self._gate_high * high_n)
        self._estimated_params = _terrain_params_for_n(self._n_est)
        self._terrain_name = _closest_preset_name(self._n_est, None)
        dists = [abs(float(preset["n"]) - self._n_est)
                 for preset in TERRAIN_PRESETS.values()]
        self._confidence = float(np.clip(1.0 - min(dists) / 0.4, 0.0, 1.0))
        return self._estimated_params, self._confidence

    def get_terrain_mpc_params(self) -> Dict[str, float]:
        return dict(self._estimated_params)

    def get_bekker_n(self) -> float:
        return float(self._n_est)

    def get_friction_angle_deg(self) -> float:
        return float(self._estimated_params["phi"])

    @property
    def output_names(self) -> Tuple[str, ...]:
        return ("n",)

    @property
    def is_joint_estimator(self) -> bool:
        return False

    def get_n_uncertainty(self) -> float:
        low_n = self._low.get_bekker_n()
        high_n = self._high.get_bekker_n()
        disagreement = self._gate_high * (1.0 - self._gate_high) * abs(high_n - low_n)
        return float(max(
            self._low.get_n_uncertainty(),
            self._high.get_n_uncertainty(),
            disagreement,
        ))

    def get_phi_uncertainty_deg(self) -> float:
        return float(self.get_n_uncertainty() * self._low._dphi_dn_deg)

    @property
    def mu_estimate(self) -> float:
        return float(self._low.mu_estimate)

    @property
    def mu_peak(self) -> float:
        return float(self._low.mu_peak)

    @property
    def loss(self) -> float:
        return 0.0

    @property
    def confidence(self) -> float:
        return self._confidence

    @property
    def total_observations(self) -> int:
        return self._total_obs

    def describe(self) -> str:
        return (
            f"n_blend={self._n_est:.3f} gate_high={self._gate_high:.2f} "
            f"low=({self._low.describe()}) high=({self._high.describe()})"
        )


class HybridJointLearnedTerrainEstimator:
    """Online joint wrapper: robust blended ``n`` plus a live phi head.

    Direct two-output joint checkpoints can learn phi cleanly offline but may
    inherit the same low-regime n bias that motivated the blended n estimator.
    This wrapper keeps both estimates live while letting the stronger n path
    own Bekker ``n`` and the joint head own friction angle ``phi``.
    """

    def __init__(
        self,
        model_dir: str,
        initial_terrain: Optional[Dict[str, float]] = None,
        *,
        update_interval: int = 1,
        verbose: bool = False,
        **kwargs,
    ):
        mdir = Path(model_dir)
        with open(mdir / "blend.json") as f:
            cfg = json.load(f)

        def child_dir(key: str) -> str:
            path = Path(cfg[key]).expanduser()
            if not path.is_absolute():
                path = (mdir / path).resolve()
            return str(path)

        n_dir = Path(child_dir("n_model_dir"))
        n_cls = (
            BlendedLearnedTerrainEstimator
            if (n_dir / "blend.json").exists()
            else LearnedTerrainEstimator
        )
        self._n_estimator = n_cls(
            model_dir=str(n_dir),
            initial_terrain=initial_terrain,
            update_interval=update_interval,
            verbose=verbose,
            **kwargs,
        )
        if self._n_estimator.is_joint_estimator:
            raise ValueError("hybrid_joint n_model_dir must expose n-only output")

        self._phi_estimator = LearnedTerrainEstimator(
            model_dir=child_dir("phi_model_dir"),
            initial_terrain=initial_terrain,
            update_interval=update_interval,
            verbose=verbose,
            **kwargs,
        )
        if not self._phi_estimator.is_joint_estimator:
            raise ValueError("hybrid_joint phi_model_dir must expose phi")

        self._apply_phi_to_mpc = bool(cfg.get("apply_phi_to_mpc", True))
        self._phi_output_mode = str(cfg.get("phi_output_mode", "learned"))
        self._phi_learned_n_max = float(cfg.get("phi_learned_n_max", 0.65))
        self._update_interval = max(int(update_interval), 1)
        self._obs_count = 0
        self._total_obs = 0
        self._estimated_params = dict(self._n_estimator.get_terrain_mpc_params())
        self._phi_est = self._select_phi_estimate(
            self._n_estimator.get_bekker_n(),
            float(self._estimated_params.get("phi", self._phi_estimator.get_friction_angle_deg())),
            self._phi_estimator.get_friction_angle_deg(),
        )
        if self._apply_phi_to_mpc:
            self._estimated_params["phi"] = self._phi_est
        self._terrain_name = "init"
        self._confidence = 0.0

    def estimate_omega_dot(self, omega: float, t: float) -> Optional[float]:
        return self._n_estimator.estimate_omega_dot(omega, t)

    def set_throttle(self, throttle: float) -> None:
        self._n_estimator.set_throttle(throttle)
        self._phi_estimator.set_throttle(throttle)

    def observe(self, *args, **kwargs) -> bool:
        n_updated = self._n_estimator.observe(*args, **kwargs)
        phi_updated = self._phi_estimator.observe(*args, **kwargs)
        if n_updated or phi_updated:
            self._obs_count += 1
            self._total_obs += 1
        return n_updated or phi_updated

    def should_update(self) -> bool:
        return self._obs_count >= self._update_interval

    def _select_phi_estimate(
        self,
        n_est: float,
        phi_manifold: float,
        phi_learned: float,
    ) -> float:
        if self._phi_output_mode == "n_manifold":
            return float(phi_manifold)
        if self._phi_output_mode == "low_n_learned_else_manifold":
            if float(n_est) <= self._phi_learned_n_max:
                return float(phi_learned)
            return float(phi_manifold)
        return float(phi_learned)

    def estimate(self) -> Tuple[Dict[str, float], float]:
        if self._n_estimator.should_update():
            params_n, conf_n = self._n_estimator.estimate()
        else:
            params_n = self._n_estimator.get_terrain_mpc_params()
            conf_n = self._n_estimator.confidence
        if self._phi_estimator.should_update():
            self._phi_estimator.estimate()
        self._obs_count = 0

        n_est = self._n_estimator.get_bekker_n()
        self._estimated_params = dict(params_n)
        phi_learned = self._phi_estimator.get_friction_angle_deg()
        phi_manifold = float(self._estimated_params.get("phi", phi_learned))
        self._phi_est = self._select_phi_estimate(
            n_est, phi_manifold, phi_learned,
        )
        if self._apply_phi_to_mpc:
            self._estimated_params["phi"] = self._phi_est
        self._terrain_name = _closest_preset_name(n_est, self._phi_est)
        self._confidence = float(conf_n)
        return self._estimated_params, self._confidence

    def get_terrain_mpc_params(self) -> Dict[str, float]:
        return dict(self._estimated_params)

    def get_bekker_n(self) -> float:
        return float(self._n_estimator.get_bekker_n())

    def get_friction_angle_deg(self) -> float:
        return float(self._phi_est)

    @property
    def output_names(self) -> Tuple[str, ...]:
        return ("n", "phi")

    @property
    def is_joint_estimator(self) -> bool:
        return True

    def get_n_uncertainty(self) -> float:
        return float(self._n_estimator.get_n_uncertainty())

    def get_phi_uncertainty_deg(self) -> float:
        return float(self._phi_estimator.get_phi_uncertainty_deg())

    @property
    def mu_estimate(self) -> float:
        return float(self._n_estimator.mu_estimate)

    @property
    def mu_peak(self) -> float:
        return float(self._n_estimator.mu_peak)

    @property
    def loss(self) -> float:
        return 0.0

    @property
    def confidence(self) -> float:
        return self._confidence

    @property
    def total_observations(self) -> int:
        return self._total_obs

    def describe(self) -> str:
        return (
            f"hybrid_n=({self._n_estimator.describe()}) "
            f"hybrid_phi=({self._phi_estimator.describe()})"
        )
