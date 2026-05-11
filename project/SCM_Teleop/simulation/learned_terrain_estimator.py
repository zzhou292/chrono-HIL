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
from train_terrain_window_mlp import (
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
        use_measured_tire_ops: bool = False,
        window_size: int = 50,
        min_excitation: float = 0.0,
        force_residual_checkpoint: Optional[str] = None,
        force_residual_gain: float = 1.0,
        direct_wheel_force_checkpoint: Optional[str] = None,
        direct_wheel_force_gain: float = 1.0,
        axle_force_observer_checkpoint: Optional[str] = None,
        wheel_force_observer_checkpoint: Optional[str] = None,
        force_gain_alpha: float = 0.0,
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
        self._n_smooth_alpha = float(smoothing_alpha)
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
        dyn = arr[:, 1:11]      # u..delta
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
        self._n_smooth += self._n_smooth_alpha * (n_pred - self._n_smooth)

        phi_pred = None
        if self._phi_index is not None:
            phi_pred = float(np.clip(pred_map["phi"], _PHI_BOUNDS[0], _PHI_BOUNDS[1]))
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
