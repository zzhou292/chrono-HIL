#!/usr/bin/env python3
"""
Online Terrain Parameter Estimator — UKF with NN Tire Model
=============================================================

Estimates the Bekker sinkage exponent *n* online using an Unscented Kalman
Filter (UKF) that couples a bicycle dynamics model with a trained NN SCM
tire surrogate.

Approach (following Dallas et al., J. Terramechanics 2020):
  1. Augment the lateral/yaw state with the terrain parameter n:
       z = [v, omega, n]     (3 states)
     where n has trivial dynamics: n_dot = 0 and longitudinal speed u is a
     measured exogenous input.
  2. Prediction step: propagate z through the bicycle model, using the
     NN tire model (parameterized by the current n estimate) to compute
     lateral forces Fy_f, Fy_r.
  3. Measurement update: fuse with observed [u, v, omega] from
     IMU/gyro/wheel-speed (sensor-realistic).
  4. The UKF's Kalman gain automatically learns which observations are
     informative for n and weights them accordingly.

Why UKF instead of optimization:
  - Handles sensor noise naturally (process/measurement covariance)
  - Propagates uncertainty (knows when it's confident)
  - No local minima (no gradient-based search)
  - Statistically principled fusion of model predictions with observations

Sensor-realistic inputs: u (wheel encoder), v (from kinematics or IMU),
omega (gyroscope), steering angle, Fz (from load model).
"""

from __future__ import annotations

import math
import pickle
from collections import deque
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import torch

from direct_wheel_force_surrogate import (
    DirectWheelForceConfig,
    DirectWheelForceSurrogate,
)
from axle_force_observer import AxleForceObserver, AxleForceObserverConfig
from force_residual_adapter import ForceResidualAdapter, ForceResidualConfig
from param_consistency import (
    HMMWV_VEHICLE_PARAMS,
    TERRAIN_PRESETS,
    TRAINING_RANGES_V6,
    terrain_preset_to_internal,
)
from wheel_force_observer import WheelForceObserver, WheelForceObserverConfig


# ═══════════════════════════════════════════════════════════════════════
# Numpy NN forward pass
# ═══════════════════════════════════════════════════════════════════════

class _NumpyMLP:
    def __init__(self, weights: dict):
        layer_ids = sorted(set(
            int(k.split('.')[1])
            for k in weights if k.startswith('layers.') and 'weight' in k
        ))
        self._layers = []
        last = layer_ids[-1]
        for i in layer_ids:
            W = np.asarray(weights[f'layers.{i}.weight'], dtype=np.float64)
            b = np.asarray(weights[f'layers.{i}.bias'], dtype=np.float64)
            self._layers.append((W, b, i < last))

    def __call__(self, x):
        h = x
        for W, b, act in self._layers:
            h = h @ W.T + b
            if act:
                h = np.tanh(h)
        return h


class _NumpyResNet:
    def __init__(self, weights: dict, n_blocks: int):
        self._W_in = np.asarray(weights['input_proj.weight'], dtype=np.float64)
        self._b_in = np.asarray(weights['input_proj.bias'], dtype=np.float64)
        self._blocks = []
        for blk in range(n_blocks):
            self._blocks.append(tuple(
                np.asarray(weights[f'blocks.{blk}.{l}.{p}'], dtype=np.float64)
                for l in ['fc1', 'fc2'] for p in ['weight', 'bias']
            ))
        self._W_out = np.asarray(weights['output_proj.weight'], dtype=np.float64)
        self._b_out = np.asarray(weights['output_proj.bias'], dtype=np.float64)

    def __call__(self, x):
        h = np.tanh(x @ self._W_in.T + self._b_in)
        for W1, b1, W2, b2 in self._blocks:
            r = h
            h = np.tanh(h @ W1.T + b1)
            h = np.tanh(h @ W2.T + b2 + r)
        return h @ self._W_out.T + self._b_out


# ═══════════════════════════════════════════════════════════════════════
# Precomputed terrain data
# ═══════════════════════════════════════════════════════════════════════

_PRESET_INTERNAL = {
    name: terrain_preset_to_internal(preset)
    for name, preset in TERRAIN_PRESETS.items()
}

_PRESET_SEQUENCE = tuple(
    sorted(
        ((name, params) for name, params in _PRESET_INTERNAL.items()),
        key=lambda item: float(item[1]["n"]),
    )
)
_N_BOUNDS = (
    float(_PRESET_SEQUENCE[0][1]["n"]),
    float(_PRESET_SEQUENCE[-1][1]["n"]),
)


# ═══════════════════════════════════════════════════════════════════════
# UKF implementation
# ═══════════════════════════════════════════════════════════════════════

def _ukf_sigma_points(x_mean, P, alpha=1.0, kappa=0.0, beta=2.0):
    """Generate sigma points and weights for the Unscented Transform."""
    L = len(x_mean)
    lam = alpha**2 * (L + kappa) - L

    # Square root of (L + lambda) * P
    S = np.linalg.cholesky((L + lam) * P)

    sigmas = np.zeros((2 * L + 1, L))
    sigmas[0] = x_mean
    for i in range(L):
        col = S[:, i]
        sigmas[i + 1] = x_mean + col
        sigmas[L + i + 1] = x_mean - col

    # Weights
    Wm = np.full(2 * L + 1, 1.0 / (2.0 * (L + lam)))
    Wc = np.full(2 * L + 1, 1.0 / (2.0 * (L + lam)))
    Wm[0] = lam / (L + lam)
    Wc[0] = lam / (L + lam) + (1.0 - alpha**2 + beta)

    return sigmas, Wm, Wc


def _logsumexp(values: np.ndarray) -> float:
    vmax = float(np.max(values))
    return vmax + float(np.log(np.sum(np.exp(values - vmax))))


# ═══════════════════════════════════════════════════════════════════════
# Main estimator
# ═══════════════════════════════════════════════════════════════════════

class TerrainParameterEstimator:
    """UKF-based online estimation of Bekker sinkage exponent n.

    Following Dallas et al. (2020) and the NN-surrogate variant:

    State vector: z = [v, omega, n]  (3 states)
    Measurement:  y = [v, omega]      (2 measurements)
    Parameter:    n has trivial dynamics (n_dot = 0), estimated by UKF.

    The NN tire model is evaluated inside the UKF prediction step to
    compute Fy_f, Fy_r as functions of (operating conditions, n).
    """

    def __init__(
        self,
        model_dir=None,
        initial_terrain: Optional[Dict[str, float]] = None,
        *,
        force_residual_checkpoint: Optional[str] = None,
        force_residual_gain: float = 1.0,
        direct_wheel_force_checkpoint: Optional[str] = None,
        direct_wheel_force_gain: float = 1.0,
        axle_force_observer_checkpoint: Optional[str] = None,
        wheel_force_observer_checkpoint: Optional[str] = None,
        use_measured_tire_ops: bool = False,
        window_size: int = 200,   # log-likelihood accumulation window
        update_interval: int = 10,
        lr: float = 0.01,         # unused
        n_steps: int = 20,        # unused
        min_excitation: float = 0.3,  # unused
        force_gain_alpha: float = 0.0,  # EMA rate for online gain calibration
    ):
        vp = HMMWV_VEHICLE_PARAMS
        self._M = vp["M"]
        self._Izz = vp["Izz"]
        self._Lf = vp["Lf"]
        self._Lr = vp["Lr"]
        self._L = vp["L"]

        self._update_interval = update_interval
        self._window_size = int(max(window_size, 10))
        self._min_excitation = float(min_excitation)
        self._use_measured_tire_ops = bool(use_measured_tire_ops)

        # Load NN
        self._np_nn = None
        if model_dir:
            self._load_nn(Path(model_dir))

        self._force_residual = None
        if force_residual_checkpoint:
            fr_cfg = ForceResidualConfig(
                checkpoint=Path(force_residual_checkpoint),
                gain=float(force_residual_gain),
                online_enabled=False,
            )
            self._force_residual = ForceResidualAdapter(fr_cfg)
        self._direct_wheel_force = None
        if direct_wheel_force_checkpoint:
            dw_cfg = DirectWheelForceConfig(
                checkpoint=Path(direct_wheel_force_checkpoint),
                gain=float(direct_wheel_force_gain),
            )
            self._direct_wheel_force = DirectWheelForceSurrogate(dw_cfg)
        self._axle_force_observer = None
        if axle_force_observer_checkpoint:
            af_cfg = AxleForceObserverConfig(checkpoint=Path(axle_force_observer_checkpoint))
            self._axle_force_observer = AxleForceObserver(af_cfg)
        self._wheel_force_observer = None
        if wheel_force_observer_checkpoint:
            wf_cfg = WheelForceObserverConfig(checkpoint=Path(wheel_force_observer_checkpoint))
            self._wheel_force_observer = WheelForceObserver(wf_cfg)

        # Fixed terrain params (everything except n)
        if initial_terrain:
            self._fixed_Kphi = float(initial_terrain['Kphi'])
            self._fixed_Kc = float(initial_terrain['Kc'])
            self._fixed_c = float(initial_terrain.get('c',
                             initial_terrain.get('cohesion', 1000.0)))
            self._fixed_phi_rad = np.radians(float(initial_terrain.get('phi',
                                  initial_terrain.get('friction_angle', 30.0))))
            self._fixed_k = float(initial_terrain.get('k',
                            initial_terrain.get('janosi_shear', 0.025)))
            init_n = float(initial_terrain['n'])
        else:
            # Default to clay
            clay = _PRESET_INTERNAL["clay"]
            self._fixed_Kphi = clay['Kphi']
            self._fixed_Kc = clay['Kc']
            self._fixed_c = clay['c']
            self._fixed_phi_rad = np.radians(clay['phi'])
            self._fixed_k = clay['k']
            init_n = 0.5

        # UKF state: [v, omega, n]
        self._x = np.array([0.0, 0.0, init_n])

        # Covariance — keep n fairly uncertain at startup.
        self._P = np.diag([0.05**2, 0.01**2, 0.3**2])

        # Process noise: n is modeled as quasi-static (n_dot = 0), so keep
        # its random walk tiny. Large Q_n makes the filter chase transients.
        self._Q = np.diag([0.02**2, 0.005**2, 0.0005**2])

        # Force-sensitive measurement noise for [ay, omega_dot].
        self._R_force = np.diag([0.35**2, 0.20**2])
        # Direct wheel-force surrogate held-out RMSE is ~458 N on scenario splits.
        # Use a slightly conservative scalar variance when sim provides wheel Fy.
        self._R_wheel_fy_var = float(500.0**2)
        force_map = np.array(
            [
                [self._M * self._Lr / self._L, self._Izz / self._L],
                [self._M * self._Lf / self._L, -self._Izz / self._L],
            ],
            dtype=np.float64,
        )
        self._R_axle = force_map @ self._R_force @ force_map.T

        # Online gain calibration — bridges rig-trained NN scale to vehicle.
        # The NN (trained on single-wheel rig data) typically overpredicts
        # vehicle-level forces by a factor of ~2-3x.  The gain adapts slowly
        # so the grid filter compares correctly-scaled predictions.
        self._force_gain = 1.0          # multiplicative scale on NN axle forces
        self._force_gain_alpha = float(force_gain_alpha)  # EMA learning rate (0 = disabled)
        self._g_inst = 1.0              # per-timestep instantaneous gain (diagnostic)

        # Bookkeeping
        self._obs_count = 0
        self._total_obs = 0
        self._last_alpha_f = 0.0
        self._last_alpha_r = 0.0
        self._last_Fz_f = 6500.0
        self._last_Fz_r = 6000.0
        self._last_kappa_f = 0.0
        self._last_kappa_r = 0.0
        self._last_sr = 0.0
        self._last_ay = 0.0
        self._last_u_meas = 0.0
        self._last_delta = 0.0
        self._last_wheel_ops = None
        self._meas_x = 0.0
        self._meas_y = 0.0
        self._meas_psi = 0.0
        self._dt = 0.01  # 100 Hz update rate
        self._n_grid = np.linspace(_N_BOUNDS[0], _N_BOUNDS[1], 25)
        prior_sigma = 1.0   # wide prior — let data drive the estimate
        self._n_prior_logw = -0.5 * ((self._n_grid - init_n) / prior_sigma) ** 2
        self._n_prior_logw -= _logsumexp(self._n_prior_logw)
        self._n_logw = self._n_prior_logw.copy()
        self._n_loglik_hist: deque = deque(maxlen=self._window_size)

        # Omega-dot filter (API compat)
        self._omega_hist: deque = deque(maxlen=7)
        self._omega_time: deque = deque(maxlen=7)

        # Output state
        self._estimated_params = dict(initial_terrain) if initial_terrain else dict(_PRESET_INTERNAL["clay"])
        self._terrain_name = "init"
        self._confidence = 0.0
        self._last_loss = 0.0
        self._mu_ema = 0.15
        self._n_ema1 = float(init_n)
        self._n_ema2 = float(init_n)

    def _load_nn(self, model_dir: Path):
        model_path = model_dir / 'best_terrain_nn.pt'
        if not model_path.exists():
            return
        checkpoint = torch.load(model_path, map_location='cpu', weights_only=False)
        sd = checkpoint.get('model_state_dict', checkpoint)

        # Remap old keys
        if any(k.startswith('layer') and not k.startswith('layers') for k in sd):
            remap = {}
            idx = 0
            while f'layer{idx+1}.weight' in sd:
                remap[f'layer{idx+1}.weight'] = f'layers.{idx}.weight'
                remap[f'layer{idx+1}.bias'] = f'layers.{idx}.bias'
                idx += 1
            sd = {remap.get(k, k): v for k, v in sd.items()}

        weights = {k: v.detach().numpy() for k, v in sd.items()}
        if 'input_proj.weight' in weights:
            nb = sum(1 for k in weights if k.startswith('blocks.') and k.endswith('.fc1.weight'))
            self._np_nn = _NumpyResNet(weights, nb)
        else:
            self._np_nn = _NumpyMLP(weights)

        with open(model_dir / 'scalers.pkl', 'rb') as f:
            scalers = pickle.load(f)
        self._X_mean = np.asarray(scalers['X'].mean_, dtype=np.float64)
        self._X_scale = np.asarray(scalers['X'].scale_, dtype=np.float64)
        self._y_mean = np.asarray(scalers['y'].mean_, dtype=np.float64)
        self._y_scale = np.asarray(scalers['y'].scale_, dtype=np.float64)

    def _terrain_params_for_n(self, n_val: float) -> Dict[str, float]:
        """Map the estimated dominant parameter n onto a consistent soil manifold.

        The UKF only estimates one latent terrain degree of freedom, but the NN
        was trained over all six terrain parameters. Holding the other five
        parameters frozen to the initial guess forces n to compensate for the
        entire preset mismatch and biases convergence. Instead, interpolate
        between the canonical literature presets as a one-dimensional manifold
        parameterized by n.
        """
        n_val = float(np.clip(n_val, _N_BOUNDS[0], _N_BOUNDS[1]))

        for idx, (_name_hi, params_hi) in enumerate(_PRESET_SEQUENCE):
            if n_val <= float(params_hi["n"]):
                if idx == 0:
                    return dict(params_hi)
                _name_lo, params_lo = _PRESET_SEQUENCE[idx - 1]
                n_lo = float(params_lo["n"])
                n_hi = float(params_hi["n"])
                if n_hi <= n_lo:
                    return dict(params_hi)
                ratio = (n_val - n_lo) / (n_hi - n_lo)
                return {
                    key: float(params_lo[key] + ratio * (params_hi[key] - params_lo[key]))
                    for key in ("Kphi", "Kc", "n", "c", "phi", "k")
                }

        return dict(_PRESET_SEQUENCE[-1][1])

    def _nn_FxFy(self, kappa, alpha, u, Fz, sr, n_val):
        """Predict single-wheel (Fx, Fy) given operating conditions + n."""
        if self._np_nn is None:
            return 0.0, 0.0
        terrain = self._terrain_params_for_n(n_val)
        x = np.array([[kappa, alpha, u, Fz, sr,
                        terrain["Kphi"], terrain["Kc"], terrain["n"],
                        terrain["c"], np.radians(terrain["phi"]), terrain["k"]]])
        x_s = (x - self._X_mean) / self._X_scale
        y_s = self._np_nn(x_s)
        y = y_s * self._y_scale + self._y_mean
        return float(y[0, 0]), float(y[0, 1])  # (Fx, Fy)

    def _direct_wheel_Fy(self, kappa, alpha, u, Fz, sr, is_front, n_val, v, omega, ay):
        if self._direct_wheel_force is None:
            return None
        terrain = self._terrain_params_for_n(n_val)
        terrain_vec = np.array(
            [
                terrain["Kphi"],
                terrain["Kc"],
                terrain["n"],
                terrain["c"],
                np.radians(terrain["phi"]),
                terrain["k"],
            ],
            dtype=np.float64,
        )
        return self._direct_wheel_force.predict_single(
            kappa=float(kappa),
            alpha=float(alpha),
            u=float(u),
            Fz=float(Fz),
            sr=float(sr),
            is_front=float(is_front),
            terrain_vec=terrain_vec,
            v=float(v),
            omega=float(omega),
            ay=float(ay),
        )

    def _residual_dFy(self, alpha_f, alpha_r, u, v, omega, Fz_f, Fz_r, n_val, ay):
        """Predict axle-level lateral-force correction from vehicle-domain traces."""
        if self._force_residual is None:
            return 0.0, 0.0
        terrain = self._terrain_params_for_n(n_val)
        terrain_vec = np.array(
            [
                terrain["Kphi"],
                terrain["Kc"],
                terrain["n"],
                terrain["c"],
                np.radians(terrain["phi"]),
                terrain["k"],
            ],
            dtype=np.float64,
        )
        dFy = self._force_residual.predict_single(
            alpha_f=float(alpha_f),
            alpha_r=float(alpha_r),
            u=float(u),
            Fz_f=float(Fz_f),
            Fz_r=float(Fz_r),
            terrain_vec=terrain_vec,
            v=float(v),
            omega=float(omega),
            ay=float(ay),
        )
        return float(dFy[0]), float(dFy[1])

    def _predicted_wheel_forces(self, state, u_meas) -> Optional[Dict[str, float]]:
        if self._direct_wheel_force is None:
            return None
        wheel_ops = self._last_wheel_ops or {}
        required = (
            "front_left_long_slip",
            "front_right_long_slip",
            "rear_left_long_slip",
            "rear_right_long_slip",
            "front_left_slip_angle",
            "front_right_slip_angle",
            "rear_left_slip_angle",
            "rear_right_slip_angle",
            "front_left_Fz",
            "front_right_Fz",
            "rear_left_Fz",
            "rear_right_Fz",
        )
        if not all(k in wheel_ops for k in required):
            return None

        v, omega, n_val = state
        u_safe = max(abs(u_meas), 0.5)
        preds: Dict[str, float] = {}
        for axle, is_front in (("front", 1.0), ("rear", 0.0)):
            sr = self._last_sr if axle == "front" else 0.0
            for side in ("left", "right"):
                prefix = f"{axle}_{side}"
                preds[f"{prefix}_Fy"] = float(self._direct_wheel_Fy(
                    float(wheel_ops[f"{prefix}_long_slip"]),
                    float(wheel_ops[f"{prefix}_slip_angle"]),
                    u_safe,
                    abs(float(wheel_ops[f"{prefix}_Fz"])),
                    sr,
                    is_front,
                    n_val,
                    v,
                    omega,
                    self._last_ay,
                ))
        return preds

    def _axle_forces(self, state, u_meas):
        """Predict axle-level lateral forces for a sigma-point state."""
        import math
        v, omega, n_val = state
        n_val = np.clip(n_val, _N_BOUNDS[0], _N_BOUNDS[1])
        u_safe = max(abs(u_meas), 0.5)

        if self._use_measured_tire_ops:
            alpha_f = float(self._last_alpha_f)
            alpha_r = float(self._last_alpha_r)
            kappa_f = float(self._last_kappa_f)
            kappa_r = float(self._last_kappa_r)
        else:
            alpha_f = self._last_delta - math.atan2(v + self._Lf * omega, u_safe)
            alpha_r = -math.atan2(v - self._Lr * omega, u_safe)
            kappa_f = float(self._last_kappa_f)
            kappa_r = float(self._last_kappa_r)

        _alpha_max = 0.55
        alpha_f = float(np.clip(alpha_f, -_alpha_max, _alpha_max))
        alpha_r = float(np.clip(alpha_r, -_alpha_max, _alpha_max))

        direct_fy_f = self._direct_wheel_Fy(
            kappa_f, alpha_f, u_safe, self._last_Fz_f, self._last_sr,
            1.0, n_val, v, omega, self._last_ay,
        )
        direct_fy_r = self._direct_wheel_Fy(
            kappa_r, alpha_r, u_safe, self._last_Fz_r, 0.0,
            0.0, n_val, v, omega, self._last_ay,
        )

        if self._direct_wheel_force is not None:
            wheel_preds = self._predicted_wheel_forces(state, u_meas)
            if wheel_preds is not None:
                Fy_f = 0.0
                Fy_r = 0.0
                for side in ("left", "right"):
                    Fy_f += wheel_preds[f"front_{side}_Fy"]
                    Fy_r += wheel_preds[f"rear_{side}_Fy"]
            elif direct_fy_f is not None and direct_fy_r is not None:
                Fy_f = 2.0 * float(direct_fy_f)
                Fy_r = 2.0 * float(direct_fy_r)
            else:
                Fy_f = 0.0
                Fy_r = 0.0
        elif self._last_wheel_ops is not None and self._use_measured_tire_ops:
            # Per-wheel NN calls — accounts for weight transfer via individual
            # Fz while using bicycle-model alpha for consistent sign convention.
            # Chrono's GetSlipAngle() returns tire-frame alpha which can have
            # OPPOSITE signs for left vs right wheels (track-width effect),
            # causing force cancellation.  Bicycle-model alpha avoids this.
            import math as _math
            alpha_f_bm = self._last_delta - _math.atan2(v + self._Lf * omega, u_safe)
            alpha_r_bm = -_math.atan2(v - self._Lr * omega, u_safe)
            _alpha_max = 0.55
            alpha_f_bm = float(np.clip(alpha_f_bm, -_alpha_max, _alpha_max))
            alpha_r_bm = float(np.clip(alpha_r_bm, -_alpha_max, _alpha_max))

            wops = self._last_wheel_ops
            Fy_f = 0.0
            Fy_r = 0.0
            _pw_debug = {}
            for side in ("left", "right"):
                fk = f"front_{side}_long_slip"
                fz = f"front_{side}_Fz"
                if fz in wops:
                    kappa_pw = float(wops[fk]) if fk in wops else self._last_kappa_f
                    _, fy_pw = self._nn_FxFy(
                        kappa_pw,
                        alpha_f_bm,
                        u_safe, abs(float(wops[fz])),
                        self._last_sr, n_val)
                    Fy_f += self._force_gain * (-1.0) * fy_pw
                    _pw_debug[f"f{side[0]}"] = (alpha_f_bm, float(wops[fz]), float(fy_pw))

                rk = f"rear_{side}_long_slip"
                rz = f"rear_{side}_Fz"
                if rz in wops:
                    kappa_pw = float(wops[rk]) if rk in wops else self._last_kappa_r
                    _, fy_pw = self._nn_FxFy(
                        kappa_pw,
                        alpha_r_bm,
                        u_safe, abs(float(wops[rz])),
                        0.0, n_val)
                    Fy_r += self._force_gain * (-1.0) * fy_pw
                    _pw_debug[f"r{side[0]}"] = (alpha_r_bm, float(wops[rz]), float(fy_pw))
            self._pw_debug = _pw_debug
        else:
            _, Fy_f_pw = self._nn_FxFy(
                kappa_f, alpha_f,
                u_safe, self._last_Fz_f, self._last_sr, n_val)
            _, Fy_r_pw = self._nn_FxFy(
                kappa_r, alpha_r,
                u_safe, self._last_Fz_r, 0.0, n_val)

            Fy_f = self._force_gain * (-2.0) * Fy_f_pw
            Fy_r = self._force_gain * (-2.0) * Fy_r_pw
            dFy_f, dFy_r = self._residual_dFy(
                alpha_f=alpha_f,
                alpha_r=alpha_r,
                u=u_safe,
                v=v,
                omega=omega,
                Fz_f=self._last_Fz_f,
                Fz_r=self._last_Fz_r,
                n_val=n_val,
                ay=self._last_ay,
            )
            Fy_f += dFy_f
            Fy_r += dFy_r
        return float(Fy_f), float(Fy_r)

    def _measurement_model(self, state, u_meas):
        """Predict measurable channels [v, omega, ay, omega_dot] from state."""
        v, omega, _n_val = state
        Fy_f, Fy_r = self._axle_forces(state, u_meas)
        ay = (Fy_f + Fy_r) / self._M
        omega_dot = (self._Lf * Fy_f - self._Lr * Fy_r) / self._Izz
        return np.array([v, omega, ay, omega_dot], dtype=np.float64)

    def _infer_axle_forces_from_sensors(self, ay: float, omega_dot: float) -> np.ndarray:
        Fy_f = (self._Izz * omega_dot + self._Lr * self._M * ay) / self._L
        Fy_r = (self._Lf * self._M * ay - self._Izz * omega_dot) / self._L
        return np.array([Fy_f, Fy_r], dtype=np.float64)

    def _observed_axle_forces(self, ay: float, omega_dot: float) -> np.ndarray:
        axle_forces = self._infer_axle_forces_from_sensors(ay, omega_dot)
        if self._axle_force_observer is None:
            return axle_forces

        wheel_ops = self._last_wheel_ops or {}
        required = (
            "front_left_long_slip",
            "front_right_long_slip",
            "rear_left_long_slip",
            "rear_right_long_slip",
            "front_left_slip_angle",
            "front_right_slip_angle",
            "rear_left_slip_angle",
            "rear_right_slip_angle",
            "front_left_Fz",
            "front_right_Fz",
            "rear_left_Fz",
            "rear_right_Fz",
        )
        if not all(k in wheel_ops for k in required):
            return axle_forces

        Fy_f, Fy_r = self._axle_force_observer.predict_axle_forces(
            Fy_f_dyn=float(axle_forces[0]),
            Fy_r_dyn=float(axle_forces[1]),
            kappa_fl=float(wheel_ops["front_left_long_slip"]),
            kappa_fr=float(wheel_ops["front_right_long_slip"]),
            kappa_rl=float(wheel_ops["rear_left_long_slip"]),
            kappa_rr=float(wheel_ops["rear_right_long_slip"]),
            alpha_fl=float(wheel_ops["front_left_slip_angle"]),
            alpha_fr=float(wheel_ops["front_right_slip_angle"]),
            alpha_rl=float(wheel_ops["rear_left_slip_angle"]),
            alpha_rr=float(wheel_ops["rear_right_slip_angle"]),
            Fz_fl=float(wheel_ops["front_left_Fz"]),
            Fz_fr=float(wheel_ops["front_right_Fz"]),
            Fz_rl=float(wheel_ops["rear_left_Fz"]),
            Fz_rr=float(wheel_ops["rear_right_Fz"]),
            sr=float(self._last_sr),
            u=float(max(abs(self._last_u_meas), 0.5)),
            v=float(self._x[0]),
            omega=float(self._x[1]),
            ay=float(ay),
            omega_dot=float(omega_dot),
        )
        return np.array([Fy_f, Fy_r], dtype=np.float64)

    def _observed_wheel_forces_from_sensors(self, ay: float, omega_dot: float) -> Optional[np.ndarray]:
        if self._wheel_force_observer is None:
            return None
        wheel_ops = self._last_wheel_ops or {}
        required = (
            "front_left_long_slip",
            "front_right_long_slip",
            "rear_left_long_slip",
            "rear_right_long_slip",
            "front_left_slip_angle",
            "front_right_slip_angle",
            "rear_left_slip_angle",
            "rear_right_slip_angle",
            "front_left_Fz",
            "front_right_Fz",
            "rear_left_Fz",
            "rear_right_Fz",
        )
        if not all(k in wheel_ops for k in required):
            return None
        axle_forces = self._infer_axle_forces_from_sensors(ay, omega_dot)
        fy_fl, fy_fr = self._wheel_force_observer.predict_wheels(
            Fy_axle=float(axle_forces[0]),
            kappa_left=float(wheel_ops["front_left_long_slip"]),
            kappa_right=float(wheel_ops["front_right_long_slip"]),
            alpha_left=float(wheel_ops["front_left_slip_angle"]),
            alpha_right=float(wheel_ops["front_right_slip_angle"]),
            Fz_left=float(wheel_ops["front_left_Fz"]),
            Fz_right=float(wheel_ops["front_right_Fz"]),
            sr=float(self._last_sr),
            u=float(max(abs(self._last_u_meas), 0.5)),
            v=float(self._x[0]),
            omega=float(self._x[1]),
            ay=float(ay),
            is_front=1.0,
        )
        fy_rl, fy_rr = self._wheel_force_observer.predict_wheels(
            Fy_axle=float(axle_forces[1]),
            kappa_left=float(wheel_ops["rear_left_long_slip"]),
            kappa_right=float(wheel_ops["rear_right_long_slip"]),
            alpha_left=float(wheel_ops["rear_left_slip_angle"]),
            alpha_right=float(wheel_ops["rear_right_slip_angle"]),
            Fz_left=float(wheel_ops["rear_left_Fz"]),
            Fz_right=float(wheel_ops["rear_right_Fz"]),
            sr=0.0,
            u=float(max(abs(self._last_u_meas), 0.5)),
            v=float(self._x[0]),
            omega=float(self._x[1]),
            ay=float(ay),
            is_front=0.0,
        )
        return np.array([fy_fl, fy_fr, fy_rl, fy_rr], dtype=np.float64)

    def _dynamics(self, state, u_meas):
        """Bicycle lateral dynamics + NN/residual force surrogate."""
        v, omega, _n_val = state
        Fy_f, Fy_r = self._axle_forces(state, u_meas)

        v_dot = (Fy_f + Fy_r) / self._M - u_meas * omega
        omega_dot = (self._Lf * Fy_f - self._Lr * Fy_r) / self._Izz

        return np.array([v_dot, omega_dot, 0.0])

    def _predict_sigma(self, sigma_pt, u_meas):
        z_dot = self._dynamics(sigma_pt, u_meas)
        z_new = sigma_pt + self._dt * z_dot
        z_new[2] = np.clip(z_new[2], _N_BOUNDS[0], _N_BOUNDS[1])
        return z_new

    # ── omega-dot (API compat) ────────────────────────────────────────

    def estimate_omega_dot(self, omega: float, t: float) -> Optional[float]:
        self._omega_hist.append(omega)
        self._omega_time.append(t)
        if len(self._omega_hist) < 5:
            return None
        omegas = list(self._omega_hist)
        times = list(self._omega_time)
        dt = (times[-1] - times[0]) / (len(times) - 1)
        if dt < 1e-6:
            return None
        if len(omegas) >= 5:
            return float((-2*omegas[-5] - omegas[-4] + omegas[-2] + 2*omegas[-1])
                         / (10.0 * dt))
        return float((omegas[-1] - omegas[-3]) / (2.0 * dt))

    # ── observe (UKF predict + update) ────────────────────────────────

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
    ) -> bool:
        """Run one UKF predict+update cycle."""
        if abs(u) < 0.5 or self._np_nn is None:
            return False
        if abs(ay_imu) < self._min_excitation:
            return False

        # Store operating conditions for sigma point propagation
        self._last_alpha_f = alpha_f
        self._last_alpha_r = alpha_r
        self._last_Fz_f = Fz_f
        self._last_Fz_r = Fz_r
        self._last_kappa_f = float(kappa if kappa_f is None else kappa_f)
        self._last_kappa_r = float(kappa if kappa_r is None else kappa_r)
        self._last_sr = sr
        self._last_ay = ay_imu
        self._last_u_meas = float(u)
        self._last_wheel_ops = dict(wheel_ops) if wheel_ops is not None else None
        self._meas_x = x_pos
        self._meas_y = y_pos
        self._meas_psi = psi
        
        # Reconstruct steering angle (delta) from slip angle since UKF needs to predict dynamic alpha
        import math
        self._last_delta = alpha_f + math.atan2(v_lateral + self._Lf * omega, max(abs(u), 0.5))

        # Treat measured v / omega as known operating conditions and update the
        # static terrain parameter directly with a 1D Bayesian grid filter.
        # For this strongly nonlinear measurement model, the grid posterior is
        # more reliable than linearizing a scalar latent with a UKF.
        self._x[0] = float(v_lateral)
        self._x[1] = float(omega)
        wheel_force_keys = (
            "front_left_Fy",
            "front_right_Fy",
            "rear_left_Fy",
            "rear_right_Fy",
        )
        measured_wheel_forces = None
        if self._last_wheel_ops is not None and all(k in self._last_wheel_ops for k in wheel_force_keys):
            measured_wheel_forces = np.array([self._last_wheel_ops[k] for k in wheel_force_keys], dtype=np.float64)
        pseudo_wheel_forces = None
        if measured_wheel_forces is None and self._axle_force_observer is None:
            pseudo_wheel_forces = self._observed_wheel_forces_from_sensors(ay_imu, omega_dot)
        use_direct_wheel_obs = self._direct_wheel_force is not None and (measured_wheel_forces is not None or pseudo_wheel_forces is not None)

        if use_direct_wheel_obs:
            y = measured_wheel_forces if measured_wheel_forces is not None else pseudo_wheel_forces
            y_grid = np.zeros((len(self._n_grid), 4), dtype=np.float64)
            for i, n_val in enumerate(self._n_grid):
                state_i = np.array([v_lateral, omega, n_val], dtype=np.float64)
                preds = self._predicted_wheel_forces(state_i, u)
                y_grid[i] = np.array([preds[f"{axle}_{side}_Fy"] for axle, side in (
                    ("front", "left"),
                    ("front", "right"),
                    ("rear", "left"),
                    ("rear", "right"),
                )], dtype=np.float64)
            innov = y.reshape(1, 4) - y_grid
            quad = np.sum((innov * innov) / self._R_wheel_fy_var, axis=1)
        else:
            y = self._observed_axle_forces(ay_imu, omega_dot)
            Rinv = np.linalg.inv(self._R_axle)
            y_grid = np.zeros((len(self._n_grid), 2), dtype=np.float64)
            for i, n_val in enumerate(self._n_grid):
                state_i = np.array([v_lateral, omega, n_val], dtype=np.float64)
                y_grid[i] = self._axle_forces(state_i, u)

            # Diagnostic: instantaneous optimal scale factor
            n_ref_idx = int(np.argmin(np.abs(self._n_grid - self._x[2])))
            y_ref = y_grid[n_ref_idx]
            y_ref_norm2 = float(np.dot(y_ref, y_ref))
            if y_ref_norm2 > 1e-6:
                self._g_inst = float(np.dot(y, y_ref)) / y_ref_norm2

            innov = y.reshape(1, 2) - y_grid
            quad = np.einsum("bi,ij,bj->b", innov, Rinv, innov)
        self._n_loglik_hist.append((-0.5 * quad).astype(np.float64))
        agg_logw = self._n_prior_logw.copy()
        for ll in self._n_loglik_hist:
            agg_logw += ll

        logw = agg_logw - _logsumexp(agg_logw)
        w = np.exp(logw)
        w /= np.sum(w)
        self._n_logw = np.log(np.clip(w, 1e-300, None))

        n_post = float(np.sum(w * self._n_grid))
        n_var = float(np.sum(w * (self._n_grid - n_post) ** 2))
        y_pred = np.sum(w.reshape(-1, 1) * y_grid, axis=0)

        # ── Online gain calibration ──────────────────────────────────
        # Compute optimal gain at current n_post to bridge rig→vehicle
        # scale gap.  g_opt minimises |y_obs − g·y_pred|².
        y_pred_norm2 = float(np.dot(y_pred, y_pred))
        if y_pred_norm2 > 1e-6:
            g_opt = float(np.dot(y, y_pred)) / y_pred_norm2
            g_opt = float(np.clip(g_opt, 0.05, 5.0))
            self._force_gain += self._force_gain_alpha * (g_opt - self._force_gain)

        self._x[2] = n_post
        self._P = np.diag([0.05**2, 0.01**2, max(n_var, 1e-10)])

        self._obs_count += 1
        self._total_obs += 1

        if self._obs_count % 10 == 0:
            ay_pred = (y_pred[0] + y_pred[1]) / self._M
            wd_pred = (self._Lf * y_pred[0] - self._Lr * y_pred[1]) / self._Izz
            n50 = int(np.argmin(np.abs(self._n_grid - 0.5)))
            n70 = int(np.argmin(np.abs(self._n_grid - 0.7)))
            n110 = len(self._n_grid) - 1
            print(
                f"    [UKF DEBUG] v_meas={v_lateral:+.2f} | "
                f"w_meas={omega:+.2f} | "
                f"ay_meas={ay_imu:+.2f}, ay_pred={ay_pred:+.2f} | "
                f"wd_meas={omega_dot:+.2f}, wd_pred={wd_pred:+.2f} | "
                f"n={self._x[2]:.2f} g={self._force_gain:.3f} g_inst={self._g_inst:.3f} | "
                f"w[0.5]={w[n50]:.3f} w[0.7]={w[n70]:.3f} w[1.1]={w[n110]:.3f} | "
                f"y_grid[0.5]={y_grid[n50,0]:+.0f},{y_grid[n50,1]:+.0f} "
                f"y_grid[1.1]={y_grid[n110,0]:+.0f},{y_grid[n110,1]:+.0f} "
                f"y_obs={y[0]:+.0f},{y[1]:+.0f}"
            )


        # Update mu EMA for display
        mu_cent = abs(u * omega) / 9.81
        if mu_cent > self._mu_ema:
            self._mu_ema += 0.10 * (mu_cent - self._mu_ema)
        else:
            self._mu_ema += 0.015 * (mu_cent - self._mu_ema)

        # 2nd-order critically damped low-pass filter (approx 0.5Hz cutoff)
        alpha = 0.01
        self._n_ema1 += alpha * (self._x[2] - self._n_ema1)
        self._n_ema2 += alpha * (self._n_ema1 - self._n_ema2)

        return True

    # ── should_update / estimate ──────────────────────────────────────

    def should_update(self) -> bool:
        return self._obs_count >= self._update_interval

    def estimate(self) -> Tuple[Dict[str, float], float]:
        """Read off the current UKF estimate of n."""
        self._obs_count = 0

        n_est = float(self._n_ema2)
        n_std = float(np.sqrt(max(self._P[2, 2], 1e-10)))

        # Confidence from uncertainty: low std = high confidence
        self._confidence = float(np.clip(1.0 - n_std / 0.3, 0.0, 1.0))

        # Map n to nearest preset for terrain name
        best_name, best_dist = "unknown", float('inf')
        for name, preset in TERRAIN_PRESETS.items():
            dist = abs(preset['n'] - n_est)
            if dist < best_dist:
                best_dist = dist
                best_name = name
        self._terrain_name = best_name

        # Build a self-consistent terrain parameter vector for the NN / MPC.
        self._estimated_params = self._terrain_params_for_n(n_est)

        self._last_loss = n_std
        return self._estimated_params, self._confidence

    # ── accessors ─────────────────────────────────────────────────────

    def get_terrain_mpc_params(self) -> Dict[str, float]:
        return dict(self._estimated_params)

    def get_bekker_n(self) -> float:
        return float(self._n_ema2)

    @property
    def mu_estimate(self) -> float:
        return self._mu_ema

    @property
    def mu_peak(self) -> float:
        return self._mu_ema

    @property
    def loss(self) -> float:
        return self._last_loss

    @property
    def confidence(self) -> float:
        return self._confidence

    @property
    def total_observations(self) -> int:
        return self._total_obs

    def describe(self) -> str:
        n_est = float(self._n_ema2)
        n_std = np.sqrt(max(self._P[2, 2], 1e-10))
        return (
            f"→ {self._terrain_name} (n={n_est:.3f}±{n_std:.3f}, "
            f"conf={self._confidence:.0%})"
        )
