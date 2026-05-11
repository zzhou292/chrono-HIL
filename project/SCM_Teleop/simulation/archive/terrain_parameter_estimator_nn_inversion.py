#!/usr/bin/env python3
"""
Online Terrain Parameter Estimator
====================================

Estimates Bekker/Mohr-Coulomb soil parameters from closed-loop vehicle dynamics
by inverting the trained NN tire model.

Sensor-realistic approach:
  - Lateral forces estimated from IMU (ay, omega_dot) + vehicle geometry
  - No direct tire force measurement required (no oracle data)
  - Gradient-based optimization through the NN to find terrain params that
    best explain observed forces

Replaces the discrete terrain classifier with continuous parameter estimation,
eliminating the clay/dirt confusion problem and supporting unknown terrains
within the training envelope.

Uses pure numpy forward pass + scipy L-BFGS-B optimization to avoid
PyTorch BLAS deadlock when coexisting with ACADOS/CasADi in the same process.

Integration:
  - Runs inside acados_mpc_controller_node (no separate process needed)
  - Publishes estimated terrain params for MPC parameter update
  - Maintains a sliding window of recent observations

Public API:
  estimator = TerrainParameterEstimator(nn_model_dir, initial_terrain_params)
  estimator.observe(alpha_f, alpha_r, u, Fz_f, Fz_r, sr, ay_imu, omega_dot)
  params, confidence = estimator.estimate()   # dict of {Kphi, Kc, n, c, phi, k}
"""

from __future__ import annotations

import pickle
from collections import deque
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import torch
from scipy.optimize import minimize

from param_consistency import (
    HMMWV_VEHICLE_PARAMS,
    TRAINING_RANGES_V6,
)


# ============================================================================
# Numpy-based NN forward passes (avoids PyTorch BLAS deadlock with ACADOS)
# ============================================================================

class _NumpyMLP:
    """Pure-numpy MLP forward pass.  Weights loaded from PyTorch checkpoint."""

    def __init__(self, weights: dict):
        layer_ids = sorted(set(
            int(k.split('.')[1])
            for k in weights if k.startswith('layers.') and 'weight' in k
        ))
        self._layers: List[Tuple[np.ndarray, np.ndarray]] = []
        self._last_idx = layer_ids[-1]
        for i in layer_ids:
            W = np.asarray(weights[f'layers.{i}.weight'], dtype=np.float64)
            b = np.asarray(weights[f'layers.{i}.bias'], dtype=np.float64)
            self._layers.append((W, b, i < self._last_idx))

    def __call__(self, x: np.ndarray) -> np.ndarray:
        """Forward pass: x shape (N, in_dim) -> (N, out_dim)."""
        h = x
        for W, b, use_tanh in self._layers:
            h = h @ W.T + b
            if use_tanh:
                h = np.tanh(h)
        return h


class _NumpyResNet:
    """Pure-numpy ResNet forward pass."""

    def __init__(self, weights: dict, n_blocks: int):
        self.n_blocks = n_blocks
        self._W_in = np.asarray(weights['input_proj.weight'], dtype=np.float64)
        self._b_in = np.asarray(weights['input_proj.bias'], dtype=np.float64)
        self._blocks: List[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = []
        for blk in range(n_blocks):
            W1 = np.asarray(weights[f'blocks.{blk}.fc1.weight'], dtype=np.float64)
            b1 = np.asarray(weights[f'blocks.{blk}.fc1.bias'], dtype=np.float64)
            W2 = np.asarray(weights[f'blocks.{blk}.fc2.weight'], dtype=np.float64)
            b2 = np.asarray(weights[f'blocks.{blk}.fc2.bias'], dtype=np.float64)
            self._blocks.append((W1, b1, W2, b2))
        self._W_out = np.asarray(weights['output_proj.weight'], dtype=np.float64)
        self._b_out = np.asarray(weights['output_proj.bias'], dtype=np.float64)

    def __call__(self, x: np.ndarray) -> np.ndarray:
        h = np.tanh(x @ self._W_in.T + self._b_in)
        for W1, b1, W2, b2 in self._blocks:
            residual = h
            h = np.tanh(h @ W1.T + b1)
            h = h @ W2.T + b2
            h = np.tanh(h + residual)
        return h @ self._W_out.T + self._b_out


# ============================================================================
# Sensor-realistic force estimation from IMU
# ============================================================================

def estimate_tire_forces_from_imu(
    ay_imu: float,
    omega_dot: float,
    *,
    M: float = HMMWV_VEHICLE_PARAMS["M"],
    Izz: float = HMMWV_VEHICLE_PARAMS["Izz"],
    Lf: float = HMMWV_VEHICLE_PARAMS["Lf"],
    Lr: float = HMMWV_VEHICLE_PARAMS["Lr"],
) -> Tuple[float, float]:
    """Estimate front/rear axle lateral forces from IMU measurements.

    From bicycle model:
        M * ay = Fy_f + Fy_r           (lateral force balance)
        Izz * omega_dot = Lf * Fy_f - Lr * Fy_r   (yaw moment)

    Solving:
        Fy_f = (M * ay * Lr + Izz * omega_dot) / (Lf + Lr)
        Fy_r = (M * ay * Lf - Izz * omega_dot) / (Lf + Lr)

    Returns (Fy_f_total, Fy_r_total) in Newtons (both wheels per axle).
    """
    L = Lf + Lr
    Fy_f = (M * ay_imu * Lr + Izz * omega_dot) / L
    Fy_r = (M * ay_imu * Lf - Izz * omega_dot) / L
    return float(Fy_f), float(Fy_r)


# ============================================================================
# Terrain parameter bounds (from training range)
# ============================================================================

_TERRAIN_BOUNDS = {
    "Kphi": TRAINING_RANGES_V6["bekker_Kphi"],
    "Kc":   TRAINING_RANGES_V6["bekker_Kc"],
    "n":    TRAINING_RANGES_V6["bekker_n"],
    "c":    TRAINING_RANGES_V6["mohr_cohesion"],
    "phi":  TRAINING_RANGES_V6["mohr_friction"],   # radians
    "k":    TRAINING_RANGES_V6["janosi_shear"],
}


# ============================================================================
# Main estimator
# ============================================================================

class TerrainParameterEstimator:
    """Online terrain parameter estimation via NN inversion.

    Uses gradient descent through the trained NN tire model to find the
    Bekker/Mohr-Coulomb parameters that best explain observed tire forces
    estimated from IMU data.

    Args:
        model_dir: Path to NN model directory (best_terrain_nn.pt + scalers.pkl)
        initial_terrain: Initial terrain params dict {Kphi, Kc, n, c, phi, k}
                        where phi is in degrees (will be converted to radians)
        window_size: Number of observations in the sliding window
        update_interval: Run optimization every N observations
        lr: Learning rate for Adam optimizer
        n_steps: Number of gradient descent steps per update
        min_excitation: Minimum |ay| (m/s²) to include observation
    """

    def __init__(
        self,
        model_dir: str | Path,
        initial_terrain: Dict[str, float],
        *,
        window_size: int = 50,
        update_interval: int = 10,
        lr: float = 0.01,
        n_steps: int = 20,
        min_excitation: float = 0.3,
    ):
        model_dir = Path(model_dir)

        # Load NN weights (use torch only for loading, then convert to numpy)
        model_path = model_dir / 'best_terrain_nn.pt'
        checkpoint = torch.load(model_path, map_location='cpu', weights_only=False)
        state_dict = checkpoint['model_state_dict'] if isinstance(checkpoint, dict) else checkpoint

        # Remap old-style keys
        if any(k.startswith('layer') and not k.startswith('layers') for k in state_dict):
            remap = {}
            idx = 0
            while f'layer{idx+1}.weight' in state_dict:
                remap[f'layer{idx+1}.weight'] = f'layers.{idx}.weight'
                remap[f'layer{idx+1}.bias'] = f'layers.{idx}.bias'
                idx += 1
            state_dict = {remap.get(k, k): v for k, v in state_dict.items()}

        weights = {k: v.detach().numpy() for k, v in state_dict.items()}

        # Detect architecture and build numpy-based NN
        if 'input_proj.weight' in weights:
            n_blocks = sum(1 for k in weights if k.startswith('blocks.') and k.endswith('.fc1.weight'))
            self._np_nn = _NumpyResNet(weights, n_blocks)
        else:
            self._np_nn = _NumpyMLP(weights)

        # Load scalers (numpy arrays)
        scaler_path = model_dir / 'scalers.pkl'
        with open(scaler_path, 'rb') as f:
            scalers = pickle.load(f)
        self._X_mean = np.asarray(scalers['X'].mean_, dtype=np.float64)
        self._X_scale = np.asarray(scalers['X'].scale_, dtype=np.float64)
        self._y_mean = np.asarray(scalers['y'].mean_, dtype=np.float64)
        self._y_scale = np.asarray(scalers['y'].scale_, dtype=np.float64)

        # Initial terrain params (convert phi from degrees to radians for NN)
        phi_rad = np.radians(initial_terrain.get('phi', initial_terrain.get('friction_angle', 30.0)))
        self._terrain_params = np.array([
            float(initial_terrain['Kphi']),
            float(initial_terrain['Kc']),
            float(initial_terrain['n']),
            float(initial_terrain.get('c', initial_terrain.get('cohesion', 1000.0))),
            phi_rad,
            float(initial_terrain.get('k', initial_terrain.get('janosi_shear', 0.025))),
        ], dtype=np.float64)

        # Store initial for regularization
        self._initial_params = self._terrain_params.copy()

        # Optimization settings
        self._lr = lr
        self._n_steps = n_steps
        self._window_size = window_size
        self._update_interval = update_interval
        self._min_excitation = min_excitation

        # Observation buffer
        self._obs_buffer: deque = deque(maxlen=window_size)
        self._obs_count = 0
        self._total_obs = 0

        # Omega-dot filter
        self._omega_buffer: deque = deque(maxlen=7)
        self._omega_time_buffer: deque = deque(maxlen=7)

        # Output state
        self._estimated_params = dict(initial_terrain)
        self._confidence = 0.0
        self._last_loss = float('inf')

        # Bounds for L-BFGS-B
        self._param_lo = np.array([
            _TERRAIN_BOUNDS["Kphi"][0],
            _TERRAIN_BOUNDS["Kc"][0],
            _TERRAIN_BOUNDS["n"][0],
            _TERRAIN_BOUNDS["c"][0],
            _TERRAIN_BOUNDS["phi"][0],
            _TERRAIN_BOUNDS["k"][0],
        ], dtype=np.float64)
        self._param_hi = np.array([
            _TERRAIN_BOUNDS["Kphi"][1],
            _TERRAIN_BOUNDS["Kc"][1],
            _TERRAIN_BOUNDS["n"][1],
            _TERRAIN_BOUNDS["c"][1],
            _TERRAIN_BOUNDS["phi"][1],
            _TERRAIN_BOUNDS["k"][1],
        ], dtype=np.float64)
        self._bounds = list(zip(self._param_lo, self._param_hi))

    def estimate_omega_dot(self, omega: float, t: float) -> Optional[float]:
        """Estimate yaw acceleration from filtered omega history.

        Uses central finite difference on buffered omega values.
        Returns None if insufficient history.
        """
        self._omega_buffer.append(omega)
        self._omega_time_buffer.append(t)

        if len(self._omega_buffer) < 5:
            return None

        # 5-point Savitzky-Golay derivative (polyorder=2, deriv=1)
        # Coefficients for equally-spaced points: [-2, -1, 0, 1, 2] / (10*h)
        omegas = list(self._omega_buffer)
        times = list(self._omega_time_buffer)
        n = len(omegas)
        dt_avg = (times[-1] - times[0]) / (n - 1)
        if dt_avg < 1e-6:
            return None

        # Central difference using last 5 points
        if n >= 5:
            omega_dot = (-2*omegas[-5] - omegas[-4] + omegas[-2] + 2*omegas[-1]) / (10.0 * dt_avg)
        else:
            omega_dot = (omegas[-1] - omegas[-3]) / (2.0 * dt_avg)

        return float(omega_dot)

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
    ) -> bool:
        """Add an observation to the sliding window.

        Args:
            kappa: Longitudinal slip ratio
            alpha_f: Front slip angle (rad)
            alpha_r: Rear slip angle (rad)
            u: Forward speed (m/s)
            Fz_f: Front axle vertical load (N, per-wheel mean)
            Fz_r: Rear axle vertical load (N, per-wheel mean)
            sr: Steering rate (rad/s)
            ay_imu: Lateral acceleration from IMU (m/s²)
            omega_dot: Yaw acceleration (rad/s²), pre-filtered

        Returns True if observation was accepted (sufficient excitation).
        """
        # Excitation gate: skip low-lateral-acceleration observations
        if abs(ay_imu) < self._min_excitation:
            return False

        # Speed gate: skip very low speeds (NN unreliable)
        if abs(u) < 1.5:
            return False

        # Estimate actual lateral forces from IMU (per-axle total)
        Fy_f_est, Fy_r_est = estimate_tire_forces_from_imu(ay_imu, omega_dot)

        # Convert to per-wheel (NN predicts single-wheel forces)
        self._obs_buffer.append({
            'kappa': kappa,
            'alpha_f': alpha_f,
            'alpha_r': alpha_r,
            'u': u,
            'Fz_f': Fz_f,
            'Fz_r': Fz_r,
            'sr': sr,
            'Fy_f': Fy_f_est / 2.0,   # per-wheel
            'Fy_r': Fy_r_est / 2.0,   # per-wheel
        })
        self._obs_count += 1
        self._total_obs += 1
        return True

    def _build_batch_arrays(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Build batch input arrays from observation buffer.

        Returns:
            ops_front: (N, 5) operating conditions for front axle
            ops_rear: (N, 5) operating conditions for rear axle
            Fy_target: (N, 2) target [Fy_f, Fy_r]
        """
        N = len(self._obs_buffer)
        ops_front = np.zeros((N, 5), dtype=np.float64)
        ops_rear = np.zeros((N, 5), dtype=np.float64)
        Fy_target = np.zeros((N, 2), dtype=np.float64)

        for i, obs in enumerate(self._obs_buffer):
            ops_front[i] = [obs['kappa'], obs['alpha_f'], obs['u'], obs['Fz_f'], obs['sr']]
            ops_rear[i] = [obs['kappa'], obs['alpha_r'], obs['u'], obs['Fz_r'], obs['sr']]
            Fy_target[i] = [obs['Fy_f'], obs['Fy_r']]

        return ops_front, ops_rear, Fy_target

    def _nn_forward(self, ops: np.ndarray, terrain_p: np.ndarray) -> np.ndarray:
        """Run NN forward pass for a batch of operating conditions (pure numpy).

        Args:
            ops: (N, 5) [kappa, alpha, u, Fz, sr]
            terrain_p: (6,) [Kphi, Kc, n, c, phi, k]

        Returns:
            (N, 2) [Fx, Fy] in Newtons
        """
        N = ops.shape[0]
        terrain_expanded = np.broadcast_to(terrain_p, (N, 6))
        x_full = np.concatenate([ops, terrain_expanded], axis=1)  # (N, 11)

        # Scale inputs
        x_scaled = (x_full - self._X_mean) / self._X_scale

        # Forward pass
        y_scaled = self._np_nn(x_scaled)

        # Unscale outputs
        return y_scaled * self._y_scale + self._y_mean  # (N, 2) [Fx, Fy]

    def should_update(self) -> bool:
        """Check if we have enough observations for an update."""
        return (self._obs_count >= self._update_interval
                and len(self._obs_buffer) >= 10)

    def estimate(self) -> Tuple[Dict[str, float], float]:
        """Run terrain parameter estimation via scipy L-BFGS-B.

        Optimizes terrain params to minimize ||NN(state, terrain) - F_imu||²
        using bounded quasi-Newton optimization (pure numpy, no PyTorch).

        Returns:
            (params_dict, confidence) where params_dict has keys matching
            TERRAIN_PRESETS format and confidence in [0, 1].
        """
        if len(self._obs_buffer) < 10:
            return self._estimated_params, self._confidence

        self._obs_count = 0

        ops_front, ops_rear, Fy_target = self._build_batch_arrays()

        # Only optimize the 2 most terrain-sensitive parameters: n (idx 2)
        # and phi (idx 4).  Keep Kphi, Kc, c, k fixed to avoid the
        # underdetermined-system problem (Fy alone cannot constrain 6 params).
        base_p = self._terrain_params.copy()   # full 6-vector
        n_idx, phi_idx = 2, 4
        x0 = np.array([base_p[n_idx], base_p[phi_idx]])
        bounds_2 = [
            (self._param_lo[n_idx], self._param_hi[n_idx]),
            (self._param_lo[phi_idx], self._param_hi[phi_idx]),
        ]
        init_2 = np.array([self._initial_params[n_idx], self._initial_params[phi_idx]])
        range_2 = np.array([
            self._param_hi[n_idx] - self._param_lo[n_idx],
            self._param_hi[phi_idx] - self._param_lo[phi_idx],
        ])

        def objective(x2: np.ndarray) -> float:
            p = base_p.copy()
            p[n_idx] = x2[0]
            p[phi_idx] = x2[1]
            pred_f = self._nn_forward(ops_front, p)  # (N, 2)
            pred_r = self._nn_forward(ops_rear, p)   # (N, 2)
            Fy_err_f = pred_f[:, 1] - Fy_target[:, 0]
            Fy_err_r = pred_r[:, 1] - Fy_target[:, 1]
            loss = float(np.mean(Fy_err_f**2) + np.mean(Fy_err_r**2))
            reg = 0.001 * float(np.sum(((x2 - init_2) / range_2) ** 2))
            return loss + reg

        result = minimize(
            objective, x0,
            method='L-BFGS-B',
            bounds=bounds_2,
            options={'maxiter': self._n_steps, 'ftol': 1e-8},
        )

        best_2 = result.x
        best_loss = objective(best_2)

        # Write optimised n and phi back into the full parameter vector
        self._terrain_params[n_idx] = best_2[0]
        self._terrain_params[phi_idx] = best_2[1]
        self._last_loss = best_loss

        # Compute confidence
        rmse = np.sqrt(best_loss / 2.0)
        self._confidence = float(np.clip(1.0 - rmse / 2000.0, 0.0, 1.0))

        # Build output dict
        p = self._terrain_params
        self._estimated_params = {
            'Kphi': float(p[0]),
            'Kc': float(p[1]),
            'n': float(p[2]),
            'c': float(p[3]),
            'cohesion': float(p[3]),
            'phi': float(np.degrees(p[4])),
            'friction_angle': float(np.degrees(p[4])),
            'k': float(p[5]),
            'janosi_shear': float(p[5]),
        }

        return self._estimated_params, self._confidence

    def get_terrain_mpc_params(self) -> Dict[str, float]:
        """Get terrain params in the format expected by MPC (phi in degrees)."""
        p = self._terrain_params
        return {
            'Kphi': float(p[0]),
            'Kc': float(p[1]),
            'n': float(p[2]),
            'c': float(p[3]),
            'phi': float(np.degrees(p[4])),  # convert rad → deg for MPC
            'k': float(p[5]),
        }

    def get_bekker_n(self) -> float:
        """Get estimated Bekker sinkage exponent n."""
        return float(self._terrain_params[2])

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
        """Human-readable summary of current estimate."""
        p = self._estimated_params
        return (
            f"Terrain estimate (conf={self._confidence:.2f}, loss={self._last_loss:.0f}): "
            f"Kphi={p['Kphi']/1e6:.2f}MPa, Kc={p['Kc']/1e3:.1f}kPa, n={p['n']:.2f}, "
            f"c={p['c']:.0f}Pa, phi={p['phi']:.1f}°, k={p['k']:.4f}m"
        )


# ============================================================================
# Quick smoke test
# ============================================================================

if __name__ == "__main__":
    import sys
    from param_consistency import TERRAIN_PRESETS, terrain_preset_to_mpc_params

    model_dir = sys.argv[1] if len(sys.argv) > 1 else "nn_models/paper_v2_mlp_16_4"
    true_terrain = sys.argv[2] if len(sys.argv) > 2 else "clay"

    true_params = TERRAIN_PRESETS[true_terrain]
    # Start from a "wrong" initial guess (dirt)
    init_params = TERRAIN_PRESETS["dirt"]

    print(f"True terrain: {true_terrain}")
    print(f"Initial guess: dirt")
    print(f"True phi={true_params['friction_angle']}°, n={true_params['n']}")

    est = TerrainParameterEstimator(
        model_dir, init_params,
        window_size=30, update_interval=5, lr=0.02, n_steps=30,
    )

    # Simulate some observations with known forces
    # (In real use, ay_imu and omega_dot come from the vehicle)
    np.random.seed(42)
    for i in range(60):
        alpha = np.random.uniform(-0.15, 0.15)
        u = np.random.uniform(3, 7)
        Fz = np.random.uniform(2500, 4500)
        # Fake IMU-derived forces (in practice these come from vehicle dynamics)
        ay = np.random.uniform(-2, 2)
        omega_dot = np.random.uniform(-0.5, 0.5)

        est.observe(
            kappa=0.0, alpha_f=alpha, alpha_r=alpha*0.5,
            u=u, Fz_f=Fz, Fz_r=Fz * 1.1, sr=0.0,
            ay_imu=ay, omega_dot=omega_dot,
        )

        if est.should_update():
            params, conf = est.estimate()
            print(f"  Step {i}: {est.describe()}")

    print(f"\nFinal: {est.describe()}")
