"""
Trajectory-matching terrain parameter estimator.

Instead of UKF which requires accurate single-step predictions,
this estimator buffers recent state history and finds the sinkage
exponent `n` that best explains the observed trajectory.

This is more robust to model mismatch because:
1. It directly minimizes trajectory prediction error
2. Multiple n values are evaluated explicitly (grid search)
3. The optimization is over a window of observations, reducing noise sensitivity
"""

import numpy as np
import torch
import pickle
from pathlib import Path
from collections import deque


class TrajectoryMatchingEstimator:
    """
    Batch-style terrain parameter estimator using trajectory matching.
    
    Maintains a sliding window of recent state observations. For each new
    observation, evaluates multiple candidate n values by predicting
    trajectories and choosing the n that minimizes prediction error.
    """
    
    def __init__(self, vehicle_params, nn_model, scaler_X, scaler_y,
                 base_terrain_params, dt, n_init=0.7,
                 window_size=8, n_grid_size=9, debug=False,
                 use_steering_rate=False):
        """
        Args:
            vehicle_params: dict with M, Izz, Lf, Lr
            nn_model:       PyTorch TerrainNN (eval mode)
            scaler_X/y:     sklearn StandardScalers for NN inputs/outputs
            base_terrain_params: dict with Kphi, Kc, c, phi, k
            dt:             timestep between observations (s)
            n_init:         initial estimate
            window_size:    number of recent observations to keep (reduced for speed)
            n_grid_size:    number of candidate n values to evaluate (reduced for speed)
            debug:          print diagnostic info
            use_steering_rate: if True, use v6 12-input format with steering_rate
        """
        self.M = vehicle_params['M']
        self.Izz = vehicle_params['Izz']
        self.Lf = vehicle_params['Lf']
        self.Lr = vehicle_params['Lr']
        
        self.nn_model = nn_model
        self.scaler_X = scaler_X
        self.scaler_y = scaler_y
        self.base_terrain = base_terrain_params
        self.dt = dt
        self._debug = debug
        self.is_v6 = use_steering_rate  # v6 format: different column order + phi in radians
        
        # Grid of candidate n values
        self.n_grid = np.linspace(0.4, 1.4, n_grid_size)
        
        # Sliding window of observations
        self.window_size = window_size
        self.obs_buffer = deque(maxlen=window_size)
        
        # Current estimate
        self._n_est = n_init
        self._n_est_smooth = n_init  # Exponential smoothed estimate
        self.smooth_alpha = 0.3  # Smoothing factor (higher = more responsive)
        
        # Error weights for trajectory matching
        self.w_v = 1.0      # Weight on lateral velocity error
        self.w_omega = 5.0  # Weight on yaw rate error (more observable)
        self.w_psi = 0.5    # Weight on heading error
        
        # History for plotting
        self.n_history = [n_init]
        self.t_history = [0.0]
        
        # Bounds
        self.n_min = 0.4
        self.n_max = 1.4
        
    def _nn_forces(self, u, v, omega, delta, n_val, steering_rate=0.0):
        """Evaluate NN lateral forces."""
        terrain = self.base_terrain
        u_safe = max(abs(u), 0.5)
        
        alpha_f = delta - np.arctan2(v + self.Lf * omega, u_safe)
        alpha_r = -np.arctan2(v - self.Lr * omega, u_safe)
        
        L = self.Lf + self.Lr
        Fz_f = self.M * 9.81 * self.Lr / L / 2.0
        Fz_r = self.M * 9.81 * self.Lf / L / 2.0
        
        results = []
        for alpha, Fz in [(alpha_f, Fz_f), (alpha_r, Fz_r)]:
            if self.is_v6:
                # v6 Dallas format (11 inputs): slip_ratio, slip_angle, velocity, Fz, steering_rate,
                #   Kphi, Kc, n, c, phi(radians), k
                phi_rad = np.radians(terrain['phi'])
                x_raw = np.array([[0.0, alpha, u_safe, Fz, steering_rate,
                                   terrain['Kphi'], terrain['Kc'], n_val,
                                   terrain['c'], phi_rad, terrain['k']]])
            else:
                # v3 legacy format (11 inputs): Fz, slip_angle, slip_ratio, camber, velocity,
                #   Kphi, Kc, n, c, phi(degrees), k
                x_raw = np.array([[Fz, alpha, 0.0, 0.0, u_safe,
                                   terrain['Kphi'], terrain['Kc'], n_val,
                                   terrain['c'], terrain['phi'], terrain['k']]])
            x_scaled = self.scaler_X.transform(x_raw)
            with torch.no_grad():
                y_scaled = self.nn_model(
                    torch.tensor(x_scaled, dtype=torch.float32)).numpy()
            y_out = self.scaler_y.inverse_transform(y_scaled)
            results.append(-2.0 * y_out[0, 1])
        
        return results[0], results[1]
    
    def _simulate_step(self, state, delta, ax, n_val, steering_rate=0.0):
        """Simulate one timestep with given n."""
        u, v, omega, psi = state
        
        Fyf, Fyr = self._nn_forces(u, v, omega, delta, n_val, steering_rate)
        
        # Dynamics
        v_dot = (Fyf + Fyr) / self.M - u * omega
        omega_dot = (Fyf * self.Lf - Fyr * self.Lr) / self.Izz
        psi_dot = omega
        u_dot = ax
        
        # Simple Euler integration (2 substeps for stability)
        n_sub = 2
        dt_sub = self.dt / n_sub
        for _ in range(n_sub):
            u = u + u_dot * dt_sub
            v = v + v_dot * dt_sub
            omega = omega + omega_dot * dt_sub
            psi = psi + psi_dot * dt_sub
        
        return (u, v, omega, psi)
    
    def _evaluate_n(self, n_val, obs_list):
        """
        Evaluate trajectory error for a candidate n value.
        
        Simulates forward from the first observation using recorded
        deltas and compares to actual observed states.
        """
        if len(obs_list) < 3:
            return float('inf')
        
        # Start from first observation
        first = obs_list[0]
        state = (first['u'], first['v'], first['omega'], first['psi'])
        
        total_error = 0.0
        count = 0
        
        # Simulate forward and compare
        for i in range(1, len(obs_list)):
            prev = obs_list[i-1]
            curr = obs_list[i]
            
            # Simulate one step
            sr = prev.get('steering_rate', 0.0)
            state = self._simulate_step(state, prev['delta'], prev['ax'], n_val, sr)
            pred_u, pred_v, pred_omega, pred_psi = state
            
            # Error vs actual
            err_v = (pred_v - curr['v'])**2
            err_omega = (pred_omega - curr['omega'])**2
            err_psi = (np.sin(pred_psi - curr['psi']))**2  # Angle error
            
            total_error += self.w_v * err_v + self.w_omega * err_omega + self.w_psi * err_psi
            count += 1
        
        return total_error / max(count, 1)
    
    def step(self, x_meas, y_meas, psi_meas, u, v, omega, delta, ax, time=None,
             steering_rate=0.0):
        """
        Update n estimate with a new observation.
        
        Args match the V2 UKF interface for compatibility.
        steering_rate: delta_dot for v6 format (rad/s)
        """
        # Buffer the observation
        obs = {
            'time': time,
            'x': x_meas, 'y': y_meas, 'psi': psi_meas,
            'u': u, 'v': v, 'omega': omega,
            'delta': delta, 'ax': ax,
            'steering_rate': steering_rate
        }
        self.obs_buffer.append(obs)
        
        # Need enough data
        if len(self.obs_buffer) < 5:
            return self._n_est
        
        # Convert buffer to list for evaluation
        obs_list = list(self.obs_buffer)
        
        # Grid search over n values
        best_n = self._n_est
        best_error = float('inf')
        
        for n_cand in self.n_grid:
            error = self._evaluate_n(n_cand, obs_list)
            if error < best_error:
                best_error = error
                best_n = n_cand
        
        # Update estimate with smoothing
        self._n_est = best_n
        self._n_est_smooth = (self.smooth_alpha * best_n + 
                              (1 - self.smooth_alpha) * self._n_est_smooth)
        
        # Debug output
        if self._debug and time is not None and int(time * 10) % 20 == 0:
            print(f"    TRAJ: n_raw={best_n:.3f} n_smooth={self._n_est_smooth:.3f} "
                  f"error={best_error:.4f}")
        
        # Record history
        if time is not None:
            self.n_history.append(float(self._n_est_smooth))
            self.t_history.append(time)
        
        return float(self._n_est_smooth)
    
    @property
    def n_estimated(self):
        return float(self._n_est_smooth)
    
    @property
    def n_hat(self):
        return self.n_estimated
    
    @property
    def bias_v(self):
        return 0.0
    
    @property
    def bias_omega(self):
        return 0.0
    
    # P property for compatibility with UKF debug code
    @property
    def P(self):
        # Return dummy covariance
        return np.diag([0.01]*7)


def load_ukf(model_path, scaler_path, vehicle_params, base_terrain_params,
             dt=0.1, n_init=0.7, measurement_noise=None, debug=False,
             **kwargs):
    """
    Load the NN and create a TrajectoryMatchingEstimator.
    
    Function name kept as load_ukf for backward compatibility.
    """
    import sys
    sys.path.append(str(Path(__file__).parent.parent / "nn_training"))
    from train_terrain_nn import TerrainNN

    checkpoint = torch.load(model_path, weights_only=False, map_location='cpu')
    state_dict = checkpoint.get('model_state_dict', checkpoint) \
        if isinstance(checkpoint, dict) else checkpoint

    hidden_sizes = None
    if isinstance(checkpoint, dict) and 'hidden_sizes' in checkpoint:
        hidden_sizes = checkpoint['hidden_sizes']

    if any(k.startswith('layer') and not k.startswith('layers.') for k in state_dict):
        old_to_new = {}
        idx = 0
        while f'layer{idx+1}.weight' in state_dict:
            old_to_new[f'layer{idx+1}.weight'] = f'layers.{idx}.weight'
            old_to_new[f'layer{idx+1}.bias'] = f'layers.{idx}.bias'
            idx += 1
        state_dict = {old_to_new.get(k, k): v for k, v in state_dict.items()}

    if hidden_sizes is None and any(k.startswith('layers.') for k in state_dict):
        layer_ids = sorted(set(int(k.split('.')[1])
                               for k in state_dict if k.startswith('layers.')))
        hidden_sizes = [state_dict[f'layers.{i}.weight'].shape[0]
                        for i in layer_ids[:-1]]

    # Detect format: v6 has phi in radians (mean < 1.0), v3 has phi in degrees (mean > 1.0)
    with open(scaler_path, 'rb') as f:
        scalers = pickle.load(f)
    
    input_size = len(scalers['X'].mean_)
    phi_mean = scalers['X'].mean_[9]  # phi is at index 9 in both formats
    is_v6 = phi_mean < 1.0  # Radians will be < 1, degrees will be >> 1
    
    nn_model = TerrainNN(input_size=input_size, output_size=2, hidden_sizes=hidden_sizes)
    nn_model.load_state_dict(state_dict)
    nn_model.eval()

    estimator = TrajectoryMatchingEstimator(
        vehicle_params=vehicle_params,
        nn_model=nn_model,
        scaler_X=scalers['X'],
        scaler_y=scalers['y'],
        base_terrain_params=base_terrain_params,
        dt=dt,
        n_init=n_init,
        debug=debug,
        use_steering_rate=is_v6,
    )

    fmt = f"v6 (phi_mean={phi_mean:.2f} rad, has steering_rate)" if is_v6 else f"v3 (phi_mean={phi_mean:.1f}°)"
    print(f"✓ TerrainEstimator (TrajMatch) initialized: n_init={n_init:.2f}, dt={dt}s, format={fmt}")
    return estimator
