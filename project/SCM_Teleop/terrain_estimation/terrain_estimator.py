"""
Online terrain parameter estimator following Dallas et al. (2021).

Implements an Unscented Kalman Filter (UKF) for estimating the sinkage
exponent `n` by propagating a bicycle model with NN-predicted lateral forces
and comparing predictions to measured vehicle states.

Reference:
    J. Dallas et al., "Terrain Adaptive Trajectory Planning and Tracking on
    Deformable Terrains," IEEE Trans. Veh. Tech., vol. 70, no. 11, 2021.

The key insight is that the UKF:
1. Propagates state [u, v, omega, n] forward using bicycle dynamics + NN forces
2. Compares PREDICTED states to MEASURED states
3. Uses the innovation (prediction error) to correct both vehicle state AND n

This avoids the noise-amplification issues of finite-differencing accelerations.
"""

import numpy as np
import torch
import pickle
from pathlib import Path


class TerrainEstimator:
    """
    Unscented Kalman Filter for sinkage exponent estimation.

    State vector: z = [u, v, omega, n]
        u     - longitudinal velocity (m/s)
        v     - lateral velocity (m/s)  
        omega - yaw rate (rad/s)
        n     - sinkage exponent (terrain parameter)

    Measurements: y = [u, v, omega] from vehicle sensors/simulation.

    Process model: Bicycle dynamics with NN-predicted lateral tire forces.
    """

    def __init__(self, vehicle_params, nn_model, scaler_X, scaler_y,
                 base_terrain_params, dt, n_init=0.7,
                 process_noise=None, measurement_noise=None, debug=False):
        """
        Args:
            vehicle_params: dict with M, Izz, Lf, Lr
            nn_model:       PyTorch TerrainNN (eval mode)
            scaler_X/y:     sklearn StandardScalers for NN inputs/outputs
            base_terrain_params: dict with Kphi, Kc, c, phi, k
            dt:             estimation interval (s)
            n_init:         initial estimate of sinkage exponent
            process_noise:  dict with 'u', 'v', 'omega', 'n' std devs (optional)
            measurement_noise: dict with 'u', 'v', 'omega' std devs (optional)
            debug:          print diagnostic info
        """
        # Vehicle parameters
        self.M = vehicle_params['M']
        self.Izz = vehicle_params['Izz']
        self.Lf = vehicle_params['Lf']
        self.Lr = vehicle_params['Lr']

        # NN model and scalers
        self.nn_model = nn_model
        self.scaler_X = scaler_X
        self.scaler_y = scaler_y
        self.base_terrain = base_terrain_params
        self.dt = dt
        self._debug = debug

        # State dimension and measurement dimension
        self.n_x = 4  # [u, v, omega, n]
        self.n_y = 2  # [v, omega] - only lateral dynamics (affected by n)
        # Note: u is NOT used as measurement because longitudinal dynamics
        # are poorly modeled (rolling resistance, terrain drag not in model).
        # Instead, u is updated directly from observation.

        # UKF parameters
        # Standard settings for good sigma point spread:
        # - alpha controls spread around mean (0.001-1, larger = more spread)
        # - beta=2 optimal for Gaussian prior  
        # - kappa often set to 0 or 3-n for guaranteed positive semi-definiteness
        # lambda = alpha^2 * (n + kappa) - n
        # We need (n + lambda) > 0 for valid sigma point scaling
        self.alpha = 0.5    # Moderate spread (was 1e-3 which gave lambda≈-4!)
        self.beta = 2.0     # Optimal for Gaussian
        self.kappa = 0.0    # Secondary scaling parameter
        self.lambd = self.alpha**2 * (self.n_x + self.kappa) - self.n_x
        # With alpha=0.5, n_x=4: lambda = 0.25*4 - 4 = -3, so n_x+lambda = 1 ✓

        # Initialize state estimate
        # z = [u, v, omega, n]
        self.z = np.array([5.0, 0.0, 0.0, n_init])

        # Initialize covariance
        # Higher initial variance for n since we're uncertain about it
        self.P = np.diag([0.5**2, 0.1**2, 0.05**2, 0.3**2])

        # Process noise covariance Q
        # n process noise allows adaptation to terrain changes
        if process_noise is None:
            process_noise = {'u': 0.1, 'v': 0.05, 'omega': 0.02, 'n': 0.01}
        self.Q = np.diag([
            process_noise['u']**2,
            process_noise['v']**2,
            process_noise['omega']**2,
            process_noise['n']**2
        ])

        # Measurement noise covariance R
        # From Table III of paper: σ_v = 0.25 m/s, σ_omega = 0.0175 rad/s
        # Only using v and omega as measurements (not u)
        if measurement_noise is None:
            measurement_noise = {'v': 0.25, 'omega': 0.0175}
        self.R = np.diag([
            measurement_noise.get('v', 0.25)**2,
            measurement_noise.get('omega', 0.0175)**2
        ])

        # Compute UKF weights
        self._compute_weights()

        # Store previous inputs for process model
        self._prev_delta = 0.0
        self._prev_ax = 0.0
        self._initialized = False

        # History for plotting
        self.n_history = [n_init]
        self.t_history = [0.0]
        self.P_history = [self.P[3, 3]]  # Variance of n

        # Bounds on n (training data range)
        self.n_min = 0.3
        self.n_max = 1.5

    def _compute_weights(self):
        """Compute UKF sigma point weights."""
        n = self.n_x
        lambd = self.lambd

        # Mean weights
        self.Wm = np.full(2 * n + 1, 1.0 / (2 * (n + lambd)))
        self.Wm[0] = lambd / (n + lambd)

        # Covariance weights
        self.Wc = np.full(2 * n + 1, 1.0 / (2 * (n + lambd)))
        self.Wc[0] = lambd / (n + lambd) + (1 - self.alpha**2 + self.beta)

    def _generate_sigma_points(self, z, P):
        """Generate 2n+1 sigma points around mean z with covariance P."""
        n = self.n_x
        sigma_pts = np.zeros((2 * n + 1, n))

        # Square root of scaled covariance
        try:
            sqrt_P = np.linalg.cholesky((n + self.lambd) * P)
        except np.linalg.LinAlgError:
            # If not positive definite, add small diagonal
            sqrt_P = np.linalg.cholesky((n + self.lambd) * (P + 1e-6 * np.eye(n)))

        sigma_pts[0] = z
        for i in range(n):
            sigma_pts[i + 1] = z + sqrt_P[i]
            sigma_pts[n + i + 1] = z - sqrt_P[i]

        return sigma_pts

    def _nn_forces(self, u, v, omega, delta, n_val):
        """
        Evaluate NN lateral forces for a single set of inputs.
        Returns (Fyf, Fyr) - total front and rear lateral forces.
        """
        terrain = self.base_terrain
        u_safe = max(abs(u), 0.5)

        # Slip angles (bicycle model)
        alpha_f = delta - np.arctan2(v + self.Lf * omega, u_safe)
        alpha_r = -np.arctan2(v - self.Lr * omega, u_safe)

        # Static vertical loads (per wheel)
        L = self.Lf + self.Lr
        Fz_f = self.M * 9.81 * self.Lr / L / 2.0
        Fz_r = self.M * 9.81 * self.Lf / L / 2.0

        results = []
        for alpha, Fz in [(alpha_f, Fz_f), (alpha_r, Fz_r)]:
            x_raw = np.array([[Fz, alpha, 0.0, 0.0, u_safe,
                               terrain['Kphi'], terrain['Kc'], n_val,
                               terrain['c'], terrain['phi'], terrain['k']]])
            x_scaled = self.scaler_X.transform(x_raw)
            with torch.no_grad():
                y_scaled = self.nn_model(
                    torch.tensor(x_scaled, dtype=torch.float32)).numpy()
            y_out = self.scaler_y.inverse_transform(y_scaled)
            # NN outputs per-wheel force; multiply by 2 for axle total
            results.append(-2.0 * y_out[0, 1])  # Index 1 is Fy

        Fyf, Fyr = results
        return Fyf, Fyr

    def _process_model(self, z, delta, ax):
        """
        Propagate state forward using bicycle dynamics.

        State: z = [u, v, omega, n]
        Inputs: delta (steering angle), ax (longitudinal acceleration)

        Returns: z_next
        """
        u, v, omega, n_val = z

        # Clamp n to valid range
        n_val = np.clip(n_val, self.n_min, self.n_max)

        # Get NN lateral forces
        Fyf, Fyr = self._nn_forces(u, v, omega, delta, n_val)

        # Bicycle model dynamics (Eq. 13 from paper):
        #   u_dot = ax  (given as input)
        #   v_dot = (Fyf + Fyr) / M - u * omega
        #   omega_dot = (Fyf * Lf - Fyr * Lr) / Izz
        #   n_dot = 0  (terrain parameter is constant)

        u_dot = ax
        v_dot = (Fyf + Fyr) / self.M - u * omega
        omega_dot = (Fyf * self.Lf - Fyr * self.Lr) / self.Izz
        n_dot = 0.0

        # Forward Euler integration
        u_next = u + u_dot * self.dt
        v_next = v + v_dot * self.dt
        omega_next = omega + omega_dot * self.dt
        n_next = n_val  # No dynamics for n

        return np.array([u_next, v_next, omega_next, n_next])

    def _measurement_model(self, z):
        """
        Measurement model: H(z) = [v, omega]
        Only extracts lateral states (affected by n through Fy).
        u is excluded because longitudinal dynamics are poorly modeled.
        """
        return z[1:3]  # [v, omega]

    def step(self, u, v, omega, delta, ax, time=None):
        """
        UKF update step.

        Args:
            u:     Measured longitudinal velocity (m/s)
            v:     Measured lateral velocity (m/s)
            omega: Measured yaw rate (rad/s)
            delta: Current steering angle (rad)
            ax:    Current longitudinal acceleration (m/s²)
            time:  Current time (optional, for logging)

        Returns:
            n_estimated (float): Updated sinkage exponent estimate
        """
        # Measurements (only v, omega - lateral dynamics)
        y = np.array([v, omega])
        
        # Store measured u for direct state update
        u_measured = u

        if not self._initialized:
            # Initialize state with first measurement
            self.z[0] = u
            self.z[1] = v
            self.z[2] = omega
            self._prev_delta = delta
            self._prev_ax = ax
            self._initialized = True
            return float(self.z[3])

        # === PREDICTION STEP ===

        # Generate sigma points
        sigma_pts = self._generate_sigma_points(self.z, self.P)

        # Propagate sigma points through process model
        sigma_pts_pred = np.zeros_like(sigma_pts)
        for i in range(2 * self.n_x + 1):
            sigma_pts_pred[i] = self._process_model(
                sigma_pts[i], self._prev_delta, self._prev_ax)
            # Clamp n to valid range
            sigma_pts_pred[i, 3] = np.clip(
                sigma_pts_pred[i, 3], self.n_min, self.n_max)

        # Predicted mean
        z_pred = np.sum(self.Wm[:, np.newaxis] * sigma_pts_pred, axis=0)

        # Predicted covariance
        P_pred = self.Q.copy()
        for i in range(2 * self.n_x + 1):
            diff = sigma_pts_pred[i] - z_pred
            P_pred += self.Wc[i] * np.outer(diff, diff)

        # === UPDATE STEP ===

        # Transform predicted sigma points through measurement model
        sigma_pts_y = np.zeros((2 * self.n_x + 1, self.n_y))
        for i in range(2 * self.n_x + 1):
            sigma_pts_y[i] = self._measurement_model(sigma_pts_pred[i])

        # Predicted measurement mean
        y_pred = np.sum(self.Wm[:, np.newaxis] * sigma_pts_y, axis=0)

        # Innovation covariance
        Pyy = self.R.copy()
        for i in range(2 * self.n_x + 1):
            diff_y = sigma_pts_y[i] - y_pred
            Pyy += self.Wc[i] * np.outer(diff_y, diff_y)

        # Cross-covariance
        Pxy = np.zeros((self.n_x, self.n_y))
        for i in range(2 * self.n_x + 1):
            diff_x = sigma_pts_pred[i] - z_pred
            diff_y = sigma_pts_y[i] - y_pred
            Pxy += self.Wc[i] * np.outer(diff_x, diff_y)

        # Kalman gain
        K = Pxy @ np.linalg.inv(Pyy)

        # Innovation (measurement residual)
        innovation = y - y_pred

        # State update
        self.z = z_pred + K @ innovation

        # Clamp n to valid range
        self.z[3] = np.clip(self.z[3], self.n_min, self.n_max)
        
        # Direct update of u from measurement (bypasses poorly modeled longitudinal dynamics)
        # This is legitimate since we observe u directly and model is unreliable for it
        self.z[0] = u_measured

        # Covariance update
        self.P = P_pred - K @ Pyy @ K.T

        # Ensure P stays symmetric and positive definite
        self.P = 0.5 * (self.P + self.P.T)
        eigvals = np.linalg.eigvalsh(self.P)
        if np.min(eigvals) < 1e-10:
            self.P += (1e-10 - np.min(eigvals)) * np.eye(self.n_x)

        # Store inputs for next prediction
        self._prev_delta = delta
        self._prev_ax = ax

        # Debug output
        if self._debug and time is not None and int(time * 10) % 20 == 0:
            innov_norm = np.linalg.norm(innovation)
            # Show Kalman gain for n from v and omega innovations
            K_n_v = K[3, 0]
            K_n_omega = K[3, 1]
            n_contrib_v = K_n_v * innovation[0]
            n_contrib_omega = K_n_omega * innovation[1]
            print(f"    UKF: n={self.z[3]:.3f} ±{np.sqrt(self.P[3,3]):.3f} "
                  f"innov_v={innovation[0]:+.3f} innov_ω={innovation[1]:+.4f} "
                  f"|innov|={innov_norm:.3f}")
            print(f"         K[n,v]={K_n_v:+.3f} K[n,ω]={K_n_omega:+.3f} "
                  f"→ Δn_v={n_contrib_v:+.4f} Δn_ω={n_contrib_omega:+.4f} "
                  f"total_Δn={(n_contrib_v+n_contrib_omega):+.4f}")

        # Record history
        if time is not None:
            self.n_history.append(float(self.z[3]))
            self.t_history.append(time)
            self.P_history.append(self.P[3, 3])

        return float(self.z[3])

    @property
    def n_estimated(self):
        return float(self.z[3])

    @property
    def n_hat(self):
        return self.n_estimated

    @property
    def bias_v(self):
        """Estimated lateral velocity bias (state - measurement)."""
        return 0.0  # No explicit bias state in this formulation

    @property
    def bias_omega(self):
        """Estimated yaw rate bias."""
        return 0.0

    @property
    def n_variance(self):
        """Variance of n estimate (P[3,3])."""
        return self.P[3, 3]


def load_ukf(model_path, scaler_path, vehicle_params, base_terrain_params,
             dt=0.1, n_init=0.7, measurement_noise=None, debug=False):
    """
    Load the NN and create a TerrainEstimator (UKF).

    Args:
        model_path:    Path to NN model checkpoint (.pt file)
        scaler_path:   Path to scalers pickle file
        vehicle_params: dict with M, Izz, Lf, Lr
        base_terrain_params: dict with Kphi, Kc, c, phi, k
        dt:            Estimation timestep (s)
        n_init:        Initial sinkage exponent estimate
        measurement_noise: dict with 'u', 'v', 'omega' noise std devs
        debug:         Print debug output

    Returns:
        TerrainEstimator instance
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

    nn_model = TerrainNN(input_size=11, output_size=2, hidden_sizes=hidden_sizes)
    nn_model.load_state_dict(state_dict)
    nn_model.eval()

    with open(scaler_path, 'rb') as f:
        scalers = pickle.load(f)

    # Convert measurement_noise dict format if provided
    meas_noise_dict = None
    if measurement_noise is not None:
        if isinstance(measurement_noise, dict):
            meas_noise_dict = measurement_noise
        else:
            # Assume it's a single value for all
            meas_noise_dict = {'u': measurement_noise, 'v': measurement_noise,
                               'omega': measurement_noise * 0.07}

    estimator = TerrainEstimator(
        vehicle_params=vehicle_params,
        nn_model=nn_model,
        scaler_X=scalers['X'],
        scaler_y=scalers['y'],
        base_terrain_params=base_terrain_params,
        dt=dt,
        n_init=n_init,
        measurement_noise=meas_noise_dict,
        debug=debug,
    )

    print(f"✓ TerrainUKF initialized: n_init={n_init:.2f}, dt={dt}s "
          f"(augmented-state UKF following Dallas et al. 2021)")
    return estimator
