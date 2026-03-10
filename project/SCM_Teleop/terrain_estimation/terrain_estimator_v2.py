"""
Online terrain parameter estimator following Dallas et al. (2021).

Implements an Unscented Kalman Filter (UKF) for estimating the sinkage
exponent `n` by propagating a bicycle model with NN-predicted lateral forces
and comparing predictions to measured vehicle states.

Reference:
    J. Dallas et al., "Terrain Adaptive Trajectory Planning and Tracking on
    Deformable Terrains," IEEE Trans. Veh. Tech., vol. 70, no. 11, 2021.

This version uses the FULL state vector [x, y, ψ, u, v, ω, n] with position
measurements [x, y, ψ, u, v, ω] as in the paper. Position measurements give
much better observability than velocities alone.
"""

import numpy as np
import torch
import pickle
from pathlib import Path


class TerrainEstimator:
    """
    Unscented Kalman Filter for sinkage exponent estimation.

    State vector: z = [x, y, psi, u, v, omega, n]  (7 states)
        x, y  - global position (m)
        psi   - heading angle (rad)
        u     - longitudinal velocity in body frame (m/s)
        v     - lateral velocity in body frame (m/s)  
        omega - yaw rate (rad/s)
        n     - sinkage exponent (terrain parameter)

    Measurements: y = [x, y, psi, u, v, omega] (6 measurements)
    
    Process model: Bicycle dynamics with NN-predicted lateral tire forces.
    """

    def __init__(self, vehicle_params, nn_model, scaler_X, scaler_y,
                 base_terrain_params, dt, n_init=0.7,
                 process_noise=None, measurement_noise=None, debug=False,
                 use_steering_rate=False):
        """
        Args:
            vehicle_params: dict with M, Izz, Lf, Lr
            nn_model:       PyTorch TerrainNN (eval mode)
            scaler_X/y:     sklearn StandardScalers for NN inputs/outputs
            base_terrain_params: dict with Kphi, Kc, c, phi, k
            dt:             estimation interval (s)
            n_init:         initial estimate of sinkage exponent
            process_noise:  dict with state noise std devs (optional)
            measurement_noise: dict with measurement noise std devs (optional)
            debug:          print diagnostic info
            use_steering_rate: if True, use v6 format (different column order + phi in radians)
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
        self.is_v6 = use_steering_rate  # v6 format: different column order + phi in radians
        self._steering_rate = 0.0  # Stored for v6 NN input

        # State dimension and measurement dimension
        self.n_x = 7  # [x, y, psi, u, v, omega, n]
        self.n_y = 6  # [x, y, psi, u, v, omega]

        # UKF parameters
        # alpha=1.0 for wider sigma point spread (better sensitivity exploration)
        # With alpha=1.0, n_x=7: lambda = 1.0*7 - 7 = 0, n_x+lambda = 7 ✓
        self.alpha = 1.0  # Maximum spread
        self.beta = 2.0
        self.kappa = 0.0
        self.lambd = self.alpha**2 * (self.n_x + self.kappa) - self.n_x

        # Initialize state estimate: [x, y, psi, u, v, omega, n]
        self.z = np.array([0.0, 0.0, 0.0, 5.0, 0.0, 0.0, n_init])

        # Initialize covariance
        # Large initial uncertainty on n to allow more exploration
        # Larger P[n,n] means sigma points explore wider range of n values
        self.P = np.diag([
            1.0**2,    # x
            1.0**2,    # y
            0.1**2,    # psi
            0.5**2,    # u
            0.1**2,    # v
            0.05**2,   # omega
            0.5**2     # n - large uncertainty for wide exploration
        ])

        # Process noise covariance Q
        # Higher process noise on n allows faster adaptation
        if process_noise is None:
            process_noise = {
                'x': 0.05, 'y': 0.05, 'psi': 0.01,
                'u': 0.1, 'v': 0.05, 'omega': 0.02,
                'n': 0.02  # Allow more adaptation (was 0.005)
            }
        self.Q = np.diag([
            process_noise['x']**2,
            process_noise['y']**2,
            process_noise['psi']**2,
            process_noise['u']**2,
            process_noise['v']**2,
            process_noise['omega']**2,
            process_noise['n']**2
        ])

        # Measurement noise covariance R (from Table III of paper)
        if measurement_noise is None:
            measurement_noise = {
                'x': 1.2, 'y': 1.2, 'psi': 0.0175,
                'u': 0.25, 'v': 0.25, 'omega': 0.0175
            }
        self.R = np.diag([
            measurement_noise.get('x', 1.2)**2,
            measurement_noise.get('y', 1.2)**2,
            measurement_noise.get('psi', 0.0175)**2,
            measurement_noise.get('u', 0.25)**2,
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
        self.P_history = [self.P[6, 6]]  # Variance of n

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
            sqrt_P = np.linalg.cholesky((n + self.lambd) * (P + 1e-6 * np.eye(n)))

        sigma_pts[0] = z
        for i in range(n):
            sigma_pts[i + 1] = z + sqrt_P[i]
            sigma_pts[n + i + 1] = z - sqrt_P[i]

        return sigma_pts

    def _nn_forces(self, u, v, omega, delta, n_val):
        """
        Evaluate NN lateral forces for a single set of inputs.
        Returns (Fyf, Fyr) - total front and rear axle lateral forces.
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
            if self.is_v6:
                # v6 Dallas format: slip_ratio, slip_angle, velocity, Fz, steering_rate,
                #   Kphi, Kc, n, c, phi(radians), k
                phi_rad = np.radians(terrain['phi'])
                x_raw = np.array([[0.0, alpha, u_safe, Fz, self._steering_rate,
                                   terrain['Kphi'], terrain['Kc'], n_val,
                                   terrain['c'], phi_rad, terrain['k']]])
            else:
                # v3 legacy format: Fz, slip_angle, slip_ratio, camber, velocity,
                #   Kphi, Kc, n, c, phi(degrees), k
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

    def _process_model(self, z, delta, ax, n_substeps=5):
        """
        Propagate state forward using bicycle dynamics with sub-stepping.

        State: z = [x, y, psi, u, v, omega, n]
        Inputs: delta (steering angle), ax (longitudinal acceleration)

        Uses n_substeps for better numerical stability and to accumulate
        the effect of n on positions through v/omega evolution.

        Returns: z_next
        """
        dt_sub = self.dt / n_substeps
        z_curr = z.copy()
        
        for _ in range(n_substeps):
            x, y, psi, u, v, omega, n_val = z_curr

            # Clamp n to valid range
            n_val = np.clip(n_val, self.n_min, self.n_max)

            # Get NN lateral forces
            Fyf, Fyr = self._nn_forces(u, v, omega, delta, n_val)

            # Bicycle model dynamics
            cos_psi = np.cos(psi)
            sin_psi = np.sin(psi)
            
            x_dot = u * cos_psi - (v + self.Lf * omega) * sin_psi
            y_dot = u * sin_psi + (v + self.Lf * omega) * cos_psi
            psi_dot = omega
            u_dot = ax
            v_dot = (Fyf + Fyr) / self.M - u * omega
            omega_dot = (Fyf * self.Lf - Fyr * self.Lr) / self.Izz
            n_dot = 0.0

            # Forward Euler integration (sub-step)
            z_curr = np.array([
                x + x_dot * dt_sub,
                y + y_dot * dt_sub,
                psi + psi_dot * dt_sub,
                u + u_dot * dt_sub,
                v + v_dot * dt_sub,
                omega + omega_dot * dt_sub,
                n_val  # No dynamics for n
            ])
        
        return z_curr

    def _measurement_model(self, z):
        """
        Measurement model: H(z) = [x, y, psi, u, v, omega]
        Extracts all observable states (everything except n).
        """
        return z[:6]

    def step(self, x_meas, y_meas, psi_meas, u, v, omega, delta, ax, time=None,
             steering_rate=0.0):
        """
        UKF update step.

        Args:
            x_meas, y_meas: Measured global position (m)
            psi_meas:      Measured heading (rad)
            u:             Measured longitudinal velocity (m/s)
            v:             Measured lateral velocity (m/s)
            omega:         Measured yaw rate (rad/s)
            delta:         Current steering angle (rad)
            ax:            Current longitudinal acceleration (m/s²)
            time:          Current time (optional, for logging)
            steering_rate: Current steering rate (rad/s) for v6 format

        Returns:
            n_estimated (float): Updated sinkage exponent estimate
        """
        # Store steering rate for v6 NN input
        self._steering_rate = steering_rate

        # Measurements
        y = np.array([x_meas, y_meas, psi_meas, u, v, omega])

        if not self._initialized:
            # Initialize state with first measurement
            self.z = np.array([x_meas, y_meas, psi_meas, u, v, omega, self.z[6]])
            self._prev_delta = delta
            self._prev_ax = ax
            self._initialized = True
            return float(self.z[6])

        # === PREDICTION STEP ===

        # Generate sigma points
        sigma_pts = self._generate_sigma_points(self.z, self.P)

        # Propagate sigma points through process model
        sigma_pts_pred = np.zeros_like(sigma_pts)
        for i in range(2 * self.n_x + 1):
            sigma_pts_pred[i] = self._process_model(
                sigma_pts[i], self._prev_delta, self._prev_ax)
            # Clamp n to valid range
            sigma_pts_pred[i, 6] = np.clip(
                sigma_pts_pred[i, 6], self.n_min, self.n_max)

        # Predicted mean
        z_pred = np.sum(self.Wm[:, np.newaxis] * sigma_pts_pred, axis=0)

        # Predicted covariance
        P_pred = self.Q.copy()
        for i in range(2 * self.n_x + 1):
            diff = sigma_pts_pred[i] - z_pred
            # Wrap angle difference for psi
            diff[2] = np.arctan2(np.sin(diff[2]), np.cos(diff[2]))
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
            diff_y[2] = np.arctan2(np.sin(diff_y[2]), np.cos(diff_y[2]))
            Pyy += self.Wc[i] * np.outer(diff_y, diff_y)

        # Cross-covariance
        Pxy = np.zeros((self.n_x, self.n_y))
        for i in range(2 * self.n_x + 1):
            diff_x = sigma_pts_pred[i] - z_pred
            diff_x[2] = np.arctan2(np.sin(diff_x[2]), np.cos(diff_x[2]))
            diff_y = sigma_pts_y[i] - y_pred
            diff_y[2] = np.arctan2(np.sin(diff_y[2]), np.cos(diff_y[2]))
            Pxy += self.Wc[i] * np.outer(diff_x, diff_y)

        # Kalman gain
        K = Pxy @ np.linalg.inv(Pyy)

        # Innovation (measurement residual)
        innovation = y - y_pred
        innovation[2] = np.arctan2(np.sin(innovation[2]), np.cos(innovation[2]))

        # State update
        self.z = z_pred + K @ innovation

        # Clamp n to valid range
        self.z[6] = np.clip(self.z[6], self.n_min, self.n_max)

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
            innov_pos = np.linalg.norm(innovation[:2])
            K_n = K[6, :]  # Kalman gain for n from all measurements
            print(f"    UKF: n={self.z[6]:.3f} ±{np.sqrt(self.P[6,6]):.3f} "
                  f"innov_pos={innov_pos:.2f}m innov_ψ={np.degrees(innovation[2]):+.1f}° "
                  f"innov_v={innovation[4]:+.3f}")
            print(f"         K[n,x]={K_n[0]:+.4f} K[n,y]={K_n[1]:+.4f} "
                  f"K[n,ψ]={K_n[2]:+.4f} K[n,v]={K_n[4]:+.4f}")

        # Record history
        if time is not None:
            self.n_history.append(float(self.z[6]))
            self.t_history.append(time)
            self.P_history.append(self.P[6, 6])

        return float(self.z[6])

    @property
    def n_estimated(self):
        return float(self.z[6])

    @property
    def n_hat(self):
        return self.n_estimated

    @property
    def bias_v(self):
        return 0.0

    @property
    def bias_omega(self):
        return 0.0

    @property
    def n_variance(self):
        return self.P[6, 6]


def load_ukf(model_path, scaler_path, vehicle_params, base_terrain_params,
             dt=0.1, n_init=0.7, measurement_noise=None, debug=False,
             use_v2=True):
    """
    Load the NN and create a TerrainEstimator (UKF).

    Args:
        model_path:    Path to NN model checkpoint (.pt file)
        scaler_path:   Path to scalers pickle file
        vehicle_params: dict with M, Izz, Lf, Lr
        base_terrain_params: dict with Kphi, Kc, c, phi, k
        dt:            Estimation timestep (s)
        n_init:        Initial sinkage exponent estimate
        measurement_noise: dict with measurement noise std devs
        debug:         Print debug output
        use_v2:        If True, use V2 estimator with position measurements

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

    with open(scaler_path, 'rb') as f:
        scalers = pickle.load(f)
    input_size = len(scalers['X'].mean_)

    nn_model = TerrainNN(input_size=input_size, output_size=2, hidden_sizes=hidden_sizes)
    nn_model.load_state_dict(state_dict)
    nn_model.eval()

    # Detect format: v6 has phi in radians (mean < 1.0), v3 has phi in degrees (mean > 1.0)
    phi_mean = scalers['X'].mean_[9]  # phi is at index 9 in both formats
    is_v6 = phi_mean < 1.0  # Radians will be < 1, degrees will be >> 1

    # Convert measurement_noise dict format if provided
    meas_noise_dict = None
    if measurement_noise is not None:
        if isinstance(measurement_noise, dict):
            meas_noise_dict = measurement_noise

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
        use_steering_rate=is_v6,
    )

    fmt = f"v6 (phi_mean={phi_mean:.2f} rad, has steering_rate)" if is_v6 else f"v3 (phi_mean={phi_mean:.1f}°)"
    print(f"✓ TerrainUKF V2 initialized: n_init={n_init:.2f}, dt={dt}s, format={fmt} "
          f"(7-state UKF with position measurements)")
    return estimator
