#!/usr/bin/env python3
"""
Dallas et al. MPC Formulation - Neural Network Terramechanics
==============================================================

Implementation of the MPC formulation from:
"Neural Network Deformable Terramechanics and Model 
Predictive Control for Off-Road Navigation"

Key features:
- 8-state dynamic bicycle model: [x, y, ψ, u, v, ω, δ, ax]
- Control inputs: steering rate (δ̇) and longitudinal jerk (Jx)  
- NN tire model embedded directly in MPC prediction dynamics
- Cost function: time-to-goal, heading penalty, control smoothness, terminal distance

Author: Implementation based on Dallas et al.
"""

import numpy as np
import casadi as ca
import torch
import pickle
from pathlib import Path


# =============================================================================
# Neural Network as CasADi Symbolic Function
# =============================================================================

class NNCasADi:
    """
    Converts a PyTorch neural network to CasADi symbolic operations.
    This allows the NN to be embedded directly in the MPC optimization.
    
    The NN predicts lateral tire forces Fyf and Fyr given:
    - Slip angles (αf, αr)
    - Normal forces (Fz)
    - Vehicle speed (u)
    - Terrain parameters
    """
    
    def __init__(self, model_path, scaler_path, terrain_params):
        """
        Load NN model and create CasADi function.
        
        Args:
            model_path: Path to PyTorch model checkpoint
            scaler_path: Path to input/output scalers
            terrain_params: Dict with SCM terrain parameters. Keys: Kphi, Kc, n, c, phi, k.
                           Friction angle phi must be in DEGREES (same as training CSV mohr_friction).
        
        Note: The NN is trained on single-wheel data. When used in the bicycle
        model MPC, it should be called with per-wheel loads (Fz/2) and the
        outputs summed (or equivalently, output * 2) per superposition.
        """
        self.terrain_params = terrain_params
        self._load_pytorch_model(model_path, scaler_path)
        self._build_casadi_function()
    
    def _load_pytorch_model(self, model_path, scaler_path):
        """Load the trained PyTorch model and scalers (supports variable architectures)."""
        import sys
        # Add nn_training directory to path for TerrainNN import
        # From simulation/, go up one level to SCM_Teleop/nn_training
        nn_training_path = Path(__file__).parent.parent / "nn_training"
        sys.path.insert(0, str(nn_training_path))
        from train_terrain_nn import TerrainNN
        
        checkpoint = torch.load(model_path, weights_only=False, map_location='cpu')
        state_dict = checkpoint.get('model_state_dict', checkpoint) if isinstance(checkpoint, dict) else checkpoint
        
        hidden_sizes = None
        if isinstance(checkpoint, dict) and 'hidden_sizes' in checkpoint:
            hidden_sizes = checkpoint['hidden_sizes']
        
        # Remap legacy keys (layer1/layer2/layer3 -> layers.0/layers.1/layers.2)
        if any(k.startswith('layer') and not k.startswith('layers.') for k in state_dict):
            old_to_new = {}
            idx = 0
            while f'layer{idx+1}.weight' in state_dict:
                old_to_new[f'layer{idx+1}.weight'] = f'layers.{idx}.weight'
                old_to_new[f'layer{idx+1}.bias'] = f'layers.{idx}.bias'
                idx += 1
            state_dict = {old_to_new.get(k, k): v for k, v in state_dict.items()}
        
        if hidden_sizes is None and any(k.startswith('layers.') for k in state_dict):
            layer_ids = sorted(set(int(k.split('.')[1]) for k in state_dict if k.startswith('layers.')))
            hidden_sizes = [state_dict[f'layers.{i}.weight'].shape[0] for i in layer_ids[:-1]]
        
        self.pytorch_model = TerrainNN(input_size=11, output_size=2, hidden_sizes=hidden_sizes)
        self.pytorch_model.load_state_dict(state_dict)
        self.pytorch_model.eval()
        
        # Extract weights as numpy arrays
        self.weights = {}
        for name, param in self.pytorch_model.named_parameters():
            self.weights[name] = param.detach().numpy()
        
        # Load scalers
        with open(scaler_path, 'rb') as f:
            scalers = pickle.load(f)
        self.scaler_X = scalers['X']
        self.scaler_y = scalers['y']
        
        # Extract scaler parameters
        self.X_mean = self.scaler_X.mean_
        self.X_scale = self.scaler_X.scale_
        self.y_mean = self.scaler_y.mean_
        self.y_scale = self.scaler_y.scale_
        
        print(f"✓ Loaded NN model: {len(self.weights)} parameter tensors")
    
    def _build_casadi_function(self):
        """Build CasADi function replicating the NN forward pass (variable depth).
        
        The sinkage exponent `n` is a symbolic parameter so the MPC can pass
        the UKF estimate at each solve without rebuilding the solver.
        
        Supports two feature orders:
        - v6 (Dallas format): slip_ratio, slip_angle, velocity, Fz, steering_rate, Kphi, Kc, n, c, phi(rad), k
        - v3 (legacy format): Fz, slip_angle, slip_ratio, camber, velocity, Kphi, Kc, n, c, phi(deg), k
        """
        alpha = ca.SX.sym('alpha')
        Fz = ca.SX.sym('Fz')
        u = ca.SX.sym('u')
        slip_ratio = ca.SX.sym('kappa')
        n_terrain = ca.SX.sym('n_terrain')
        steering_rate = ca.SX.sym('steering_rate')
        
        terrain = self.terrain_params
        
        # Detect model format based on scaler characteristics:
        # v3: phi mean ~35 (degrees), Fz in position 0
        # v6: phi mean ~0.38 (radians), slip_ratio in position 0
        phi_mean = self.X_mean[9]  # phi is at index 9 in both formats
        is_v6_format = phi_mean < 1.0  # Radians will be < 1.0, degrees will be > 1.0
        
        if is_v6_format:
            # v6 Dallas feature order: slip_ratio, slip_angle, velocity, Fz, steering_rate, 
            # Kphi, Kc, n, c, phi(radians), k
            # Note: terrain_params['phi'] is in DEGREES, convert to radians for v6
            phi_rad = np.radians(terrain['phi'])
            x_input = ca.vertcat(
                slip_ratio, alpha, u, Fz, steering_rate,
                terrain['Kphi'], terrain['Kc'], n_terrain,
                terrain['c'], phi_rad, terrain['k']
            )
            self.model_format = 'v6'
        else:
            # v3 legacy feature order: Fz, slip_angle, slip_ratio, camber, velocity,
            # Kphi, Kc, n, c, phi(degrees), k
            # Note: v3 expects phi in DEGREES, terrain_params has degrees
            camber = 0.0
            x_input = ca.vertcat(
                Fz, alpha, slip_ratio, camber, u,
                terrain['Kphi'], terrain['Kc'], n_terrain,
                terrain['c'], terrain['phi'], terrain['k']
            )
            self.model_format = 'v3'
        
        print(f"  Model format detected: {self.model_format} (phi_mean={phi_mean:.2f})")
        
        x_scaled = (x_input - self.X_mean.reshape(-1, 1)) / self.X_scale.reshape(-1, 1)
        
        # Discover layers dynamically from weight keys (layers.0.weight, layers.1.weight, ...)
        layer_indices = sorted(set(
            int(k.split('.')[1]) for k in self.weights if k.startswith('layers.')
        ))
        
        h = x_scaled
        for i in layer_indices:
            W = self.weights[f'layers.{i}.weight']
            b = self.weights[f'layers.{i}.bias']
            h = ca.mtimes(W, h) + b.reshape(-1, 1)
            if i < layer_indices[-1]:
                h = ca.tanh(h)
        
        y_scaled = h
        y_output = y_scaled * self.y_scale.reshape(-1, 1) + self.y_mean.reshape(-1, 1)
        
        Fx = y_output[0]
        Fy = y_output[1]
        
        self.predict_tire_force = ca.Function(
            'nn_tire',
            [alpha, Fz, u, slip_ratio, n_terrain, steering_rate],
            [Fx, Fy],
            ['alpha', 'Fz', 'u', 'kappa', 'n_terrain', 'steering_rate'],
            ['Fx', 'Fy']
        )
        self.n_nominal = terrain['n']
        
        n_params = sum(W.shape[0]*W.shape[1] for k, W in self.weights.items() if 'weight' in k)
        print(f"✓ Built CasADi symbolic NN ({len(layer_indices)} layers, {n_params} weight params)")
    
    def predict(self, alpha, Fz, u, kappa=0.0, n_terrain=None, steering_rate=0.0):
        """
        Predict tire forces using the CasADi function.
        Can accept both numeric and symbolic inputs.
        
        Args:
            alpha: Slip angle (rad)
            Fz: Normal force (N)
            u: Longitudinal velocity (m/s)
            kappa: Longitudinal slip ratio (default 0.0)
            n_terrain: Sinkage exponent (default: nominal from terrain config)
            steering_rate: Steering rate (rad/s) - required for v6 model (default 0.0)
        """
        if n_terrain is None:
            n_terrain = self.n_nominal
        Fx, Fy = self.predict_tire_force(alpha, Fz, u, kappa, n_terrain, steering_rate)
        return Fx, Fy
    
    def predict_numeric(self, alpha, Fz, u, kappa=0.0, n_terrain=None, steering_rate=0.0):
        """Evaluate numerically and return float values (for debugging)."""
        if n_terrain is None:
            n_terrain = self.n_nominal
        Fx, Fy = self.predict_tire_force(alpha, Fz, u, kappa, n_terrain, steering_rate)
        return float(Fx), float(Fy)
    
    def debug_compare(self, alpha_deg, Fz, u, kappa=0.0, Cf=80000, Cr=80000):
        """
        Compare NN prediction to linear model for debugging.
        
        Args:
            alpha_deg: Slip angle in DEGREES for readability
            Fz: Normal force per wheel (N)
            u: Vehicle speed (m/s)
            kappa: Slip ratio
            Cf, Cr: Cornering stiffness for comparison
        """
        import numpy as np
        alpha_rad = np.radians(alpha_deg)
        
        # NN prediction (single wheel)
        Fx_nn, Fy_nn = self.predict_numeric(alpha_rad, Fz, u, kappa)
        
        # Linear model (single wheel cornering stiffness / 2)
        Fy_linear = (Cf / 2) * alpha_rad  # Per-wheel
        
        # Saturate at friction limit
        mu = 0.4  # Soft terrain
        Fy_max = mu * Fz
        Fy_linear_sat = np.clip(Fy_linear, -Fy_max, Fy_max)
        
        print(f"  Slip angle: {alpha_deg:.1f}° ({alpha_rad:.3f} rad)")
        print(f"  Fz (per wheel): {Fz:.0f} N")
        print(f"  Speed: {u:.1f} m/s, kappa: {kappa:.3f}")
        print(f"  NN Fy (wheel):     {Fy_nn:8.1f} N")
        print(f"  Linear Fy (wheel): {Fy_linear:8.1f} N (saturated: {Fy_linear_sat:.1f})")
        print(f"  Ratio NN/Linear:   {Fy_nn/Fy_linear:.2f}" if abs(Fy_linear) > 10 else "")
        return Fx_nn, Fy_nn, Fy_linear


# =============================================================================
# Dallas et al. MPC Formulation
# =============================================================================

class DallasMPC:
    """
    Dallas et al. MPC with 8-state dynamic bicycle model and NN tire forces.
    
    State vector: z = [x, y, ψ, u, v, ω, δ, ax]
        x: global x position (front axle)
        y: global y position (front axle)
        ψ: yaw angle
        u: longitudinal speed
        v: lateral speed
        ω: yaw rate
        δ: steering angle (state, not control!)
        ax: longitudinal acceleration (state, not control!)
    
    Control vector: ζ = [δ̇, Jx]
        δ̇: steering rate
        Jx: longitudinal jerk
    
    The key insight: steering and acceleration are STATES (integrated from
    rate/jerk controls), which produces smoother behavior and allows 
    explicit constraints on rates.
    """
    
    def __init__(self, nn_casadi=None, params=None, dt=0.1, N=20,
                 nn_scale=1.0, nn_sign=1, kappa_mode='zero'):
        """
        Initialize Dallas MPC.
        
        Args:
            nn_casadi: NNCasADi instance for tire force prediction
            params: Vehicle parameters dict
            dt: Time step for discretization
            N: Prediction horizon (number of steps)
            nn_scale: Scale factor for NN force predictions (default 1.0)
            nn_sign: Deprecated, kept for backward compat (sign verified empirically as -1)
            kappa_mode: 'zero' = assume pure lateral slip (no combined slip),
                        'approx' = approximate kappa from ax/(mu*g)
        """
        self.kappa_mode = kappa_mode
        self.nn_scale = nn_scale
        # Vehicle parameters — queried from Chrono HMMWV_Full() at init
        self.M = 2573.0      # kg — GetVehicle().GetMass()
        self.Izz = 3570.0    # kg*m^2 — GetChassisBody().GetInertiaXX().z
        self.Lf = 1.593      # m — front axle x (1.6486) minus CG x (0.056)
        self.Lr = 1.709      # m — CG x (0.056) minus rear axle x (-1.6534)
        self.L = self.Lf + self.Lr  # 3.302 m
        
        # Tire parameters (used if no NN) - legacy, kept for compatibility
        self.Cf = 80000.0    # Front cornering stiffness (N/rad)
        self.Cr = 80000.0    # Rear cornering stiffness (N/rad)
        
        # Pacejka Magic Formula coefficients (from HMMWV_Pac02Tire.tir)
        # Fy = D * sin(C * atan(B*α - E*(B*α - atan(B*α))))
        # where D = μ * Fz (peak force)
        # Source: chrono/data/vehicle/hmmwv/tire/HMMWV_Pac02Tire.tir
        self.pacejka_B = 8.77   # Stiffness factor (derived: |PKY1|/(PCY1*PDY1) = 10.289/1.174)
        self.pacejka_C = 1.5874 # Shape factor (PCY1 from .tir file)
        self.pacejka_E = 0.376  # Curvature factor (PEY1 from .tir file)
        self.mu = 0.74          # Peak friction (PDY1 from .tir file)
        
        if params:
            self.__dict__.update(params)
        
        self.dt = dt
        self.N = N
        self.nx = 8   # [x, y, psi, u, v, omega, delta, ax]
        self.nu = 2   # [delta_dot, Jx]
        
        # Neural network tire model
        self.nn_casadi = nn_casadi
        self.use_nn = nn_casadi is not None
        
        # State constraints (from Dallas paper)
        self.u_min = 0.5      # Min longitudinal speed (m/s)
        self.u_max = 20.0     # Max longitudinal speed (m/s)
        self.delta_min = -0.528 # Min steering angle (rad) — GetVehicle().GetMaxSteeringAngle()
        self.delta_max = 0.528  # Max steering angle (rad)
        self.ax_min = -3.5    # Min acceleration (m/s^2)
        self.ax_max = 3.5     # Max acceleration (m/s^2) - increased for deformable terrain headroom
        
        # Control constraints 
        self.delta_dot_min = -0.5  # Min steering rate (rad/s)
        self.delta_dot_max = 0.7   # Max steering rate (rad/s)
        self.Jx_min = -5.0         # Min jerk (m/s^3)
        self.Jx_max = 5.0          # Max jerk (m/s^3)
        
        # Cost weights (from Dallas paper Eq. 11)
        self.w_t = 0.0       # Time weight (disabled for fixed horizon)
        self.w_psi = 10.0    # Heading alignment weight
        self.w_delta_dot = 1.0    # Steering rate penalty
        self.w_Jx = 1.0      # Jerk penalty
        self.w_terminal = 50.0    # Terminal distance weight
        
        # Additional state tracking weights
        self.w_lateral = 100.0   # Lateral error weight (prevent right-swing)
        self.w_heading = 20.0     # Heading error weight
        self.w_speed = 30.0       # Speed tracking weight (high to overcome unmodeled SCM drag)
        
        self._setup_solver()
        
        # Warm start storage
        self.X_warm = None
        self.U_warm = None
    
    def _setup_solver(self):
        """Build the CasADi NLP solver for the OCP"""
        nx, nu, N, dt = self.nx, self.nu, self.N, self.dt
        M, Izz, Lf, Lr = self.M, self.Izz, self.Lf, self.Lr
        
        # =====================================================================
        # Define symbolic state and control vectors
        # =====================================================================
        z = ca.SX.sym('z', nx)  # State
        zeta = ca.SX.sym('zeta', nu)  # Control
        n_terrain_sym = ca.SX.sym('n_terrain')  # Sinkage exponent (from UKF or nominal)
        sr_meas_sym = ca.SX.sym('sr_meas')  # Measured steering rate (rad/s) - from vehicle, not control
        
        # Extract states
        x_pos = z[0]     # Global x
        y_pos = z[1]     # Global y
        psi = z[2]       # Yaw angle
        u = z[3]         # Longitudinal velocity
        v = z[4]         # Lateral velocity
        omega = z[5]     # Yaw rate
        delta = z[6]     # Steering angle (STATE)
        ax = z[7]        # Longitudinal acceleration (STATE)
        
        # Extract controls
        delta_dot = zeta[0]  # Steering rate
        Jx = zeta[1]         # Longitudinal jerk
        
        # =====================================================================
        # Tire Forces - NN or Linear Model
        # =====================================================================
        
        # Safe speed for slip angle calculation
        u_safe = ca.fmax(ca.fabs(u), 0.5)
        
        # Slip angles (from Dallas Eq. 13)
        alpha_f = delta - ca.atan2(v + Lf * omega, u_safe)
        alpha_r = -ca.atan2(v - Lr * omega, u_safe)
        
        # Normal forces: static weight distribution (no load transfer).
        # Training data used MEASURED Fz from the rig; here we use predicted Fz for the whole horizon.
        # Optional: pass measured Fz at initial step when available from the vehicle (see NMPC_AND_DATA_VERIFICATION.md).
        Fz_f_axle = M * 9.81 * Lr / (Lf + Lr)
        Fz_r_axle = M * 9.81 * Lf / (Lf + Lr)
        
        # Per-wheel loads (2 wheels per axle) — same as data collection (single wheel, then ×2 for axle)
        Fz_f_wheel = Fz_f_axle / 2.0
        Fz_r_wheel = Fz_r_axle / 2.0
        
        # Longitudinal slip ratio (kappa).
        # The NN was trained with true kinematic kappa from the tire rig, but the
        # bicycle model has no wheel speed state. Two options:
        #   'zero'   - assume pure lateral slip (kappa=0). Avoids feeding the NN
        #              a bad approximation; Dallas et al. likely did this.
        #   'approx' - estimate kappa ~ ax/(mu*g). Rough but couples long/lat dynamics.
        if self.kappa_mode == 'approx':
            mu_terrain = 0.4
            kappa = ca.fmax(ca.fmin(ax / (mu_terrain * 9.81), 0.3), -0.3)
        else:
            kappa = 0.0
        
        if self.use_nn and self.nn_casadi is not None:
            # For NN predictions in dynamics, use sr_meas_sym (measured steering rate).
            # This is a PARAMETER (like n_terrain), NOT the control delta_dot.
            # Dallas paper: steering rate affects tire forces, but it should be measured,
            # not derived from control. Passing delta_dot causes optimization instability
            # due to high sensitivity (~1000N per rad/s).
            Fxf_wheel, Fyf_wheel = self.nn_casadi.predict(alpha_f, Fz_f_wheel, u_safe, kappa, n_terrain_sym, sr_meas_sym)
            Fxr_wheel, Fyr_wheel = self.nn_casadi.predict(alpha_r, Fz_r_wheel, u_safe, kappa, n_terrain_sym, sr_meas_sym)
            # -2.0: verified empirically — NN outputs tire-frame Fy (positive alpha →
            # negative Fy), body-frame needs opposite sign, and ×2 sums both wheels.
            Fyf = -2.0 * self.nn_scale * Fyf_wheel
            Fyr = -2.0 * self.nn_scale * Fyr_wheel
            
            # For traction constraint: query Fx at REFERENCE slip ratio (not kappa=0).
            # At kappa=0, NN predicts rolling resistance (negative Fx), not traction capacity.
            # Use kappa=0.08 (near peak traction) to get available driving force.
            # Use steering_rate=0.0 for traction (doesn't depend on steering dynamics).
            kappa_ref = 0.08
            Fxf_traction, _ = self.nn_casadi.predict(0.0, Fz_f_wheel, u_safe, kappa_ref, n_terrain_sym, 0.0)
            Fxr_traction, _ = self.nn_casadi.predict(0.0, Fz_r_wheel, u_safe, kappa_ref, n_terrain_sym, 0.0)
            # Total available traction force (×2 for both wheels per axle)
            Fx_traction = 2.0 * (Fxf_traction + Fxr_traction)
        else:
            # Pacejka Magic Formula tire model (simplified)
            # Fy = D * sin(C * atan(B*α - E*(B*α - atan(B*α))))
            B = self.pacejka_B
            C = self.pacejka_C
            E = self.pacejka_E
            
            # Peak force D = μ * Fz (with combined slip reduction)
            lateral_limit_factor = ca.sqrt(ca.fmax(1.0 - (kappa/0.2)**2, 0.1))
            Df = self.mu * Fz_f_axle * lateral_limit_factor
            Dr = self.mu * Fz_r_axle * lateral_limit_factor
            
            # Magic formula: Fy = D * sin(C * atan(B*α - E*(B*α - atan(B*α))))
            Baf = B * alpha_f
            Bar = B * alpha_r
            Fyf = Df * ca.sin(C * ca.atan(Baf - E * (Baf - ca.atan(Baf))))
            Fyr = Dr * ca.sin(C * ca.atan(Bar - E * (Bar - ca.atan(Bar))))
            
            # Available traction from friction limit
            Fx_traction = self.mu * (Fz_f_axle + Fz_r_axle)
        
        # =====================================================================
        # State Evolution - Dallas Eq. (13)
        # =====================================================================
        zdot = ca.vertcat(
            # ẋ = u*cos(ψ) - (v + Lf*ω)*sin(ψ)
            u * ca.cos(psi) - (v + Lf * omega) * ca.sin(psi),
            # ẏ = u*sin(ψ) + (v + Lf*ω)*cos(ψ)
            u * ca.sin(psi) + (v + Lf * omega) * ca.cos(psi),
            # ψ̇ = ω
            omega,
            # u̇ = ax
            ax,
            # v̇ = (Fyf + Fyr)/M - u*ω
            (Fyf + Fyr) / M - u * omega,
            # ω̇ = (Fyf*Lf - Fyr*Lr)/Izz
            (Fyf * Lf - Fyr * Lr) / Izz,
            # δ̇ = δ_dot (control input)
            delta_dot,
            # ȧx = Jx (control input)
            Jx
        )
        
        self.f = ca.Function('f', [z, zeta, n_terrain_sym, sr_meas_sym], [zdot])
        
        # Create function to compute available traction force for constraints
        # This lets us constrain ax ≤ Fx_traction / M at each time step
        self.f_Fx = ca.Function('f_Fx', [z, n_terrain_sym, sr_meas_sym], [Fx_traction],
                                ['z', 'n_terrain', 'sr_meas'], ['Fx_traction'])
        
        # =====================================================================
        # Optimal Control Problem Setup
        # =====================================================================
        
        # Decision variables: states at each time step + controls
        Z = ca.SX.sym('Z', nx, N+1)
        U = ca.SX.sym('U', nu, N)
        
        # Parameters: [z0(8), x_goal, y_goal, psi_goal, v_target, n_terrain, sr_meas,
        #              x_ref(N+1), y_ref(N+1), psi_ref(N+1), v_ref(N+1)]
        n_ref = 4 * (N + 1)  # Reference trajectory
        P = ca.SX.sym('P', nx + 6 + n_ref)
        
        # Extract parameters
        z0 = P[:nx]
        x_goal = P[nx]
        y_goal = P[nx + 1]
        psi_goal = P[nx + 2]
        v_target = P[nx + 3]
        n_terrain_param = P[nx + 4]
        sr_meas_param = P[nx + 5]  # Measured steering rate (constant for horizon)
        
        ref_start = nx + 6
        
        # =====================================================================
        # Cost Function - Dallas Eq. (11)
        # =====================================================================
        cost = 0
        g_eq = []    # Equality constraints (dynamics)
        g_ineq = []  # Inequality constraints (traction)
        
        # Initial state constraint
        g_eq.append(Z[:, 0] - z0)
        
        # Distance to goal at initial time (for normalization)
        x0_pos = z0[0]
        y0_pos = z0[1]
        d0 = ca.sqrt((x0_pos - x_goal)**2 + (y0_pos - y_goal)**2) + 1e-6
        
        for k in range(N):
            # Current state
            zk = Z[:, k]
            uk = U[:, k]
            x_k, y_k, psi_k, u_k = zk[0], zk[1], zk[2], zk[3]
            delta_dot_k, Jx_k = uk[0], uk[1]
            
            # Reference at this time step
            ref_idx = ref_start + k * 4
            x_ref_k = P[ref_idx]
            y_ref_k = P[ref_idx + 1]
            psi_ref_k = P[ref_idx + 2]
            v_ref_k = P[ref_idx + 3]
            
            # -----------------------------------------------------------------
            # Control effort
            # -----------------------------------------------------------------
            cost += (self.w_delta_dot * delta_dot_k**2 + self.w_Jx * Jx_k**2) * dt
            
            # -----------------------------------------------------------------
            # Path tracking: CROSS-TRACK (y) error only.
            # Do NOT penalize x-error: the speed cost handles longitudinal progress.
            # Including (x_k - x_ref_k)^2 causes countersteering when the MPC
            # accelerates past the reference, since x-error dominates and y
            # becomes unconstrained.
            # -----------------------------------------------------------------
            crosstrack_error = (y_k - y_ref_k)**2
            heading_error = (psi_k - psi_ref_k)**2
            speed_error = (u_k - v_ref_k)**2
            
            cost += self.w_lateral * crosstrack_error
            cost += self.w_heading * heading_error
            cost += self.w_speed * speed_error
            
            # -----------------------------------------------------------------
            # Control rate penalty for smoothness
            # -----------------------------------------------------------------
            if k > 0:
                du = U[:, k] - U[:, k-1]
                cost += 10.0 * ca.dot(du, du)
            
            # -----------------------------------------------------------------
            # Dynamics constraint (RK4 integration)
            # -----------------------------------------------------------------
            k1 = self.f(zk, uk, n_terrain_param, sr_meas_param)
            k2 = self.f(zk + dt/2 * k1, uk, n_terrain_param, sr_meas_param)
            k3 = self.f(zk + dt/2 * k2, uk, n_terrain_param, sr_meas_param)
            k4 = self.f(zk + dt * k3, uk, n_terrain_param, sr_meas_param)
            z_next = zk + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
            
            g_eq.append(Z[:, k+1] - z_next)
            
            # -----------------------------------------------------------------
            # Traction constraint: commanded force M*ax must not exceed
            # available tire force Fx_total from NN prediction.
            # For acceleration: M*ax ≤ Fx_total  →  M*ax - Fx_total ≤ 0
            # For braking: M*ax ≥ -Fx_braking  →  -M*ax - Fx_braking ≤ 0
            # We use Fx_total for both (symmetric for simplicity)
            # -----------------------------------------------------------------
            ax_k = zk[7]
            Fx_avail = self.f_Fx(zk, n_terrain_param, sr_meas_param)
            
            # Acceleration traction limit: M*ax - Fx_avail ≤ 0
            g_ineq.append(M * ax_k - Fx_avail)
            
            # Braking traction limit: -M*ax - |Fx_avail| ≤ 0
            # (braking force magnitude limited by same traction)
            g_ineq.append(-M * ax_k - ca.fabs(Fx_avail))
        
        # Combine constraints: equality first, then inequality
        g = g_eq + g_ineq
        
        # =====================================================================
        # Terminal Cost: cross-track + heading only (no x-distance).
        # Euclidean distance to (x_goal, y_goal) caused countersteering because
        # the vehicle overshoots x_goal when accelerating, making the x-component
        # dominate and the y-component irrelevant.
        # =====================================================================
        z_final = Z[:, N]
        x_f, y_f, psi_f = z_final[0], z_final[1], z_final[2]
        
        cost += self.w_terminal * (y_f - y_goal)**2
        cost += self.w_terminal * 0.5 * (psi_f - psi_goal)**2
        
        # =====================================================================
        # Build NLP
        # =====================================================================
        opt_vars = ca.vertcat(ca.reshape(Z, -1, 1), ca.reshape(U, -1, 1))
        
        nlp = {
            'x': opt_vars,
            'f': cost,
            'g': ca.vertcat(*g),
            'p': P
        }
        
        opts = {
            'ipopt.print_level': 0,
            'print_time': 0,
            'ipopt.max_iter': 100,
            'ipopt.warm_start_init_point': 'yes',
            'ipopt.tol': 1e-3,
            'ipopt.acceptable_tol': 5e-3,
            'ipopt.acceptable_iter': 5,
            'ipopt.mu_strategy': 'adaptive',
        }
        
        self.solver = ca.nlpsol('dallas_mpc', 'ipopt', nlp, opts)
        
        # =====================================================================
        # Variable Bounds
        # =====================================================================
        lbx, ubx = [], []
        
        # State bounds for each time step
        for i in range(N + 1):
            lbx += [-1e6, -1e6, -1e6,              # x, y, psi unconstrained
                    self.u_min,                     # u (speed)
                    -10.0, -5.0,                    # v, omega
                    self.delta_min, self.ax_min]   # delta, ax
            ubx += [1e6, 1e6, 1e6,
                    self.u_max,
                    10.0, 5.0,
                    self.delta_max, self.ax_max]
        
        # Control bounds
        for i in range(N):
            lbx += [self.delta_dot_min, self.Jx_min]
            ubx += [self.delta_dot_max, self.Jx_max]
        
        self.lbx = np.array(lbx)
        self.ubx = np.array(ubx)
        
        # Constraint bounds:
        # - Equality constraints for dynamics: nx*(N+1) constraints = 0
        # - Inequality constraints for traction: 2*N constraints ≤ 0
        n_eq = nx * (N + 1)   # Dynamics equality constraints
        n_ineq = 2 * N        # Traction inequality constraints (accel + brake per step)
        
        self.lbg = np.concatenate([
            np.zeros(n_eq),           # Equality: g = 0
            -np.inf * np.ones(n_ineq) # Inequality: -inf ≤ g ≤ 0
        ])
        self.ubg = np.concatenate([
            np.zeros(n_eq),           # Equality: g = 0  
            np.zeros(n_ineq)          # Inequality: g ≤ 0
        ])
        
        print(f"✓ Dallas MPC solver built: {nx} states, {nu} controls, N={N}")
        print(f"  Traction constraints: {n_ineq} inequality constraints (Fx-limited ax)")
    
    def solve(self, z0, x_ref, y_ref, psi_ref, v_ref, x_goal, y_goal, psi_goal,
              n_terrain=None, sr_meas=0.0):
        """
        Solve the MPC optimal control problem.
        
        Args:
            z0: Current state [x, y, psi, u, v, omega, delta, ax]
            x_ref, y_ref, psi_ref, v_ref: Reference trajectory (N+1 points)
            x_goal, y_goal, psi_goal: Goal position and heading
            n_terrain: Sinkage exponent from UKF (None => use nominal from terrain config)
            sr_meas: Measured steering rate (rad/s). This is the observed/filtered steering
                     rate from the vehicle, NOT the MPC control input. Default 0.0.
            
        Returns:
            delta_dot: Steering rate command
            Jx: Jerk command  
            Z_opt: Predicted state trajectory
            U_opt: Control sequence
        """
        nx, nu, N = self.nx, self.nu, self.N
        
        if n_terrain is None:
            n_terrain = self.nn_casadi.n_nominal if self.nn_casadi is not None else 1.1
        
        # Build parameter vector: [z0, x_goal, y_goal, psi_goal, v_target, n_terrain, sr_meas, refs...]
        p = list(z0)
        p += [x_goal, y_goal, psi_goal, v_ref[0], n_terrain, sr_meas]
        
        for k in range(N + 1):
            p += [x_ref[k], y_ref[k], psi_ref[k], v_ref[k]]
        
        # Initial guess with warm start
        if self.X_warm is not None:
            Z_init = np.hstack([self.X_warm[:, 1:], self.X_warm[:, -1:]])
            U_init = np.hstack([self.U_warm[:, 1:], self.U_warm[:, -1:]])
        else:
            Z_init = np.tile(np.array(z0).reshape(-1, 1), (1, N + 1))
            U_init = np.zeros((nu, N))
        
        x0_nlp = np.concatenate([Z_init.flatten('F'), U_init.flatten('F')])
        
        # Solve
        try:
            sol = self.solver(
                x0=x0_nlp,
                lbx=self.lbx, ubx=self.ubx,
                lbg=self.lbg, ubg=self.ubg,
                p=p
            )
            
            # Extract solution
            sol_x = sol['x'].full().flatten()
            Z_opt = sol_x[:nx * (N + 1)].reshape((nx, N + 1), order='F')
            U_opt = sol_x[nx * (N + 1):].reshape((nu, N), order='F')
            
            # Save for warm start
            self.X_warm = Z_opt
            self.U_warm = U_opt
            
            # Return first control: steering rate and jerk
            return U_opt[0, 0], U_opt[1, 0], Z_opt, U_opt
            
        except Exception as e:
            print(f"MPC solve failed: {e}")
            return 0.0, 0.0, None, None
    
    def get_steering_and_throttle(self, delta_dot, Jx, current_delta, current_ax, dt):
        """
        Convert rate/jerk controls to actual steering and throttle commands.
        
        Args:
            delta_dot: Steering rate from MPC
            Jx: Longitudinal jerk from MPC
            current_delta: Current steering angle (state)
            current_ax: Current acceleration (state)
            dt: Time step
            
        Returns:
            delta: New steering angle
            throttle: Throttle command [0, 1] (or brake if negative)
        """
        # Integrate steering rate to get angle
        new_delta = current_delta + delta_dot * dt
        new_delta = np.clip(new_delta, self.delta_min, self.delta_max)
        
        # Integrate jerk to get acceleration
        new_ax = current_ax + Jx * dt
        new_ax = np.clip(new_ax, self.ax_min, self.ax_max)
        
        # Convert acceleration to throttle/brake
        if new_ax >= 0:
            throttle = new_ax / self.ax_max
        else:
            throttle = new_ax / abs(self.ax_min)  # Negative for braking
        
        return new_delta, throttle


# =============================================================================
# Test / Demo
# =============================================================================

def test_dallas_mpc():
    """Test the Dallas MPC with a simple trajectory"""
    print("\n" + "="*60)
    print("Dallas et al. MPC Test")
    print("="*60)
    
    # Test without NN first
    print("\n--- Testing Linear Tire Model ---")
    mpc = DallasMPC(nn_casadi=None, dt=0.1, N=30)
    
    # Initial state: [x, y, psi, u, v, omega, delta, ax]
    z0 = np.array([0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0])
    
    # Reference: straight line
    N = mpc.N
    t_ref = np.linspace(0, (N + 1) * 0.1, N + 1)
    v_const = 5.0
    x_ref = v_const * t_ref
    y_ref = np.zeros(N + 1)
    psi_ref = np.zeros(N + 1)
    v_ref = v_const * np.ones(N + 1)
    
    # Goal
    x_goal, y_goal, psi_goal = x_ref[-1], y_ref[-1], 0.0
    
    # Solve
    delta_dot, Jx, Z_opt, U_opt = mpc.solve(
        z0, x_ref, y_ref, psi_ref, v_ref,
        x_goal, y_goal, psi_goal
    )
    
    if Z_opt is not None:
        print(f"✓ Solve succeeded")
        print(f"  First control: δ̇ = {delta_dot:.4f} rad/s, Jx = {Jx:.4f} m/s³")
        print(f"  Final predicted position: ({Z_opt[0, -1]:.2f}, {Z_opt[1, -1]:.2f})")
        print(f"  Final predicted speed: {Z_opt[3, -1]:.2f} m/s")
    else:
        print("✗ Solve failed")
        return
    
    # Simulate closed-loop
    print("\n--- Closed-Loop Simulation (Lane Change) ---")
    
    # Lane change reference
    z_sim = z0.copy()
    dt = 0.1
    sim_time = 5.0
    n_steps = int(sim_time / dt)
    
    trajectory = [z_sim.copy()]
    controls = []
    
    for i in range(n_steps):
        t = i * dt
        
        # Generate reference: lane change from y=0 to y=3
        t_future = t + np.arange(N + 1) * dt
        x_ref = z_sim[3] * t_future + z_sim[0]  # Extrapolate x at current speed
        
        # Lane change profile (smooth sigmoid)
        y_target = 3.0 if t > 1.0 else 0.0
        y_ref = y_target * np.ones(N + 1)
        if 1.0 <= t < 3.0:
            blend = (t - 1.0) / 2.0
            y_ref = blend * 3.0 * np.ones(N + 1)
        
        psi_ref = np.zeros(N + 1)
        v_ref = 5.0 * np.ones(N + 1)
        
        x_goal = x_ref[-1]
        y_goal = y_ref[-1]
        psi_goal = 0.0
        
        # Solve MPC
        delta_dot, Jx, Z_opt, U_opt = mpc.solve(
            z_sim, x_ref, y_ref, psi_ref, v_ref,
            x_goal, y_goal, psi_goal
        )
        
        if Z_opt is None:
            print(f"  Step {i}: solve failed, using zero control")
            delta_dot, Jx = 0.0, 0.0
        
        controls.append([delta_dot, Jx])
        
        # Apply control and integrate (simple Euler for simulation)
        z_sim[6] += delta_dot * dt  # Update steering angle
        z_sim[7] += Jx * dt          # Update acceleration
        z_sim[6] = np.clip(z_sim[6], -0.6, 0.6)
        z_sim[7] = np.clip(z_sim[7], -2.6, 1.9)
        
        # Update other states using dynamics
        u, v, omega, delta, ax = z_sim[3], z_sim[4], z_sim[5], z_sim[6], z_sim[7]
        psi = z_sim[2]
        
        # Simple dynamics (without full tire model for test)
        z_sim[0] += (u * np.cos(psi) - (v + mpc.Lf * omega) * np.sin(psi)) * dt
        z_sim[1] += (u * np.sin(psi) + (v + mpc.Lf * omega) * np.cos(psi)) * dt
        z_sim[2] += omega * dt
        z_sim[3] += ax * dt
        z_sim[3] = np.clip(z_sim[3], 0.5, 20.0)
        
        # Simplified lateral dynamics
        Fyf = mpc.Cf * (delta - np.arctan2(v + mpc.Lf * omega, max(u, 0.5)))
        Fyr = mpc.Cr * (-np.arctan2(v - mpc.Lr * omega, max(u, 0.5)))
        z_sim[4] += ((Fyf + Fyr) / mpc.M - u * omega) * dt
        z_sim[5] += ((Fyf * mpc.Lf - Fyr * mpc.Lr) / mpc.Izz) * dt
        
        trajectory.append(z_sim.copy())
    
    traj = np.array(trajectory)
    
    # Report results
    final_y = traj[-1, 1]
    target_y = 3.0
    error = abs(final_y - target_y)
    
    print(f"  Final position: ({traj[-1, 0]:.2f}, {traj[-1, 1]:.2f})")
    print(f"  Final heading: {np.degrees(traj[-1, 2]):.2f}°")
    print(f"  Final speed: {traj[-1, 3]:.2f} m/s")
    print(f"  Lane change error: {error:.3f} m")
    
    if error < 0.5:
        print("  ✓ Lane change successful!")
    else:
        print(f"  ⚠ Lane change incomplete (target y=3.0)")
    
    return traj, controls


def test_with_nn():
    """Test Dallas MPC with NN tire model"""
    print("\n" + "="*60)
    print("Dallas MPC with NN Tire Model")
    print("="*60)
    
    # Paths
    model_path = Path(__file__).parent / "nn_models" / "best_terrain_nn.pt"
    scaler_path = Path(__file__).parent / "nn_models" / "scalers.pkl"
    
    if not model_path.exists():
        print(f"⚠ NN model not found at {model_path}")
        print("  Run test with linear model only")
        return
    
    # Soft terrain parameters
    terrain_params = {
        'Kphi': 2.1e6,
        'Kc': 500,
        'n': 1.38,
        'c': 300,
        'phi': 26,
        'k': 0.048
    }
    
    print("\n--- Loading NN into CasADi ---")
    nn_casadi = NNCasADi(model_path, scaler_path, terrain_params)
    
    # Test NN function
    print("\n--- Testing NN CasADi Function ---")
    alpha_test = 0.05  # 5 degrees
    Fz_test = 5000.0
    u_test = 5.0
    
    Fx, Fy = nn_casadi.predict(alpha_test, Fz_test, u_test)
    print(f"  α = {np.degrees(alpha_test):.1f}°, Fz = {Fz_test:.0f} N, u = {u_test:.1f} m/s")
    print(f"  NN predicts: Fx = {float(Fx):.1f} N, Fy = {float(Fy):.1f} N")
    
    # Create MPC with NN
    print("\n--- Creating Dallas MPC with NN ---")
    mpc = DallasMPC(nn_casadi=nn_casadi, dt=0.1, N=30)
    
    # Test solve
    z0 = np.array([0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0, 0.0])
    N = mpc.N
    x_ref = 5.0 * np.linspace(0, (N + 1) * 0.1, N + 1)
    y_ref = np.zeros(N + 1)
    psi_ref = np.zeros(N + 1)
    v_ref = 5.0 * np.ones(N + 1)
    
    delta_dot, Jx, Z_opt, U_opt = mpc.solve(
        z0, x_ref, y_ref, psi_ref, v_ref,
        x_ref[-1], y_ref[-1], 0.0
    )
    
    if Z_opt is not None:
        print(f"✓ NN-MPC solve succeeded")
        print(f"  Control: δ̇ = {delta_dot:.4f} rad/s, Jx = {Jx:.4f} m/s³")
    else:
        print("✗ NN-MPC solve failed")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Dallas et al. MPC")
    parser.add_argument('--nn', action='store_true', help='Test with NN tire model')
    args = parser.parse_args()
    
    # Run basic test
    test_dallas_mpc()
    
    if args.nn:
        test_with_nn()
