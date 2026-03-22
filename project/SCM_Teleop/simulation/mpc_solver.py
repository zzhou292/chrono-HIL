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
        
        # Detect temporal model
        self.temporal_K = 1
        if isinstance(checkpoint, dict) and 'temporal_K' in checkpoint:
            self.temporal_K = checkpoint['temporal_K']
        
        input_size = self.temporal_K * 5 + 6 if self.temporal_K > 1 else 11
        
        if self.temporal_K > 1:
            from train_temporal_nn import TerrainTemporalNN
            self.pytorch_model = TerrainTemporalNN(
                input_size=input_size, output_size=2,
                hidden_sizes=hidden_sizes, temporal_K=self.temporal_K)
        else:
            self.pytorch_model = TerrainNN(input_size=input_size, output_size=2, hidden_sizes=hidden_sizes)
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
        
        if self.temporal_K > 1:
            print(f"✓ Loaded TEMPORAL NN model: K={self.temporal_K}, input_dim={input_size}, {len(self.weights)} param tensors")
        else:
            print(f"✓ Loaded NN model: {len(self.weights)} parameter tensors")
    
    def _build_casadi_function(self):
        """Build CasADi function replicating the NN forward pass (variable depth).
        
        The sinkage exponent `n` is a symbolic parameter so the MPC can pass
        the UKF estimate at each solve without rebuilding the solver.
        
        Supports two feature orders:
        - v6 (Dallas format): slip_ratio, slip_angle, velocity, Fz, steering_rate, Kphi, Kc, n, c, phi(rad), k
        - v3 (legacy format): Fz, slip_angle, slip_ratio, camber, velocity, Kphi, Kc, n, c, phi(deg), k
        """
        # Dispatch to temporal builder if temporal model
        if self.temporal_K > 1:
            self._build_temporal_casadi_functions()
            return
        
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

        # Build batched version for efficient MPC dynamics (6 or 8 tire evals at once)
        self._build_batched_function(layer_indices)
    
    def _build_batched_function(self, layer_indices):
        """Build a CasADi function that evaluates the NN for B samples at once.

        Instead of calling predict() B times (each creating an independent
        symbolic sub-graph), we stack B input vectors into a [11×B] matrix
        and push them through the same weight matrices with a single set of
        ca.mtimes calls.  CasADi's AD then differentiates through one graph
        instead of B, which dramatically cuts Jacobian computation.
        """
        B = ca.SX.sym('B_size')  # not used; batch size is implicit
        # Symbolic inputs: each column of alphas/Fzs/... is one sample
        MAX_BATCH = 8  # upper bound (6 lateral + 2 traction)
        alphas = ca.SX.sym('alphas', MAX_BATCH)
        Fzs    = ca.SX.sym('Fzs',    MAX_BATCH)
        us     = ca.SX.sym('us',     MAX_BATCH)
        kappas = ca.SX.sym('kappas', MAX_BATCH)
        n_ts   = ca.SX.sym('n_ts',   MAX_BATCH)
        srs    = ca.SX.sym('srs',    MAX_BATCH)

        terrain = self.terrain_params
        if self.model_format == 'v6':
            phi_rad = np.radians(terrain['phi'])
            rows = [kappas.T, alphas.T, us.T, Fzs.T, srs.T]
            for val in [terrain['Kphi'], terrain['Kc']]:
                rows.append(ca.repmat(ca.DM(val), 1, MAX_BATCH))
            rows.append(n_ts.T)
            for val in [terrain['c'], phi_rad, terrain['k']]:
                rows.append(ca.repmat(ca.DM(val), 1, MAX_BATCH))
        else:
            rows = [Fzs.T, alphas.T, kappas.T,
                    ca.repmat(ca.DM(0.0), 1, MAX_BATCH), us.T]
            for val in [terrain['Kphi'], terrain['Kc']]:
                rows.append(ca.repmat(ca.DM(val), 1, MAX_BATCH))
            rows.append(n_ts.T)
            for val in [terrain['c'], terrain['phi'], terrain['k']]:
                rows.append(ca.repmat(ca.DM(val), 1, MAX_BATCH))

        X_batch = ca.vertcat(*rows)  # [11 x MAX_BATCH]

        # Normalise
        X_mean = ca.DM(self.X_mean.reshape(-1, 1))
        X_scale = ca.DM(self.X_scale.reshape(-1, 1))
        H = (X_batch - ca.repmat(X_mean, 1, MAX_BATCH)) / ca.repmat(X_scale, 1, MAX_BATCH)

        for i in layer_indices:
            W = ca.DM(self.weights[f'layers.{i}.weight'])
            b = ca.DM(self.weights[f'layers.{i}.bias']).reshape((-1, 1))
            H = ca.mtimes(W, H) + ca.repmat(b, 1, MAX_BATCH)
            if i < layer_indices[-1]:
                H = ca.tanh(H)

        y_mean  = ca.DM(self.y_mean.reshape(-1, 1))
        y_scale = ca.DM(self.y_scale.reshape(-1, 1))
        Y = H * ca.repmat(y_scale, 1, MAX_BATCH) + ca.repmat(y_mean, 1, MAX_BATCH)  # [2 x B]

        Fxs_out = Y[0, :].T  # [MAX_BATCH x 1]
        Fys_out = Y[1, :].T

        self._BATCH = MAX_BATCH
        self.predict_batch = ca.Function(
            'nn_tire_batch',
            [alphas, Fzs, us, kappas, n_ts, srs],
            [Fxs_out, Fys_out],
            ['alphas', 'Fzs', 'us', 'kappas', 'n_ts', 'srs'],
            ['Fxs', 'Fys']
        )
        print(f"  + Batched NN function built (max {MAX_BATCH} simultaneous evaluations)")

    def _build_temporal_casadi_functions(self):
        """Build CasADi functions for temporal NN (K > 1).

        The temporal model input is [current_ops(5), history((K-1)*5), terrain(6)].
        History is a frozen vector of per-tire operating conditions from recent
        observations, passed as an NLP parameter.
        """
        K = self.temporal_K
        terrain = self.terrain_params
        phi_rad = np.radians(terrain['phi'])
        self.model_format = 'v6_temporal'
        self.n_nominal = terrain['n']

        layer_indices = sorted(set(
            int(k.split('.')[1]) for k in self.weights if k.startswith('layers.')
        ))
        n_params = sum(W.shape[0]*W.shape[1] for k, W in self.weights.items() if 'weight' in k)

        # --- Scalar function (for debug / single eval) ---
        alpha = ca.SX.sym('alpha')
        Fz_sym = ca.SX.sym('Fz')
        u_sym = ca.SX.sym('u')
        kap = ca.SX.sym('kappa')
        n_t = ca.SX.sym('n_terrain')
        sr = ca.SX.sym('sr')
        hist = ca.SX.sym('hist', (K - 1) * 5)

        current_ops = ca.vertcat(kap, alpha, u_sym, Fz_sym, sr)
        terrain_vec = ca.vertcat(
            terrain['Kphi'], terrain['Kc'], n_t,
            terrain['c'], phi_rad, terrain['k'])
        x_in = ca.vertcat(current_ops, hist, terrain_vec)

        x_s = (x_in - self.X_mean.reshape(-1, 1)) / self.X_scale.reshape(-1, 1)
        h = x_s
        for i in layer_indices:
            W = self.weights[f'layers.{i}.weight']
            b = self.weights[f'layers.{i}.bias']
            h = ca.mtimes(W, h) + b.reshape(-1, 1)
            if i < layer_indices[-1]:
                h = ca.tanh(h)
        y_out = h * self.y_scale.reshape(-1, 1) + self.y_mean.reshape(-1, 1)

        self.predict_tire_force = ca.Function(
            'nn_tire_temporal',
            [alpha, Fz_sym, u_sym, kap, n_t, sr, hist],
            [y_out[0], y_out[1]],
            ['alpha', 'Fz', 'u', 'kappa', 'n_terrain', 'steering_rate', 'history'],
            ['Fx', 'Fy'])

        # --- Batched function: accepts pre-built [input_dim x 8] matrix ---
        MAX_BATCH = 8
        input_dim = K * 5 + 6
        X_batch = ca.SX.sym('X_batch', input_dim, MAX_BATCH)

        X_mean = ca.DM(self.X_mean.reshape(-1, 1))
        X_scale = ca.DM(self.X_scale.reshape(-1, 1))
        H = (X_batch - ca.repmat(X_mean, 1, MAX_BATCH)) / ca.repmat(X_scale, 1, MAX_BATCH)
        for i in layer_indices:
            W = ca.DM(self.weights[f'layers.{i}.weight'])
            b = ca.DM(self.weights[f'layers.{i}.bias']).reshape((-1, 1))
            H = ca.mtimes(W, H) + ca.repmat(b, 1, MAX_BATCH)
            if i < layer_indices[-1]:
                H = ca.tanh(H)
        y_mean = ca.DM(self.y_mean.reshape(-1, 1))
        y_scale = ca.DM(self.y_scale.reshape(-1, 1))
        Y = H * ca.repmat(y_scale, 1, MAX_BATCH) + ca.repmat(y_mean, 1, MAX_BATCH)

        self._BATCH = MAX_BATCH
        self.predict_batch_temporal = ca.Function(
            'nn_tire_batch_temporal',
            [X_batch], [Y[0, :].T, Y[1, :].T],
            ['X_batch'], ['Fxs', 'Fys'])

        # Keep predict_batch as alias for non-temporal code that checks _BATCH
        self.predict_batch = None  # not valid for temporal; use predict_batch_temporal

        print(f"✓ Built temporal CasADi NN (K={K}, {len(layer_indices)} layers, {n_params} weight params)")
        print(f"  + Batched temporal function: [{input_dim}×{MAX_BATCH}] input matrix")

    # Convenience: scalar predict unchanged
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
        if self.temporal_K > 1:
            import numpy as np
            hist = np.zeros((self.temporal_K - 1) * 5)
            Fx, Fy = self.predict_tire_force(alpha, Fz, u, kappa, n_terrain, steering_rate, hist)
        else:
            Fx, Fy = self.predict_tire_force(alpha, Fz, u, kappa, n_terrain, steering_rate)
        return Fx, Fy
    
    def predict_numeric(self, alpha, Fz, u, kappa=0.0, n_terrain=None, steering_rate=0.0):
        """Evaluate numerically and return float values (for debugging)."""
        if n_terrain is None:
            n_terrain = self.n_nominal
        if self.temporal_K > 1:
            import numpy as np
            hist = np.zeros((self.temporal_K - 1) * 5)
            Fx, Fy = self.predict_tire_force(alpha, Fz, u, kappa, n_terrain, steering_rate, hist)
        else:
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
                 nn_scale=1.0, nn_sign=1, kappa_mode='zero',
                 lateral_load_transfer=True, tire_model='nn'):
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
            lateral_load_transfer: If True, use 4 NN calls per step (outer+inner per axle).
                                   If False, use 2 NN calls with mean Fz per axle (faster).
            tire_model: 'nn', 'pacejka', 'tmeasy', or 'linear'. When 'nn', uses nn_casadi.
                        Falls back to pacejka if nn_casadi is None and tire_model is 'nn'.
        """
        self.tire_model = tire_model
        self.kappa_mode = kappa_mode
        self.nn_scale = nn_scale
        self.lateral_load_transfer = lateral_load_transfer
        # Vehicle parameters — defaults from param_consistency.HMMWV_VEHICLE_PARAMS
        # Overridden by `params` dict if provided.
        from param_consistency import HMMWV_VEHICLE_PARAMS as _defaults
        self.M = _defaults["M"]
        self.Izz = _defaults["Izz"]
        self.Lf = _defaults["Lf"]
        self.Lr = _defaults["Lr"]
        self.L = _defaults["L"]
        self.h_cg = _defaults.get("h_cg", 0.65)  # CG height for load transfer
        self.T = _defaults.get("T", 1.8194)  # Track width for lateral load transfer
        
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
        
        # TMeasy lateral force parameters (per tire, from HMMWV data)
        # Degressive model: linear region → parabolic transition → saturation plateau
        # dFy0: initial slope (cornering stiffness) at α=0 (N/rad per tire)
        # Fym: peak lateral force per tire (N)
        # alpha_m: slip angle at peak force (rad)
        # alpha_slide: slip angle at full sliding (rad)
        self.tmeasy_dFy0 = 40000.0   # ~Cf/2 per tire
        self.tmeasy_Fym = 4000.0     # peak Fy per tire ≈ μ * Fz_per_tire
        self.tmeasy_alpha_m = 0.12   # ~7 degrees
        self.tmeasy_alpha_slide = 0.25  # ~14 degrees
        
        if params:
            self.__dict__.update(params)
        
        self.dt = dt
        self.N = N
        self.nx = 8   # [x, y, psi, u, v, omega, delta, ax]
        self.nu = 2   # [delta_dot, Jx]
        
        # Neural network tire model
        self.nn_casadi = nn_casadi
        self.use_nn = nn_casadi is not None and self.tire_model == 'nn'
        
        # State constraints (from Dallas paper)
        self.u_min = 0.5      # Min longitudinal speed (m/s)
        self.u_max = 20.0     # Max longitudinal speed (m/s)
        self.delta_min = -0.528 # Min steering angle (rad) — GetVehicle().GetMaxSteeringAngle()
        self.delta_max = 0.528  # Max steering angle (rad)
        self.ax_min = -2.6    # Min acceleration (m/s^2) — Dallas paper value
        self.ax_max = 1.9     # Max acceleration (m/s^2) — Dallas paper value
        
        # Control constraints 
        self.delta_dot_min = -0.5  # Min steering rate (rad/s)
        self.delta_dot_max = 0.5   # Max steering rate (rad/s)
        self.Jx_min = -3.0         # Min jerk (m/s^3)
        self.Jx_max = 3.0          # Max jerk (m/s^3)
        
        # Cost weights (from Dallas paper Eq. 11)
        self.w_t = 0.0       # Time weight (disabled for fixed horizon)
        self.w_psi = 10.0    # Heading alignment weight
        self.w_delta_dot = 50.0   # Steering rate penalty
        self.w_Jx = 20.0      # Jerk penalty
        self.w_terminal = 50.0    # Terminal distance weight
        
        # Additional state tracking weights
        self.w_lateral = 120.0   # Lateral error weight (prevent right-swing)
        self.w_heading = 20.0     # Heading error weight
        self.w_speed = 10.0       # Speed tracking weight (high to overcome unmodeled SCM drag)
        self.w_continuity = 100.0 # Inter-solve continuity (penalise U[:,0] jump from last solve)
        
        # Last applied control (for continuity penalty)
        self.last_u0 = np.zeros(self.nu)
        self.have_last_u0 = False
        
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
        
        # Temporal NN: frozen per-tire history from recent observations
        K_t = getattr(self.nn_casadi, 'temporal_K', 1) if self.nn_casadi else 1
        self._temporal_mode = (K_t > 1)
        hist_dim = (K_t - 1) * 5 if self._temporal_mode else 0
        if self._temporal_mode:
            hist_front_sym = ca.SX.sym('hist_front', hist_dim)
            hist_rear_sym = ca.SX.sym('hist_rear', hist_dim)
        
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
        
        # Normal forces with longitudinal load transfer (Dallas paper Sec. V results
        # reference load shift during braking/acceleration).
        # Fz_f = (M*g*Lr - M*ax*h_cg) / L,  Fz_r = (M*g*Lf + M*ax*h_cg) / L
        h_cg = self.h_cg
        Fz_f_axle = (M * 9.81 * Lr - M * ax * h_cg) / (Lf + Lr)
        Fz_r_axle = (M * 9.81 * Lf + M * ax * h_cg) / (Lf + Lr)
        
        # Per-wheel mean loads
        Fz_f_mean = Fz_f_axle / 2.0
        Fz_r_mean = Fz_r_axle / 2.0

        # Lateral load transfer: ΔFz = M * ay * h_cg / T, where ay ≈ u * omega
        T = self.T
        ay = u * omega  # centripetal acceleration (good approx for steady-state cornering)
        dFz_f = M * ay * h_cg / T / 2.0  # per-wheel delta (divide by 2: half to each axle)
        dFz_r = M * ay * h_cg / T / 2.0
        # Clamp to avoid negative Fz (wheel lift) — keep at least 10% of mean
        Fz_f_outer = ca.fmin(Fz_f_mean + dFz_f, Fz_f_mean * 1.9)
        Fz_f_inner = ca.fmax(Fz_f_mean - dFz_f, Fz_f_mean * 0.1)
        Fz_r_outer = ca.fmin(Fz_r_mean + dFz_r, Fz_r_mean * 1.9)
        Fz_r_inner = ca.fmax(Fz_r_mean - dFz_r, Fz_r_mean * 0.1)

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
        
        if self.use_nn and self.nn_casadi is not None and self._temporal_mode:
            # ---- Temporal batched NN evaluation ----
            # Construct [input_dim x 8] matrix with current ops + frozen history + terrain.
            B = self.nn_casadi._BATCH  # 8
            terrain = self.nn_casadi.terrain_params
            phi_rad = np.radians(terrain['phi'])
            t_vec = ca.vertcat(terrain['Kphi'], terrain['Kc'], n_terrain_sym,
                               terrain['c'], phi_rad, terrain['k'])
            # Use κ=0.15 (near peak traction for soft terrain) so the NN
            # returns a positive net Fx.  At κ=0.08 the NN predicts negative
            # net force for sand (resistance > gross traction) which would
            # force ax ≤ 0 and prevent the vehicle from reaching target speed.
            kappa_ref = 0.15

            if self.lateral_load_transfer:
                slot_ops = [
                    ca.vertcat(kappa, alpha_f, u_safe, Fz_f_outer, sr_meas_sym),
                    ca.vertcat(kappa, alpha_f, u_safe, Fz_f_inner, sr_meas_sym),
                    ca.vertcat(kappa, alpha_r, u_safe, Fz_r_outer, sr_meas_sym),
                    ca.vertcat(kappa, alpha_r, u_safe, Fz_r_inner, sr_meas_sym),
                    ca.vertcat(kappa_ref, 0.0, u_safe, Fz_f_mean, 0.0),
                    ca.vertcat(kappa_ref, 0.0, u_safe, Fz_r_mean, 0.0),
                    ca.vertcat(kappa_ref, 0.0, u_safe, Fz_f_mean, 0.0),
                    ca.vertcat(kappa_ref, 0.0, u_safe, Fz_r_mean, 0.0),
                ]
                slot_hist = [hist_front_sym, hist_front_sym,
                             hist_rear_sym,  hist_rear_sym,
                             hist_front_sym, hist_rear_sym,
                             hist_front_sym, hist_rear_sym]
            else:
                slot_ops = [
                    ca.vertcat(kappa, alpha_f, u_safe, Fz_f_mean, sr_meas_sym),
                    ca.vertcat(kappa, alpha_r, u_safe, Fz_r_mean, sr_meas_sym),
                    ca.vertcat(kappa_ref, 0.0, u_safe, Fz_f_mean, 0.0),
                    ca.vertcat(kappa_ref, 0.0, u_safe, Fz_r_mean, 0.0),
                    ca.vertcat(0.0, 0.0, u_safe, Fz_f_mean, 0.0),
                    ca.vertcat(0.0, 0.0, u_safe, Fz_r_mean, 0.0),
                    ca.vertcat(0.0, 0.0, u_safe, Fz_f_mean, 0.0),
                    ca.vertcat(0.0, 0.0, u_safe, Fz_r_mean, 0.0),
                ]
                slot_hist = [hist_front_sym, hist_rear_sym,
                             hist_front_sym, hist_rear_sym,
                             hist_front_sym, hist_rear_sym,
                             hist_front_sym, hist_rear_sym]

            cols = [ca.vertcat(slot_ops[i], slot_hist[i], t_vec) for i in range(B)]
            X_batch = ca.horzcat(*cols)
            Fxs_all, Fys_all = self.nn_casadi.predict_batch_temporal(X_batch)

            if self.lateral_load_transfer:
                Fyf = -self.nn_scale * (Fys_all[0] + Fys_all[1])
                Fyr = -self.nn_scale * (Fys_all[2] + Fys_all[3])
                Fx_traction = 2.0 * (Fxs_all[4] + Fxs_all[5])
            else:
                Fyf = -self.nn_scale * 2.0 * Fys_all[0]
                Fyr = -self.nn_scale * 2.0 * Fys_all[1]
                Fx_traction = 2.0 * (Fxs_all[2] + Fxs_all[3])

        elif self.use_nn and self.nn_casadi is not None:
            # ---- Batched NN evaluation (non-temporal) ----
            # Pack all tire queries into a single batched call (6 lateral + 2 traction = 8).
            # This creates one symbolic sub-graph instead of 8 separate ones,
            # massively reducing the CasADi NLP size and Jacobian cost.
            B = self.nn_casadi._BATCH  # 8

            if self.lateral_load_transfer:
                # Slots 0-3: lateral forces (outer/inner x front/rear)
                # Slots 4-5: traction constraint (front/rear at kappa=0.15, alpha=0)
                # Slots 6-7: unused padding (repeat slot 4/5 values)
                kappa_ref = 0.15
                a_vec = ca.vertcat(alpha_f, alpha_f, alpha_r, alpha_r,
                                   0.0, 0.0, 0.0, 0.0)
                fz_vec = ca.vertcat(Fz_f_outer, Fz_f_inner, Fz_r_outer, Fz_r_inner,
                                    Fz_f_mean, Fz_r_mean, Fz_f_mean, Fz_r_mean)
                u_vec = ca.repmat(u_safe, B, 1)
                k_vec = ca.vertcat(kappa, kappa, kappa, kappa,
                                   kappa_ref, kappa_ref, kappa_ref, kappa_ref)
                n_vec = ca.repmat(n_terrain_sym, B, 1)
                sr_vec = ca.vertcat(sr_meas_sym, sr_meas_sym, sr_meas_sym, sr_meas_sym,
                                    0.0, 0.0, 0.0, 0.0)

                Fxs_all, Fys_all = self.nn_casadi.predict_batch(
                    a_vec, fz_vec, u_vec, k_vec, n_vec, sr_vec)

                Fyf = -self.nn_scale * (Fys_all[0] + Fys_all[1])  # outer+inner front
                Fyr = -self.nn_scale * (Fys_all[2] + Fys_all[3])  # outer+inner rear
                Fx_traction = 2.0 * (Fxs_all[4] + Fxs_all[5])    # front+rear traction
            else:
                # 2 lateral + 2 traction = 4 real, padded to 8
                kappa_ref = 0.15
                a_vec = ca.vertcat(alpha_f, alpha_r, 0.0, 0.0,
                                   0.0, 0.0, 0.0, 0.0)
                fz_vec = ca.vertcat(Fz_f_mean, Fz_r_mean, Fz_f_mean, Fz_r_mean,
                                    Fz_f_mean, Fz_r_mean, Fz_f_mean, Fz_r_mean)
                u_vec = ca.repmat(u_safe, B, 1)
                k_vec = ca.vertcat(kappa, kappa, kappa_ref, kappa_ref,
                                   0.0, 0.0, 0.0, 0.0)
                n_vec = ca.repmat(n_terrain_sym, B, 1)
                sr_vec = ca.vertcat(sr_meas_sym, sr_meas_sym, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0)

                Fxs_all, Fys_all = self.nn_casadi.predict_batch(
                    a_vec, fz_vec, u_vec, k_vec, n_vec, sr_vec)

                Fyf = -self.nn_scale * 2.0 * Fys_all[0]
                Fyr = -self.nn_scale * 2.0 * Fys_all[1]
                Fx_traction = 2.0 * (Fxs_all[2] + Fxs_all[3])
        else:
            # ---- Analytical tire models (no NN) ----
            # Combined slip reduction factor (kappa influence on lateral force)
            lateral_limit_factor = ca.sqrt(ca.fmax(1.0 - (kappa/0.2)**2, 0.1))

            if self.tire_model == 'pacejka':
                # Pacejka Magic Formula tire model (simplified)
                # Fy = D * sin(C * atan(B*α - E*(B*α - atan(B*α))))
                Bp = self.pacejka_B
                Cp = self.pacejka_C
                Ep = self.pacejka_E

                # Peak force D = μ * Fz (with combined slip reduction)
                Df = self.mu * Fz_f_axle * lateral_limit_factor
                Dr = self.mu * Fz_r_axle * lateral_limit_factor

                # Magic formula: Fy = D * sin(C * atan(B*α - E*(B*α - atan(B*α))))
                Baf = Bp * alpha_f
                Bar = Bp * alpha_r
                Fyf = Df * ca.sin(Cp * ca.atan(Baf - Ep * (Baf - ca.atan(Baf))))
                Fyr = Dr * ca.sin(Cp * ca.atan(Bar - Ep * (Bar - ca.atan(Bar))))

            elif self.tire_model == 'tmeasy':
                # TMeasy degressive lateral force model
                # Three regions: linear (|α| < α_m), parabolic transition (α_m..α_slide),
                # and sliding (|α| > α_slide) where Fy ≈ Fym (saturated).
                dFy0 = self.tmeasy_dFy0
                Fym = self.tmeasy_Fym
                am = self.tmeasy_alpha_m
                a_slide = self.tmeasy_alpha_slide

                # Scale peak force with vertical load ratio and combined slip
                Fz_nom = (Fz_f_axle + Fz_r_axle) / 2.0
                Fz_f_ratio = Fz_f_axle / (2.0 * ca.fmax(Fz_nom, 1.0))
                Fz_r_ratio = Fz_r_axle / (2.0 * ca.fmax(Fz_nom, 1.0))

                # TMeasy formula: smooth approximation using CasADi
                # For |α| <= α_m: Fy = dFy0 * α (linear)
                # For α_m < |α| < α_slide: parabolic transition to peak
                # Use smooth blend:  Fy = Fym * sin(π/2 * α / α_m) clipped at Fym
                #   (good approximation of the degressive characteristic)
                Fyf_per_tire = Fym * Fz_f_ratio * lateral_limit_factor * ca.sin(
                    ca.fmin(1.5707963 * alpha_f / ca.fmax(am, 1e-4), 1.5707963))
                Fyr_per_tire = Fym * Fz_r_ratio * lateral_limit_factor * ca.sin(
                    ca.fmin(1.5707963 * alpha_r / ca.fmax(am, 1e-4), 1.5707963))
                # Axle total (2 tires per axle)
                Fyf = 2.0 * Fyf_per_tire
                Fyr = 2.0 * Fyr_per_tire

            elif self.tire_model == 'linear':
                # Simple linear cornering stiffness: Fy = -C * α
                # (No saturation — valid for small slip angles only)
                Fyf = -self.Cf * alpha_f * lateral_limit_factor
                Fyr = -self.Cr * alpha_r * lateral_limit_factor

            else:
                # Default fallback: Pacejka
                Bp = self.pacejka_B
                Cp = self.pacejka_C
                Ep = self.pacejka_E
                Df = self.mu * Fz_f_axle * lateral_limit_factor
                Dr = self.mu * Fz_r_axle * lateral_limit_factor
                Baf = Bp * alpha_f
                Bar = Bp * alpha_r
                Fyf = Df * ca.sin(Cp * ca.atan(Baf - Ep * (Baf - ca.atan(Baf))))
                Fyr = Dr * ca.sin(Cp * ca.atan(Bar - Ep * (Bar - ca.atan(Bar))))

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
        
        if self._temporal_mode:
            self.f = ca.Function('f',
                [z, zeta, n_terrain_sym, sr_meas_sym, hist_front_sym, hist_rear_sym],
                [zdot, Fx_traction],
                ['z', 'zeta', 'n_terrain', 'sr_meas', 'hist_front', 'hist_rear'],
                ['zdot', 'Fx_traction'])
        else:
            self.f = ca.Function('f', [z, zeta, n_terrain_sym, sr_meas_sym],
                                 [zdot, Fx_traction],
                                 ['z', 'zeta', 'n_terrain', 'sr_meas'],
                                 ['zdot', 'Fx_traction'])
        
        # =====================================================================
        # Optimal Control Problem Setup
        # =====================================================================
        
        # Decision variables: states at each time step + controls
        Z = ca.SX.sym('Z', nx, N+1)
        U = ca.SX.sym('U', nu, N)
        
        # Parameters: [z0(8), x_goal, y_goal, psi_goal, v_target, n_terrain, sr_meas,
        #              last_u0(2), w_cont(1),
        #              hist_front(hist_dim), hist_rear(hist_dim),   <-- temporal only
        #              x_ref(N+1), y_ref(N+1), psi_ref(N+1), v_ref(N+1)]
        n_ref = 4 * (N + 1)  # Reference trajectory
        n_cont = nu + 1       # last_u0(2) + w_cont(1)
        n_hist = 2 * hist_dim  # front + rear history (0 if non-temporal)
        P = ca.SX.sym('P', nx + 6 + n_cont + n_hist + n_ref)
        
        # Extract parameters
        z0 = P[:nx]
        x_goal = P[nx]
        y_goal = P[nx + 1]
        psi_goal = P[nx + 2]
        v_target = P[nx + 3]
        n_terrain_param = P[nx + 4]
        sr_meas_param = P[nx + 5]  # Measured steering rate (constant for horizon)
        last_u0 = P[nx + 6 : nx + 6 + nu]  # Last applied control
        w_cont = P[nx + 6 + nu]             # Continuity weight (0 on first solve)
        
        hist_base = nx + 6 + n_cont
        if self._temporal_mode:
            hist_front_param = P[hist_base : hist_base + hist_dim]
            hist_rear_param = P[hist_base + hist_dim : hist_base + 2 * hist_dim]
        
        ref_start = hist_base + n_hist
        
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
            # Path tracking: true cross-track error (perpendicular to path tangent).
            # e_ct = (y-y_ref)*cos(psi_ref) - (x-x_ref)*sin(psi_ref)
            # This is correct even when the path heading is non-zero.
            # -----------------------------------------------------------------
            e_ct = (y_k - y_ref_k) * ca.cos(psi_ref_k) - (x_k - x_ref_k) * ca.sin(psi_ref_k)
            heading_error = (psi_k - psi_ref_k)**2
            speed_error = (u_k - v_ref_k)**2
            
            cost += self.w_lateral * e_ct**2
            cost += self.w_heading * heading_error
            cost += self.w_speed * speed_error
            
            # -----------------------------------------------------------------
            # Control rate penalty for smoothness
            # -----------------------------------------------------------------
            if k > 0:
                du = U[:, k] - U[:, k-1]
                cost += 200.0 * ca.dot(du, du)
            
            # -----------------------------------------------------------------
            # Dynamics constraint.
            # f() returns (zdot, Fx_traction); Fx_traction from the first
            # evaluation is used for the traction constraint below.
            # NN (smooth tanh) works well with Euler; Pacejka's sharper
            # sin(C*atan(..)) nonlinearities need RK4 at dt=0.1s.
            # -----------------------------------------------------------------
            if self._temporal_mode:
                # Temporal NN with Euler integration — history frozen across horizon
                zdot, Fx_avail = self.f(zk, uk, n_terrain_param, sr_meas_param,
                                         hist_front_param, hist_rear_param)
                z_next = zk + dt * zdot
            elif self.use_nn:
                # Euler integration (1 f-eval per step)
                zdot, Fx_avail = self.f(zk, uk, n_terrain_param, sr_meas_param)
                z_next = zk + dt * zdot
            else:
                # RK4 integration (4 f-evals per step)
                k1, Fx_avail = self.f(zk, uk, n_terrain_param, sr_meas_param)
                k2, _ = self.f(zk + dt / 2 * k1, uk, n_terrain_param, sr_meas_param)
                k3, _ = self.f(zk + dt / 2 * k2, uk, n_terrain_param, sr_meas_param)
                k4, _ = self.f(zk + dt * k3, uk, n_terrain_param, sr_meas_param)
                z_next = zk + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            
            g_eq.append(Z[:, k+1] - z_next)
            
            # -----------------------------------------------------------------
            # Traction constraint: commanded force M*ax must not exceed
            # available tire force Fx_total from NN prediction.
            # Fx_avail is from the k1 evaluation above (same batch call).
            # -----------------------------------------------------------------
            ax_k = zk[7]
            
            # Acceleration traction limit: M*ax - Fx_avail ≤ 0
            g_ineq.append(M * ax_k - Fx_avail)
            
            # Braking traction limit: -M*ax - |Fx_avail| ≤ 0
            # (braking force magnitude limited by same traction)
            g_ineq.append(-M * ax_k - ca.fabs(Fx_avail))
        
        # Combine constraints: equality first, then inequality
        g = g_eq + g_ineq
        
        # =====================================================================
        # Inter-solve continuity: penalise U[:,0] jumping from last applied control.
        # Prevents the first control of each new solve from being wildly different
        # from what was applied on the previous solve, reducing throttle/brake toggling.
        # =====================================================================
        cost += w_cont * ca.dot(U[:, 0] - last_u0, U[:, 0] - last_u0)
        
        # =====================================================================
        # Terminal Cost: true cross-track + heading (no x-distance).
        # =====================================================================
        z_final = Z[:, N]
        x_f, y_f, psi_f = z_final[0], z_final[1], z_final[2]
        e_ct_terminal = (y_f - y_goal) * ca.cos(psi_goal) - (x_f - x_goal) * ca.sin(psi_goal)
        
        cost += self.w_terminal * e_ct_terminal**2
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
            'ipopt.acceptable_iter': 3,    # accept after 3 consecutive "good enough" iters
            'ipopt.mu_strategy': 'adaptive',
            # Better scaling helps when variables span very different magnitudes
            # (position in metres vs slip angles in milliradians)
            'ipopt.nlp_scaling_method': 'gradient-based',
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
              n_terrain=None, sr_meas=0.0,
              hist_front=None, hist_rear=None):
        """
        Solve the MPC optimal control problem.
        
        Args:
            z0: Current state [x, y, psi, u, v, omega, delta, ax]
            x_ref, y_ref, psi_ref, v_ref: Reference trajectory (N+1 points)
            x_goal, y_goal, psi_goal: Goal position and heading
            n_terrain: Sinkage exponent from UKF (None => use nominal from terrain config)
            sr_meas: Measured steering rate (rad/s). This is the observed/filtered steering
                     rate from the vehicle, NOT the MPC control input. Default 0.0.
            hist_front: Flattened per-tire history for front tires [(K-1)*5 array].
                        Order per timestep: [kappa, alpha_f, u, Fz_f, sr].
                        Timesteps ordered most-recent-first: [t-1, t-2, ...].
                        Only used when the NN is temporal (K > 1).
            hist_rear: Same for rear tires.
            
        Returns:
            delta_dot: Steering rate command
            Jx: Jerk command  
            Z_opt: Predicted state trajectory
            U_opt: Control sequence
        """
        nx, nu, N = self.nx, self.nu, self.N
        
        if n_terrain is None:
            n_terrain = self.nn_casadi.n_nominal if self.nn_casadi is not None else 1.1
        
        # Build parameter vector:
        # [z0, x_goal, y_goal, psi_goal, v_target, n_terrain, sr_meas,
        #  last_u0(2), w_cont(1), [hist_front, hist_rear], refs...]
        p = list(z0)
        p += [x_goal, y_goal, psi_goal, v_ref[0], n_terrain, sr_meas]
        
        # Inter-solve continuity
        if self.have_last_u0:
            p += list(self.last_u0) + [self.w_continuity]
        else:
            p += [0.0, 0.0, 0.0]  # No penalty on first solve
        
        # Temporal history (frozen per-tire obs for the whole horizon)
        if self._temporal_mode:
            K_t = self.nn_casadi.temporal_K
            hdim = (K_t - 1) * 5
            if hist_front is None:
                hist_front = np.zeros(hdim)
            if hist_rear is None:
                hist_rear = np.zeros(hdim)
            p += list(hist_front[:hdim]) + list(hist_rear[:hdim])
        
        for k in range(N + 1):
            p += [x_ref[k], y_ref[k], psi_ref[k], v_ref[k]]
        
        # Initial guess with warm start
        if self.X_warm is not None:
            # Shift previous solution by one step (standard MPC warm start)
            Z_init = np.hstack([self.X_warm[:, 1:], self.X_warm[:, -1:]])
            U_init = np.hstack([self.U_warm[:, 1:], self.U_warm[:, -1:]])
        else:
            # Cold start: forward-simulate along the reference instead of just
            # tiling z0.  This gives IPOPT a near-feasible starting trajectory
            # and is critical for Pacejka which uses the more expensive RK4
            # integrator — tiling z0 (vehicle stationary) leaves IPOPT with an
            # infeasible initial point during the speed-up transient, causing
            # Maximum_Iterations_Exceeded on every solve until the first success.
            _dt = self.dt
            Z_init = np.zeros((nx, N + 1))
            U_init = np.zeros((nu, N))
            Z_init[:, 0] = np.array(z0)
            for k in range(N):
                zk   = Z_init[:, k]
                xk, yk, psik, uk, vk, omegak, deltak, axk = zk[:8]

                # Proportional heading + lateral correction toward reference
                psi_err    = np.arctan2(np.sin(psi_ref[k] - psik),
                                        np.cos(psi_ref[k] - psik))
                y_err      = (y_ref[k] - yk) * np.cos(psik) - (x_ref[k] - xk) * np.sin(psik)
                delta_des  = float(np.clip(psi_err * 1.5 + y_err * 0.2, -0.5, 0.5))
                ddot_k     = float(np.clip((delta_des - deltak) / _dt, -0.5, 0.5))

                # Proportional acceleration toward target speed
                spd_err    = float(v_ref[k]) - uk
                ax_des     = float(np.clip(spd_err * 0.8, -3.0, 3.0))
                jerk_k     = float(np.clip((ax_des - axk) / _dt, -5.0, 5.0))

                U_init[:, k] = [ddot_k, jerk_k]

                # Simple kinematic rollout (bicycle model, no tire forces)
                ax_next    = float(np.clip(axk + _dt * jerk_k,    -3.0, 3.0))
                delta_next = float(np.clip(deltak + _dt * ddot_k, -0.5, 0.5))
                u_next     = max(0.1, uk + _dt * axk)
                v_next     = vk * 0.9   # gentle damping
                omega_next = omegak * 0.9
                psi_next   = psik + _dt * omegak
                x_next     = xk + _dt * (uk * np.cos(psik) - vk * np.sin(psik))
                y_next     = yk + _dt * (uk * np.sin(psik) + vk * np.cos(psik))

                Z_init[:8, k + 1] = [x_next, y_next, psi_next, u_next,
                                     v_next, omega_next, delta_next, ax_next]
                if nx > 8:
                    Z_init[8:, k + 1] = zk[8:]  # pass through any extra states
        
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
            
            # Store diagnostics (accessible after solve)
            self.last_cost = float(sol['f'])
            stats = self.solver.stats()
            self.last_solver_status = stats.get('return_status', 'unknown')
            self.last_iter_count = stats.get('iter_count', -1)

            # Selective warm start:
            # - Converged / acceptable: always save — best possible warm start.
            # - Maximum_Iterations_Exceeded: save — the iterate is still interior-
            #   feasible (IPOPT just ran out of budget), so it's a good starting
            #   point for the next solve and cures the cold-start cascade.
            # - Infeasible_Problem_Detected / Restoration_Failed: do NOT save —
            #   the iterate is far outside the feasible set; feeding it back
            #   immediately re-triggers infeasibility on the next call.
            _save_statuses = {
                'Solve_Succeeded',
                'Solved_To_Acceptable_Level',
                'Maximum_Iterations_Exceeded',
            }
            if self.last_solver_status in _save_statuses:
                self.X_warm = Z_opt
                self.U_warm = U_opt

            # Update last applied control for continuity penalty
            self.last_u0 = U_opt[:, 0].copy()
            self.have_last_u0 = True

            # Return first control: steering rate and jerk
            return U_opt[0, 0], U_opt[1, 0], Z_opt, U_opt

        except Exception as e:
            print(f"MPC solve failed: {e}")
            self.last_cost = float('nan')
            self.last_solver_status = f'EXCEPTION: {e}'
            self.last_iter_count = -1
            return 0.0, 0.0, None, None

    def integrate_controls(self, delta_dot, Jx, current_delta, current_ax, dt):
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
