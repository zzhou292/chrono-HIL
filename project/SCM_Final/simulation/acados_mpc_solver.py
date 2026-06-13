#!/usr/bin/env python3
"""
ACADOS MPC Solver — Neural Network Terramechanics
===================================================

Drop-in replacement for the CasADi+IPOPT MPC solver, using ACADOS
with SQP-RTI for significantly faster solve times.

The vehicle dynamics, cost function, and constraints are identical to
mpc_solver.py.  The NN tire model is loaded via the unified
nn_tire_model.py interface and embedded as a CasADi symbolic expression
in the ACADOS model (same approach as before — ACADOS auto-differentiates
through it).

ACADOS advantages over plain CasADi+IPOPT:
  - SQP-RTI: 1 QP per MPC step instead of full interior-point convergence
  - C code generation + compilation: eliminates Python overhead
  - Condensing / partial condensing: exploits OCP structure
  - Feedback phase preparation: even lower latency

Usage:
    from acados_mpc_solver import AcadosMPC
    from nn_tire_model import load_nn_tire_model

    nn = load_nn_tire_model('nn_models/v6_mlp_16_4', terrain_params)
    mpc = AcadosMPC(nn_tire_model=nn, dt=DEFAULT_MPC_DT, N=DEFAULT_MPC_HORIZON_STEPS)

    delta_cmd, Jx, Z_opt, U_opt = mpc.solve(z0, x_ref, y_ref, psi_ref, v_ref,
                                             x_goal, y_goal, psi_goal,
                                             terrain_params=tp)
"""

from __future__ import annotations

import os
import sys
import shutil
import hashlib
import numpy as np
import casadi as ca
from pathlib import Path
import hashlib
import json

# ACADOS needs to know where its C library lives
_ACADOS_SOURCE = os.environ.get('ACADOS_SOURCE_DIR', os.path.expanduser('~/acados'))
os.environ.setdefault('ACADOS_SOURCE_DIR', _ACADOS_SOURCE)
os.environ['LD_LIBRARY_PATH'] = (
    os.path.join(_ACADOS_SOURCE, 'lib') + ':' + os.environ.get('LD_LIBRARY_PATH', ''))

# Pre-load ACADOS shared libs so ctypes.CDLL can find them at runtime
# (setting LD_LIBRARY_PATH after process start doesn't help ctypes)
import ctypes
_acados_lib_dir = os.path.join(_ACADOS_SOURCE, 'lib')
for _lib_name in ['libblasfeo.so', 'libhpipm.so', 'libqpOASES_e.so', 'libacados.so']:
    _lib_path = os.path.join(_acados_lib_dir, _lib_name)
    if os.path.isfile(_lib_path):
        ctypes.CDLL(_lib_path, mode=ctypes.RTLD_GLOBAL)

from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

# Reuse project-level vehicle/terrain params
sys.path.insert(0, str(Path(__file__).parent))
from param_consistency import HMMWV_VEHICLE_PARAMS
from analytical_tire_models import (
    get_tire_forces as _analytical_tire_forces,
    get_oracle_pacejka_params,
)


# Default OCP discretisation (controller node and precompile_solvers must agree)
DEFAULT_MPC_DT = 0.1
DEFAULT_MPC_HORIZON_STEPS = 40


# ============================================================================
# Vehicle parameters
# ============================================================================

_VP = HMMWV_VEHICLE_PARAMS
_M = _VP['M']           # 2573.0 kg
_Izz = _VP['Izz']       # 3570.0 kg·m²
_Lf = _VP['Lf']         # 1.593 m
_Lr = _VP['Lr']         # 1.709 m
_L = _VP['L']           # 3.302 m
_h_cg = _VP['h_cg']     # 0.65 m
_T = _VP['T']           # 1.8194 m


_libc = ctypes.CDLL(None)   # libc handle for fflush

class _SuppressC:
    """Context manager that silences C-level writes to stdout/stderr.

    ACADOS/HPIPM emits MINSTEP messages via C printf() (fd 1), which bypasses
    Python's sys.stdout and cannot be suppressed by print_level=0 alone.
    Strategy:
      1. Flush Python + C stdio before diverting (avoids pre-buffered leakage).
      2. Redirect fd 1 and fd 2 to /dev/null at the OS level.
      3. Flush C stdio again inside the blackout zone before restoring fds so
         that anything buffered during the C call drains to /dev/null, not the
         terminal.
    """
    def __enter__(self):
        sys.stdout.flush()
        sys.stderr.flush()
        _libc.fflush(None)          # flush all C stdio streams
        self._saved = (os.dup(1), os.dup(2))
        self._devnull = os.open(os.devnull, os.O_WRONLY)
        os.dup2(self._devnull, 1)
        os.dup2(self._devnull, 2)
        return self

    def __exit__(self, *_):
        _libc.fflush(None)          # drain any buffered C output → /dev/null
        os.dup2(self._saved[0], 1)
        os.dup2(self._saved[1], 2)
        os.close(self._devnull)
        os.close(self._saved[0])
        os.close(self._saved[1])


class AcadosMPC:
    """
    ACADOS-based MPC for off-road autonomous driving.

    Direct steering-angle formulation: the MPC optimizes road-wheel angle δ
    and longitudinal jerk Jx each stage.  Augmented states [δ_prev, Jx_prev]
    pair with a heavy Δu penalty for smooth commands (limits straight-road
    chatter).  Effective steering rate is penalised via (δ − δ_prev)² / dt.

    State:   z = [x, y, ψ, u, v, ω, ax, δ_prev, Jx_prev]   (nx = 9)
    Control: ζ = [δ, Jx]                                   (nu = 2)
    """

    def __init__(self, nn_tire_model=None, dt=None, N=None,
                 lateral_load_transfer=True, kappa_mode='approx',
                 build_dir=None, tire_model='nn', build_solver=True,
                 symbolic_rates=None, no_temporal_staged=False,
                 friction_angle_deg=None, rate_feature_dt=None,
                 oracle_terrain=None, speed_weight: float = 70.0,
                 speed_cost_mode: str = 'symmetric',
                 obstacle_weight: float = 5e3,
                 longitudinal_force_balance: bool = False):
        """
        Args:
            nn_tire_model: Instance of NNTireModel from nn_tire_model.py.
                           Required when tire_model='nn', ignored otherwise.
            dt: Time step [s]
            N:  Prediction horizon steps
            lateral_load_transfer: Use 4-wheel Fz model (NN mode only)
            kappa_mode: 'zero' or 'approx'
            build_dir: Where to generate ACADOS C code.  Defaults to
                       /tmp/acados_mpc_{model_tag}/
            tire_model: 'nn', 'pacejka', 'pacejka-oracle', or 'tmeasy'.
            oracle_terrain: Terrain name ('clay', 'sand', 'dirt') required
                            when tire_model='pacejka-oracle'.  Used to look up
                            terrain-specific Pacejka params (mu=tan(phi_terrain))
                            from PACEJKA_ORACLE in analytical_tire_models.py.
            build_solver: If False, skip ACADOS codegen/compile and keep
                          this object as a dynamics/OCP factory.
            symbolic_rates: If True, compute dκ, dα, du, sr symbolically
                           inside the OCP rather than freezing them as
                           parameters.  Adds 3 states
                           (α_f_prev, α_r_prev, δ_sr_prev).
                           Defaults to True when a rate model is used.
            rate_feature_dt: Time base [s] used for symbolic finite-difference
                           rate channels (dα and steering rate). Set this to
                           the same effective_dt used when training the rate
                           model (e.g. nn_rate_sample_dt) for consistency.
                           Defaults to dt when not provided.
            speed_weight: Quadratic stage weight on (u - v_ref)^2. Lower values
                          let the MPC accept slower speed through turns instead
                          of accelerating/braking to chase the reference profile.
            speed_cost_mode: 'symmetric' tracks v_ref from both sides.
                             'overspeed' only penalizes u > v_ref, so the MPC
                             treats v_ref as a speed cap instead of a speed
                             command.
            obstacle_weight: Smooth obstacle-barrier weight. Higher values make
                             the in-horizon autonomous MPC avoid rocks more
                             aggressively.
        Longitudinal channel design note:
            The OCP closes the longitudinal momentum equation as u̇ = ax,
            where ax is the commanded longitudinal acceleration state and
            throttle = ax/ax_max in the controller node (calibrated open-
            loop gain). Soil-dependent peak grip enters the longitudinal
            plan through the *traction-budget constraint*
                |M·ax| ≤ Fx_traction(κ_ref, α=0, Fz, terrain_params),
            where Fx_traction is the surrogate tire model's prediction of
            achievable per-axle longitudinal force on the current terrain
            at the optimal slip ratio. On soft soil this constraint binds
            and clips the planned ax (and therefore the throttle command)
            to the terrain's physical limit. We previously experimented
            with replacing u̇ = ax by u̇ = Fx_op/M (NN tire force at the
            live operating point); empirically this degraded both lateral
            and longitudinal tracking because ax then plays two
            inconsistent roles -- throttle surrogate via the integrator
            and slip-ratio surrogate via κ ≈ ax/(μg) -- so that channel
            was removed.
        """
        if dt is None:
            dt = DEFAULT_MPC_DT
        if N is None:
            N = DEFAULT_MPC_HORIZON_STEPS
        if dt <= 0:
            raise ValueError(f"dt must be > 0, got {dt}")
        if N <= 0:
            raise ValueError(f"N must be > 0, got {N}")
        self.tire_model = tire_model
        self.nn_tire_model = nn_tire_model
        self.dt = dt
        self.N = N
        self.nx = 9    # [x, y, ψ, u, v, ω, ax, δ_prev, Jx_prev]
        self.nu = 2    # [δ, Jx]  (Jx -> κ̇ in force-balance mode)
        self.lateral_load_transfer = lateral_load_transfer
        self.kappa_mode = kappa_mode
        # Principled longitudinal force balance (gated): state idx6 becomes the
        # slip ratio κ (control κ̇), u̇ = ΣFx(κ)/M from the surrogate.
        self._force_balance = bool(longitudinal_force_balance)
        self._kappa_fb_max = 0.2      # slip-ratio box bound
        self._kappa_dot_max = 4.0     # slip-rate (κ̇) box bound [1/s]

        # Oracle Pacejka params (terrain-specific mu/B), pre-resolved at init
        # so they are embedded as constants in the CasADi expression tree.
        self._oracle_pacejka_params: dict = {}
        if tire_model == 'pacejka-oracle':
            if oracle_terrain is None:
                raise ValueError(
                    "tire_model='pacejka-oracle' requires oracle_terrain "
                    "(one of 'clay', 'sand', 'dirt')"
                )
            self._oracle_pacejka_params = get_oracle_pacejka_params(oracle_terrain)
            print(f"  [Oracle Pacejka] terrain={oracle_terrain} "
                  f"B={self._oracle_pacejka_params['B']} "
                  f"mu={self._oracle_pacejka_params['mu']:.3f}")

        # Vehicle params (exposed for controller node)
        self.Lf = _Lf
        self.Lr = _Lr
        self.M = _M
        self.Izz = _Izz
        self.h_cg = _h_cg
        self.T = _T
        self.use_nn = (nn_tire_model is not None and tire_model == 'nn')

        # Bounds
        self.u_min, self.u_max = 0.5, 20.0  # 0.5 keeps OCP well-conditioned at low speed
        self.delta_min, self.delta_max = -0.528, 0.528
        self.ax_min, self.ax_max = -2.6, 1.9
        self.delta_dot_min, self.delta_dot_max = -1.0, 1.0
        self.Jx_min, self.Jx_max = -2.0, 2.0
        self.jounce_max = 5.0    # m/s^4, max |ΔJx|/dt
        self.ay_max = 6.0        # m/s^2, max lateral acceleration (default; terrain-adapted at solve time)

        # Terrain-aware ay_max: on low-friction surfaces the physical limit
        # is μ·g.  We allow up to the physical limit (no safety margin) since
        # the speed profiler already applies its own conservative margin and
        # the NN tire model captures the true force envelope.
        if friction_angle_deg is not None:
            mu = float(np.tan(np.radians(friction_angle_deg)))
            ay_physical = mu * 9.81
            self.ay_max = min(self.ay_max, ay_physical+1)

        # Cost weights
        self.w_x = 10.0          # x-position tracking (light — prevents corner-cutting)
        self.w_y = 10.0          # y-position tracking (heavy — keeps the MPC honest about lateral errors)
        self.w_lateral = 200.0   # y-position tracking
        self.w_heading = 100.0
        self.w_speed = float(speed_weight)
        if speed_cost_mode not in ('symmetric', 'overspeed'):
            raise ValueError("speed_cost_mode must be 'symmetric' or 'overspeed'")
        self.speed_cost_mode = speed_cost_mode
        # Strong (δ−δ_prev)²/dt term smooths wheel angle (straights / RTI).
        self.w_delta_dot = 80.0
        self.w_Jx = 150.0
        self.w_ax = 20.0          # disabled — was crippling speed tracking on low-friction terrains
        self.w_terminal = 50.0
        self.w_du = 100.0   # penalty on (Jx − Jx_prev)²
        self.w_steer = 10.0
        self.max_steer_rate = self.delta_dot_max  # rad/s, for kinematic warm-start

        # Obstacle avoidance: smooth softplus barrier in OCP cost.
        # N_OBS positions injected as per-stage parameters; inactive = far-away placeholder.
        self._n_obs = 3          # fixed: max obstacles tracked per solve
        self.w_obstacle = float(obstacle_weight)

        # NN integration mode
        self._temporal_mode = (self.use_nn and nn_tire_model.temporal_K > 1)
        self._rate_mode = (self.use_nn and nn_tire_model.rate_augmented)
        self._gru_mode = (self.use_nn and hasattr(nn_tire_model, 'gru_h_dim')
                          and nn_tire_model.gru_h_dim > 0)

        # Symbolic rate propagation: compute dκ, dα, du, sr from state/control
        # at each shooting node instead of freezing them as parameters.
        # Adds α_f_prev and α_r_prev to the state vector (nx 9 → 12).
        if symbolic_rates is None:
            symbolic_rates = self._rate_mode
        self._symbolic_rate_mode = bool(symbolic_rates) and self._rate_mode
        if self._symbolic_rate_mode:
            self.nx = 12  # [x, y, ψ, u, v, ω, ax, δ_prev, Jx_prev, αf_prev, αr_prev, δ_sr_prev]

        # Symbolic steering rate for ALL NN models (not just rate-augmented).
        # Adds δ_sr_prev as one extra state so the optimizer can
        # differentiate F_y through δ̇ at each horizon stage instead of
        # using a frozen parameter that's constant across the horizon.
        # Skipped when _symbolic_rate_mode is active (it already includes δ_sr_prev).
        self._symbolic_sr = self.use_nn and not self._symbolic_rate_mode
        if self._symbolic_sr:
            self._sr_state_idx = self.nx
            self.nx += 1  # e.g. 9 → 10 for static, temporal, GRU

        # Temporal history propagation: instead of freezing (K-1) history
        # frames as identical parameters at all N stages, compute what
        # the history WOULD be at each future stage based on the previous
        # solve's predicted trajectory, and set stage-varying parameters.
        # This gives the optimiser correct Fy predictions along the
        # horizon without adding extra states (nx stays at 9), avoiding
        # the ill-conditioned Gauss-Newton Hessian that caused 100% QP
        # failure with rolling history states.
        self._temporal_rolling = False
        self._temporal_staged = self._temporal_mode and not no_temporal_staged
        self._n_hist_states = 0
        if self._temporal_staged:
            K = nn_tire_model.temporal_K
            self._hist_feat_scale = np.array([0.3, 0.5, 5.0, 5000.0, 0.5])

        # Time base for symbolic finite-difference rate channels. Should match
        # the effective sample interval used during training.
        if rate_feature_dt is None:
            rate_feature_dt = self.dt
        self._rate_feature_dt = float(max(1e-3, rate_feature_dt))

        # State-tracking time constants for symbolic auxiliary states.
        # Keep these tied to MPC discretization for numerically stable
        # forward-Euler integration (dt/tau <= 1 at the nominal setting).
        # Rate scaling is handled separately via _rate_feature_dt.
        self._tau_alpha = self.dt
        self._tau_sr = self.dt

        # nn_scale
        self.nn_scale = 1.0

        # Rear Fy scale: compensates for bicycle-model alpha_r underprediction
        # due to suspension compliance steer missing from the simplified model.
        # Note: empirically, values > 1 cause the MPC to understeer (reduce
        # front delta because it overestimates rear capability), worsening CTE.
        # The MPC self-corrects by oversteering when it underestimates rear Fy.
        self.rear_fy_scale = 1.0

        # Rear alpha scale: multiplies the rear slip angle sent to the NN.
        # The bicycle model alpha_r underestimates actual tire slip by ~2.5×
        # due to missing suspension compliance steer.  Scaling alpha_r up
        # changes the NN operating point (nonlinear), distinct from rear_fy_scale.
        self.rear_alpha_scale = 1.0

        # Solver tuning (model-adaptive)
        self._lm = 1e-3
        self._qp_iter_max = 50
        self._nlp_max_iter = 1
        # Static NN: scale Levenberg-Marquardt regularisation with model
        # complexity.  Larger networks produce sharper Jacobians whose
        # Gauss-Newton Hessian is more ill-conditioned for HPIPM, causing
        # QP_FAILURE (status 4) especially at low-speed start-up.
        # Symbolic sr adds a 1/τ_sr Jacobian channel; bump LM slightly.
        if (self.use_nn and not self._temporal_mode and not self._rate_mode
                and not self._gru_mode and nn_tire_model is not None):
            n_p = nn_tire_model.n_params
            self._lm = min(max(1e-3, n_p / 400.0 * 1e-3), 1e-2)
            if self._symbolic_sr:
                self._lm = max(self._lm, 2e-2)
        # GRU observer decoder (e.g. 27→32→32→2) creates deep expression
        # trees similar to ResNet temporals → needs heavier LM and extra
        # SQP iterations to keep QP well-conditioned.
        if self._gru_mode and nn_tire_model is not None:
            n_p = nn_tire_model.n_params
            self._lm = min(max(5e-3, n_p / 400.0 * 1e-3), 5e-2)
            self._nlp_max_iter = 3
        # Symbolic rate mode: the dα channel adds Jacobian sensitivity
        # through the 1/τ finite-difference factor.  Heavier LM dampens
        # the QP steps.
        if self._symbolic_rate_mode:
            self._lm = max(self._lm, 2e-2)
        # Non-symbolic rate mode (rates frozen as parameters): still needs
        # LM comparable to symbolic mode — the wider NN input (11 vs 8)
        # creates similar conditioning issues.
        elif self._rate_mode:
            self._lm = max(self._lm, 2e-2)
        # Temporal staged mode: history is frozen as a parameter (no extra
        # states → nx=9).  QP is well-conditioned.  Use SQP_RTI for speed.
        # The wide input dimension (K×5+6) creates ill-conditioned Hessians;
        # scale LM with K to compensate.
        if self._temporal_staged:
            K = nn_tire_model.temporal_K if nn_tire_model else 3
            self._lm = max(self._lm, 2e-2 * K)
            self._nlp_max_iter = 1  # SQP_RTI
        # Temporal rolling history: shift-register states add coupled
        # dynamics and NN Jacobians through history channels create
        # ill-conditioned Gauss-Newton Hessians.  Heavy LM + extra QP
        # iterations are needed.  Use SQP_RTI (1 NLP iter) since the
        # first QP typically solves but subsequent SQP iterations hit
        # MINSTEP due to the NN nonlinearity through history states.
        if self._temporal_rolling:
            self._lm = max(self._lm, 5e-2)
            self._qp_iter_max = 100
            self._nlp_max_iter = 1  # SQP_RTI: one QP per solve
        # ResNet temporal models have deeper expression trees → more
        # regularisation and extra SQP iterations to help convergence.
        elif self._temporal_mode and nn_tire_model is not None:
            if 'resnet' in nn_tire_model.model_type:
                self._lm = 1e-2
                self._nlp_max_iter = 3
            else:
                # Temporal MLP: deeper input stacking than static → slightly more SQP work.
                self._lm = max(self._lm, 5e-3)
                self._nlp_max_iter = max(self._nlp_max_iter, 2)
        # Analytical tire models (Pacejka/TMeasy) have steep nonlinearities.
        # With 3 RK4 substeps the chain-rule sensitivities amplify conditioning
        # issues → higher LM regularisation needed to avoid MINSTEP.
        if not self.use_nn:
            self._lm = 5e-2
            self._nlp_max_iter = 3

        if self.use_nn:
            model_tag = nn_tire_model.model_type
        else:
            model_tag = tire_model  # pacejka / pacejka-oracle / tmeasy
        self._build_dir_user_provided = build_dir is not None
        if build_dir is None:
            build_dir = Path(f'/tmp/acados_mpc_{model_tag}')
        self._build_dir = Path(build_dir)

        # Build the ACADOS OCP solver unless we are used as an OCP factory.
        self._solver = None
        if build_solver:
            self._build_ocp()
        else:
            self._compute_param_layout()

        # Warm-start state
        self._prev_U = None
        self._prev_Z = None
        self._last_u0 = np.zeros(self.nu)
        self.last_iter_count = 0
        self.last_solver_status = ''

    @property
    def temporal_mode(self) -> bool:
        """True if NN uses stacked temporal history (K>1)."""
        return self._temporal_mode

    @property
    def rate_mode(self) -> bool:
        """True if NN uses rate-augmented inputs."""
        return self._rate_mode

    @property
    def gru_mode(self) -> bool:
        """True if NN uses GRU latent-state observer."""
        return self._gru_mode

    # ------------------------------------------------------------------
    # OCP construction
    # ------------------------------------------------------------------

    def _compute_param_layout(self):
        """Compute parameter dimension and offsets (needed before OCP build and at solve time)."""
        n_terrain = 6  # Kphi, Kc, n, c, phi, k
        n_ref_per_stage = 4  # x_ref_k, y_ref_k, psi_ref_k, v_ref_k
        n_extra = 1  # sr_meas (frozen)
        n_kappa_ref = 2  # front / rear runtime peak-slip references for traction query
        n_hist = 0
        if self._temporal_mode and not self._temporal_rolling:
            K = self.nn_tire_model.temporal_K
            n_hist = 2 * (K - 1) * 5  # front + rear
        n_rate = 0
        if self._rate_mode and not self._symbolic_rate_mode:
            n_rate = 6  # rates_front(3) + rates_rear(3)
        n_gru = 0
        if self._gru_mode:
            n_gru = 2 * self.nn_tire_model.gru_h_dim  # h_front + h_rear

        n_force_resid = 2  # dFy_f_resid, dFy_r_resid (per-stage force corrections)
        n_dynamics_resid = 3  # du_dot_resid, dv_dot_resid, domega_dot_resid
        n_obs_params = self._n_obs * 3   # [x_j, y_j, r_j] for each of N_OBS obstacles
        self._np_per_stage = n_terrain + n_ref_per_stage + n_extra + n_kappa_ref + n_hist + n_rate + n_gru + n_force_resid + n_dynamics_resid + n_obs_params
        self._tp_off = 0
        self._ref_off = n_terrain
        self._sr_off = n_terrain + n_ref_per_stage
        self._kappa_ref_front_off = n_terrain + n_ref_per_stage + n_extra
        self._kappa_ref_rear_off = self._kappa_ref_front_off + 1
        self._hist_off = n_terrain + n_ref_per_stage + n_extra + n_kappa_ref
        self._rate_off = self._hist_off + n_hist
        self._gru_off = self._rate_off + n_rate
        self._force_resid_off = self._gru_off + n_gru  # force residuals (2 slots)
        self._dynamics_resid_off = self._force_resid_off + n_force_resid  # dynamics residuals (3 slots)
        self._obs_off = self._dynamics_resid_off + n_dynamics_resid  # obstacle params (N_OBS * 3)

    def build_nominal_ocp(self, code_export_directory=None) -> AcadosOcp:
        """Build and return an AcadosOcp object matching this MPC configuration.

        This does not compile the solver; it is intended for workflows that
        need direct access to the nominal OCP (e.g. l4acados residual wrappers).
        """
        nx, nu, N, dt = self.nx, self.nu, self.N, self.dt
        self._compute_param_layout()

        ocp = AcadosOcp()

        # ---- Model (uses _np_per_stage and offsets) ----
        model = self._build_acados_model()
        ocp.model = model

        # ---- Dimensions ----
        ocp.dims.N = N
        ocp.dims.np = self._np_per_stage
        ocp.parameter_values = np.zeros(self._np_per_stage)

        # ---- Cost ----
        ocp.cost.cost_type = 'EXTERNAL'
        ocp.cost.cost_type_e = 'EXTERNAL'

        # ---- Constraints: bounds ----
        # In force-balance mode state idx6 is the slip ratio κ (not ax) and the
        # control / idx8 are κ̇ (not jerk), so retune those box bounds.
        _x6_lo, _x6_hi = ((-self._kappa_fb_max, self._kappa_fb_max)
                          if self._force_balance else (self.ax_min, self.ax_max))
        _u1_lo, _u1_hi = ((-self._kappa_dot_max, self._kappa_dot_max)
                          if self._force_balance else (self.Jx_min, self.Jx_max))
        ocp.constraints.lbx = np.array([self.u_min, -10.0, -5.0, _x6_lo,
                                        self.delta_min, _u1_lo])
        ocp.constraints.ubx = np.array([self.u_max, 10.0, 5.0, _x6_hi,
                                        self.delta_max, _u1_hi])
        ocp.constraints.idxbx = np.array([3, 4, 5, 6, 7, 8])

        if self._symbolic_rate_mode:
            # Append bounds for α_f_prev (9), α_r_prev (10), δ_sr_prev (11)
            ocp.constraints.lbx = np.append(ocp.constraints.lbx, [-0.8, -0.8, self.delta_min])
            ocp.constraints.ubx = np.append(ocp.constraints.ubx, [0.8, 0.8, self.delta_max])
            ocp.constraints.idxbx = np.append(ocp.constraints.idxbx, [9, 10, 11])
        elif self._symbolic_sr:
            # Append bounds for δ_sr_prev at _sr_state_idx
            ocp.constraints.lbx = np.append(ocp.constraints.lbx, [self.delta_min])
            ocp.constraints.ubx = np.append(ocp.constraints.ubx, [self.delta_max])
            ocp.constraints.idxbx = np.append(ocp.constraints.idxbx, [self._sr_state_idx])

        if self._temporal_rolling:
            # No box bounds on history states.  They are determined by the
            # shift-register dynamics (exact copy of current operating point)
            # which is already bounded through base state constraints.
            # Removing bounds avoids infeasibility at startup (when history
            # may contain zeros) and reduces QP complexity.
            pass

        ocp.constraints.lbx_e = ocp.constraints.lbx.copy()
        ocp.constraints.ubx_e = ocp.constraints.ubx.copy()
        ocp.constraints.idxbx_e = ocp.constraints.idxbx.copy()

        ocp.constraints.lbu = np.array([self.delta_min, _u1_lo])
        ocp.constraints.ubu = np.array([self.delta_max, _u1_hi])
        ocp.constraints.idxbu = np.array([0, 1])

        # ---- Polytopic constraints: steering rate & jounce ----
        nc_lin = 2
        C_poly = np.zeros((nc_lin, nx))
        D_poly = np.zeros((nc_lin, nu))
        C_poly[0, 7] = -1.0
        D_poly[0, 0] = 1.0
        C_poly[1, 8] = -1.0
        D_poly[1, 1] = 1.0
        ocp.constraints.C = C_poly
        ocp.constraints.D = D_poly
        ocp.constraints.lg = np.array([
            self.delta_dot_min * dt,
            -self.jounce_max * dt,
        ])
        ocp.constraints.ug = np.array([
            self.delta_dot_max * dt,
            self.jounce_max * dt,
        ])

        ocp.constraints.x0 = np.zeros(nx)

        # ---- Nonlinear path constraints ----
        if hasattr(model, 'con_h_expr') and hasattr(model.con_h_expr, 'shape'):
            nh = model.con_h_expr.shape[0]
            ocp.constraints.lh = -1e9 * np.ones(nh)
            ocp.constraints.uh = np.zeros(nh)



        # ---- Solver options ----
        # Use standard HPIPM for all modes.  Full condensing and qpOASES
        # don't improve temporal rolling convergence.
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI' if self._nlp_max_iter == 1 else 'SQP'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        if self.use_nn:
            ocp.solver_options.sim_method_num_stages = 1
            ocp.solver_options.sim_method_num_steps = 1
        else:
            ocp.solver_options.sim_method_num_stages = 4
            ocp.solver_options.sim_method_num_steps = 3
        ocp.solver_options.nlp_solver_max_iter = self._nlp_max_iter
        ocp.solver_options.qp_solver_iter_max = self._qp_iter_max
        ocp.solver_options.tf = N * dt
        ocp.solver_options.levenberg_marquardt = self._lm
        ocp.solver_options.qp_solver_warm_start = 2
        ocp.solver_options.print_level = 0

        if self.use_nn and self.nn_tire_model.n_params > 5000:
            ocp.solver_options.ext_fun_compile_flags = '-O0'

        if code_export_directory is not None:
            ocp.code_export_directory = str(code_export_directory)

        return ocp

    def _build_ocp(self):
        """Construct the full ACADOS OCP and compile the solver."""
        nx, nu, N, dt = self.nx, self.nu, self.N, self.dt

        # Compute param layout (needed for fingerprint and solve)
        self._compute_param_layout()

        # --- Cache: skip full OCP build if compiled solver matches ---
        fingerprint = self._compute_fingerprint()
        if not getattr(self, '_build_dir_user_provided', False):
            # The fingerprint includes terrain/friction and solver options.
            # Keying the default cache only by model tag lets parallel
            # mixed-terrain runs delete/rebuild the same directory while other
            # processes are inside it.  Add a short fingerprint hash to make
            # concurrent cache misses independent.
            model_tag = self.nn_tire_model.model_type if self.use_nn else self.tire_model
            fp_hash = hashlib.sha1(fingerprint.encode('utf-8')).hexdigest()[:10]
            # Keyed by (model, fingerprint) and SHARED across processes — the
            # solver is terrain-independent (terrain enters as a runtime
            # parameter, not the fingerprint), so every run with the same model
            # + solver config reuses one compiled .so.  We deliberately do NOT
            # append os.getpid(): the controller is a fresh subprocess per run,
            # so a PID-keyed dir forced a ~60 s cold recompile on every one of
            # the thousands of sweep runs.  Concurrent cold builds are
            # serialized by the file lock below.
            self._build_dir = Path(f'/tmp/acados_mpc_{model_tag}_{fp_hash}')
        fp_file = self._build_dir / '.fingerprint'
        so_file = self._build_dir / 'c_generated_code' / 'libacados_ocp_solver_bicycle.so'
        json_file = self._build_dir / 'acados_ocp.json'

        def _load_if_cached() -> bool:
            if (fp_file.exists() and so_file.exists() and json_file.exists()
                    and fp_file.read_text().strip() == fingerprint):
                print(f"Reusing cached ACADOS solver in {self._build_dir}")
                old_cwd = os.getcwd()
                os.chdir(str(self._build_dir))
                try:
                    self._solver = AcadosOcpSolver(
                        None, json_file=str(json_file),
                        build=False, generate=False)
                finally:
                    os.chdir(old_cwd)
                _tire_tag = self.nn_tire_model.model_type if self.use_nn else self.tire_model
                print(f"✓ ACADOS solver built: {nx} states, {nu} controls, N={N}, "
                      f"dt={dt}, tire={_tire_tag}")
                return True
            return False

        if _load_if_cached():
            return

        # --- Cache miss: serialize codegen across concurrent workers ---
        # The build dir is shared per (model, fingerprint).  In a multi-model
        # sweep several workers can cache-miss the same dir at once; without a
        # lock they would clobber each other's generated C / .so.  The first to
        # grab the lock builds; the rest block, then find the now-valid cache on
        # re-check and reuse it.  (Held until the build finishes; the OS frees
        # the fd if the process dies mid-build.)
        import fcntl
        _lock_path = Path(f"{self._build_dir}.lock")
        _lock_path.parent.mkdir(parents=True, exist_ok=True)
        _build_lock_f = open(_lock_path, "w")
        fcntl.flock(_build_lock_f, fcntl.LOCK_EX)
        if _load_if_cached():
            fcntl.flock(_build_lock_f, fcntl.LOCK_UN)
            _build_lock_f.close()
            return

        # --- Cache miss: build full OCP ---
        # Large NN models (>5k params) generate huge CasADi C files that
        # can trip gcc at -O2.  Use two-phase build at -O0 for those.
        _NN_PARAM_THRESHOLD = 5000
        nn_params = self.nn_tire_model.n_params if self.use_nn else 0
        use_O0_workaround = nn_params > _NN_PARAM_THRESHOLD

        ocp = self.build_nominal_ocp(
            code_export_directory=str(self._build_dir / 'c_generated_code')
        )

        # Generate + compile from scratch
        json_file = self._build_dir / 'acados_ocp.json'
        print(f"Building ACADOS solver in {self._build_dir} ...")
        if self._build_dir.exists():
            shutil.rmtree(self._build_dir)
        self._build_dir.mkdir(parents=True, exist_ok=True)

        old_cwd = os.getcwd()
        os.chdir(str(self._build_dir))
        try:
            if use_O0_workaround:
                # Two-phase build: generate code, patch Makefile to -O0,
                # compile with system gcc.  Conda's cc wrapper injects
                # hard-coded -O2 that causes ICE on large C files.
                import subprocess
                self._solver = AcadosOcpSolver(
                    ocp, json_file=str(json_file),
                    generate=True, build=False)
                makefile = self._build_dir / 'c_generated_code' / 'Makefile'
                if makefile.exists():
                    txt = makefile.read_text()
                    makefile.write_text(txt.replace('-O2', '-O0'))
                cgen = self._build_dir / 'c_generated_code'
                env = os.environ.copy()
                env.pop('CFLAGS', None)
                env.pop('CXXFLAGS', None)
                subprocess.check_call(
                    ['make', 'CC=/usr/bin/gcc',
                     'CFLAGS=-fPIC -std=c99 -O0', 'shared_lib'],
                    cwd=str(cgen), env=env)
                self._solver = AcadosOcpSolver(
                    None, json_file=str(json_file),
                    build=False, generate=False)
                print(f"  (used -O0 workaround for large model: {nn_params} params)")
            else:
                # Normal compilation — works for small/medium models and
                # analytical tire models. Compiler uses default -O2.
                self._solver = AcadosOcpSolver(
                    ocp, json_file=str(json_file))
        finally:
            os.chdir(old_cwd)

        # Write fingerprint for next time
        fp_file = self._build_dir / '.fingerprint'
        fp_file.write_text(fingerprint)

        _tire_tag = self.nn_tire_model.model_type if self.use_nn else self.tire_model
        print(f"✓ ACADOS solver built: {nx} states, {nu} controls, N={N}, "
              f"dt={dt}, tire={_tire_tag}")

        # Release the build lock so workers blocked on this dir can proceed.
        try:
            import fcntl
            fcntl.flock(_build_lock_f, fcntl.LOCK_UN)
            _build_lock_f.close()
        except Exception:
            pass

    def reset_warmstart(self):
        """Clear cached trajectory so next solve uses kinematic rollout."""
        self._prev_Z = None
        self._prev_U = None
        self._last_u0 = np.zeros(self.nu)

    def compile_solver(self):
        """Build ACADOS solver if this instance was created with build_solver=False."""
        if self._solver is None:
            self._build_ocp()

    def _compute_fingerprint(self) -> str:
        """Hash of NN weights + solver config so we can cache compiled code."""
        h = hashlib.sha256()
        # Solver config
        cfg = json.dumps({
            'N': self.N, 'dt': self.dt, 'nx': self.nx, 'nu': self.nu,
            'lat_transfer': self.lateral_load_transfer,
            'kappa_mode': self.kappa_mode,
            'force_balance': self._force_balance,
            'np': self._np_per_stage,
            'tire_model': self.tire_model,
            'qp_solver': 'PARTIAL_CONDENSING_HPIPM',
            'levenberg_marquardt': self._lm,
            'qp_solver_iter_max': self._qp_iter_max,
            'nlp_solver_max_iter': self._nlp_max_iter,
            'print_level': 0,
            'w_delta_dot': self.w_delta_dot,
            'w_Jx': self.w_Jx,
            'w_ax': self.w_ax,
            'steer_angle_mpc': True,
            'w_x': self.w_x,
            'w_lateral': self.w_lateral,
            'w_heading': self.w_heading,
            'w_speed': self.w_speed,
            'speed_cost_mode': self.speed_cost_mode,
            'w_du': self.w_du,
            'w_steer': self.w_steer,
            'w_terminal': self.w_terminal,
            'jounce_max': self.jounce_max,
            'ay_max': self.ay_max,
            'use_nn': self.use_nn,
            'symbolic_rate_mode': self._symbolic_rate_mode,
            'symbolic_sr': self._symbolic_sr,
            'tau_alpha': self._tau_alpha if self._symbolic_rate_mode else 0,
            'tau_sr': self._tau_sr if (self._symbolic_rate_mode or self._symbolic_sr) else 0,
            'rate_feature_dt': self._rate_feature_dt,
            'temporal_rolling': self._temporal_rolling,
            'temporal_staged': self._temporal_staged,
            'hist_feat_scale': list(self._hist_feat_scale) if self._temporal_rolling else [],
            'runtime_kappa_ref': True,
            'rear_fy_scale': self.rear_fy_scale,
            'rear_alpha_scale': self.rear_alpha_scale,
            'ext_fun_compile_flags': '-O0' if (self.use_nn and self.nn_tire_model.n_params > 5000) else '',
            'oracle_pacejka_params': self._oracle_pacejka_params,
            'n_obs': self._n_obs,
            'w_obstacle': self.w_obstacle,
        }, sort_keys=True)
        h.update(cfg.encode())
        # NN weights
        nn = self.nn_tire_model
        if nn is not None:
            h.update(nn.model_type.encode())
            for name in sorted(nn._weights.keys()):
                h.update(name.encode())
                h.update(nn._weights[name].tobytes())
            h.update(nn._X_mean.tobytes())
            h.update(nn._X_scale.tobytes())
        return h.hexdigest()

    def _estimate_peak_slip_runtime(self, Fz: float, u: float, terrain_params: dict,
                                    hist=None, rates=None, sr_meas: float = 0.0,
                                    gru_h=None) -> float:
        """Estimate the positive slip ratio that maximizes predicted traction.

        This is evaluated numerically outside the CasADi graph so we can adapt
        the traction-reference query online without embedding a nested
        optimization inside ACADOS.
        """
        nn = self.nn_tire_model
        if nn is None:
            return 0.15

        u_eval = float(max(abs(u), 0.5))
        Fz_eval = float(max(Fz, 100.0))
        n_terrain_val = terrain_params.get('n', nn.n_nominal)

        kappa_grid = np.linspace(0.02, 0.3, 15)
        best_kappa = 0.15
        best_fx = -np.inf

        for kappa in kappa_grid:
            Fx, _ = nn.predict_numeric(
                alpha=0.0,
                Fz=Fz_eval,
                u=u_eval,
                kappa=float(kappa),
                n_terrain=n_terrain_val,
                steering_rate=sr_meas,
                terrain_params=terrain_params,
                hist=hist,
                rates=rates,
                **(dict(gru_h=gru_h) if gru_h is not None else {}),
            )
            if np.isfinite(Fx) and Fx > best_fx:
                best_fx = Fx
                best_kappa = float(kappa)

        return float(np.clip(best_kappa, 0.02, 0.3))

    def _compute_runtime_kappa_refs(self, z0, terrain_params, sr_meas=0.0,
                                    hist_front=None, hist_rear=None,
                                    rates_front=None, rates_rear=None,
                                    gru_h_front=None, gru_h_rear=None):
        """Compute front/rear peak-slip references from the current operating point."""
        if not self.use_nn:
            return 0.15, 0.15

        u_vel = float(z0[3])
        ax = float(np.clip(z0[6], self.ax_min, self.ax_max))
        M, Lf, Lr = _M, _Lf, _Lr
        h_cg = _h_cg

        Fz_f_axle = (M * 9.81 * Lr - M * ax * h_cg) / (Lf + Lr)
        Fz_r_axle = (M * 9.81 * Lf + M * ax * h_cg) / (Lf + Lr)
        Fz_f_mean = Fz_f_axle / 2.0
        Fz_r_mean = Fz_r_axle / 2.0

        kappa_ref_front = self._estimate_peak_slip_runtime(
            Fz=Fz_f_mean,
            u=u_vel,
            terrain_params=terrain_params,
            hist=hist_front,
            rates=rates_front,
            sr_meas=sr_meas,
            gru_h=gru_h_front,
        )
        kappa_ref_rear = self._estimate_peak_slip_runtime(
            Fz=Fz_r_mean,
            u=u_vel,
            terrain_params=terrain_params,
            hist=hist_rear,
            rates=rates_rear,
            sr_meas=sr_meas,
            gru_h=gru_h_rear,
        )
        return kappa_ref_front, kappa_ref_rear

    def _build_acados_model(self) -> AcadosModel:
        """Define the CasADi symbolic model for ACADOS."""
        model = AcadosModel()
        model.name = 'bicycle'

        nx, nu = self.nx, self.nu
        M, Izz, Lf, Lr = _M, _Izz, _Lf, _Lr
        h_cg, T = _h_cg, _T

        # ---- Symbolic variables ----
        x = ca.SX.sym('x', nx)  # [x, y, ψ, u, v, ω, ax, δ_prev, Jx_prev, (αf_prev, αr_prev, δ_sr_prev)]
        u_ctrl = ca.SX.sym('u', nu)  # [δ, Jx]
        xdot = ca.SX.sym('xdot', nx)

        # Parameters per stage
        p = ca.SX.sym('p', self._np_per_stage)

        # Unpack parameters
        Kphi_sym = p[self._tp_off + 0]
        Kc_sym   = p[self._tp_off + 1]
        n_terrain_val = p[self._tp_off + 2]
        c_sym    = p[self._tp_off + 3]
        phi_sym  = p[self._tp_off + 4]
        k_sym    = p[self._tp_off + 5]
        x_ref    = p[self._ref_off + 0]
        y_ref    = p[self._ref_off + 1]
        psi_ref  = p[self._ref_off + 2]
        v_ref    = p[self._ref_off + 3]
        sr_meas  = p[self._sr_off]
        kappa_ref_front = p[self._kappa_ref_front_off]
        kappa_ref_rear = p[self._kappa_ref_rear_off]
        dFy_f_resid = p[self._force_resid_off + 0]
        dFy_r_resid = p[self._force_resid_off + 1]
        du_dot_resid = p[self._dynamics_resid_off + 0]
        dv_dot_resid = p[self._dynamics_resid_off + 1]
        domega_dot_resid = p[self._dynamics_resid_off + 2]

        # Unpack state
        px, py, psi = x[0], x[1], x[2]
        u_vel, v_vel, omega = x[3], x[4], x[5]
        ax = x[6]
        delta_prev = x[7]
        Jx_prev = x[8]

        # Extra states for symbolic rate propagation
        if self._symbolic_rate_mode:
            alpha_f_prev = x[9]
            alpha_r_prev = x[10]
            delta_sr_prev = x[11]
        elif self._symbolic_sr:
            delta_sr_prev = x[self._sr_state_idx]

        # Unpack controls (commanded road-wheel angle + longitudinal jerk)
        delta_cmd = u_ctrl[0]
        Jx = u_ctrl[1]

        # ---- Tire force computation ----
        u_safe = ca.fmax(ca.fabs(u_vel), 0.5)
        alpha_f = delta_cmd - ca.atan2(v_vel + Lf * omega, u_safe)
        alpha_r = -ca.atan2(v_vel - Lr * omega, u_safe)

        # Rear alpha correction for suspension compliance steer
        if self.rear_alpha_scale != 1.0:
            alpha_r = alpha_r * self.rear_alpha_scale

        # Clamp slip angles to training-data range to prevent NN
        # extrapolation; beyond saturation, more alpha does not produce
        # more force, so clamping also gives the solver correct gradients.
        _alpha_max = 0.55
        alpha_f = ca.fmax(ca.fmin(alpha_f, _alpha_max), -_alpha_max)
        alpha_r = ca.fmax(ca.fmin(alpha_r, _alpha_max), -_alpha_max)

        # Normal forces with longitudinal load transfer
        Fz_f_axle = (M * 9.81 * Lr - M * ax * h_cg) / (Lf + Lr)
        Fz_r_axle = (M * 9.81 * Lf + M * ax * h_cg) / (Lf + Lr)
        Fz_f_mean = Fz_f_axle / 2.0
        Fz_r_mean = Fz_r_axle / 2.0

        # Lateral load transfer
        ay = u_vel * omega
        dFz_f = M * ay * h_cg / T / 2.0
        dFz_r = M * ay * h_cg / T / 2.0
        Fz_f_outer = ca.fmin(Fz_f_mean + dFz_f, Fz_f_mean * 1.9)
        Fz_f_inner = ca.fmax(Fz_f_mean - dFz_f, Fz_f_mean * 0.1)
        Fz_r_outer = ca.fmin(Fz_r_mean + dFz_r, Fz_r_mean * 1.9)
        Fz_r_inner = ca.fmax(Fz_r_mean - dFz_r, Fz_r_mean * 0.1)

        # Slip ratio — terrain-adaptive using friction angle φ
        mu_terrain = ca.fmax(ca.tan(phi_sym), 0.1)  # phi_sym in radians; always defined
        if self._force_balance:
            # Force-balance mode: state x[6] (aliased here as `ax`) IS the
            # longitudinal slip ratio κ, a genuine control variable (κ̇ = u[1]),
            # not the ax/(μg) proxy. The surrogate's Fx(κ) then drives u̇ directly.
            kappa = ca.fmax(ca.fmin(ax, self._kappa_fb_max), -self._kappa_fb_max)
        elif self.kappa_mode == 'approx':
            kappa = ca.fmax(ca.fmin(ax / (mu_terrain * 9.81), 0.8), -0.8)
        else:
            kappa = 0.0

        # Sinkage exponent is stage-parameterized (runtime terrain adaptation).
        nn = self.nn_tire_model

        # Symbolic steering rate: differentiable w.r.t. optimizer's δ.
        # Used by all NN models so that sr varies across the horizon and
        # the optimizer sees ∂Fy/∂δ through the steering-rate channel.
        if self._symbolic_rate_mode or self._symbolic_sr:
            sr_eff = (delta_cmd - delta_sr_prev) / self._rate_feature_dt
        else:
            sr_eff = sr_meas

        Fx_traction = ca.SX(0.0)
        # Fx_op: longitudinal wheel force at the *live* operating point
        # (current κ, α, Fz per wheel).  Currently unused in the dynamics
        # (the longitudinal channel is u̇ = ax + du_dot_resid; soil enters
        # via the Fx_traction budget constraint).  Kept assigned for
        # diagnostics / future learned-residual adapters.
        Fx_op = ca.SX(0.0)

        if nn is not None and self._temporal_mode:
            # --- Temporal batched ---
            K = nn.temporal_K
            hist_dim = (K - 1) * 5
            if self._temporal_rolling:
                # History lives in the state vector in SCALED space.
                # De-scale to raw before passing to NN.
                hf_off = 9
                hr_off = 9 + hist_dim
                sc = ca.DM(np.tile(self._hist_feat_scale, K - 1))
                hist_front = x[hf_off: hf_off + hist_dim] * sc
                hist_rear = x[hr_off: hr_off + hist_dim] * sc
            else:
                hist_front = p[self._hist_off: self._hist_off + hist_dim]
                hist_rear = p[self._hist_off + hist_dim: self._hist_off + 2 * hist_dim]
            t_vec = ca.vertcat(Kphi_sym, Kc_sym, n_terrain_val, c_sym, phi_sym, k_sym)

            if self.lateral_load_transfer:
                slot_ops = [
                    ca.vertcat(kappa, alpha_f, u_safe, Fz_f_outer, sr_eff),
                    ca.vertcat(kappa, alpha_f, u_safe, Fz_f_inner, sr_eff),
                    ca.vertcat(kappa, alpha_r, u_safe, Fz_r_outer, 0.0),
                    ca.vertcat(kappa, alpha_r, u_safe, Fz_r_inner, 0.0),
                    ca.vertcat(kappa_ref_front, 0.0, u_safe, Fz_f_mean, 0.0),
                    ca.vertcat(kappa_ref_rear, 0.0, u_safe, Fz_r_mean, 0.0),
                    ca.vertcat(kappa_ref_front, 0.0, u_safe, Fz_f_mean, 0.0),
                    ca.vertcat(kappa_ref_rear, 0.0, u_safe, Fz_r_mean, 0.0),
                ]
                slot_hist = [hist_front, hist_front, hist_rear, hist_rear,
                             hist_front, hist_rear, hist_front, hist_rear]
            else:
                slot_ops = [
                    ca.vertcat(kappa, alpha_f, u_safe, Fz_f_mean, sr_eff),
                    ca.vertcat(kappa, alpha_r, u_safe, Fz_r_mean, 0.0),
                    ca.vertcat(kappa_ref_front, 0.0, u_safe, Fz_f_mean, 0.0),
                    ca.vertcat(kappa_ref_rear, 0.0, u_safe, Fz_r_mean, 0.0),
                    ca.vertcat(0.0, 0.0, u_safe, Fz_f_mean, 0.0),
                    ca.vertcat(0.0, 0.0, u_safe, Fz_r_mean, 0.0),
                    ca.vertcat(0.0, 0.0, u_safe, Fz_f_mean, 0.0),
                    ca.vertcat(0.0, 0.0, u_safe, Fz_r_mean, 0.0),
                ]
                slot_hist = [hist_front, hist_rear, hist_front, hist_rear,
                             hist_front, hist_rear, hist_front, hist_rear]

            B = nn._BATCH
            cols = [ca.vertcat(slot_ops[i], slot_hist[i], t_vec) for i in range(B)]
            X_batch = ca.horzcat(*cols)
            Fxs_all, Fys_all = nn.predict_batch_temporal(X_batch)

            if self.lateral_load_transfer:
                Fyf = -self.nn_scale * (Fys_all[0] + Fys_all[1])
                Fyr = -self.nn_scale * (Fys_all[2] + Fys_all[3])
                Fx_traction = 2.0 * (Fxs_all[4] + Fxs_all[5])
                # Live operating-point Fx: 4 wheels at current (κ, α, Fz)
                Fx_op = (Fxs_all[0] + Fxs_all[1]
                         + Fxs_all[2] + Fxs_all[3])
            else:
                Fyf = -self.nn_scale * 2.0 * Fys_all[0]
                Fyr = -self.nn_scale * 2.0 * Fys_all[1]
                Fx_traction = 2.0 * (Fxs_all[2] + Fxs_all[3])
                # Live operating-point Fx: per-axle mean × 2 (L+R wheels)
                Fx_op = 2.0 * (Fxs_all[0] + Fxs_all[1])

        elif nn is not None and self._rate_mode:
            # --- Rate-augmented batched ---
            # Compute rate features: either symbolically from state/control
            # (Markovian) or from frozen parameters (legacy).
            if self._symbolic_rate_mode:
                # dκ: derivative of slip ratio approximation κ ≈ ax/(μg)
                #     dκ/dt = Jx / (μg)  where μ = tan(φ)
                if self._force_balance:
                    # κ is the state x[6]; its rate κ̇ is the control u[1] (Jx slot).
                    sym_dk = Jx
                elif self.kappa_mode == 'approx':
                    sym_dk = Jx / (mu_terrain * 9.81)
                else:
                    sym_dk = ca.SX(0.0)
                # dα: finite-difference channel scaled to the training
                # feature time base (separate from auxiliary-state dynamics).
                _dt_feat = self._rate_feature_dt
                sym_da_f = (alpha_f - alpha_f_prev) / _dt_feat
                sym_da_r = (alpha_r - alpha_r_prev) / _dt_feat
                # du: longitudinal-accel rate feature. In force-balance mode the
                # accel is u̇=Fx_op/M (circular with the surrogate), so use the
                # quasi-steady value 0; otherwise it is the ax state.
                sym_du = ca.SX(0.0) if self._force_balance else ax

                # Package per-axle: front uses sym_da_f, rear uses sym_da_r
                rates_f_dk = sym_dk
                rates_f_da = sym_da_f
                rates_f_du = sym_du
                rates_r_dk = sym_dk
                rates_r_da = sym_da_r
                rates_r_du = sym_du
            else:
                rates_front = p[self._rate_off: self._rate_off + 3]
                rates_rear = p[self._rate_off + 3: self._rate_off + 6]
                rates_f_dk = rates_front[0]
                rates_f_da = rates_front[1]
                rates_f_du = rates_front[2]
                rates_r_dk = rates_rear[0]
                rates_r_da = rates_rear[1]
                rates_r_du = rates_rear[2]

            # sr_eff is already computed above (symbolic for all NN models).

            B = nn._BATCH

            if self.lateral_load_transfer:
                a_vec = ca.vertcat(alpha_f, alpha_f, alpha_r, alpha_r, 0.0, 0.0, 0.0, 0.0)
                fz_vec = ca.vertcat(Fz_f_outer, Fz_f_inner, Fz_r_outer, Fz_r_inner,
                                    Fz_f_mean, Fz_r_mean, Fz_f_mean, Fz_r_mean)
                u_vec = ca.repmat(u_safe, B, 1)
                k_vec = ca.vertcat(kappa, kappa, kappa, kappa,
                                   kappa_ref_front, kappa_ref_rear, kappa_ref_front, kappa_ref_rear)
                n_vec = ca.repmat(n_terrain_val, B, 1)
                sr_vec = ca.vertcat(sr_eff, sr_eff, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                dk_vec = ca.vertcat(rates_f_dk, rates_f_dk, rates_r_dk, rates_r_dk,
                                    rates_f_dk, rates_r_dk, rates_f_dk, rates_r_dk)
                da_vec = ca.vertcat(rates_f_da, rates_f_da, rates_r_da, rates_r_da,
                                    0.0, 0.0, 0.0, 0.0)
                du_vec = ca.vertcat(rates_f_du, rates_f_du, rates_r_du, rates_r_du,
                                    rates_f_du, rates_r_du, rates_f_du, rates_r_du)

                if hasattr(nn, 'predict_batch_axle_rate'):
                    axle_vec = ca.vertcat(0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0, 1.0)
                    Fxs_all, Fys_all = nn.predict_batch_axle_rate(
                        axle_vec, a_vec, fz_vec, u_vec, k_vec, n_vec, sr_vec,
                        dk_vec, da_vec, du_vec, Kphi_sym, Kc_sym, c_sym, phi_sym, k_sym)
                else:
                    Fxs_all, Fys_all = nn.predict_batch_rate(
                        a_vec, fz_vec, u_vec, k_vec, n_vec, sr_vec,
                        dk_vec, da_vec, du_vec, Kphi_sym, Kc_sym, c_sym, phi_sym, k_sym)
                Fyf = -self.nn_scale * (Fys_all[0] + Fys_all[1])
                Fyr = -self.nn_scale * (Fys_all[2] + Fys_all[3])
                Fx_traction = 2.0 * (Fxs_all[4] + Fxs_all[5])
                Fx_op = (Fxs_all[0] + Fxs_all[1]
                         + Fxs_all[2] + Fxs_all[3])
            else:
                a_vec = ca.vertcat(alpha_f, alpha_r, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                fz_vec = ca.vertcat(Fz_f_mean, Fz_r_mean, Fz_f_mean, Fz_r_mean,
                                    Fz_f_mean, Fz_r_mean, Fz_f_mean, Fz_r_mean)
                u_vec = ca.repmat(u_safe, B, 1)
                k_vec = ca.vertcat(kappa, kappa, kappa_ref_front, kappa_ref_rear, 0.0, 0.0, 0.0, 0.0)
                n_vec = ca.repmat(n_terrain_val, B, 1)
                sr_vec = ca.vertcat(sr_eff, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                dk_vec = ca.vertcat(rates_f_dk, rates_r_dk, rates_f_dk, rates_r_dk,
                                    0.0, 0.0, 0.0, 0.0)
                da_vec = ca.vertcat(rates_f_da, rates_r_da, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                du_vec = ca.vertcat(rates_f_du, rates_r_du, rates_f_du, rates_r_du,
                                    0.0, 0.0, 0.0, 0.0)

                if hasattr(nn, 'predict_batch_axle_rate'):
                    axle_vec = ca.vertcat(0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0)
                    Fxs_all, Fys_all = nn.predict_batch_axle_rate(
                        axle_vec, a_vec, fz_vec, u_vec, k_vec, n_vec, sr_vec,
                        dk_vec, da_vec, du_vec, Kphi_sym, Kc_sym, c_sym, phi_sym, k_sym)
                else:
                    Fxs_all, Fys_all = nn.predict_batch_rate(
                        a_vec, fz_vec, u_vec, k_vec, n_vec, sr_vec,
                        dk_vec, da_vec, du_vec, Kphi_sym, Kc_sym, c_sym, phi_sym, k_sym)
                Fyf = -self.nn_scale * 2.0 * Fys_all[0]
                Fyr = -self.nn_scale * 2.0 * Fys_all[1]
                Fx_traction = 2.0 * (Fxs_all[2] + Fxs_all[3])
                Fx_op = 2.0 * (Fxs_all[0] + Fxs_all[1])

        elif nn is not None and self._gru_mode:
            # --- GRU observer batched ---
            h_dim = nn.gru_h_dim
            h_front = p[self._gru_off: self._gru_off + h_dim]
            h_rear = p[self._gru_off + h_dim: self._gru_off + 2 * h_dim]
            B = nn._BATCH

            if self.lateral_load_transfer:
                a_vec = ca.vertcat(alpha_f, alpha_f, alpha_r, alpha_r, 0.0, 0.0, 0.0, 0.0)
                fz_vec = ca.vertcat(Fz_f_outer, Fz_f_inner, Fz_r_outer, Fz_r_inner,
                                    Fz_f_mean, Fz_r_mean, Fz_f_mean, Fz_r_mean)
                u_vec = ca.repmat(u_safe, B, 1)
                k_vec = ca.vertcat(kappa, kappa, kappa, kappa,
                                   kappa_ref_front, kappa_ref_rear, kappa_ref_front, kappa_ref_rear)
                n_vec = ca.repmat(n_terrain_val, B, 1)
                sr_vec = ca.vertcat(sr_eff, sr_eff, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                h_mat = ca.horzcat(h_front, h_front, h_rear, h_rear,
                                   h_front, h_rear, h_front, h_rear)

                Fxs_all, Fys_all = nn.predict_batch_gru(
                    a_vec, fz_vec, u_vec, k_vec, n_vec, sr_vec,
                    h_mat, Kphi_sym, Kc_sym, c_sym, phi_sym, k_sym)
                Fyf = -self.nn_scale * (Fys_all[0] + Fys_all[1])
                Fyr = -self.nn_scale * (Fys_all[2] + Fys_all[3])
                Fx_traction = 2.0 * (Fxs_all[4] + Fxs_all[5])
                Fx_op = (Fxs_all[0] + Fxs_all[1]
                         + Fxs_all[2] + Fxs_all[3])
            else:
                a_vec = ca.vertcat(alpha_f, alpha_r, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                fz_vec = ca.vertcat(Fz_f_mean, Fz_r_mean, Fz_f_mean, Fz_r_mean,
                                    Fz_f_mean, Fz_r_mean, Fz_f_mean, Fz_r_mean)
                u_vec = ca.repmat(u_safe, B, 1)
                k_vec = ca.vertcat(kappa, kappa, kappa_ref_front, kappa_ref_rear, 0.0, 0.0, 0.0, 0.0)
                n_vec = ca.repmat(n_terrain_val, B, 1)
                sr_vec = ca.vertcat(sr_eff, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                h_mat = ca.horzcat(h_front, h_rear, h_front, h_rear,
                                   h_front, h_rear, h_front, h_rear)

                Fxs_all, Fys_all = nn.predict_batch_gru(
                    a_vec, fz_vec, u_vec, k_vec, n_vec, sr_vec,
                    h_mat, Kphi_sym, Kc_sym, c_sym, phi_sym, k_sym)
                Fyf = -self.nn_scale * 2.0 * Fys_all[0]
                Fyr = -self.nn_scale * 2.0 * Fys_all[1]
                Fx_traction = 2.0 * (Fxs_all[2] + Fxs_all[3])
                Fx_op = 2.0 * (Fxs_all[0] + Fxs_all[1])

        elif nn is not None:
            # --- Static batched (MLP or ResNet) ---
            B = nn._BATCH

            if self.lateral_load_transfer:
                a_vec = ca.vertcat(alpha_f, alpha_f, alpha_r, alpha_r, 0.0, 0.0, 0.0, 0.0)
                fz_vec = ca.vertcat(Fz_f_outer, Fz_f_inner, Fz_r_outer, Fz_r_inner,
                                    Fz_f_mean, Fz_r_mean, Fz_f_mean, Fz_r_mean)
                u_vec = ca.repmat(u_safe, B, 1)
                k_vec = ca.vertcat(kappa, kappa, kappa, kappa,
                                   kappa_ref_front, kappa_ref_rear, kappa_ref_front, kappa_ref_rear)
                n_vec = ca.repmat(n_terrain_val, B, 1)
                sr_vec = ca.vertcat(sr_eff, sr_eff, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

                Fxs_all, Fys_all = nn.predict_batch(
                    a_vec, fz_vec, u_vec, k_vec, n_vec, sr_vec,
                    Kphi_sym, Kc_sym, c_sym, phi_sym, k_sym)

                Fyf = -self.nn_scale * (Fys_all[0] + Fys_all[1])
                Fyr = -self.nn_scale * self.rear_fy_scale * (Fys_all[2] + Fys_all[3])
                Fx_traction = 2.0 * (Fxs_all[4] + Fxs_all[5])
                Fx_op = (Fxs_all[0] + Fxs_all[1]
                         + Fxs_all[2] + Fxs_all[3])
            else:
                a_vec = ca.vertcat(alpha_f, alpha_r, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                fz_vec = ca.vertcat(Fz_f_mean, Fz_r_mean, Fz_f_mean, Fz_r_mean,
                                    Fz_f_mean, Fz_r_mean, Fz_f_mean, Fz_r_mean)
                u_vec = ca.repmat(u_safe, B, 1)
                k_vec = ca.vertcat(kappa, kappa, kappa_ref_front, kappa_ref_rear, 0.0, 0.0, 0.0, 0.0)
                n_vec = ca.repmat(n_terrain_val, B, 1)
                sr_vec = ca.vertcat(sr_eff, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

                Fxs_all, Fys_all = nn.predict_batch(
                    a_vec, fz_vec, u_vec, k_vec, n_vec, sr_vec,
                    Kphi_sym, Kc_sym, c_sym, phi_sym, k_sym)

                Fyf = -self.nn_scale * 2.0 * Fys_all[0]
                Fyr = -self.nn_scale * self.rear_fy_scale * 2.0 * Fys_all[1]
                Fx_traction = 2.0 * (Fxs_all[2] + Fxs_all[3])
                Fx_op = 2.0 * (Fxs_all[0] + Fxs_all[1])
        else:
            # Analytical tire model (pacejka / pacejka-oracle / tmeasy)
            if self.tire_model == 'pacejka-oracle':
                # Oracle: terrain-fitted params embedded as constants — fair
                # comparison upper bound against the NN surrogate.
                Fyf, Fyr, Fx_traction = _analytical_tire_forces(
                    'pacejka', alpha_f, alpha_r, Fz_f_axle, Fz_r_axle, kappa,
                    **self._oracle_pacejka_params)
            else:
                Fyf, Fyr, Fx_traction = _analytical_tire_forces(
                    self.tire_model, alpha_f, alpha_r, Fz_f_axle, Fz_r_axle, kappa)
            # Analytical Fx is already evaluated at the live (κ, α, Fz),
            # so the operating-point Fx is identical to Fx_traction.  Note
            # that for terrain-agnostic models (Pacejka, TMeasy) this does
            # NOT model SCM sinkage / rolling drag — that mismatch is
            # exactly what 'tire_fx' coupling exposes for those baselines.
            Fx_op = Fx_traction

        # ---- Dynamics: ẋ = f(x, u) ----
        # δ_prev and Jx_prev follow (u - u_prev)/dt so the stage cost can
        # penalise Δδ and ΔJx consistently across the horizon.
        #
        # Force residuals dFy_f_resid/dFy_r_resid are per-stage parameters
        # that add learned corrections to the NN (or analytical) tire forces.
        # When no force residual adapter is active they are set to zero.
        _dt = self.dt

        # Speed-dependent force fade: bicycle-model slip angles become
        # unreliable below ~2 m/s (the training-data velocity floor),
        # so smoothly attenuate lateral forces to prevent spurious
        # predictions at low speed.  Ramps linearly 0 → 1 over [0.5, 2] m/s.
        # Applied to both NN and analytical models for consistent conditioning.
        _speed_fade = ca.fmin(1.0, ca.fmax(0.0, (ca.fabs(u_vel) - 0.5) / 1.5))
        Fyf_total = (Fyf + dFy_f_resid) * _speed_fade
        Fyr_total = (Fyr + dFy_r_resid) * _speed_fade
        # Dynamics residuals: additive corrections to state derivatives.
        # Fade at low speed (same as lateral forces) to avoid spurious
        # corrections when the model is poorly conditioned.
        _dyn_fade = _speed_fade

        # Longitudinal momentum equation: u̇ = ax + du_dot_resid.
        # ax is the commanded longitudinal acceleration state; throttle and
        # brake are mapped open-loop from ax in the controller node
        # (throttle = ax/ax_max, calibrated gain).  Soil-dependent peak
        # grip enters the longitudinal plan through the traction-budget
        # constraint |M·ax| ≤ Fx_traction(κ_ref, terrain_params) below, not
        # through a u̇ = Fx_op/M substitution -- empirical A/B benchmarks
        # showed the latter degrades both lateral and longitudinal tracking
        # because ax then plays two inconsistent roles (throttle surrogate
        # via the integrator AND slip-ratio surrogate via κ ≈ ax/(μg)).
        # du_dot_resid is preserved as an additive online residual that a
        # learned dynamics adapter (e.g. GP) can populate at runtime.
        if self._force_balance:
            # Principled longitudinal force balance: u̇ = ΣFx(κ, Fz, n̂)/M, with
            # Fx_op the surrogate's net longitudinal force at the live slip κ=x[6]
            # (drag, traction, and soil-dependence all from the one learned
            # curve). No kinematic u̇=ax, no du_dot_resid patch.
            u_dot_long = Fx_op / M + du_dot_resid * _dyn_fade
        else:
            u_dot_long = ax + du_dot_resid * _dyn_fade

        f_expl_base = ca.vertcat(
            u_vel * ca.cos(psi) - (v_vel + Lf * omega) * ca.sin(psi),  # ẋ
            u_vel * ca.sin(psi) + (v_vel + Lf * omega) * ca.cos(psi),  # ẏ
            omega,                                                       # ψ̇
            u_dot_long,                                                  # u̇
            (Fyf_total + Fyr_total) / M - u_vel * omega + dv_dot_resid * _dyn_fade,  # v̇
            (Fyf_total * Lf - Fyr_total * Lr) / Izz + domega_dot_resid * _dyn_fade,  # ω̇
            Jx,                                                          # ȧx
            (delta_cmd - delta_prev) / _dt,                              # δ_prev
            (Jx - Jx_prev) / _dt,                                        # Jx_prev
        )

        if self._symbolic_rate_mode:
            # α_prev / δ_sr_prev track current slip angle / steering via
            # first-order filters with time constants τ_alpha / τ_sr tied to
            # MPC dt for stable Euler integration. Rate-feature scaling uses
            # _rate_feature_dt separately.
            _tau_a = self._tau_alpha
            _tau_s = self._tau_sr
            f_expl = ca.vertcat(
                f_expl_base,
                (alpha_f - alpha_f_prev) / _tau_a,                       # α_f_prev
                (alpha_r - alpha_r_prev) / _tau_a,                       # α_r_prev
                (delta_cmd - delta_sr_prev) / _tau_s,                    # δ_sr_prev
            )
        elif self._temporal_rolling:
            # Shift-register dynamics for rolling temporal history.
            # History states are in SCALED space (divided by _hist_feat_scale).
            # With forward Euler (ERK, 1 stage, 1 step), (src - tgt)/dt
            # gives exact discrete shift: tgt_next = tgt + dt*(src-tgt)/dt = src.
            K = nn.temporal_K
            hist_dim = (K - 1) * 5
            hf_off = 9
            hr_off = 9 + hist_dim
            sc5 = ca.DM(self._hist_feat_scale)
            # Current operating point → scale to match history states
            cur_front_scaled = ca.vertcat(kappa, alpha_f, u_safe, Fz_f_mean, sr_eff) / sc5
            cur_rear_scaled = ca.vertcat(kappa, alpha_r, u_safe, Fz_r_mean, 0.0) / sc5
            f_hist_parts = []
            # Front axle: frame[0] ← current_scaled, frame[j] ← frame[j-1]
            for j in range(K - 1):
                src = cur_front_scaled if j == 0 else x[hf_off + (j - 1) * 5: hf_off + j * 5]
                tgt = x[hf_off + j * 5: hf_off + (j + 1) * 5]
                f_hist_parts.append((src - tgt) / _dt)
            # Rear axle: same shift pattern
            for j in range(K - 1):
                src = cur_rear_scaled if j == 0 else x[hr_off + (j - 1) * 5: hr_off + j * 5]
                tgt = x[hr_off + j * 5: hr_off + (j + 1) * 5]
                f_hist_parts.append((src - tgt) / _dt)
            f_expl = ca.vertcat(f_expl_base, *f_hist_parts)
        elif self._symbolic_sr:
            # Single extra state: δ_sr_prev tracks commanded δ via first-order
            # filter, giving a smooth symbolic steering rate.
            _tau_s = self._tau_sr
            f_expl = ca.vertcat(
                f_expl_base,
                (delta_cmd - delta_sr_prev) / _tau_s,                    # δ_sr_prev
            )
        else:
            f_expl = f_expl_base

        model.x = x
        model.u = u_ctrl
        model.xdot = xdot
        model.p = p
        model.f_expl_expr = f_expl
        model.f_impl_expr = xdot - f_expl

        # ---- Stage cost (EXTERNAL) ----
        _ct = self.dt
        d_delta = delta_cmd - delta_prev
        d_jx = Jx - Jx_prev

        # Wrap heading error to [-pi, pi] so the optimizer sees a smooth,
        # shortest-angle objective around the branch cut.
        psi_err = ca.atan2(ca.sin(psi - psi_ref), ca.cos(psi - psi_ref))

        # Keep legacy weight scaling to preserve closed-loop tuning while
        # using wrapped heading error.
        speed_err = u_vel - v_ref
        if self.speed_cost_mode == 'overspeed':
            speed_cost_err = 0.5 * (speed_err + ca.sqrt(speed_err**2 + 1e-4))
        else:
            speed_cost_err = speed_err

        stage_cost = (
            self.w_delta_dot / _ct * d_delta**2 +
            self.w_Jx * _ct * Jx**2 +
            self.w_ax * ax**2 +
            self.w_du * d_jx**2 +
            self.w_steer * delta_cmd**2 +
            self.w_x * (px - x_ref)**2 +
            self.w_y * (py - y_ref)**2 +
            self.w_lateral * (py - y_ref)**2 +
            self.w_heading * psi_err**2 +
            self.w_speed * speed_cost_err**2
        )
        # Obstacle avoidance: smooth softplus barrier penalty.
        # Uses softplus(k*(r - d))/k as a smooth approximation to max(0, r - d).
        # Unlike fmax(0, r²-d²)², the softplus has:
        #   - No derivative discontinuity (infinitely differentiable)
        #   - Nonzero gradient *outside* the obstacle (early warning for SQP-RTI)
        #   - Well-conditioned Hessian (no extreme eigenvalue jumps)
        # Inactive obstacles at (1e4, 1e4) with r=0.001 → negligible penalty.
        _obs_k = 1.5   # softplus sharpness: penalty extends ~1/k metres outside r
        for j in range(self._n_obs):
            x_obs_j = p[self._obs_off + 3 * j + 0]
            y_obs_j = p[self._obs_off + 3 * j + 1]
            r_obs_j = p[self._obs_off + 3 * j + 2]
            dist_j = ca.sqrt((px - x_obs_j)**2 + (py - y_obs_j)**2 + 1e-6)
            penetration_j = r_obs_j - dist_j   # positive when inside obstacle
            # softplus: log(1 + exp(k*x)) / k  ≈ max(0,x) but smooth everywhere
            softplus_j = ca.log(1.0 + ca.exp(_obs_k * penetration_j)) / _obs_k
            stage_cost = stage_cost + self.w_obstacle * softplus_j**2

        model.cost_expr_ext_cost = stage_cost

        # ---- Terminal cost ----
        # At terminal stage the refs are repurposed: x/y_ref = x/y_goal, psi_ref = psi_goal
        psi_err_e = ca.atan2(ca.sin(psi - psi_ref), ca.cos(psi - psi_ref))
        terminal_cost = (
            self.w_terminal * 0.25 * (px - x_ref)**2 +
            self.w_terminal * (py - y_ref)**2 +
            self.w_terminal * 0.5 * psi_err_e**2
        )
        # Add obstacle penalty at terminal stage too (same smooth softplus barrier).
        for j in range(self._n_obs):
            x_obs_j = p[self._obs_off + 3 * j + 0]
            y_obs_j = p[self._obs_off + 3 * j + 1]
            r_obs_j = p[self._obs_off + 3 * j + 2]
            dist_j = ca.sqrt((px - x_obs_j)**2 + (py - y_obs_j)**2 + 1e-6)
            penetration_j = r_obs_j - dist_j
            softplus_j = ca.log(1.0 + ca.exp(_obs_k * penetration_j)) / _obs_k
            terminal_cost = terminal_cost + self.w_obstacle * softplus_j**2

        model.cost_expr_ext_cost_e = terminal_cost

        # ---- Nonlinear path constraints: h(x,u,p) ≤ 0 ----
        h_list = []
        self._traction_h_indices = []

        # Lateral acceleration: |u·ω| ≤ ay_max  (prevents infeasible cornering)
        ay_expr = u_vel * omega
        h_list.append(ay_expr - self.ay_max)
        h_list.append(-ay_expr - self.ay_max)

        # Traction budget (NN models only — analytical models embed traction
        # limits in the tire force curves themselves).
        # Skip for GRU mode: the decoder's Fx predictions are unreliable
        # (consistently negative) making the constraint infeasible.
        # Skip in force-balance mode: x[6] is κ (not ax), and the traction limit
        # is already intrinsic to the surrogate's Fx(κ) curve driving u̇.
        if self.use_nn and not self._gru_mode and not self._force_balance:
            self._traction_h_indices = [len(h_list), len(h_list) + 1]
            h_list.append(M * ax - Fx_traction)
            h_list.append(-M * ax - ca.fabs(Fx_traction))

        model.con_h_expr = ca.vertcat(*h_list)

        return model

    # ------------------------------------------------------------------
    # Trajectory initialisation helpers
    # ------------------------------------------------------------------

    def _init_kinematic_rollout(self, z0, x_ref, y_ref, psi_ref, v_ref):
        """Initialise solver stages with a kinematic rollout along the reference.

        Used for cold start and as a retry fallback after QP failure.
        """
        N, _dt = self.N, self.dt
        zk = np.array(z0, dtype=float)
        for k in range(N):
            xk, yk, psik, uk, vk, omegak, axk, dprev, _jxprev = zk[:9]
            psi_err = np.arctan2(np.sin(psi_ref[k] - psik),
                                 np.cos(psi_ref[k] - psik))
            y_err = (y_ref[k] - yk) * np.cos(psik) - (x_ref[k] - xk) * np.sin(psik)
            delta_des = float(np.clip(psi_err * 1.5 + y_err * 0.2,
                                      self.delta_min, self.delta_max))
            delta_k = float(np.clip(
                delta_des,
                dprev - self.max_steer_rate * _dt,
                dprev + self.max_steer_rate * _dt))
            spd_err = float(v_ref[k]) - uk
            ax_des = float(np.clip(spd_err * 0.8, self.ax_min, self.ax_max))
            jerk_k = float(np.clip((ax_des - axk) / _dt,
                                   self.Jx_min, self.Jx_max))

            self._solver.set(k, 'x', zk)
            self._solver.set(k, 'u', np.array([delta_k, jerk_k]))

            ax_next = float(np.clip(axk + _dt * jerk_k,
                                    self.ax_min, self.ax_max))
            u_next = float(np.clip(uk + _dt * axk, self.u_min, self.u_max))
            # Kinematic warm-start (standard): no arbitrary damping factors.
            v_next = 0.0
            omega_next = float(np.clip(
                u_next * np.tan(delta_k) / max(self.Lf + self.Lr, 1e-6),
                -5.0, 5.0,
            ))
            psi_next = psik + _dt * omega_next
            x_next = xk + _dt * (u_next * np.cos(psik))
            y_next = yk + _dt * (u_next * np.sin(psik))
            zk_base = [x_next, y_next, psi_next, u_next,
                       v_next, omega_next, ax_next, delta_k, jerk_k]
            if self._symbolic_rate_mode:
                # α_prev states: seed from the current rollout command (delta_k),
                # matching the symbolic dynamics channel alpha_f(delta_cmd).
                u_s = max(uk, 0.5)
                Lf, Lr = self.Lf, self.Lr
                af = delta_k - np.arctan2(vk + Lf * omegak, u_s)
                ar = -np.arctan2(vk - Lr * omegak, u_s)
                # δ_sr_prev tracks steering command history; use delta_k for
                # stage-consistent warm-start continuity.
                zk_base.extend([af, ar, delta_k])
            elif self._symbolic_sr:
                # δ_sr_prev tracks steering command history; use delta_k for
                # stage-consistent warm-start continuity.
                zk_base.append(delta_k)
            elif self._temporal_rolling:
                # Shift-register: push current ops (scaled), drop oldest frame
                K = self.nn_tire_model.temporal_K
                hist_dim = (K - 1) * 5
                hf_off = 9
                hr_off = 9 + hist_dim
                sc = self._hist_feat_scale
                u_s = max(uk, 0.5)
                Lf, Lr = self.Lf, self.Lr
                af = dprev - np.arctan2(vk + Lf * omegak, u_s)
                ar = -np.arctan2(vk - Lr * omegak, u_s)
                Fz_f = (self.M * 9.81 * Lr - self.M * axk * self.h_cg) / (Lf + Lr) / 2.0
                Fz_r = (self.M * 9.81 * Lf + self.M * axk * self.h_cg) / (Lf + Lr) / 2.0
                if self.kappa_mode == 'approx':
                    kap = float(np.clip(axk / (0.4 * 9.81), -0.3, 0.3))
                else:
                    kap = 0.0
                cur_f = np.array([kap, af, u_s, Fz_f, 0.0]) / sc
                cur_r = np.array([kap, ar, u_s, Fz_r, 0.0]) / sc
                old_hf = list(zk[hf_off: hf_off + hist_dim])
                old_hr = list(zk[hr_off: hr_off + hist_dim])
                # frame[0] ← current_scaled, frame[j] ← frame[j-1], oldest dropped
                new_hf = list(cur_f) + old_hf[: hist_dim - 5]
                new_hr = list(cur_r) + old_hr[: hist_dim - 5]
                zk_base.extend(new_hf + new_hr)
            zk = np.array(zk_base)
        self._solver.set(N, 'x', zk)

    # ------------------------------------------------------------------
    # Stage-varying history for temporal models
    # ------------------------------------------------------------------

    def _compute_staged_history(self, k, hf_meas, hr_meas, z0, h_dim):
        """Compute the history parameter for horizon stage k.

        Stage 0: uses the measured history directly.
        Stage k>0: shifts the history forward k times using the previous
        solve's predicted trajectory to synthesise operating-point frames.

        The shift-register logic mirrors what would happen if history were
        a state: frame[0] ← current op-point, frame[j] ← frame[j-1],
        oldest frame dropped.
        """
        if k == 0 or self._prev_Z is None:
            return hf_meas.copy(), hr_meas.copy()

        # Build the sequence of shifted histories for stages 1..k.
        # Starting from measured history, push operating points from
        # the previous predicted trajectory (shifted by 1 for warm-start alignment).
        hf = hf_meas.copy()
        hr = hr_meas.copy()
        Z = self._prev_Z  # (nx, N+1)
        Lf, Lr = self.Lf, self.Lr
        M = self.M
        h_cg = self.h_cg
        K_temp = self.nn_tire_model.temporal_K

        for j in range(k):
            # Index into previous trajectory (shifted by 1 for warm-start)
            idx = min(j + 1, Z.shape[1] - 1)
            xk, yk, psik, uk, vk, omegak, axk, dk, jxk = Z[:9, idx]

            u_s = max(uk, 0.5)
            alpha_f = dk - np.arctan2(vk + Lf * omegak, u_s)
            alpha_r = -np.arctan2(vk - Lr * omegak, u_s)
            Fz_f = (M * 9.81 * Lr - M * axk * h_cg) / (Lf + Lr) / 2.0
            Fz_r = (M * 9.81 * Lf + M * axk * h_cg) / (Lf + Lr) / 2.0
            if self.kappa_mode == 'approx':
                kap = float(np.clip(axk / (0.4 * 9.81), -0.3, 0.3))
            else:
                kap = 0.0

            cur_f = np.array([kap, alpha_f, u_s, Fz_f, 0.0])
            cur_r = np.array([kap, alpha_r, u_s, Fz_r, 0.0])

            # Shift: new = [cur | old[:-5]]
            hf = np.concatenate([cur_f, hf[:h_dim - 5]])
            hr = np.concatenate([cur_r, hr[:h_dim - 5]])

        return hf, hr

    # ------------------------------------------------------------------
    # Solve
    # ------------------------------------------------------------------

    def solve(self, z0, x_ref, y_ref, psi_ref, v_ref,
              x_goal, y_goal, psi_goal,
              n_terrain=None, sr_meas=0.0,
              terrain_params=None,
              hist_front=None, hist_rear=None,
              rates_front=None, rates_rear=None,
              gru_h_front=None, gru_h_rear=None,
              force_residuals=None,
              dynamics_residuals=None,
              obstacles=None):
        """
        Solve the MPC.

        Returns:
            delta_cmd, Jx, Z_opt, U_opt  — first control is road-wheel angle δ [rad]
        """
        if self._solver is None:
            raise RuntimeError("ACADOS solver is not built. Call compile_solver() first.")

        nx, nu, N = self.nx, self.nu, self.N
        nn = self.nn_tire_model

        # Input validation: fail fast with clear diagnostics.
        z0 = np.asarray(z0, dtype=float).reshape(-1)
        if z0.size != nx:
            raise ValueError(f"z0 has length {z0.size}, expected {nx}")

        x_ref = np.asarray(x_ref, dtype=float).reshape(-1)
        y_ref = np.asarray(y_ref, dtype=float).reshape(-1)
        psi_ref = np.asarray(psi_ref, dtype=float).reshape(-1)
        v_ref = np.asarray(v_ref, dtype=float).reshape(-1)
        exp_ref_len = N + 1
        if not (x_ref.size == y_ref.size == psi_ref.size == v_ref.size == exp_ref_len):
            raise ValueError(
                "Reference arrays must all have length N+1 "
                f"(expected {exp_ref_len}, got x={x_ref.size}, y={y_ref.size}, "
                f"psi={psi_ref.size}, v={v_ref.size})"
            )

        if force_residuals is not None:
            force_residuals = np.asarray(force_residuals, dtype=float)
            if force_residuals.shape != (N + 1, 2):
                raise ValueError(
                    f"force_residuals must have shape ({N + 1}, 2), "
                    f"got {force_residuals.shape}"
                )

        if dynamics_residuals is not None:
            dynamics_residuals = np.asarray(dynamics_residuals, dtype=float)
            if dynamics_residuals.shape != (N + 1, 3):
                raise ValueError(
                    f"dynamics_residuals must have shape ({N + 1}, 3), "
                    f"got {dynamics_residuals.shape}"
                )

        # Resolve terrain params
        tp_default = {
            'Kphi': 0.0, 'Kc': 0.0, 'n': 1.1, 'c': 0.0, 'phi': 0.0, 'k': 0.0
        }
        if nn is not None:
            tp_default.update(nn._terrain_nominals)
        if terrain_params is not None:
            tp_default.update(terrain_params)
        tp = tp_default

        if n_terrain is None:
            if nn is not None:
                terrain_n = float(tp.get('n', nn.n_nominal))
            else:
                terrain_n = float(tp.get('n', 1.1))
        else:
            terrain_n = float(n_terrain)

        # Project x0 to hard state bounds to avoid infeasibility from transient
        # estimator spikes at the measured stage.
        z0 = np.array(z0, dtype=float, copy=True)
        z0[3] = np.clip(z0[3], self.u_min, self.u_max)
        z0[4] = np.clip(z0[4], -10.0, 10.0)
        z0[5] = np.clip(z0[5], -5.0, 5.0)
        z0[6] = np.clip(z0[6], self.ax_min, self.ax_max)
        z0[7] = np.clip(z0[7], self.delta_min, self.delta_max)
        z0[8] = np.clip(z0[8], self.Jx_min, self.Jx_max)
        if self._symbolic_rate_mode:
            z0[9] = np.clip(z0[9], -0.8, 0.8)
            z0[10] = np.clip(z0[10], -0.8, 0.8)
            z0[11] = np.clip(z0[11], self.delta_min, self.delta_max)
        elif self._symbolic_sr:
            z0[self._sr_state_idx] = np.clip(
                z0[self._sr_state_idx], self.delta_min, self.delta_max
            )

        # Terrain friction angle in the units expected by the selected NN
        # checkpoint.  Rig v6 checkpoints used radians; closed-loop vehicle
        # checkpoints use degrees.  The NN loader detects this from scalers.
        if self.use_nn and hasattr(self.nn_tire_model, 'phi_feature_value'):
            phi_val = self.nn_tire_model.phi_feature_value(float(tp['phi']))
        else:
            phi_raw = float(tp['phi'])
            phi_val = np.radians(phi_raw) if abs(phi_raw) > np.pi else phi_raw

        terrain_vec = [tp['Kphi'], tp['Kc'], terrain_n, tp['c'], phi_val, tp['k']]

        tp_for_refs = dict(tp)
        tp_for_refs['n'] = terrain_n

        kappa_ref_front, kappa_ref_rear = self._compute_runtime_kappa_refs(
            z0,
            terrain_params=tp_for_refs,
            sr_meas=sr_meas,
            hist_front=hist_front,
            hist_rear=hist_rear,
            rates_front=rates_front,
            rates_rear=rates_rear,
            gru_h_front=gru_h_front,
            gru_h_rear=gru_h_rear,
        )

        # Set initial state
        self._solver.set(0, 'lbx', z0)
        self._solver.set(0, 'ubx', z0)

        # Build flat obstacle parameter list: [x0,y0,r0, x1,y1,r1, ...]
        # Inactive obstacles placed far away (penalty = 0 at any reachable position).
        _obs_flat = [1e4, 1e4, 0.001] * self._n_obs
        if obstacles:
            for j, obs in enumerate(obstacles[:self._n_obs]):
                _obs_flat[3 * j + 0] = float(obs[0])
                _obs_flat[3 * j + 1] = float(obs[1])
                _obs_flat[3 * j + 2] = float(obs[2])

        # Build per-stage parameter vectors
        _p_base = []
        for k in range(N):
            p_k = list(terrain_vec) + [
                x_ref[k], y_ref[k], psi_ref[k], v_ref[k],
                sr_meas, kappa_ref_front, kappa_ref_rear,
            ]
            if self._temporal_staged:
                K = nn.temporal_K
                h_dim = (K - 1) * 5
                hf = hist_front if hist_front is not None else np.zeros(h_dim)
                hr = hist_rear if hist_rear is not None else np.zeros(h_dim)
                # Compute stage-varying history by shifting the measured
                # history forward using the previous solve's trajectory.
                hf_k, hr_k = self._compute_staged_history(
                    k, hf, hr, z0, h_dim)
                p_k += list(hf_k) + list(hr_k)
            elif self._temporal_mode and not self._temporal_rolling:
                K = nn.temporal_K
                h_dim = (K - 1) * 5
                hf = hist_front if hist_front is not None else np.zeros(h_dim)
                hr = hist_rear if hist_rear is not None else np.zeros(h_dim)
                p_k += list(hf) + list(hr)
            if self._rate_mode and not self._symbolic_rate_mode:
                rf = rates_front if rates_front is not None else np.zeros(3)
                rr = rates_rear if rates_rear is not None else np.zeros(3)
                p_k += list(rf) + list(rr)
            if self._gru_mode:
                hd = nn.gru_h_dim
                ghf = gru_h_front if gru_h_front is not None else np.zeros(hd)
                ghr = gru_h_rear if gru_h_rear is not None else np.zeros(hd)
                p_k += list(ghf) + list(ghr)
            # Force residual corrections (always slots before dynamics/obstacle params)
            if force_residuals is not None:
                p_k += [float(force_residuals[k, 0]), float(force_residuals[k, 1])]
            else:
                p_k += [0.0, 0.0]
            # Dynamics residual corrections [du_dot, dv_dot, domega_dot]
            if dynamics_residuals is not None:
                p_k += [float(dynamics_residuals[k, 0]),
                        float(dynamics_residuals[k, 1]),
                        float(dynamics_residuals[k, 2])]
            else:
                p_k += [0.0, 0.0, 0.0]
            # Obstacle avoidance parameters (always last 3*N_OBS slots)
            p_k += _obs_flat
            _p_base.append(p_k)

        # Warm-start: shift previous solution by 1 step.
        _have_prev = (self._prev_Z is not None and self._prev_U is not None)
        if _have_prev:
            _speed_gap = abs(z0[3] - self._prev_Z[3, 0])
            if _speed_gap > 2.0:
                _have_prev = False

        if _have_prev:
            for k in range(N):
                idx = min(k + 1, N - 1)
                self._solver.set(k, 'x', self._prev_Z[:, idx])
                self._solver.set(k, 'u', self._prev_U[:, idx])
            self._solver.set(N, 'x', self._prev_Z[:, -1])
        else:
            self._init_kinematic_rollout(z0, x_ref, y_ref, psi_ref, v_ref)

        for k in range(N):
            self._solver.set(k, 'p', np.array(_p_base[k]))

        # Terminal stage parameters: x/y_ref = x/y_goal, psi_ref = psi_goal
        p_e = list(terrain_vec) + [
            x_goal, y_goal, psi_goal, v_ref[-1],
            sr_meas, kappa_ref_front, kappa_ref_rear,
        ]
        if self._temporal_staged:
            K = nn.temporal_K
            h_dim = (K - 1) * 5
            hf = hist_front if hist_front is not None else np.zeros(h_dim)
            hr = hist_rear if hist_rear is not None else np.zeros(h_dim)
            hf_e, hr_e = self._compute_staged_history(N, hf, hr, z0, h_dim)
            p_e += list(hf_e) + list(hr_e)
        elif self._temporal_mode and not self._temporal_rolling:
            K = nn.temporal_K
            h_dim = (K - 1) * 5
            hf = hist_front if hist_front is not None else np.zeros(h_dim)
            hr = hist_rear if hist_rear is not None else np.zeros(h_dim)
            p_e += list(hf) + list(hr)
        if self._rate_mode and not self._symbolic_rate_mode:
            rf = rates_front if rates_front is not None else np.zeros(3)
            rr = rates_rear if rates_rear is not None else np.zeros(3)
            p_e += list(rf) + list(rr)
        if self._gru_mode:
            hd = nn.gru_h_dim
            ghf = gru_h_front if gru_h_front is not None else np.zeros(hd)
            ghr = gru_h_rear if gru_h_rear is not None else np.zeros(hd)
            p_e += list(ghf) + list(ghr)
        # Force residual corrections (terminal stage)
        if force_residuals is not None:
            p_e += [float(force_residuals[N, 0]), float(force_residuals[N, 1])]
        else:
            p_e += [0.0, 0.0]
        # Dynamics residual corrections (terminal stage)
        if dynamics_residuals is not None:
            p_e += [float(dynamics_residuals[N, 0]),
                    float(dynamics_residuals[N, 1]),
                    float(dynamics_residuals[N, 2])]
        else:
            p_e += [0.0, 0.0, 0.0]
        # Obstacle avoidance parameters (terminal stage, same as stage params)
        p_e += _obs_flat

        self._solver.set(N, 'p', np.array(p_e))

        # Solve — suppress C-level HPIPM MINSTEP/stderr noise.
        with _SuppressC():
            status = self._solver.solve()

        # On QP_FAILURE (status 4) with warm-start, retry from kinematic rollout.
        # MINSTEP (status 3) and MAX_ITER (status 2) still yield usable RTI primal
        # solutions — do not retry those (the warm-start is usually better).
        if status == 4 and _have_prev:
            self._init_kinematic_rollout(z0, x_ref, y_ref, psi_ref, v_ref)
            with _SuppressC():
                status = self._solver.solve()

        self.last_solver_status = str(status)

        # Extract solution regardless of MINSTEP/MAX_ITER — primal is still usable.
        # Only fall back to hold on true QP_FAILURE (status 4) or NaN output.
        Z_opt = np.zeros((nx, N + 1))
        U_opt = np.zeros((nu, N))
        for k in range(N):
            Z_opt[:, k] = self._solver.get(k, 'x')
            U_opt[:, k] = self._solver.get(k, 'u')
        Z_opt[:, N] = self._solver.get(N, 'x')

        # Cache warm-start: keep on MINSTEP/success, clear on QP_FAILURE.
        if status == 4:
            self._prev_Z = None
            self._prev_U = None
            # Make the silent hold-fallback visible: QP_FAILURE means the
            # commanded (delta, Jx) below are stale/degraded. Rate-limit the
            # warning so a bad patch of terrain doesn't flood stderr.
            self._qp_fail_count = getattr(self, "_qp_fail_count", 0) + 1
            if self._qp_fail_count <= 3 or self._qp_fail_count % 50 == 0:
                import sys as _sys
                print(f"  [MPC] WARNING: QP_FAILURE (status 4) — holding last command "
                      f"(failure #{self._qp_fail_count})", file=_sys.stderr, flush=True)
        else:
            self._prev_Z = Z_opt.copy()
            self._prev_U = U_opt.copy()

        delta_cmd = float(U_opt[0, 0])
        Jx = float(U_opt[1, 0])

        # Get solver stats
        try:
            self.last_iter_count = int(self._solver.get_stats('sqp_iter'))
        except Exception:
            self.last_iter_count = 1
        try:
            self.last_cost = float(self._solver.get_cost())
        except Exception:
            self.last_cost = float('nan')

        # Numerical sanity check: only reject NaN/Inf outputs (not MINSTEP status).
        if (not np.isfinite(delta_cmd) or not np.isfinite(Jx) or
                not np.isfinite(Z_opt).all() or not np.isfinite(U_opt).all()):
            self.reset_warmstart()
            return 0.0, 0.0, None, None

        self._last_u0 = U_opt[:, 0].copy()
        return delta_cmd, Jx, Z_opt, U_opt


# ============================================================================
# CLI smoke test
# ============================================================================

if __name__ == '__main__':
    import time
    sys.path.insert(0, str(Path(__file__).parent))
    from param_consistency import TERRAIN_PRESETS
    from nn_tire_model import load_nn_tire_model

    terrain = 'sand'
    preset = TERRAIN_PRESETS[terrain]
    tp = {
        'Kphi': preset['Kphi'], 'Kc': preset['Kc'], 'n': preset['n'],
        'c': preset['cohesion'], 'phi': preset['friction_angle'], 'k': preset['janosi_shear'],
    }

    # Try to load a model
    model_dir = Path(__file__).parent.parent / 'nn_models' / 'v6_sweep_12_2'
    if not model_dir.exists():
        # Fallback to any sweep model
        models_root = Path(__file__).parent.parent / 'nn_models'
        candidates = sorted(models_root.glob('sweep_mlp_*'))
        if candidates:
            model_dir = candidates[0]
        else:
            print("No NN model found. Exiting.")
            sys.exit(1)

    nn = load_nn_tire_model(model_dir, tp)
    mpc = AcadosMPC(nn_tire_model=nn, dt=DEFAULT_MPC_DT, N=DEFAULT_MPC_HORIZON_STEPS)

    # Simple test scenario
    N = mpc.N
    dt = mpc.dt
    v_target = 5.0
    t = np.arange(N + 1) * dt
    x_ref = v_target * t
    y_ref = 2.0 * np.sin(0.3 * x_ref)
    psi_ref = np.arctan2(np.gradient(y_ref), np.gradient(x_ref))
    v_ref = np.full(N + 1, v_target)

    z0 = np.zeros(mpc.nx)
    z0[:6] = np.array([x_ref[0], y_ref[0], psi_ref[0], v_target, 0.0, 0.0])
    # ax, δ_prev, Jx_prev remain 0

    # Warm up
    for _ in range(5):
        dc, jx, Z, U = mpc.solve(z0, x_ref, y_ref, psi_ref, v_ref,
                                   x_ref[-1], y_ref[-1], psi_ref[-1],
                                   terrain_params=tp)

    # Benchmark
    times = []
    for _ in range(50):
        t0 = time.perf_counter()
        dc, jx, Z, U = mpc.solve(z0, x_ref, y_ref, psi_ref, v_ref,
                                   x_ref[-1], y_ref[-1], psi_ref[-1],
                                   terrain_params=tp)
        times.append((time.perf_counter() - t0) * 1000)
        z0 = Z[:, 1]  # advance

    print(f"\n--- Benchmark (50 solves) ---")
    print(f"Mean:   {np.mean(times):.2f} ms")
    print(f"Median: {np.median(times):.2f} ms")
    print(f"p95:    {np.percentile(times, 95):.2f} ms")
    print(f"Hz:     {1000/np.mean(times):.1f}")
    print(f"δ={dc:.4f} rad  Jx={jx:.4f}")
