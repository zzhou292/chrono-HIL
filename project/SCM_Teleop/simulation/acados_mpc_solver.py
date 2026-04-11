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
    mpc = AcadosMPC(nn_tire_model=nn, dt=0.1, N=20)

    delta_cmd, Jx, Z_opt, U_opt = mpc.solve(z0, x_ref, y_ref, psi_ref, v_ref,
                                             x_goal, y_goal, psi_goal,
                                             terrain_params=tp)
"""

from __future__ import annotations

import os
import sys
import shutil
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
from analytical_tire_models import get_tire_forces as _analytical_tire_forces


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

    def __init__(self, nn_tire_model=None, dt=0.1, N=20,
                 lateral_load_transfer=True, kappa_mode='zero',
                 build_dir=None, tire_model='nn'):
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
            tire_model: 'nn', 'pacejka', 'tmeasy', or 'linear'.
        """
        self.tire_model = tire_model
        self.nn_tire_model = nn_tire_model
        self.dt = dt
        self.N = N
        self.nx = 9    # [x, y, ψ, u, v, ω, ax, δ_prev, Jx_prev]
        self.nu = 2    # [δ, Jx]
        self.lateral_load_transfer = lateral_load_transfer
        self.kappa_mode = kappa_mode

        # Vehicle params (exposed for controller node)
        self.Lf = _Lf
        self.Lr = _Lr
        self.M = _M
        self.Izz = _Izz
        self.h_cg = _h_cg
        self.T = _T
        self.use_nn = (nn_tire_model is not None and tire_model == 'nn')

        # Bounds
        self.u_min, self.u_max = 0.5, 20.0
        self.delta_min, self.delta_max = -0.528, 0.528
        self.ax_min, self.ax_max = -2.6, 1.9
        self.delta_dot_min, self.delta_dot_max = -0.5, 0.5
        self.Jx_min, self.Jx_max = -3.0, 3.0

        # Cost weights
        self.w_lateral = 120.0
        self.w_heading = 20.0
        self.w_speed = 10.0
        # Strong (δ−δ_prev)²/dt term smooths wheel angle (straights / RTI).
        self.w_delta_dot = 800.0
        self.w_Jx = 20.0
        self.w_terminal = 50.0
        self.w_du = 500.0   # penalty on (Jx − Jx_prev)²
        self.w_steer = 0.0
        self.max_steer_rate = self.delta_dot_max  # rad/s, for kinematic warm-start

        # NN integration mode
        self._temporal_mode = (self.use_nn and nn_tire_model.temporal_K > 1)
        self._rate_mode = (self.use_nn and nn_tire_model.rate_augmented)

        # nn_scale
        self.nn_scale = 1.0

        # Solver tuning (model-adaptive)
        self._lm = 1e-3
        self._qp_iter_max = 50
        self._nlp_max_iter = 1
        # Static NN: scale Levenberg-Marquardt regularisation with model
        # complexity.  Larger networks produce sharper Jacobians whose
        # Gauss-Newton Hessian is more ill-conditioned for HPIPM, causing
        # QP_FAILURE (status 4) especially at low-speed start-up.
        if (self.use_nn and not self._temporal_mode and not self._rate_mode
                and nn_tire_model is not None):
            n_p = nn_tire_model.n_params
            self._lm = min(max(1e-3, n_p / 400.0 * 1e-3), 1e-2)
        # ResNet temporal models have deeper expression trees → more
        # regularisation and extra SQP iterations to help convergence.
        if self._temporal_mode and nn_tire_model is not None:
            if 'resnet' in nn_tire_model.model_type:
                self._lm = 1e-2
                self._nlp_max_iter = 3
            else:
                # Temporal MLP: deeper input stacking than static → slightly more SQP work.
                self._lm = max(self._lm, 5e-3)
                self._nlp_max_iter = max(self._nlp_max_iter, 2)
        # Analytical tire models (Pacejka/TMeasy) have steep nonlinearities.
        # SQP with multiple iterations + FULL_CONDENSING_QPOASES avoids the
        # MINSTEP errors that plague HPIPM with these dynamics.
        if not self.use_nn:
            self._lm = 5e-2
            self._nlp_max_iter = 3

        # Build dir
        if self.use_nn:
            model_tag = nn_tire_model.model_type
        else:
            model_tag = tire_model  # pacejka / tmeasy / linear
        if build_dir is None:
            build_dir = Path(f'/tmp/acados_mpc_{model_tag}')
        self._build_dir = Path(build_dir)

        # Build the ACADOS OCP solver
        self._build_ocp()

        # Warm-start state
        self._prev_U = None
        self._prev_Z = None
        self._last_u0 = np.zeros(self.nu)
        self.last_iter_count = 0
        self.last_solver_status = ''

    # ------------------------------------------------------------------
    # OCP construction
    # ------------------------------------------------------------------

    def _compute_param_layout(self):
        """Compute parameter dimension and offsets (needed before OCP build and at solve time)."""
        n_terrain = 5  # Kphi, Kc, c, phi, k
        n_ref_per_stage = 3  # y_ref_k, psi_ref_k, v_ref_k
        n_extra = 1  # sr_meas (frozen)
        n_hist = 0
        if self._temporal_mode:
            K = self.nn_tire_model.temporal_K
            n_hist = 2 * (K - 1) * 5  # front + rear
        n_rate = 0
        if self._rate_mode:
            n_rate = 6  # rates_front(3) + rates_rear(3)

        self._np_per_stage = n_terrain + n_ref_per_stage + n_extra + n_hist + n_rate
        self._tp_off = 0
        self._ref_off = n_terrain
        self._sr_off = n_terrain + n_ref_per_stage
        self._hist_off = n_terrain + n_ref_per_stage + n_extra
        self._rate_off = self._hist_off + n_hist

    def _build_ocp(self):
        """Construct the full ACADOS OCP and compile the solver."""
        nx, nu, N, dt = self.nx, self.nu, self.N, self.dt

        # Compute param layout (needed for fingerprint and solve)
        self._compute_param_layout()

        # --- Cache: skip full OCP build if compiled solver matches ---
        fingerprint = self._compute_fingerprint()
        fp_file = self._build_dir / '.fingerprint'
        so_file = self._build_dir / 'c_generated_code' / 'libacados_ocp_solver_bicycle.so'
        json_file = self._build_dir / 'acados_ocp.json'

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
            return

        # --- Cache miss: build full OCP ---
        ocp = AcadosOcp()

        # ---- Model (uses _np_per_stage and offsets) ----
        model = self._build_acados_model()
        ocp.model = model

        # ---- Dimensions ----
        ocp.dims.N = N
        ocp.dims.np = self._np_per_stage
        ocp.parameter_values = np.zeros(self._np_per_stage)

        # ---- Cost ----
        # We use EXTERNAL cost type (CasADi expression).
        ocp.cost.cost_type = 'EXTERNAL'
        ocp.cost.cost_type_e = 'EXTERNAL'

        # ---- Constraints: bounds ----
        # State bounds (indices: u=3, v=4, ω=5, ax=6, δ_prev=7, Jx_prev=8)
        ocp.constraints.lbx = np.array([self.u_min, -10.0, -5.0, self.ax_min,
                                        self.delta_min, self.Jx_min])
        ocp.constraints.ubx = np.array([self.u_max, 10.0, 5.0, self.ax_max,
                                        self.delta_max, self.Jx_max])
        ocp.constraints.idxbx = np.array([3, 4, 5, 6, 7, 8])

        # Terminal state bounds (same)
        ocp.constraints.lbx_e = ocp.constraints.lbx.copy()
        ocp.constraints.ubx_e = ocp.constraints.ubx.copy()
        ocp.constraints.idxbx_e = ocp.constraints.idxbx.copy()

        # Control bounds: δ and Jx
        ocp.constraints.lbu = np.array([self.delta_min, self.Jx_min])
        ocp.constraints.ubu = np.array([self.delta_max, self.Jx_max])
        ocp.constraints.idxbu = np.array([0, 1])

        # Initial state (will be set at solve time)
        ocp.constraints.x0 = np.zeros(nx)

        # ---- Nonlinear constraints: traction (NN models only) ----
        # h(x, u, p) <= 0: [M*ax - Fx_avail, -M*ax - |Fx_avail|]
        # Analytical models skip this — qpOASES can't handle one-sided
        # nonlinear constraints (declares INFEASIBLE every solve).
        if hasattr(model.con_h_expr, 'shape'):
            nh = model.con_h_expr.shape[0]
            ocp.constraints.lh = -1e9 * np.ones(nh)
            ocp.constraints.uh = np.zeros(nh)

        # ---- Solver options ----
        # PARTIAL_CONDENSING_HPIPM for all models.  Previous MINSTEP errors
        # with analytical tire models were caused by the hard traction
        # constraints (now removed for analytical).  qpOASES was tried but
        # declares INFEASIBLE on most QPs due to its "no one-sided
        # constraints" limitation interacting with the box bounds.
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI' if self._nlp_max_iter == 1 else 'SQP'
        # GAUSS_NEWTON approximation for all models: EXTERNAL cost is purely
        # quadratic in states/controls, so the GN Hessian equals the exact cost
        # Hessian.  EXACT Hessian (including constraint curvature) was tested
        # but worsened HPIPM conditioning with Pacejka dynamics.
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.integrator_type = 'ERK'
        if self.use_nn:
            ocp.solver_options.sim_method_num_stages = 1  # Euler (NN is smooth)
        else:
            ocp.solver_options.sim_method_num_stages = 4  # RK4 for analytical tire models
        ocp.solver_options.sim_method_num_steps = 1
        ocp.solver_options.nlp_solver_max_iter = self._nlp_max_iter
        ocp.solver_options.qp_solver_iter_max = self._qp_iter_max
        ocp.solver_options.tf = N * dt
        ocp.solver_options.levenberg_marquardt = self._lm
        ocp.solver_options.qp_solver_warm_start = 2     # full warm-start QP
        ocp.solver_options.print_level = 0             # suppress MINSTEP warnings

        # Large NN models (>5k params) generate huge CasADi C files that
        # cause gcc ICE at -O2.  Use -O0 + system gcc for those; let
        # small models compile normally with -O2 for better runtime perf.
        _NN_PARAM_THRESHOLD = 5000
        nn_params = self.nn_tire_model.n_params if self.use_nn else 0
        use_O0_workaround = nn_params > _NN_PARAM_THRESHOLD

        if use_O0_workaround:
            ocp.solver_options.ext_fun_compile_flags = '-O0'

        # Code generation
        ocp.code_export_directory = str(self._build_dir / 'c_generated_code')

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

    def reset_warmstart(self):
        """Clear cached trajectory so next solve uses kinematic rollout."""
        self._prev_Z = None
        self._prev_U = None
        self._last_u0 = np.zeros(self.nu)

    def _compute_fingerprint(self) -> str:
        """Hash of NN weights + solver config so we can cache compiled code."""
        h = hashlib.sha256()
        # Solver config
        cfg = json.dumps({
            'N': self.N, 'dt': self.dt, 'nx': self.nx, 'nu': self.nu,
            'lat_transfer': self.lateral_load_transfer,
            'kappa_mode': self.kappa_mode,
            'np': self._np_per_stage,
            'tire_model': self.tire_model,
            'qp_solver': 'PARTIAL_CONDENSING_HPIPM',
            'levenberg_marquardt': self._lm,
            'qp_solver_iter_max': self._qp_iter_max,
            'nlp_solver_max_iter': self._nlp_max_iter,
            'print_level': 0,
            'w_delta_dot': self.w_delta_dot,
            'w_Jx': self.w_Jx,
            'steer_angle_mpc': True,
            'w_lateral': self.w_lateral,
            'w_heading': self.w_heading,
            'w_speed': self.w_speed,
            'w_du': self.w_du,
            'w_steer': self.w_steer,
            'w_terminal': self.w_terminal,
            'use_nn': self.use_nn,
            'ext_fun_compile_flags': '-O0' if (self.use_nn and self.nn_tire_model.n_params > 5000) else '',
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

    def _build_acados_model(self) -> AcadosModel:
        """Define the CasADi symbolic model for ACADOS."""
        model = AcadosModel()
        model.name = 'bicycle'

        nx, nu = self.nx, self.nu
        M, Izz, Lf, Lr = _M, _Izz, _Lf, _Lr
        h_cg, T = _h_cg, _T

        # ---- Symbolic variables ----
        x = ca.SX.sym('x', nx)  # [x, y, ψ, u, v, ω, ax, δ_prev, Jx_prev]
        u_ctrl = ca.SX.sym('u', nu)  # [δ, Jx]
        xdot = ca.SX.sym('xdot', nx)

        # Parameters per stage
        p = ca.SX.sym('p', self._np_per_stage)

        # Unpack parameters
        Kphi_sym = p[self._tp_off + 0]
        Kc_sym   = p[self._tp_off + 1]
        c_sym    = p[self._tp_off + 2]
        phi_sym  = p[self._tp_off + 3]
        k_sym    = p[self._tp_off + 4]
        y_ref    = p[self._ref_off + 0]
        psi_ref  = p[self._ref_off + 1]
        v_ref    = p[self._ref_off + 2]
        sr_meas  = p[self._sr_off]

        # Unpack state
        px, py, psi = x[0], x[1], x[2]
        u_vel, v_vel, omega = x[3], x[4], x[5]
        ax = x[6]
        delta_prev = x[7]
        Jx_prev = x[8]

        # Unpack controls (commanded road-wheel angle + longitudinal jerk)
        delta_cmd = u_ctrl[0]
        Jx = u_ctrl[1]

        # ---- Tire force computation ----
        u_safe = ca.fmax(ca.fabs(u_vel), 0.5)
        alpha_f = delta_cmd - ca.atan2(v_vel + Lf * omega, u_safe)
        alpha_r = -ca.atan2(v_vel - Lr * omega, u_safe)

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

        # Slip ratio
        if self.kappa_mode == 'approx':
            kappa = ca.fmax(ca.fmin(ax / (0.4 * 9.81), 0.3), -0.3)
        else:
            kappa = 0.0

        # Sinkage exponent from terrain (packed as n = c_sym placeholder... no,
        # we need n_terrain. Let's encode it. Actually, in MPC we typically fix
        # n_terrain as the nominal from the preset. The 5 terrain params we pass
        # are Kphi, Kc, c, phi, k. n is fixed at model.n_nominal.)
        nn = self.nn_tire_model
        n_terrain_val = nn.n_nominal if nn is not None else 1.1

        kappa_ref = 0.15
        Fx_traction = ca.SX(0.0)

        if nn is not None and self._temporal_mode:
            # --- Temporal batched ---
            K = nn.temporal_K
            hist_dim = (K - 1) * 5
            hist_front = p[self._hist_off: self._hist_off + hist_dim]
            hist_rear = p[self._hist_off + hist_dim: self._hist_off + 2 * hist_dim]
            t_vec = ca.vertcat(Kphi_sym, Kc_sym, n_terrain_val, c_sym, phi_sym, k_sym)

            if self.lateral_load_transfer:
                slot_ops = [
                    ca.vertcat(kappa, alpha_f, u_safe, Fz_f_outer, sr_meas),
                    ca.vertcat(kappa, alpha_f, u_safe, Fz_f_inner, sr_meas),
                    ca.vertcat(kappa, alpha_r, u_safe, Fz_r_outer, sr_meas),
                    ca.vertcat(kappa, alpha_r, u_safe, Fz_r_inner, sr_meas),
                    ca.vertcat(kappa_ref, 0.0, u_safe, Fz_f_mean, 0.0),
                    ca.vertcat(kappa_ref, 0.0, u_safe, Fz_r_mean, 0.0),
                    ca.vertcat(kappa_ref, 0.0, u_safe, Fz_f_mean, 0.0),
                    ca.vertcat(kappa_ref, 0.0, u_safe, Fz_r_mean, 0.0),
                ]
                slot_hist = [hist_front, hist_front, hist_rear, hist_rear,
                             hist_front, hist_rear, hist_front, hist_rear]
            else:
                slot_ops = [
                    ca.vertcat(kappa, alpha_f, u_safe, Fz_f_mean, sr_meas),
                    ca.vertcat(kappa, alpha_r, u_safe, Fz_r_mean, sr_meas),
                    ca.vertcat(kappa_ref, 0.0, u_safe, Fz_f_mean, 0.0),
                    ca.vertcat(kappa_ref, 0.0, u_safe, Fz_r_mean, 0.0),
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
            else:
                Fyf = -self.nn_scale * 2.0 * Fys_all[0]
                Fyr = -self.nn_scale * 2.0 * Fys_all[1]
                Fx_traction = 2.0 * (Fxs_all[2] + Fxs_all[3])

        elif nn is not None and self._rate_mode:
            # --- Rate-augmented batched ---
            rates_front = p[self._rate_off: self._rate_off + 3]
            rates_rear = p[self._rate_off + 3: self._rate_off + 6]
            B = nn._BATCH

            if self.lateral_load_transfer:
                a_vec = ca.vertcat(alpha_f, alpha_f, alpha_r, alpha_r, 0.0, 0.0, 0.0, 0.0)
                fz_vec = ca.vertcat(Fz_f_outer, Fz_f_inner, Fz_r_outer, Fz_r_inner,
                                    Fz_f_mean, Fz_r_mean, Fz_f_mean, Fz_r_mean)
                u_vec = ca.repmat(u_safe, B, 1)
                k_vec = ca.vertcat(kappa, kappa, kappa, kappa,
                                   kappa_ref, kappa_ref, kappa_ref, kappa_ref)
                n_vec = ca.repmat(n_terrain_val, B, 1)
                sr_vec = ca.vertcat(sr_meas, sr_meas, sr_meas, sr_meas, 0.0, 0.0, 0.0, 0.0)
                dk_vec = ca.vertcat(rates_front[0], rates_front[0], rates_rear[0], rates_rear[0],
                                    rates_front[0], rates_rear[0], rates_front[0], rates_rear[0])
                da_vec = ca.vertcat(rates_front[1], rates_front[1], rates_rear[1], rates_rear[1],
                                    0.0, 0.0, 0.0, 0.0)
                du_vec = ca.vertcat(rates_front[2], rates_front[2], rates_rear[2], rates_rear[2],
                                    rates_front[2], rates_rear[2], rates_front[2], rates_rear[2])

                Fxs_all, Fys_all = nn.predict_batch_rate(
                    a_vec, fz_vec, u_vec, k_vec, n_vec, sr_vec,
                    dk_vec, da_vec, du_vec, Kphi_sym, Kc_sym, c_sym, phi_sym, k_sym)
                Fyf = -self.nn_scale * (Fys_all[0] + Fys_all[1])
                Fyr = -self.nn_scale * (Fys_all[2] + Fys_all[3])
                Fx_traction = 2.0 * (Fxs_all[4] + Fxs_all[5])
            else:
                a_vec = ca.vertcat(alpha_f, alpha_r, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                fz_vec = ca.vertcat(Fz_f_mean, Fz_r_mean, Fz_f_mean, Fz_r_mean,
                                    Fz_f_mean, Fz_r_mean, Fz_f_mean, Fz_r_mean)
                u_vec = ca.repmat(u_safe, B, 1)
                k_vec = ca.vertcat(kappa, kappa, kappa_ref, kappa_ref, 0.0, 0.0, 0.0, 0.0)
                n_vec = ca.repmat(n_terrain_val, B, 1)
                sr_vec = ca.vertcat(sr_meas, sr_meas, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                dk_vec = ca.vertcat(rates_front[0], rates_rear[0], rates_front[0], rates_rear[0],
                                    0.0, 0.0, 0.0, 0.0)
                da_vec = ca.vertcat(rates_front[1], rates_rear[1], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                du_vec = ca.vertcat(rates_front[2], rates_rear[2], rates_front[2], rates_rear[2],
                                    0.0, 0.0, 0.0, 0.0)

                Fxs_all, Fys_all = nn.predict_batch_rate(
                    a_vec, fz_vec, u_vec, k_vec, n_vec, sr_vec,
                    dk_vec, da_vec, du_vec, Kphi_sym, Kc_sym, c_sym, phi_sym, k_sym)
                Fyf = -self.nn_scale * 2.0 * Fys_all[0]
                Fyr = -self.nn_scale * 2.0 * Fys_all[1]
                Fx_traction = 2.0 * (Fxs_all[2] + Fxs_all[3])

        elif nn is not None:
            # --- Static batched (MLP or ResNet) ---
            B = nn._BATCH

            if self.lateral_load_transfer:
                a_vec = ca.vertcat(alpha_f, alpha_f, alpha_r, alpha_r, 0.0, 0.0, 0.0, 0.0)
                fz_vec = ca.vertcat(Fz_f_outer, Fz_f_inner, Fz_r_outer, Fz_r_inner,
                                    Fz_f_mean, Fz_r_mean, Fz_f_mean, Fz_r_mean)
                u_vec = ca.repmat(u_safe, B, 1)
                k_vec = ca.vertcat(kappa, kappa, kappa, kappa,
                                   kappa_ref, kappa_ref, kappa_ref, kappa_ref)
                n_vec = ca.repmat(n_terrain_val, B, 1)
                sr_vec = ca.vertcat(sr_meas, sr_meas, sr_meas, sr_meas, 0.0, 0.0, 0.0, 0.0)

                Fxs_all, Fys_all = nn.predict_batch(
                    a_vec, fz_vec, u_vec, k_vec, n_vec, sr_vec,
                    Kphi_sym, Kc_sym, c_sym, phi_sym, k_sym)

                Fyf = -self.nn_scale * (Fys_all[0] + Fys_all[1])
                Fyr = -self.nn_scale * (Fys_all[2] + Fys_all[3])
                Fx_traction = 2.0 * (Fxs_all[4] + Fxs_all[5])
            else:
                a_vec = ca.vertcat(alpha_f, alpha_r, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                fz_vec = ca.vertcat(Fz_f_mean, Fz_r_mean, Fz_f_mean, Fz_r_mean,
                                    Fz_f_mean, Fz_r_mean, Fz_f_mean, Fz_r_mean)
                u_vec = ca.repmat(u_safe, B, 1)
                k_vec = ca.vertcat(kappa, kappa, kappa_ref, kappa_ref, 0.0, 0.0, 0.0, 0.0)
                n_vec = ca.repmat(n_terrain_val, B, 1)
                sr_vec = ca.vertcat(sr_meas, sr_meas, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

                Fxs_all, Fys_all = nn.predict_batch(
                    a_vec, fz_vec, u_vec, k_vec, n_vec, sr_vec,
                    Kphi_sym, Kc_sym, c_sym, phi_sym, k_sym)

                Fyf = -self.nn_scale * 2.0 * Fys_all[0]
                Fyr = -self.nn_scale * 2.0 * Fys_all[1]
                Fx_traction = 2.0 * (Fxs_all[2] + Fxs_all[3])
        else:
            # Analytical tire model (pacejka / tmeasy / linear)
            Fyf, Fyr, Fx_traction = _analytical_tire_forces(
                self.tire_model, alpha_f, alpha_r, Fz_f_axle, Fz_r_axle, kappa)

        # ---- Dynamics: ẋ = f(x, u) ----
        # δ_prev and Jx_prev follow (u - u_prev)/dt so the stage cost can
        # penalise Δδ and ΔJx consistently across the horizon.
        _dt = self.dt
        f_expl = ca.vertcat(
            u_vel * ca.cos(psi) - (v_vel + Lf * omega) * ca.sin(psi),  # ẋ
            u_vel * ca.sin(psi) + (v_vel + Lf * omega) * ca.cos(psi),  # ẏ
            omega,                                                       # ψ̇
            ax,                                                          # u̇
            (Fyf + Fyr) / M - u_vel * omega,                            # v̇
            (Fyf * Lf - Fyr * Lr) / Izz,                                # ω̇
            Jx,                                                          # ȧx
            (delta_cmd - delta_prev) / _dt,                              # δ_prev
            (Jx - Jx_prev) / _dt,                                        # Jx_prev
        )

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

        stage_cost = (
            self.w_delta_dot / _ct * d_delta**2 +
            self.w_Jx * _ct * Jx**2 +
            self.w_du * d_jx**2 +
            self.w_steer * delta_cmd**2 +
            self.w_lateral * (py - y_ref)**2 +
            self.w_heading * (psi - psi_ref)**2 +
            self.w_speed * (u_vel - v_ref)**2
        )
        model.cost_expr_ext_cost = stage_cost

        # ---- Terminal cost ----
        # At terminal stage the refs are repurposed: y_ref = y_goal, psi_ref = psi_goal
        terminal_cost = (
            self.w_terminal * (py - y_ref)**2 +
            self.w_terminal * 0.5 * (psi - psi_ref)**2
        )
        model.cost_expr_ext_cost_e = terminal_cost

        # ---- Traction constraints: h(x,u,p) ≤ 0 ----
        # [M*ax - Fx_traction, -M*ax - |Fx_traction|]
        # Only apply for NN models where traction budget is learned info
        # not embedded in the dynamics.  For analytical models (Pacejka,
        # TMeasy, linear), traction is implicit in the tire force curves
        # and qpOASES cannot handle one-sided nonlinear constraints —
        # it declares INFEASIBLE on every solve.
        if self.use_nn:
            traction_h = ca.vertcat(
                M * ax - Fx_traction,
                -M * ax - ca.fabs(Fx_traction),
            )
            model.con_h_expr = traction_h

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
            xk, yk, psik, uk, vk, omegak, axk, dprev, _jxprev = zk
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
            u_next = max(self.u_min, uk + _dt * axk)
            v_next = vk * 0.9
            omega_next = omegak * 0.9
            psi_next = psik + _dt * omegak
            x_next = xk + _dt * (uk * np.cos(psik) - vk * np.sin(psik))
            y_next = yk + _dt * (uk * np.sin(psik) + vk * np.cos(psik))
            zk = np.array([x_next, y_next, psi_next, u_next,
                           v_next, omega_next, ax_next, delta_k, jerk_k])
        self._solver.set(N, 'x', zk)

    # ------------------------------------------------------------------
    # Solve
    # ------------------------------------------------------------------

    def solve(self, z0, x_ref, y_ref, psi_ref, v_ref,
              x_goal, y_goal, psi_goal,
              n_terrain=None, sr_meas=0.0,
              terrain_params=None,
              hist_front=None, hist_rear=None,
              rates_front=None, rates_rear=None):
        """
        Solve the MPC.

        Returns:
            delta_cmd, Jx, Z_opt, U_opt  — first control is road-wheel angle δ [rad]
        """
        nx, nu, N = self.nx, self.nu, self.N
        nn = self.nn_tire_model

        # Resolve terrain params
        if terrain_params is not None:
            tp = terrain_params
        elif nn is not None:
            tp = nn._terrain_nominals
        else:
            tp = {'Kphi': 0, 'Kc': 0, 'c': 0, 'phi': 0, 'k': 0}

        model_fmt = nn.model_format if nn else 'v6'
        if model_fmt in ('v6', 'v6_temporal', 'v8_rate'):
            phi_val = np.radians(tp['phi'])
        else:
            phi_val = tp['phi']

        terrain_vec = [tp['Kphi'], tp['Kc'], tp['c'], phi_val, tp['k']]

        # Set initial state
        self._solver.set(0, 'lbx', z0)
        self._solver.set(0, 'ubx', z0)

        # Build per-stage parameter vectors
        _p_base = []
        for k in range(N):
            p_k = list(terrain_vec) + [y_ref[k], psi_ref[k], v_ref[k], sr_meas]
            if self._temporal_mode:
                K = nn.temporal_K
                h_dim = (K - 1) * 5
                hf = hist_front if hist_front is not None else np.zeros(h_dim)
                hr = hist_rear if hist_rear is not None else np.zeros(h_dim)
                p_k += list(hf) + list(hr)
            if self._rate_mode:
                rf = rates_front if rates_front is not None else np.zeros(3)
                rr = rates_rear if rates_rear is not None else np.zeros(3)
                p_k += list(rf) + list(rr)
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

        # Terminal stage parameters: y_ref = y_goal, psi_ref = psi_goal
        p_e = list(terrain_vec) + [y_goal, psi_goal, v_ref[-1], sr_meas]
        if self._temporal_mode:
            K = nn.temporal_K
            h_dim = (K - 1) * 5
            hf = hist_front if hist_front is not None else np.zeros(h_dim)
            hr = hist_rear if hist_rear is not None else np.zeros(h_dim)
            p_e += list(hf) + list(hr)
        if self._rate_mode:
            rf = rates_front if rates_front is not None else np.zeros(3)
            rr = rates_rear if rates_rear is not None else np.zeros(3)
            p_e += list(rf) + list(rr)

        self._solver.set(N, 'p', np.array(p_e))

        # Solve
        status = self._solver.solve()

        # On QP failure (status 4) with warm-start, retry from kinematic
        # rollout.  The warm-start trajectory from a previously failed QP
        # is often the root cause of cascading failures.
        if status == 4 and _have_prev:
            self._init_kinematic_rollout(z0, x_ref, y_ref, psi_ref, v_ref)
            status = self._solver.solve()

        self.last_solver_status = str(status)

        # Extract solution
        Z_opt = np.zeros((nx, N + 1))
        U_opt = np.zeros((nu, N))
        for k in range(N):
            Z_opt[:, k] = self._solver.get(k, 'x')
            U_opt[:, k] = self._solver.get(k, 'u')
        Z_opt[:, N] = self._solver.get(N, 'x')

        # Cache for warm-start.  On QP failure (status 4), purge the
        # cache so the next call uses a kinematic rollout instead of
        # propagating a poor solution that causes more QP failures.
        if status == 4:
            self._prev_Z = None
            self._prev_U = None
        else:
            self._prev_Z = Z_opt.copy()
            self._prev_U = U_opt.copy()

        delta_cmd = float(U_opt[0, 0])
        Jx = float(U_opt[1, 0])
        self._last_u0 = U_opt[:, 0].copy()

        # Get solver stats
        try:
            self.last_iter_count = int(self._solver.get_stats('sqp_iter'))
        except Exception:
            self.last_iter_count = 1
        try:
            self.last_cost = float(self._solver.get_cost())
        except Exception:
            self.last_cost = float('nan')

        # Only reject solutions with actual NaN/Inf -- ACADOS RTI commonly
        # returns non-zero status (e.g. max-iterations) with a usable solution.
        if (not np.isfinite(delta_cmd) or not np.isfinite(Jx) or
                not np.isfinite(Z_opt).all() or not np.isfinite(U_opt).all()):
            self.reset_warmstart()
            return 0.0, 0.0, None, None

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
    mpc = AcadosMPC(nn_tire_model=nn, dt=0.1, N=20)

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
