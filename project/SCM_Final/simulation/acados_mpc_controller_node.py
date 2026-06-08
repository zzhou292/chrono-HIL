#!/usr/bin/env python3
"""
ACADOS MPC Controller Node (Decoupled)
========================================

ACADOS SQP-RTI MPC controller node (this module implements the
decoupled controller; the repo does not ship acados_mpc_controller.py).

All helper classes (DelayEstimator, StatePredictor, ControlIntegrator,
TrackingAnalytics, TireHistoryTracker, RateTracker) are imported from
mpc_helpers module to avoid code duplication.

NN tire models are loaded via the unified nn_tire_model.py interface which
auto-detects model type from the checkpoint.

Subscribes: VehicleState from simulation node
Publishes:  ControlCommand to simulation node

Usage:
    python acados_mpc_controller_node.py --nn-model vehicle_rate_64_32_lhs --terrain sand --path sinusoidal
    python acados_mpc_controller_node.py --model pacejka --terrain dirt --path lane_change
"""

import argparse
import collections
import csv
import json
import math
import os
import sys
import tempfile
from typing import Optional, Tuple
import time as wall_time
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from hil_messages import (
    VehicleState, ControlCommand, SimStatus,
    ZMQPublisher, ZMQSubscriber,
    ctrl_pub_endpoint, sim_sub_endpoint,
)
from param_consistency import (
    get_vehicle_params_for_demo, get_terrain_preset,
    terrain_preset_to_internal,
    TERRAIN_PRESETS,
    STANDARD_GRAVITY_M_S2,
)

# Reuse helper classes from mpc_helpers (shared module)
from mpc_helpers import (
    DelayEstimator,
    StatePredictor,
    quat_to_yaw,
    ControlIntegrator,
    TrackingAnalytics,
    TireHistoryTracker,
    RateTracker,
    GRUHiddenTracker,
)

# ACADOS solver + unified NN loader
from acados_mpc_solver import (
    AcadosMPC,
    DEFAULT_MPC_DT,
    DEFAULT_MPC_HORIZON_STEPS,
)
from nn_tire_model import load_nn_tire_model
from analytical_tire_models import get_tire_forces as analytical_tire_forces
from learned_terrain_estimator import (
    BlendedLearnedTerrainEstimator,
    HybridJointLearnedTerrainEstimator,
    LearnedTerrainEstimator,
)

# Terrain classifier (optional)
try:
    from terrain_classifier.messages import (
        TerrainEstimate, terrain_sub_endpoint,
    )
    TERRAIN_CLASSIFIER_AVAILABLE = True
except ImportError:
    TERRAIN_CLASSIFIER_AVAILABLE = False

# Lookup tables (same as original controller node)
_TERRAIN_N_LOOKUP = {name: preset["n"] for name, preset in TERRAIN_PRESETS.items()}
_TERRAIN_PARAMS_LOOKUP = {
    name: terrain_preset_to_internal(preset)
    for name, preset in TERRAIN_PRESETS.items()
}
_DEFAULT_TERRAIN_CLASS = "clay"

# Subset of terrain_preset_to_internal keys passed into the OCP each stage
TERRAIN_MPC_PARAM_KEYS = ("Kphi", "Kc", "n", "c", "phi", "k")

# Minimum forward speed represented in MPC state (physical bound).
MPC_STATE_MIN_FORWARD_SPEED_MPS = 0.0
# Speed epsilon used only in slip-angle/rate feature computations.
SLIP_CALC_MIN_SPEED_MPS = 0.5

from path_utils import make_path_function
from tire_input_features import (
    VehicleGeometry,
    compute_bicycle_operating_point,
    fz_with_lateral_transfer,
    kappa_from_wheel_pair,
    kappa_from_wheel_speed,
    lateral_load_transfer_dFz,
    pack_rich_vehicle_tire_csv_row,
    pack_vehicle_tire_csv_row,
    write_rich_vehicle_tire_csv_header,
    write_vehicle_tire_csv_header,
)


def _resolve_project_path(path_like: str) -> Path:
    """Resolve relative runtime artifact paths from the project root."""
    path = Path(path_like).expanduser()
    if path.is_absolute():
        return path.resolve()
    return (Path(__file__).resolve().parent.parent / path).resolve()


def _config_dict_from_cli(args) -> dict:
    """Authoritative bootstrap for ACADOS (do not block on sim)."""
    vehicle_params = get_vehicle_params_for_demo()
    # ``--controller-prior-terrain`` (when set) decouples the controller's
    # assumed terrain from the plant terrain ``--terrain``.  This lets us
    # exercise the wrong-prior case: run the plant on, say, sand while the
    # controller's static prior is dirt.  When unset (default), the
    # controller's prior tracks the plant terrain (the matched/oracle case).
    prior_name = getattr(args, "controller_prior_terrain", None) or args.terrain
    tp = get_terrain_preset(prior_name)
    terrain_params = terrain_preset_to_internal(tp)
    return {
        "vehicle_params": vehicle_params,
        "terrain_params": terrain_params,
        "terrain_preset": prior_name,
        "path_type": args.path,
        "v_target": args.speed,
        "sim_time": args.time,
        "sine_amplitude": args.sine_amplitude,
        "sine_wavelength": args.sine_wavelength,
        "lead_in": args.lead_in,
    }


def _unpack_run_config(config: dict, args) -> tuple:
    vehicle_params = config["vehicle_params"]
    terrain_params = config["terrain_params"]
    v_target = float(config.get("v_target", args.speed))
    path_type = config.get("path_type", args.path)
    sine_amp = float(config.get("sine_amplitude", args.sine_amplitude))
    sine_wl = float(config.get("sine_wavelength", args.sine_wavelength))
    lead_in = float(config.get("lead_in", args.lead_in))
    terrain_name = config.get("terrain_preset", args.terrain)
    return (
        vehicle_params,
        terrain_params,
        v_target,
        path_type,
        sine_amp,
        sine_wl,
        lead_in,
        terrain_name,
    )


def _drain_latest_sim_config(state_sub) -> Optional[dict]:
    """Non-blocking: return the most recent SimStatus config payload in the queue, if any."""
    latest = None
    while True:
        result = state_sub.recv(timeout_ms=0)
        if result is None:
            break
        _, msg = result
        if isinstance(msg, SimStatus) and msg.event == "config" and msg.config:
            latest = msg.config
    return latest


def _send_ready_control_ping(ctrl_pub, integrator: ControlIntegrator) -> None:
    """Neutral command so chrono_sim_node can exit --wait-for-controller before VehicleState exists."""
    cmd = ControlCommand(
        time=0.0,
        wall_time=wall_time.time(),
        seq=0,
        steering=0.0,
        throttle=0.0,
        braking=0.0,
        delta=float(integrator.steering_angle),
        acceleration=float(integrator.acceleration),
        delta_dot=0.0,
        jerk=0.0,
        solve_time_ms=0.0,
        mpc_cost=0.0,
    )
    ctrl_pub.send(cmd)


def _acados_build_directory(args, tire_model: str, nn_model_id: Optional[str]) -> Path:
    """Resolve ACADOS codegen/cache directory (CLI > env > temp root)."""
    if args.acados_build_dir:
        return Path(args.acados_build_dir).expanduser().resolve()
    root = Path(os.environ.get("ACADOS_MPC_BUILD_ROOT", tempfile.gettempdir()))
    tag = (nn_model_id or tire_model).replace("/", "_")
    # NOTE: intentionally NOT keyed by os.getpid().  The controller is a fresh
    # subprocess per sweep run, so a PID-keyed build dir defeats the acados
    # codegen cache and forces a ~60 s recompile on every run.  The dir is
    # shared per-model and the solver validates a fingerprint + holds a build
    # lock, so concurrent workers reuse the compiled .so safely.
    return (root / f"acados_mpc_{tag}").resolve()


def _terrain_estimate_bundle(
    terrain_classifier: bool,
    use_prediction: bool,
    terrain_name: str,
    terrain_params: dict,
) -> Tuple[str, float, dict, float]:
    """Class name, Bekker n, terrain dict for OCP, classifier confidence."""
    if terrain_classifier and use_prediction:
        return (
            _DEFAULT_TERRAIN_CLASS,
            float(_TERRAIN_N_LOOKUP[_DEFAULT_TERRAIN_CLASS]),
            dict(_TERRAIN_PARAMS_LOOKUP[_DEFAULT_TERRAIN_CLASS]),
            0.0,
        )
    return (
        terrain_name,
        float(terrain_params.get("n", 1.1)),
        {k: terrain_params[k] for k in TERRAIN_MPC_PARAM_KEYS},
        1.0,
    )


def _wrap_to_pi(angle_rad: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (angle_rad + np.pi) % (2.0 * np.pi) - np.pi


# =============================================================================
# Main controller loop
# =============================================================================

def run_controller_node(args):
    print("=" * 60)
    print("ACADOS MPC Controller Node (Decoupled)")
    print("=" * 60)

    # ------------------------------------------------------------------
    # ZMQ: subscribe first; ACADOS builds from CLI without waiting for sim.
    # If chrono already published config, we merge it after warmup (queue drain).
    # ------------------------------------------------------------------
    state_sub = ZMQSubscriber(sim_sub_endpoint(args.sim_host, args.sim_port))
    print(f"  Subscribing to state from {args.sim_host}:{args.sim_port}")
    print("  ACADOS will compile from CLI now (no wait for sim config).")

    config = _config_dict_from_cli(args)
    (
        vehicle_params,
        terrain_params,
        v_target,
        path_type,
        sine_amp,
        sine_wl,
        lead_in,
        terrain_name,
    ) = _unpack_run_config(config, args)

    # ------------------------------------------------------------------
    # Build MPC (ACADOS)
    # ------------------------------------------------------------------
    dt_mpc = float(args.mpc_dt)
    N_horizon = int(args.mpc_n)
    tire_model = args.model

    # Load NN tire model (only for nn mode)
    nn_tire = None
    nn_model_version: Optional[str] = None
    if tire_model == 'nn':
        base_path = Path(__file__).parent.parent
        nn_model_version = args.nn_model
        requested_model = Path(nn_model_version).expanduser()
        if requested_model.is_absolute() or len(requested_model.parts) > 1:
            model_dir = requested_model if requested_model.is_absolute() else base_path / requested_model
        else:
            model_dir = base_path / "nn_models" / nn_model_version
        if not model_dir.exists():
            print(f"  ERROR: NN model directory not found: {model_dir}")
            sys.exit(1)

        nn_tire = load_nn_tire_model(str(model_dir), terrain_params)
        print(f"  NN model loaded: {nn_tire.model_type} "
              f"(input_dim={nn_tire.input_dim}, params={nn_tire.n_params})")

    acados_build_dir = _acados_build_directory(args, tire_model, nn_model_version)

    # Solver uses symbolic ax-based kappa for the prediction horizon;
    # 'measured' mode only affects the controller's operating point.
    _solver_kappa = 'approx' if args.kappa == 'measured' else args.kappa

    mpc = AcadosMPC(
        nn_tire_model=nn_tire,
        dt=dt_mpc,
        N=N_horizon,
        lateral_load_transfer=not args.no_lat_transfer,
        kappa_mode=_solver_kappa,
        tire_model=tire_model,
        build_dir=acados_build_dir,
        symbolic_rates=args.symbolic_rates,
        no_temporal_staged=getattr(args, 'no_temporal_staged', False),
        friction_angle_deg=terrain_params.get('phi'),
        rate_feature_dt=float(args.nn_rate_sample_dt),
        oracle_terrain=(terrain_name if tire_model == 'pacejka-oracle' else None),
        speed_weight=float(args.speed_weight),
        speed_cost_mode=args.speed_cost_mode,
        obstacle_weight=float(args.obstacle_weight),
    )
    if tire_model == 'nn':
        model_label = f"ACADOS-NN ({nn_tire.model_type})"
    else:
        model_label = f"ACADOS-{tire_model}"
    print(f"  MPC built: {model_label}, N={N_horizon}, dt={dt_mpc}s")

    # Temporal history tracker
    tire_hist = None
    if mpc.temporal_mode:
        K_t = nn_tire.temporal_K
        tire_hist = TireHistoryTracker(K_t)
        # Pre-fill with plausible values so the NN doesn't see OOD zeros
        # (zero Fz/u in history causes near-zero force predictions under RTI)
        L = mpc.Lf + mpc.Lr
        Fz_f0 = mpc.M * STANDARD_GRAVITY_M_S2 * mpc.Lr / L / 2.0
        Fz_r0 = mpc.M * STANDARD_GRAVITY_M_S2 * mpc.Lf / L / 2.0
        for _ in range(K_t - 1):
            tire_hist.update(0.0, 0.0, 0.5, Fz_f0, 0.0,
                             0.0, 0.0, Fz_r0, 0.0)
        print(
            f"  Temporal history: K={K_t}, {K_t - 1} past frames, "
            f"push every {args.nn_temporal_hist_dt:g}s sim time (match training dt_nn)"
        )

    # Rate tracker for rate-augmented NN (skip if symbolic rates)
    rate_tracker = None
    if mpc.rate_mode and not mpc._symbolic_rate_mode:
        rate_tracker = RateTracker(sample_dt=args.nn_rate_sample_dt)
        print(
            f"  Rate-augmented NN: dkappa/dt, dalpha/dt, du/dt over "
            f">={args.nn_rate_sample_dt:g}s sim intervals (match train_rate_nn effective_dt)"
        )
    elif mpc._symbolic_rate_mode:
        print("  Symbolic rate mode: rates computed inside MPC dynamics (no RateTracker needed)")

    # GRU hidden-state tracker for GRU observer NN
    gru_tracker = None
    if mpc.gru_mode:
        gru_tracker = GRUHiddenTracker(nn_tire)
        print(f"  GRU observer: h_dim={nn_tire.gru_h_dim}, stepping every MPC cycle")

    # Warmup ACADOS solver (first solves trigger JIT)
    print("  Warming up ACADOS solver...", end="", flush=True)
    v_warm = float(
        np.clip(
            max(MPC_STATE_MIN_FORWARD_SPEED_MPS, v_target),
            mpc.u_min,
            mpc.u_max,
        )
    )
    z0_warm = np.zeros(mpc.nx)
    z0_warm[3] = v_warm
    x_ref_w = np.linspace(0, v_warm * dt_mpc * N_horizon, N_horizon + 1)
    y_ref_w = np.zeros(N_horizon + 1)
    psi_ref_w = np.zeros(N_horizon + 1)
    v_ref_w = v_warm * np.ones(N_horizon + 1)
    for _ in range(int(args.warmup_iters)):
        warm_kwargs = dict(terrain_params=terrain_params)
        if tire_hist is not None:
            warm_kwargs['hist_front'] = tire_hist.front
            warm_kwargs['hist_rear'] = tire_hist.rear
        if gru_tracker is not None:
            warm_kwargs['gru_h_front'] = gru_tracker.front
            warm_kwargs['gru_h_rear'] = gru_tracker.rear
        mpc.solve(z0_warm, x_ref_w, y_ref_w, psi_ref_w, v_ref_w, x_ref_w[-1], 0, 0,
                  **warm_kwargs)
        print(".", end="", flush=True)
    mpc.reset_warmstart()  # force kinematic rollout from real z0
    print(" done!")

    # Merge sim-published config if it is already in the ZMQ queue (sim started first).
    sim_cfg = _drain_latest_sim_config(state_sub)
    if sim_cfg:
        # When the user explicitly fixes the controller's terrain prior via
        # --controller-prior-terrain, do NOT let the sim's broadcast config
        # silently overwrite it (the sim publishes the plant terrain, which
        # would collapse the wrong-prior ablation back into matched-prior).
        if getattr(args, "controller_prior_terrain", None):
            sim_cfg = {k: v for k, v in sim_cfg.items()
                       if k not in ("terrain_params", "terrain_preset")}
        config.update(sim_cfg)
        (
            vehicle_params,
            terrain_params,
            v_target,
            path_type,
            sine_amp,
            sine_wl,
            lead_in,
            terrain_name,
        ) = _unpack_run_config(config, args)
        print("  Merged sim config from ZMQ queue (chrono published before/during ACADOS init).")
    else:
        print("  No sim config in queue yet — using CLI. Match --terrain/--path/--speed/--lead-in to chrono, "
              "or start sim before controller so config is buffered.")

    # If classifier estimates are consumed by MPC, start from conservative default.
    if args.use_prediction:
        default_tp = get_terrain_preset(_DEFAULT_TERRAIN_CLASS)
        terrain_params = terrain_preset_to_internal(default_tp)
        print(f"  Terrain: classifier prediction mode "
              f"(default={_DEFAULT_TERRAIN_CLASS}, GT={terrain_name} hidden from MPC)")
    elif args.terrain_classifier:
        print(f"  Terrain: {terrain_name} (classifier telemetry only)")
    else:
        print(f"  Terrain: {terrain_name}")
    print(f"  Path: {path_type}, v_target: {v_target} m/s")
    if lead_in > 0:
        print(f"  Lead-in: {lead_in:.0f}m straight before path")

    # ------------------------------------------------------------------
    # Timestamped run directory
    # ------------------------------------------------------------------
    model_tag = f"acados_{nn_tire.model_type}" if nn_tire is not None else f"acados_{tire_model}"
    run_ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    run_dir = Path(args.plot_dir) / f"{run_ts}_{terrain_name}_{path_type}_{model_tag}"
    run_dir.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------
    # Reference path
    # ------------------------------------------------------------------
    ref_path = make_path_function(
        path_type=path_type,
        v_target=v_target,
        sine_amplitude=sine_amp,
        sine_wavelength=sine_wl,
        use_closest_point=not args.no_path_reindex,
        lead_in=lead_in,
        csv_dir=str(run_dir),
        friction_angle_deg=terrain_params.get('phi'),
        ay_safety=float(getattr(args, 'ay_safety', 0.65)),
    )
    path_func = ref_path.get_reference

    # ------------------------------------------------------------------
    # Live debug plotter (optional)
    # ------------------------------------------------------------------
    _live_plotter = None
    if args.live_plot:
        from live_debug_plotter import LiveDebugPlotter
        _live_plotter = LiveDebugPlotter(ref_path, update_every=args.live_plot_every)
        print("  Live debug plotter: ON")

    # ------------------------------------------------------------------
    # Transport delay compensation
    # ------------------------------------------------------------------
    delay_est = DelayEstimator(initial_delay=args.initial_delay)
    state_predictor = StatePredictor(vehicle_params, dt_prop=args.state_predict_dt)
    control_buffer = collections.deque(maxlen=int(args.control_buffer_len))

    # ------------------------------------------------------------------
    # Control integrator
    # ------------------------------------------------------------------
    integrator = ControlIntegrator(
        mpc, v_target=v_target,
        dob_ki=float(getattr(args, "dob_ki", 0.15)),
        dob_max=float(getattr(args, "dob_max", 0.35)),
        dob_bleed=float(getattr(args, "dob_bleed", 0.5)),
    )

    # ------------------------------------------------------------------
    # Tracking analytics
    # ------------------------------------------------------------------
    analytics = TrackingAnalytics(
        ref_path=ref_path,
        v_target=v_target,
        rms_time_start=args.rms_time_start,
        path_type=path_type,
    )

    # ------------------------------------------------------------------
    # Publisher for control commands
    # ------------------------------------------------------------------
    ctrl_pub = ZMQPublisher(ctrl_pub_endpoint(args.ctrl_port))
    print(f"  Publishing controls on port {args.ctrl_port}")

    # ------------------------------------------------------------------
    # Terrain classifier subscription (optional)
    # ------------------------------------------------------------------
    terrain_sub = None
    if args.terrain_classifier and TERRAIN_CLASSIFIER_AVAILABLE:
        tc_endpoint = terrain_sub_endpoint(args.sim_host, args.tc_port)
        terrain_sub = ZMQSubscriber(tc_endpoint)
        print(f"  Subscribing to terrain classifier on {tc_endpoint}")
    elif args.terrain_classifier and not TERRAIN_CLASSIFIER_AVAILABLE:
        print("  WARNING: --terrain-classifier requested but module not available")

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    seq = 0
    last_state: VehicleState = None
    solve_times = []
    last_sim_time = None
    # Pending prediction targets: (sim_time_target, predicted 9-state at target).
    pred_targets = collections.deque(maxlen=max(64, 4 * int(mpc.N)))
    pred_pos_err_hist = []
    pred_psi_err_hist = []
    pred_u_err_hist = []
    pred_v_err_hist = []
    pred_omega_err_hist = []
    pred_age_hist = []
    # Previous applied road-wheel angle used to estimate realized steering-rate
    # at the current state sample.
    prev_applied_delta = float(integrator.steering_angle)
    # Effective δ̇ for delay predictor buffer and NN steering_rate feature.
    last_delta_dot_cmd = 0.0
    last_Jx_cmd = 0.0
    # Measured-ax state (from IMU, complementary-filtered)
    _measured_ax = 0.0
    _ax_filter_tau = args.ax_filter_tau
    # Velocity state EMA filter — smooths noisy [u, v, omega] before MPC and GP observations.
    # The main source of noise corruption: σ_v=0.05 m/s vs signal ~0.1-0.5 m/s (SNR 2-10:1),
    # and dynamics GP targets are (z_meas - z_pred)/dt which amplifies noise by ~1/dt.
    _vel_filter_tau = float(getattr(args, 'vel_filter_tau', 0.0))
    _vel_filt_u: Optional[float] = None
    _vel_filt_v: Optional[float] = None
    _vel_filt_omega: Optional[float] = None
    terrain_class_est, n_terrain_est, terrain_params_est, terrain_confidence = (
        _terrain_estimate_bundle(
            args.terrain_classifier,
            args.use_prediction,
            terrain_name,
            terrain_params,
        )
    )

    res_pred_du = 0.0
    res_pred_dv = 0.0
    res_pred_domega = 0.0

    # ------------------------------------------------------------------
    # Online terrain parameter estimator (speed-capability voting)
    # ------------------------------------------------------------------
    terrain_estimator = None
    _te_omega_prev = None
    _te_time_prev = None
    # Cache of the most recent live terrain estimate. Every ControlCommand
    # carries this snapshot so the sim-side safety shield can re-condition
    # its NN surrogate and tighten the friction-cone gate by sigma_phi.
    latest_terrain_update = {
        "seq": 0, "n": None, "phi_deg": None, "phi_sigma_deg": None,
        "Kphi": None, "Kc": None, "c": None, "k": None,
        "terrain_class": None, "confidence": None,
    }
    if args.terrain_estimator and args.model == "nn":
        # Start estimator from a NEUTRAL default — midpoint of the n range.
        # Use dirt preset as base (n=0.7, middle of [0.3, 1.3]).
        # This follows Dallas et al. who initialize with a "wrong" but not
        # extreme initial guess.
        _te_default = terrain_preset_to_internal(get_terrain_preset("dirt"))
        if getattr(args, "terrain_estimator_backend", "learned") == "fused":
            # Regime-aware fusion of the window-MLP and the NN-UKF.
            from fused_terrain_estimator import FusedTerrainEstimator
            _fy_dir = (str(Path(args.learned_terrain_model_dir).resolve())
                       if args.learned_terrain_model_dir else None)
            terrain_estimator = FusedTerrainEstimator(
                initial_terrain=_te_default,
                update_interval=args.te_update_interval,
                verbose=bool(getattr(args, "te_verbose", False)),
                fy_model_dir=_fy_dir, q_n=float(getattr(args, "nn_ukf_q_n", 0.01)))
            _te_src_desc = "backend=fused [MLP+NN-UKF regime blend]"
        elif getattr(args, "terrain_estimator_backend", "learned") == "nn_ukf":
            # Online Dallas-style state-augmented UKF (whole-vehicle Fy surrogate).
            from dallas_ukf_terrain_estimator import DallasUKFTerrainEstimator
            # For nn_ukf, --learned-terrain-model-dir (if given) selects the
            # vehicle_fy surrogate dir (used for the data-scaling study).
            _fy_dir = (str(Path(args.learned_terrain_model_dir).resolve())
                       if args.learned_terrain_model_dir else None)
            terrain_estimator = DallasUKFTerrainEstimator(
                model_dir=_fy_dir,
                initial_terrain=_te_default,
                update_interval=args.te_update_interval,
                verbose=bool(getattr(args, "te_verbose", False)),
                q_n=float(getattr(args, "nn_ukf_q_n", 0.01)),
            )
            _te_src_desc = "backend=nn_ukf [online Dallas UKF]"
        else:
            default_model_name = "terrain_window_mlp"
            learned_dir = (Path(args.learned_terrain_model_dir).resolve()
                           if args.learned_terrain_model_dir else
                           Path(__file__).parent.parent / "nn_models" /
                           default_model_name)
            blend_cfg_path = learned_dir / "blend.json"
            if blend_cfg_path.exists():
                try:
                    with blend_cfg_path.open() as f:
                        blend_cfg = json.load(f)
                except Exception:
                    blend_cfg = {}
                if blend_cfg.get("type") == "hybrid_joint":
                    learned_estimator_cls = HybridJointLearnedTerrainEstimator
                else:
                    learned_estimator_cls = BlendedLearnedTerrainEstimator
            else:
                learned_estimator_cls = LearnedTerrainEstimator
            terrain_estimator = learned_estimator_cls(
                model_dir=str(learned_dir),
                initial_terrain=_te_default,
                update_interval=args.te_update_interval,
                verbose=bool(getattr(args, "te_verbose", False)),
                window_size=args.te_window,
                min_excitation=args.te_min_excitation,
            )
            _te_src_desc = f"model={learned_dir.name}, window={args.te_window}"
        # Override MPC terrain params to the conservative default too,
        # so the MPC starts blind and adapts as the estimator learns.
        terrain_params_est = dict(_te_default)
        n_terrain_est = _te_default['n']
        terrain_class_est = "estimating"
        terrain_confidence = 0.0
        estimator_outputs = ",".join(terrain_estimator.output_names)
        print(
            f"  Terrain estimator: ON  (mode={args.terrain_estimator_mode}, "
            f"outputs={estimator_outputs}, init=dirt/n=0.7, "
            f"update_every={args.te_update_interval}, {_te_src_desc})"
        )
    elif args.terrain_estimator and args.model != "nn":
        print("  WARNING: --terrain-estimator requires --model nn (disabled)")

    # ------------------------------------------------------------------
    # Simple online Fy bias estimator
    # ------------------------------------------------------------------
    # Maintains a signed EMA of (actual − pred) Fy error.
    # The signed bias captures the net directional force correction needed;
    # the EMA's natural lag provides damping on sinusoidal paths.
    _fy_bias_signed_f = 0.0
    _fy_bias_signed_r = 0.0
    _FY_BIAS_ALPHA = 0.10   # EMA smoothing (signed)
    _FY_BIAS_CLIP = 3000.0  # max correction [N]
    _FY_BIAS_MIN_SPEED = 2.0  # only update above this speed

    # ------------------------------------------------------------------
    # Diagnostic CSV logger
    # ------------------------------------------------------------------
    csv_file = None
    csv_writer = None
    csv_path = None
    tire_csv_file = None
    tire_csv_writer = None
    tire_csv_path = None
    tire_csv_rows = 0
    rich_tire_csv_file = None
    rich_tire_csv_writer = None
    rich_tire_csv_path = None
    rich_tire_csv_rows = 0
    if not args.no_csv:
        csv_path = run_dir / f"diag_{terrain_name}_{path_type}_{model_tag}.csv"
        csv_file = open(csv_path, "w", newline="")
        csv_header = [
            "sim_time", "wall_time", "seq",
            "x_fa_meas", "y_fa_meas", "psi_meas", "u_meas", "v_meas", "omega_meas",
            "x_fa_true", "y_fa_true", "psi_true", "u_true",
            "x_fa_comp", "y_fa_comp", "psi_comp", "u_comp", "v_comp", "omega_comp",
            "ax_state", "delta_prev_state",
            "x_ref_0", "y_ref_0", "psi_ref_0", "v_ref_0",
            "delta_dot", "Jx", "mpc_cost", "solver_status", "solver_iters",
            "terrain_class_est", "terrain_confidence", "n_terrain_est",
            "phi_terrain_est_deg", "phi_terrain_estimator_deg",
            "phi_terrain_sigma_deg", "terrain_update_applied",
            "residual_enabled", "residual_du_pred", "residual_dv_pred", "residual_domega_pred",
            "residual_updates", "residual_last_loss", "residual_last_update_ms",
            "steering", "throttle", "braking", "steering_angle", "acceleration",
            "tau_one_way_ms", "tau_solve_ms", "tau_comp_ms", "solve_time_ms",
            "crosstrack_err", "heading_err_deg", "speed_err",
            "pred1_age_s", "pred1_pos_err_m", "pred1_psi_err_deg",
            "pred1_u_err_mps", "pred1_v_err_mps", "pred1_omega_err_radps",
            "actual_Fy_front", "actual_Fy_rear", "pred_Fy_front", "pred_Fy_rear",
            "alpha_f", "alpha_r", "Fz_f_mean", "Fz_r_mean",
            "kappa_diag", "kappa_meas_diag", "sr_diag", "u_safe_diag", "speed_fade_diag",
        ]
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(csv_header)
        print(f"  Diagnostic CSV: {csv_path}")

    if args.log_tire_csv:
        tire_csv_path = Path(args.log_tire_csv).expanduser().resolve()
        tire_csv_path.parent.mkdir(parents=True, exist_ok=True)
        tire_csv_file = open(tire_csv_path, "w", newline="")
        tire_csv_writer = csv.writer(tire_csv_file)
        tire_csv_writer.writerow(write_vehicle_tire_csv_header())
        print(f"  MPC-aligned tire training CSV: {tire_csv_path}")

    if args.log_rich_tire_csv:
        rich_tire_csv_path = Path(args.log_rich_tire_csv).expanduser().resolve()
        rich_tire_csv_path.parent.mkdir(parents=True, exist_ok=True)
        rich_tire_csv_file = open(rich_tire_csv_path, "w", newline="")
        rich_tire_csv_writer = csv.writer(rich_tire_csv_file)
        rich_tire_csv_writer.writerow(write_rich_vehicle_tire_csv_header())
        print(f"  Rich sensor-realistic tire training CSV: {rich_tire_csv_path}")

    tire_geom = VehicleGeometry(
        Lf=float(mpc.Lf),
        Lr=float(mpc.Lr),
        M=float(mpc.M),
        h_cg=float(mpc.h_cg),
        T=float(mpc.T),
    )

    print(f"  Delay compensation: {'ON' if not args.no_delay_comp else 'OFF'} "
          f"(initial τ={args.initial_delay * 1000:.0f}ms)")
    print(f"  Running ACADOS controller loop...")
    print("  ACADOS init complete — sending ready signal(s) to sim (then waiting for VehicleState)...")

    # Unblock chrono_sim_node --wait-for-controller before any VehicleState is published.
    for _ in range(3):
        _send_ready_control_ping(ctrl_pub, integrator)
        wall_time.sleep(0.05)

    running = True
    last_ready_ping = wall_time.time()
    ready_ping_interval_s = float(args.ready_ping_interval_s)
    last_tire_hist_time = None  # sim time of last temporal history push
    while running:
        now = wall_time.time()
        if last_state is None and now - last_ready_ping >= ready_ping_interval_s:
            _send_ready_control_ping(ctrl_pub, integrator)
            last_ready_ping = now

        # --- Receive state ---
        result = state_sub.recv(timeout_ms=int(args.zmq_recv_timeout_ms))
        if result is None:
            continue

        topic, msg = result

        if isinstance(msg, SimStatus):
            if msg.event == "stop":
                print("  Received stop signal from sim node.")
                running = False
                break
            if (
                msg.event == "config"
                and msg.config
                and last_state is None
            ):
                # Sim published config after our post-warmup drain (e.g. very fast ACADOS cache).
                sim_payload = msg.config
                if getattr(args, "controller_prior_terrain", None):
                    sim_payload = {k: v for k, v in sim_payload.items()
                                   if k not in ("terrain_params", "terrain_preset")}
                config.update(sim_payload)
                (
                    vehicle_params,
                    terrain_params,
                    v_target,
                    path_type,
                    sine_amp,
                    sine_wl,
                    lead_in,
                    terrain_name,
                ) = _unpack_run_config(config, args)
                state_predictor = StatePredictor(vehicle_params, dt_prop=args.state_predict_dt)
                ref_path = make_path_function(
                    path_type=path_type,
                    v_target=v_target,
                    sine_amplitude=sine_amp,
                    sine_wavelength=sine_wl,
                    use_closest_point=not args.no_path_reindex,
                    lead_in=lead_in,
                    csv_dir=str(run_dir),
                    friction_angle_deg=terrain_params.get('phi'),
                    ay_safety=float(getattr(args, 'ay_safety', 0.65)),
                )
                path_func = ref_path.get_reference
                analytics.ref_path = ref_path
                analytics.v_target = v_target
                analytics.path_type = path_type
                integrator.v_target = v_target
                if not args.terrain_estimator:
                    # Only use ground-truth terrain when estimator is OFF.
                    # When estimator is ON, keep the conservative init (clay)
                    # and let the estimator discover the terrain online.
                    terrain_class_est, n_terrain_est, terrain_params_est, terrain_confidence = (
                        _terrain_estimate_bundle(
                            args.terrain_classifier,
                            args.use_prediction,
                            terrain_name,
                            terrain_params,
                        )
                    )
                print("  Applied sim config from stream (received after ACADOS init).")
            continue

        if not isinstance(msg, VehicleState):
            continue

        last_state = msg
        recv_time = wall_time.time()
        if last_sim_time is None:
            dt_ctrl = dt_mpc
        else:
            dt_ctrl = float(np.clip(msg.time - last_sim_time, 1e-3, 0.2))
        last_sim_time = msg.time
        terrain_update_applied = 0

        # --- Poll terrain classifier (non-blocking) ---
        if terrain_sub is not None:
            tc_result = terrain_sub.recv(timeout_ms=0)
            if tc_result is not None:
                _, tc_msg = tc_result
                tc_class = None
                tc_conf = 0.0
                if isinstance(tc_msg, TerrainEstimate):
                    tc_class = tc_msg.terrain_class
                    tc_conf = tc_msg.confidence
                elif isinstance(tc_msg, dict) and 'terrain_class' in tc_msg:
                    tc_class = tc_msg['terrain_class']
                    tc_conf = tc_msg.get('confidence', 0.0)
                if tc_class and tc_class in _TERRAIN_N_LOOKUP:
                    terrain_class_est = tc_class
                    terrain_confidence = tc_conf
                    if args.use_prediction and tc_conf >= args.prediction_min_confidence:
                        n_terrain_est = _TERRAIN_N_LOOKUP[tc_class]
                        terrain_params_est = dict(_TERRAIN_PARAMS_LOOKUP[tc_class])
                        terrain_update_applied = 1

        # Update delay estimate
        delay_est.update_transport(msg.wall_time, recv_time)

        # --- Build state vector ---
        psi = quat_to_yaw(msg.quat_e0, msg.quat_e1, msg.quat_e2, msg.quat_e3)
        Lf = mpc.Lf

        # Transform CG → front axle
        x_fa = msg.x_cg + Lf * np.cos(psi)
        y_fa = msg.y_cg + Lf * np.sin(psi)

        # Complementary filter fuses the model prediction (ax + Jx*dt, smooth
        # but drifts) with the IMU reading (noisy but accurate in the mean).
        # High-frequency content comes from the model; DC correction from IMU.
        ax_raw = float(np.clip(msg.ax, mpc.ax_min, mpc.ax_max))
        if _ax_filter_tau > 0 and dt_ctrl > 1e-6:
            alpha = min(dt_ctrl / (_ax_filter_tau + dt_ctrl), 1.0)
            ax_pred = float(np.clip(
                _measured_ax + last_Jx_cmd * dt_ctrl,
                mpc.ax_min, mpc.ax_max))
            _measured_ax = (1.0 - alpha) * ax_pred + alpha * ax_raw
        else:
            _measured_ax = ax_raw

        # MPC state [x,y,ψ,u,v,ω, ax, δ_prev, Jx_prev] (direct δ control)
        ax_for_state = _measured_ax
        z0_measured = np.array([
            x_fa, y_fa, psi,
            max(msg.u, MPC_STATE_MIN_FORWARD_SPEED_MPS),
            msg.v, msg.omega,
            ax_for_state,
            integrator.steering_angle,
            last_Jx_cmd,
        ])

        # Velocity state EMA filter: smooth u, v, omega before MPC and GP observations.
        # Reduces noise corruption in both MPC initial condition and dynamics GP targets.
        if _vel_filter_tau > 0 and dt_ctrl > 1e-6:
            _alpha_vel = min(dt_ctrl / (_vel_filter_tau + dt_ctrl), 1.0)
            if _vel_filt_u is None:
                # First measurement: initialise to raw values
                _vel_filt_u = float(z0_measured[3])
                _vel_filt_v = float(z0_measured[4])
                _vel_filt_omega = float(z0_measured[5])
            else:
                _vel_filt_u = (1.0 - _alpha_vel) * _vel_filt_u + _alpha_vel * float(z0_measured[3])
                _vel_filt_v = (1.0 - _alpha_vel) * _vel_filt_v + _alpha_vel * float(z0_measured[4])
                _vel_filt_omega = (1.0 - _alpha_vel) * _vel_filt_omega + _alpha_vel * float(z0_measured[5])
            z0_measured[3] = _vel_filt_u
            z0_measured[4] = _vel_filt_v
            z0_measured[5] = _vel_filt_omega

        # Append α_f_prev, α_r_prev, δ_sr_prev for symbolic rate mode (nx=12)
        # Use z0_measured[3,4,5] (post-filter) for consistency with the filtered MPC state.
        if mpc._symbolic_rate_mode:
            u_s = max(float(z0_measured[3]), SLIP_CALC_MIN_SPEED_MPS)
            Lr = mpc.Lr
            af = integrator.steering_angle - np.arctan2(float(z0_measured[4]) + Lf * float(z0_measured[5]), u_s)
            ar = -np.arctan2(float(z0_measured[4]) - Lr * float(z0_measured[5]), u_s)
            delta_sr0 = integrator.steering_angle  # sr starts at 0
            z0_measured = np.append(z0_measured, [af, ar, delta_sr0])
        elif mpc._symbolic_sr:
            # δ_sr_prev: set to current δ so that sr ≈ 0 at measurement time
            z0_measured = np.append(z0_measured, [integrator.steering_angle])

        # Append rolling temporal history frames for temporal rolling mode
        # History states are stored in SCALED space (divided by per-feature
        # scale factors) for QP conditioning.
        if mpc._temporal_rolling and tire_hist is not None:
            sc = np.tile(mpc._hist_feat_scale, mpc.nn_tire_model.temporal_K - 1)
            z0_measured = np.append(z0_measured,
                                    np.concatenate([tire_hist.front / sc,
                                                    tire_hist.rear / sc]))

        # Evaluate matured 1-step prediction targets from previous solves.
        pred_age = float("nan")
        pred_pos_err = float("nan")
        pred_psi_err_deg = float("nan")
        pred_u_err = float("nan")
        pred_v_err = float("nan")
        pred_omega_err = float("nan")
        while pred_targets and float(msg.time) >= float(pred_targets[0][0]):
            t_pred, z_pred = pred_targets.popleft()
            z_pred = np.asarray(z_pred, dtype=float).reshape(-1)
            if z_pred.size < 6:
                continue
            pred_age = float(msg.time - t_pred)
            pred_pos_err = float(np.hypot(
                float(z0_measured[0]) - float(z_pred[0]),
                float(z0_measured[1]) - float(z_pred[1]),
            ))
            psi_err = _wrap_to_pi(float(z0_measured[2]) - float(z_pred[2]))
            pred_psi_err_deg = float(np.degrees(psi_err))
            pred_u_err = float(z0_measured[3] - z_pred[3])
            pred_v_err = float(z0_measured[4] - z_pred[4])
            pred_omega_err = float(z0_measured[5] - z_pred[5])

            pred_age_hist.append(pred_age)
            pred_pos_err_hist.append(pred_pos_err)
            pred_psi_err_hist.append(abs(psi_err))
            pred_u_err_hist.append(abs(pred_u_err))
            pred_v_err_hist.append(abs(pred_v_err))
            pred_omega_err_hist.append(abs(pred_omega_err))

        # --- Delay compensation ---
        # StatePredictor: 8-state [x,y,ψ,u,v,ω, δ, ax]; carry Jx_prev through.
        if not args.no_delay_comp:
            tau = delay_est.compensation_delay
            z8 = np.array([
                z0_measured[0], z0_measured[1], z0_measured[2],
                z0_measured[3], z0_measured[4], z0_measured[5],
                z0_measured[7],  # δ
                z0_measured[6],  # ax
            ])
            z8_pred, _, jx_prev_pred = state_predictor.propagate(
                z8,
                control_buffer,
                tau,
                sim_time_s=float(msg.time),
                command_lag_s=float(delay_est.one_way_delay),
                return_last_cmd=True,
            )
            z0 = np.array([
                z8_pred[0], z8_pred[1], z8_pred[2],
                z8_pred[3], z8_pred[4], z8_pred[5],
                z8_pred[7],  # ax
                z8_pred[6],  # δ_prev
                jx_prev_pred,  # Jx_prev at compensated time
            ])
            if mpc._symbolic_rate_mode:
                # Recompute α from predicted state for consistency
                u_s_pred = max(z8_pred[3], SLIP_CALC_MIN_SPEED_MPS)
                af_pred = z8_pred[6] - np.arctan2(z8_pred[4] + Lf * z8_pred[5], u_s_pred)
                ar_pred = -np.arctan2(z8_pred[4] - mpc.Lr * z8_pred[5], u_s_pred)
                delta_sr0_pred = z8_pred[6]  # predicted δ → sr starts at 0
                z0 = np.append(z0, [af_pred, ar_pred, delta_sr0_pred])
            elif mpc._symbolic_sr:
                # δ_sr_prev: set to predicted δ so sr ≈ 0 at compensated time
                z0 = np.append(z0, [z8_pred[6]])
            # Temporal rolling: append same history as measured (delay comp
            # doesn't change the tire history frames).  Scaled space.
            if mpc._temporal_rolling and tire_hist is not None:
                sc = np.tile(mpc._hist_feat_scale, mpc.nn_tire_model.temporal_K - 1)
                z0 = np.append(z0,
                               np.concatenate([tire_hist.front / sc,
                                               tire_hist.rear / sc]))
        else:
            z0 = z0_measured
            tau = 0.0

        res_pred_du = 0.0
        res_pred_dv = 0.0
        res_pred_domega = 0.0

        # --- Parse obstacle positions early (needed for reference modification) ---
        _obs_raw_early = getattr(msg, 'obstacles', None)
        _obs_list_mpc = []
        if _obs_raw_early and len(_obs_raw_early) >= 3:
            for _oi in range(0, len(_obs_raw_early) - 2, 3):
                _ox = float(_obs_raw_early[_oi])
                _oy = float(_obs_raw_early[_oi + 1])
                _or = float(_obs_raw_early[_oi + 2]) + 3.5  # 3.5 m margin (vehicle half-width + clearance + speed buffer)
                _obs_list_mpc.append((_ox, _oy, _or))

        # --- Generate reference trajectory ---
        x_ref, y_ref, psi_ref, v_ref, x_goal, y_goal, psi_goal = path_func(
            msg.time, z0, mpc.N, mpc.dt
        )

        _path_done = ref_path.is_complete(threshold=2.0)
        if _path_done:
            v_ref[:] = 0.0


        # --- Compute per-tire operating conditions (shared tire_input_features.py) ---
        delta_meas_now = float(integrator.steering_angle)
        _terrain_mu = math.tan(math.radians(float(terrain_params_est['phi'])))
        # Measured kappa from wheel speed sensors (available when kappa_mode='measured')
        _meas_kappa = kappa_from_wheel_speed(
            msg.wheel_omega_fl, msg.wheel_omega_fr,
            msg.wheel_omega_rl, msg.wheel_omega_rr,
            msg.u,
        )
        # Use filtered velocities for operating point so derived features
        # are computed from the same noise-smoothed state that MPC receives.
        _u_obs = float(z0_measured[3])   # filtered (or raw if filter disabled)
        _v_obs = float(z0_measured[4])
        _omega_obs = float(z0_measured[5])
        kappa_h, alpha_f_h, alpha_r_h, u_safe_h, Fz_f_h, Fz_r_h = (
            compute_bicycle_operating_point(
                delta_meas_now,
                _u_obs,
                _v_obs,
                _omega_obs,
                _measured_ax,
                geom=tire_geom,
                kappa_mode=args.kappa,
                terrain_mu=_terrain_mu,
                measured_kappa=_meas_kappa,
            )
        )
        # Realized road-wheel rate over the last control interval.
        sr_h = (delta_meas_now - prev_applied_delta) / max(dt_ctrl, 1e-4)

        # --- Step GRU observer (before solve, using current operating point) ---
        if gru_tracker is not None:
            gru_tracker.step(
                kappa_h, alpha_f_h, u_safe_h, Fz_f_h, sr_h,
                kappa_h, alpha_r_h, Fz_r_h, 0.0,
                terrain_params_est,
            )

        # --- Solve MPC ---
        t0_solve = wall_time.time()
        solve_kwargs = dict(
            n_terrain=n_terrain_est,
            sr_meas=sr_h,
            terrain_params=terrain_params_est,
        )
        if tire_hist is not None:
            solve_kwargs['hist_front'] = tire_hist.front
            solve_kwargs['hist_rear'] = tire_hist.rear
        if rate_tracker is not None:
            solve_kwargs['rates_front'] = rate_tracker.front
            solve_kwargs['rates_rear'] = rate_tracker.rear
        if gru_tracker is not None:
            solve_kwargs['gru_h_front'] = gru_tracker.front
            solve_kwargs['gru_h_rear'] = gru_tracker.rear

        # Obstacle avoidance: pass parsed obstacle list to OCP solver.
        # ``--mpc-blind-obstacles`` lets the safety filter be the *sole*
        # obstacle-avoider, which is the realistic test for a teleoperator
        # who can't see the rocks ahead of time and relies on the shield.
        if _obs_list_mpc and not getattr(args, 'mpc_blind_obstacles', False):
            solve_kwargs['obstacles'] = _obs_list_mpc

        delta_cmd, Jx, Z_opt, U_opt = mpc.solve(
            z0, x_ref, y_ref, psi_ref, v_ref,
            x_goal, y_goal, psi_goal,
            **solve_kwargs,
        )
        t_solve = wall_time.time() - t0_solve
        solve_times.append(t_solve)
        delay_est.update_solve(t_solve)

        if Z_opt is None:
            # Solver fallback path: solver may return a hold command.
            if not np.isfinite(delta_cmd):
                delta_cmd = float(z0[7])
            if not np.isfinite(Jx):
                Jx = float(z0[8])
        elif Z_opt.shape[1] > 1:
            # Queue one-step-ahead state prediction for measurement residuals.
            pred_targets.append((
                float(msg.time + mpc.dt),
                np.array(Z_opt[:9, 1], dtype=float, copy=True),
            ))

        if not np.isfinite(delta_cmd):
            delta_cmd = integrator.steering_angle
        if not np.isfinite(Jx):
            Jx = 0.0

        # Suppress steering during lead-in acceleration phase
        if lead_in > 0 and z0[0] < lead_in and msg.u < args.lead_in_speed_fraction * v_target:
            delta_cmd = 0.0

        # ---- End-of-path override: brake cleanly to a stop ----
        # The speed profile in ReferencePath already ramps v_ref → 0 over
        # the last 5 m, so the MPC should naturally command braking.  However,
        # integrator windup and latency can leave residual forward thrust after
        # the path ends.  When the path is exhausted, force the integrator
        # acceleration to maximum deceleration so the vehicle stops within the
        # physical braking distance rather than coasting past.
        if _path_done:
            if seq % 50 == 0 and msg.u > 0.1:
                print(f"  [PATH DONE] t={msg.time:.1f}s  u={msg.u:.2f} m/s"
                      f"  — braking to stop")
            if msg.u < 0.1:
                delta_cmd = 0.0

        # Post-MPC rate limiter: enforce max steer rate in real control dt
        # (MPC internal dt=0.1s >> control dt~0.012s, so the MPC's polytopic
        # constraint allows jumps that are too large for a single control step)
        max_delta_change = mpc.max_steer_rate * max(dt_ctrl, 1e-4)
        delta_cmd = float(np.clip(
            delta_cmd,
            integrator.steering_angle - max_delta_change,
            integrator.steering_angle + max_delta_change))

        # Effective δ̇ for delay buffer, diagnostics, and NN steering_rate feature
        delta_dot = (delta_cmd - integrator.steering_angle) / max(dt_ctrl, 1e-4)
        last_delta_dot_cmd = float(delta_dot)
        last_Jx_cmd = float(Jx)

        if tire_csv_writer is not None and msg.tire_forces:
            tfw = msg.tire_forces
            req = (
                "front_left_Fx", "front_right_Fx", "front_left_Fy", "front_right_Fy",
                "rear_left_Fx", "rear_right_Fx", "rear_left_Fy", "rear_right_Fy",
            )
            if all(k in tfw for k in req):
                fxm = 0.5 * (tfw["front_left_Fx"] + tfw["front_right_Fx"])
                fym = 0.5 * (tfw["front_left_Fy"] + tfw["front_right_Fy"])
                tire_csv_writer.writerow(
                    pack_vehicle_tire_csv_row(
                        int(args.log_scenario_id),
                        float(msg.time),
                        kappa_h,
                        alpha_f_h,
                        u_safe_h,
                        Fz_f_h,
                        float(delta_dot),
                        terrain_params_est,
                        float(fxm),
                        float(fym),
                    )
                )
                tire_csv_rows += 1
                # Log the rear axle as a separate sequence.  Static training
                # treats scenario_id as metadata, but temporal/rate training
                # groups by scenario_id to build windows; offsetting the rear
                # id prevents front/rear samples at the same timestep from
                # being stitched into one artificial tire history.
                fxm_r = 0.5 * (tfw["rear_left_Fx"] + tfw["rear_right_Fx"])
                fym_r = 0.5 * (tfw["rear_left_Fy"] + tfw["rear_right_Fy"])
                tire_csv_writer.writerow(
                    pack_vehicle_tire_csv_row(
                        int(args.log_scenario_id) + 1_000_000,
                        float(msg.time),
                        kappa_h,
                        alpha_r_h,
                        u_safe_h,
                        Fz_r_h,
                        0.0,
                        terrain_params_est,
                        float(fxm_r),
                        float(fym_r),
                    )
                )
                tire_csv_rows += 1

        # --- Update tire history after solve (training uses ~dt_nn between frames) ---
        if tire_hist is not None:
            sr_feat = float(delta_dot)
            t_sim = float(msg.time)
            if (
                last_tire_hist_time is None
                or (t_sim - last_tire_hist_time) >= args.nn_temporal_hist_dt
            ):
                tire_hist.update(
                    kappa_h, alpha_f_h, u_safe_h, Fz_f_h, sr_feat,
                    kappa_h, alpha_r_h, Fz_r_h, 0.0,
                )
                last_tire_hist_time = t_sim

        # --- Update rate tracker (training uses record_dt×subsample between diffs) ---
        if rate_tracker is not None:
            rate_tracker.update(
                float(msg.time),
                kappa_h, alpha_f_h, u_safe_h,
                kappa_h, alpha_r_h, u_safe_h,
            )

        # --- Apply controls: set δ directly, integrate ax from Jx ---
        integrator.steering_angle = float(np.clip(
            delta_cmd, -mpc.delta_max, mpc.delta_max))
        _saved_delta = integrator.steering_angle

        # Default: MPC uses IMU-measured ax so it sees terrain drag and
        # compensates via the speed cost.  The integrator accumulates ax
        # normally (acceleration += Jx*dt), providing integral action for
        # throttle.
        v_ref_now = float(v_ref[0]) if len(v_ref) else float(v_target)

        _, throttle, braking = integrator.update(
            0.0, Jx, dt_ctrl, msg.u,
            v_ref_now=v_ref_now,
        )

        # Force stopping if path is done
        if _path_done:
            integrator.acceleration = 0.0
            throttle = 0.0
            if msg.u > 0.1:
                braking = 1.0
            else:
                braking = 1.0

        integrator.steering_angle = _saved_delta
        steering = float(np.clip(
            integrator.steering_angle * integrator.steering_gain, -1.0, 1.0))

        # Planned excitation injection (off by default). Adds a small
        # sinusoidal steering perturbation so the terrain estimator gets
        # slip variance even when the NMPC's nominal trajectory is too
        # smooth to discriminate soils. This is a controller-architecture
        # knob, not a tweak to the estimator — the estimator sees the
        # vehicle response without any privileged information.
        if float(getattr(args, 'excitation_steer_amp', 0.0)) > 0.0:
            amp = float(args.excitation_steer_amp)
            per = max(float(args.excitation_steer_period), 1e-3)
            steering = float(np.clip(
                steering + amp * math.sin(2.0 * math.pi * msg.time / per),
                -1.0, 1.0))

        if rich_tire_csv_writer is not None and msg.tire_forces:
            tfw = msg.tire_forces
            req = (
                "front_left_Fx", "front_right_Fx", "front_left_Fy", "front_right_Fy",
                "rear_left_Fx", "rear_right_Fx", "rear_left_Fy", "rear_right_Fy",
            )
            if all(k in tfw for k in req):
                dFz_kin = lateral_load_transfer_dFz(msg.u, msg.omega, geom=tire_geom)
                dFz_imu = float(tire_geom.M * float(msg.ay) * tire_geom.h_cg / tire_geom.T / 2.0)
                kappa_front = kappa_from_wheel_pair(
                    msg.wheel_omega_fl, msg.wheel_omega_fr, msg.u
                )
                kappa_rear = kappa_from_wheel_pair(
                    msg.wheel_omega_rl, msg.wheel_omega_rr, msg.u
                )

                fxm_f = 0.5 * (tfw["front_left_Fx"] + tfw["front_right_Fx"])
                fym_f = 0.5 * (tfw["front_left_Fy"] + tfw["front_right_Fy"])
                rich_tire_csv_writer.writerow(
                    pack_rich_vehicle_tire_csv_row(
                        int(args.log_scenario_id),
                        float(msg.time),
                        0,
                        kappa_h,
                        alpha_f_h,
                        u_safe_h,
                        Fz_f_h,
                        float(delta_dot),
                        float(delta_cmd),
                        float(msg.u),
                        float(msg.v),
                        float(msg.omega),
                        float(msg.ax),
                        float(msg.ay),
                        float(_meas_kappa),
                        float(kappa_front),
                        float(0.5 * (msg.wheel_omega_fl + msg.wheel_omega_fr)),
                        float(msg.wheel_omega_fl),
                        float(msg.wheel_omega_fr),
                        float(dFz_kin),
                        float(dFz_imu),
                        float(throttle),
                        float(braking),
                        float(integrator.acceleration),
                        float(Jx),
                        terrain_params_est,
                        float(fxm_f),
                        float(fym_f),
                    )
                )
                rich_tire_csv_rows += 1

                fxm_r = 0.5 * (tfw["rear_left_Fx"] + tfw["rear_right_Fx"])
                fym_r = 0.5 * (tfw["rear_left_Fy"] + tfw["rear_right_Fy"])
                rich_tire_csv_writer.writerow(
                    pack_rich_vehicle_tire_csv_row(
                        int(args.log_scenario_id) + 1_000_000,
                        float(msg.time),
                        1,
                        kappa_h,
                        alpha_r_h,
                        u_safe_h,
                        Fz_r_h,
                        float(delta_dot),
                        float(delta_cmd),
                        float(msg.u),
                        float(msg.v),
                        float(msg.omega),
                        float(msg.ax),
                        float(msg.ay),
                        float(_meas_kappa),
                        float(kappa_rear),
                        float(0.5 * (msg.wheel_omega_rl + msg.wheel_omega_rr)),
                        float(msg.wheel_omega_rl),
                        float(msg.wheel_omega_rr),
                        float(dFz_kin),
                        float(dFz_imu),
                        float(throttle),
                        float(braking),
                        float(integrator.acceleration),
                        float(Jx),
                        terrain_params_est,
                        float(fxm_r),
                        float(fym_r),
                    )
                )
                rich_tire_csv_rows += 1

        # For next cycle's realized-rate estimate, keep the delta from this
        # cycle's measured state sample (before applying the new command).
        prev_applied_delta = delta_meas_now

        # --- Record in control buffer for delay compensation ---
        control_buffer.append((msg.time, delta_dot, Jx))

        # --- Record tracking analytics ---
        tf = msg.tire_forces or {}
        true_x = tf.get('true_x_cg')
        if true_x is not None:
            true_psi = tf['true_psi']
            true_x_fa = true_x + Lf * np.cos(true_psi)
            true_y_fa = tf['true_y_cg'] + Lf * np.sin(true_psi)
            true_u = tf['true_u']
            analytics.record(
                msg.time, true_x_fa, true_y_fa, true_psi, true_u,
                v_ref_now=v_ref_now,
            )
        else:
            analytics.record(
                msg.time, z0_measured[0], z0_measured[1], psi, msg.u,
                v_ref_now=v_ref_now,
            )
        analytics.record_control(
            msg.time, steering, throttle, braking,
            integrator.steering_angle, integrator.acceleration,
            t_solve * 1000.0, delay_est.compensation_delay * 1000.0,
        )

        # --- Live debug plotter update ---
        if _live_plotter is not None:
            ct_now = analytics.crosstrack_errors[-1] if analytics.crosstrack_errors else 0.0
            _live_plotter.update(
                z0, Z_opt, x_ref, y_ref, v_ref,
                sim_time=msg.time, u_meas=msg.u,
                steering_angle=integrator.steering_angle,
                ax_state=float(z0[6]),
                mpc_cost=getattr(mpc, 'last_cost', 0.0),
                crosstrack_err=ct_now,
                obstacles=_obs_list_mpc if _obs_list_mpc else None,
            )

        # --- Record Fy: Chrono actual vs model predicted ---
        # Use the same (current/pre-command) operating point as the state sample.
        # For kappa, log both measured slip and OCP-consistent slip so the
        # force comparison is interpreted against the model actually optimized.
        kappa_meas_diag = float(_meas_kappa)
        if mpc.kappa_mode == 'approx':
            mu_diag = max(_terrain_mu, 1e-3)
            kappa_diag = float(np.clip(_measured_ax / (mu_diag * 9.81), -0.8, 0.8))
        elif mpc.kappa_mode == 'zero':
            kappa_diag = 0.0
        else:
            kappa_diag = kappa_meas_diag
        alpha_f, alpha_r = alpha_f_h, alpha_r_h
        u_safe = u_safe_h
        Fz_f_mean, Fz_r_mean = Fz_f_h, Fz_r_h
        sr_diag = sr_h

        # Clamp slip angles to training-data range (matches MPC solver).
        _alpha_max = 0.55
        alpha_f = float(max(-_alpha_max, min(_alpha_max, alpha_f)))
        alpha_r = float(max(-_alpha_max, min(_alpha_max, alpha_r)))

        # No low-speed force fade: diagnostics should reflect direct model output.
        _speed_fade = 1.0

        if msg.tire_forces is not None:
            tf = msg.tire_forces
            actual_Fy_f = tf.get('front_left_Fy', 0) + tf.get('front_right_Fy', 0)
            actual_Fy_r = tf.get('rear_left_Fy', 0) + tf.get('rear_right_Fy', 0)

            if nn_tire is not None:
                hist_f = tire_hist.front if tire_hist is not None else None
                hist_r = tire_hist.rear if tire_hist is not None else None
                rates_f = rate_tracker.front if rate_tracker is not None else None
                rates_r = rate_tracker.rear if rate_tracker is not None else None
                if mpc.lateral_load_transfer:
                    dFz = lateral_load_transfer_dFz(msg.u, msg.omega, geom=tire_geom)
                    Fz_fo, Fz_fi, Fz_ro, Fz_ri = fz_with_lateral_transfer(
                        Fz_f_mean, Fz_r_mean, dFz
                    )
                    _, Fy_fo = nn_tire.predict_numeric(
                        alpha_f, Fz_fo, u_safe,
                        kappa=kappa_diag, n_terrain=n_terrain_est, steering_rate=sr_diag,
                        terrain_params=terrain_params_est, hist=hist_f, rates=rates_f)
                    _, Fy_fi = nn_tire.predict_numeric(
                        alpha_f, Fz_fi, u_safe,
                        kappa=kappa_diag, n_terrain=n_terrain_est, steering_rate=sr_diag,
                        terrain_params=terrain_params_est, hist=hist_f, rates=rates_f)
                    _, Fy_ro = nn_tire.predict_numeric(
                        alpha_r, Fz_ro, u_safe,
                        kappa=kappa_diag, n_terrain=n_terrain_est, steering_rate=0.0,
                        terrain_params=terrain_params_est, hist=hist_r, rates=rates_r)
                    _, Fy_ri = nn_tire.predict_numeric(
                        alpha_r, Fz_ri, u_safe,
                        kappa=kappa_diag, n_terrain=n_terrain_est, steering_rate=0.0,
                        terrain_params=terrain_params_est, hist=hist_r, rates=rates_r)
                    pred_Fy_f = -(Fy_fo + Fy_fi)
                    pred_Fy_r = -(Fy_ro + Fy_ri)
                else:
                    _, Fy_fw = nn_tire.predict_numeric(
                        alpha_f, Fz_f_mean, u_safe,
                        kappa=kappa_diag, n_terrain=n_terrain_est, steering_rate=sr_diag,
                        terrain_params=terrain_params_est, hist=hist_f, rates=rates_f)
                    _, Fy_rw = nn_tire.predict_numeric(
                        alpha_r, Fz_r_mean, u_safe,
                        kappa=kappa_diag, n_terrain=n_terrain_est, steering_rate=0.0,
                        terrain_params=terrain_params_est, hist=hist_r, rates=rates_r)
                    pred_Fy_f = -2.0 * Fy_fw
                    pred_Fy_r = -2.0 * Fy_rw
            else:
                # Analytical tire model (pacejka, pacejka-oracle, tmeasy)
                _anal_model = 'pacejka' if tire_model == 'pacejka-oracle' else tire_model
                _anal_kwargs = mpc._oracle_pacejka_params if tire_model == 'pacejka-oracle' else {}
                Fyf, Fyr, _ = analytical_tire_forces(
                    _anal_model, alpha_f, alpha_r,
                    2.0 * Fz_f_mean, 2.0 * Fz_r_mean, kappa_diag,
                    **_anal_kwargs,
                )
                pred_Fy_f = float(Fyf)
                pred_Fy_r = float(Fyr)

            analytics.record_tire_forces(
                msg.time, actual_Fy_f, actual_Fy_r, pred_Fy_f, pred_Fy_r)

            # Update online Fy bias estimator (signed EMA)
            if msg.u > _FY_BIAS_MIN_SPEED:
                af, ar, pf, pr = float(actual_Fy_f), float(actual_Fy_r), float(pred_Fy_f), float(pred_Fy_r)
                signed_err_f = af - pf
                signed_err_r = ar - pr
                _fy_bias_signed_f = (1 - _FY_BIAS_ALPHA) * _fy_bias_signed_f + _FY_BIAS_ALPHA * signed_err_f
                _fy_bias_signed_r = (1 - _FY_BIAS_ALPHA) * _fy_bias_signed_r + _FY_BIAS_ALPHA * signed_err_r
                _fy_bias_signed_f = float(np.clip(_fy_bias_signed_f, -_FY_BIAS_CLIP, _FY_BIAS_CLIP))
                _fy_bias_signed_r = float(np.clip(_fy_bias_signed_r, -_FY_BIAS_CLIP, _FY_BIAS_CLIP))

            # --- Online terrain parameter estimator (sensor-realistic) ---
            if terrain_estimator is not None:
                try:
                    # Use raw target speed (NOT terrain-adapted v_ref) so
                    # the estimator sees speed capability directly.
                    _te_omega_dot = terrain_estimator.estimate_omega_dot(
                        msg.omega, msg.time
                    )
                    if _te_omega_dot is not None:
                        terrain_estimator.observe(
                            kappa=kappa_diag,
                            alpha_f=float(alpha_f),
                            alpha_r=float(alpha_r),
                            u=float(u_safe),
                            Fz_f=float(Fz_f_mean),
                            Fz_r=float(Fz_r_mean),
                            sr=float(sr_diag),
                            ay_imu=float(msg.ay),
                            omega_dot=_te_omega_dot,
                            omega=float(msg.omega),
                            pred_Fy_f=float(pred_Fy_f),
                            pred_Fy_r=float(pred_Fy_r),
                            v_ref=float(v_target),
                            v_lateral=float(msg.v),
                            x_pos=float(msg.x_cg),
                            y_pos=float(msg.y_cg),
                            psi=float(np.arctan2(
                                2*(msg.quat_e0*msg.quat_e3 + msg.quat_e1*msg.quat_e2),
                                1 - 2*(msg.quat_e2**2 + msg.quat_e3**2))),
                            ax_cmd=float(z0_measured[6]),
                            sim_time=float(msg.time),
                            wheel_omegas=(
                                float(msg.wheel_omega_fl),
                                float(msg.wheel_omega_fr),
                                float(msg.wheel_omega_rl),
                                float(msg.wheel_omega_rr),
                            ),
                            ax_imu=float(msg.ax),
                            az_imu=float(getattr(msg, "az", 0.0)),
                            roll_rate=float(getattr(msg, "omega_x", 0.0)),
                            pitch_rate=float(getattr(msg, "omega_y", 0.0)),
                            throttle_cmd=float(throttle),
                        )

                    if terrain_estimator.should_update():
                        _te_params, _te_conf = terrain_estimator.estimate()
                        if _te_conf >= args.te_min_confidence:
                            # Update MPC terrain params. Note: ``_te_mpc['n']``
                            # is clipped to the canonical interpolation range
                            # (clay-n .. sand-n) so the MPC tire model stays
                            # inside its training distribution. For diagnostics
                            # and figures we log the *unclipped* smoothed value
                            # via ``get_bekker_n()`` so OOD predictions remain
                            # visible.
                            _te_mpc = terrain_estimator.get_terrain_mpc_params()
                            terrain_params_est = _te_mpc
                            n_terrain_est = float(terrain_estimator.get_bekker_n())
                            terrain_confidence = _te_conf
                            terrain_update_applied = 1
                            terrain_class_est = getattr(
                                terrain_estimator, '_terrain_name', 'estimated'
                            )
                            # Cache the live estimate so every subsequent
                            # ControlCommand carries it to the sim-side
                            # safety shield (see ControlCommand.terrain_*
                            # fields). Piggybacking on ctrl_pub avoids
                            # ZMQ_CONFLATE dropping a separate channel.
                            latest_terrain_update.update({
                                "seq": latest_terrain_update["seq"] + 1,
                                "n": float(_te_mpc['n']),
                                "phi_deg": float(_te_mpc['phi']),
                                "phi_sigma_deg": float(
                                    terrain_estimator.get_phi_uncertainty_deg()
                                ),
                                "Kphi": float(_te_mpc['Kphi']),
                                "Kc": float(_te_mpc['Kc']),
                                "c": float(_te_mpc['c']),
                                "k": float(_te_mpc['k']),
                                "terrain_class": str(terrain_class_est),
                                "confidence": float(_te_conf),
                            })
                except Exception as _te_exc:
                    import traceback
                    print(f"[TERRAIN-EST] Error: {_te_exc}", flush=True)
                    traceback.print_exc()

        # --- Publish command ---
        cmd = ControlCommand(
            time=msg.time,
            wall_time=wall_time.time(),
            seq=seq,
            steering=steering,
            throttle=throttle,
            braking=braking,
            delta=integrator.steering_angle,
            acceleration=integrator.acceleration,
            delta_dot=delta_dot,
            jerk=Jx,
            solve_time_ms=t_solve * 1000.0,
            terrain_n=latest_terrain_update["n"],
            terrain_phi_deg=latest_terrain_update["phi_deg"],
            terrain_phi_sigma_deg=latest_terrain_update["phi_sigma_deg"],
            terrain_Kphi=latest_terrain_update["Kphi"],
            terrain_Kc=latest_terrain_update["Kc"],
            terrain_c=latest_terrain_update["c"],
            terrain_k=latest_terrain_update["k"],
            terrain_class=latest_terrain_update["terrain_class"],
            terrain_confidence=latest_terrain_update["confidence"],
            terrain_update_seq=latest_terrain_update["seq"],
        )
        ctrl_pub.send(cmd)

        # --- Write diagnostic CSV row ---
        if csv_writer is not None:
            tf = msg.tire_forces or {}
            true_x = tf.get('true_x_cg')
            if true_x is not None:
                true_psi_v = tf['true_psi']
                true_x_fa = true_x + Lf * np.cos(true_psi_v)
                true_y_fa = tf['true_y_cg'] + Lf * np.sin(true_psi_v)
                true_u_v = tf['true_u']
            else:
                true_x_fa = z0_measured[0]
                true_y_fa = z0_measured[1]
                true_psi_v = psi
                true_u_v = msg.u

            ct_err = analytics.crosstrack_errors[-1] if analytics.crosstrack_errors else 0
            hd_err = np.degrees(analytics.heading_errors[-1]) if analytics.heading_errors else 0
            sp_err = analytics.speed_errors[-1] if analytics.speed_errors else 0

            fy_af = analytics.actual_Fy_front[-1] if analytics.actual_Fy_front else ''
            fy_ar = analytics.actual_Fy_rear[-1] if analytics.actual_Fy_rear else ''
            fy_nf = analytics.pred_Fy_front[-1] if analytics.pred_Fy_front else ''
            fy_nr = analytics.pred_Fy_rear[-1] if analytics.pred_Fy_rear else ''

            mpc_cost = getattr(mpc, 'last_cost', float('nan'))
            solver_status = getattr(mpc, 'last_solver_status', '')
            solver_iters = getattr(mpc, 'last_iter_count', -1)
            res_updates = 0
            res_last_loss = float("nan")
            res_last_update_ms = 0.0
            phi_applied_deg = float(terrain_params_est.get('phi', float('nan')))
            phi_estimator_deg = phi_applied_deg
            if terrain_estimator is not None:
                try:
                    phi_estimator_deg = float(terrain_estimator.get_friction_angle_deg())
                except Exception:
                    phi_estimator_deg = phi_applied_deg

            csv_writer.writerow([
                f"{msg.time:.4f}", f"{recv_time:.6f}", seq,
                f"{z0_measured[0]:.6f}", f"{z0_measured[1]:.6f}",
                f"{psi:.6f}", f"{msg.u:.4f}", f"{msg.v:.4f}", f"{msg.omega:.6f}",
                f"{true_x_fa:.6f}", f"{true_y_fa:.6f}",
                f"{true_psi_v:.6f}", f"{true_u_v:.4f}",
                f"{z0[0]:.6f}", f"{z0[1]:.6f}", f"{z0[2]:.6f}",
                f"{z0[3]:.4f}", f"{z0[4]:.4f}", f"{z0[5]:.6f}",
                f"{z0[6]:.6f}", f"{z0[7]:.4f}",
                f"{x_ref[0]:.6f}", f"{y_ref[0]:.6f}",
                f"{psi_ref[0]:.6f}", f"{v_ref[0]:.4f}",
                f"{delta_dot:.6f}", f"{Jx:.6f}",
                f"{mpc_cost:.4f}", solver_status, solver_iters,
                terrain_class_est, f"{terrain_confidence:.4f}", f"{n_terrain_est:.6f}",
                f"{phi_applied_deg:.6f}",
                f"{phi_estimator_deg:.6f}",
                f"{latest_terrain_update['phi_sigma_deg']:.6f}" if latest_terrain_update["phi_sigma_deg"] is not None else "",
                terrain_update_applied,
                0,
                f"{res_pred_du:.6f}", f"{res_pred_dv:.6f}", f"{res_pred_domega:.6f}",
                res_updates, f"{res_last_loss:.6f}", f"{res_last_update_ms:.2f}",
                f"{steering:.6f}", f"{throttle:.4f}", f"{braking:.4f}",
                f"{integrator.steering_angle:.6f}", f"{integrator.acceleration:.4f}",
                f"{delay_est.one_way_delay*1000:.2f}",
                f"{delay_est.solve_time*1000:.2f}",
                f"{delay_est.compensation_delay*1000:.2f}",
                f"{t_solve*1000:.2f}",
                f"{ct_err:.6f}", f"{hd_err:.4f}", f"{sp_err:.4f}",
                f"{pred_age:.6f}", f"{pred_pos_err:.6f}", f"{pred_psi_err_deg:.4f}",
                f"{pred_u_err:.6f}", f"{pred_v_err:.6f}", f"{pred_omega_err:.6f}",
                fy_af, fy_ar, fy_nf, fy_nr,
                f"{alpha_f:.6f}", f"{alpha_r:.6f}",
                f"{Fz_f_mean:.1f}", f"{Fz_r_mean:.1f}",
                f"{kappa_diag:.6f}", f"{kappa_meas_diag:.6f}", f"{sr_diag:.6f}",
                f"{u_safe:.4f}", f"{_speed_fade:.4f}",
            ])

        seq += 1

        # --- Periodic report ---
        if int(args.status_every_n) > 0 and seq % int(args.status_every_n) == 0:
            mean_ms = np.mean(solve_times[-20:]) * 1000
            tau_ms = delay_est.compensation_delay * 1000
            trk = analytics.periodic_summary()
            tc_str = f"  terrain={terrain_class_est}({terrain_confidence:.0%})" if terrain_sub else ""
            te_str = ""
            if terrain_estimator is not None:
                te_str = (f"  TE={terrain_class_est}({terrain_confidence:.0%})"
                          f"[μ={terrain_estimator.mu_estimate:.3f}]")
            print(f"  t={msg.time:.1f}s  solve={mean_ms:.1f}ms  "
                  f"τ_comp={tau_ms:.1f}ms  {trk}  "
                  f"u={msg.u:.2f}m/s{tc_str}{te_str}")

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    if solve_times:
        st = np.array(solve_times)
        ct_arr = np.array(analytics.crosstrack_errors) if analytics.crosstrack_errors else None
        avg_cte = np.mean(np.abs(ct_arr)) if ct_arr is not None else float("nan")
        print(f"\n  ACADOS Controller Summary ({model_label}):")
        print(f"    Total solves:   {len(st)}")
        print(f"    Mean solve:     {np.mean(st)*1000:.2f} ms")
        print(f"    Max solve:      {np.max(st)*1000:.2f} ms")
        print(f"    Effective rate: {1.0/np.mean(st):.1f} Hz")
        print(f"    Avg |CTE|:      {avg_cte:.4f} m")
        print(f"    Final τ_comp:   {delay_est.compensation_delay*1000:.1f} ms")
        if pred_pos_err_hist:
            print("    1-step prediction residuals:")
            print(f"      Mean pos:     {np.mean(pred_pos_err_hist):.4f} m")
            print(f"      RMS pos:      {np.sqrt(np.mean(np.square(pred_pos_err_hist))):.4f} m")
            print(f"      Mean |ψ|:      {np.degrees(np.mean(pred_psi_err_hist)):.2f}°")
            print(f"      Mean |u|:      {np.mean(pred_u_err_hist):.3f} m/s")
            print(f"      Mean |v|:      {np.mean(pred_v_err_hist):.3f} m/s")
            print(f"      Mean |ω|:      {np.mean(pred_omega_err_hist):.4f} rad/s")

    print(analytics.final_summary())

    # Close CSV
    if csv_file is not None:
        csv_file.close()
        if csv_path is not None:
            print(f"  Diagnostic CSV written: {csv_path} ({seq} rows)")
    if tire_csv_file is not None:
        tire_csv_file.close()
        if tire_csv_path is not None:
            print(f"  Tire training CSV written: {tire_csv_path} ({tire_csv_rows} rows)")
    if rich_tire_csv_file is not None:
        rich_tire_csv_file.close()
        if rich_tire_csv_path is not None:
            print(
                f"  Rich tire training CSV written: {rich_tire_csv_path} "
                f"({rich_tire_csv_rows} rows)"
            )

    if not args.no_plot:
        analytics.plot_results(
            plot_dir=str(run_dir),
            terrain_name=terrain_name,
            model_label=model_label,
        )

    if _live_plotter is not None:
        _live_plotter.close()

    ctrl_pub.close()
    state_sub.close()
    if terrain_sub is not None:
        terrain_sub.close()


# =============================================================================
# Entry point
# =============================================================================

def main():
    p = argparse.ArgumentParser(description="ACADOS MPC Controller Node (decoupled)")
    p.add_argument("--mpc-blind-obstacles", action="store_true",
                   help="Drop obstacle data on the controller side so the NMPC is a "
                        "pure path-tracker. The downstream safety shield (MPPI/NMPC/"
                        "DOB-CBF) becomes the sole collision-avoidance layer — "
                        "useful as a proxy for an oblivious teleoperator.")

    # Model (NN or analytical tire model)
    p.add_argument("--model", default="nn",
                   choices=["nn", "pacejka", "pacejka-oracle", "tmeasy"],
                   help="Tire model: nn (neural network), pacejka (rigid-terrain "
                        "defaults), pacejka-oracle (terrain-fitted mu/B, fair "
                        "comparison upper bound), or tmeasy")
    p.add_argument("--nn-model", default="vehicle_rate_64_32_lhs",
                   help="NN model version directory (only used when --model nn)")
    p.add_argument("--kappa", default="measured", choices=["zero", "approx", "measured"])
    p.add_argument("--no-lat-transfer", action="store_true",
                   help="Disable lateral load transfer (2 NN calls vs 4)")
    p.add_argument(
        "--nn-temporal-hist-dt",
        type=float,
        default=0.1,
        help="Sim seconds between temporal NN history pushes; must match "
             "train_temporal_nn / temporal ResNet dt_nn (default 0.1)",
    )
    p.add_argument(
        "--nn-rate-sample-dt",
        type=float,
        default=0.05,
        help="Sim seconds between rate-NN finite-difference anchors; match "
             "train_rate_nn record_dt×subsample (default 0.005×10=0.05)",
    )
    p.add_argument(
        "--symbolic-rates",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Compute rate features (dκ, dα, du, sr) symbolically inside the "
             "MPC dynamics instead of freezing them as parameters (default: on). "
             "Adds 3 extra states (α_f_prev, α_r_prev, δ_sr_prev) for finite-"
             "difference/rate channels. Use --no-symbolic-rates to disable.",
    )
    p.add_argument(
        "--no-temporal-staged",
        action="store_true",
        help="Disable stage-varying temporal history (freeze measured history "
             "across all horizon stages). Diagnostic flag.",
    )
    # Path
    p.add_argument("--path", default="lane_change",
                   choices=["lane_change", "double_lane_change", "right_left", "sinusoidal"])
    p.add_argument("--speed", type=float, default=5.0, help="Target speed (m/s)")
    p.add_argument(
        "--speed-weight",
        type=float,
        default=70.0,
        help="Stage-cost weight on (u - v_ref)^2. Lower values keep the MPC "
             "from chasing reference speed as aggressively in turns.",
    )
    p.add_argument(
        "--speed-cost-mode",
        choices=["symmetric", "overspeed"],
        default="symmetric",
        help="'symmetric' tracks v_ref from both sides; 'overspeed' treats "
             "v_ref as a cap and does not reward accelerating up to it.",
    )
    p.add_argument(
        "--obstacle-weight",
        type=float,
        default=5e3,
        help="Stage/terminal soft obstacle-barrier weight for autonomous MPC obstacle avoidance.",
    )
    p.add_argument("--sine-amplitude", type=float, default=2.0)
    p.add_argument("--sine-wavelength", type=float, default=30.0)
    p.add_argument("--lead-in", type=float, default=0.0,
                   help="Straight lead-in distance (m) before path starts")
    p.add_argument("--no-path-reindex", action="store_true")

    # Terrain
    p.add_argument("--terrain", default="sand", choices=["sand", "clay", "dirt"])
    p.add_argument("--controller-prior-terrain", default=None,
                   choices=["sand", "clay", "dirt"],
                   help="Override what terrain the controller's static prior assumes. "
                        "Defaults to --terrain (matched-prior). Set to a different "
                        "value to exercise the wrong-prior case in ablations.")
    p.add_argument("--ay-safety", type=float, default=0.65,
                   help="Fraction of the Coulomb lateral-accel limit the "
                        "curvature-limited speed profile may use in turns. "
                        "Higher = faster cornering reference (speed sweeps).")
    p.add_argument("--time", type=float, default=15.0, help="Expected sim duration")

    # Delay compensation
    p.add_argument("--no-delay-comp", action="store_true",
                   help="Disable transport delay compensation in MPC")
    p.add_argument("--initial-delay", type=float, default=0.02,
                   help="Initial one-way delay estimate (s)")
    p.add_argument(
        "--state-predict-dt",
        type=float,
        default=0.005,
        help="Delay-compensation predictor RK substep (s); smaller = finer τ forward roll",
    )
    p.add_argument(
        "--control-buffer-len",
        type=int,
        default=50,
        help="Max past (δ̇, Jx) samples kept for delay compensation",
    )

    # MPC / solver build (defaults match acados_mpc_solver.DEFAULT_*)
    p.add_argument(
        "--mpc-dt",
        type=float,
        default=DEFAULT_MPC_DT,
        help="MPC discretisation step [s]; must match precompiled solver if you use one",
    )
    p.add_argument(
        "--mpc-n",
        type=int,
        default=DEFAULT_MPC_HORIZON_STEPS,
        help="MPC horizon length (stages); must match precompiled solver if you use one",
    )
    p.add_argument(
        "--acados-build-dir",
        default=None,
        metavar="DIR",
        help="Exact directory for ACADOS codegen (overrides ACADOS_MPC_BUILD_ROOT / tmp)",
    )
    p.add_argument(
        "--warmup-iters",
        type=int,
        default=5,
        help="Dummy solves before connecting to sim (JIT / first-factor warm-up)",
    )
    p.add_argument(
        "--zmq-recv-timeout-ms",
        type=int,
        default=200,
        help="State subscriber poll timeout (ms)",
    )
    p.add_argument(
        "--ready-ping-interval-s",
        type=float,
        default=0.25,
        help="While waiting for first VehicleState, re-send neutral ControlCommand period (s)",
    )
    p.add_argument(
        "--status-every-n",
        type=int,
        default=20,
        help="Print timing/tracking line every N control steps (0 = disable)",
    )
    p.add_argument(
        "--lead-in-speed-fraction",
        type=float,
        default=0.8,
        help="During lead-in, zero steering while u below this fraction of v_target",
    )

    # Throttle disturbance observer (asymmetric velocity-error DOB)
    p.add_argument(
        "--dob-ki",
        type=float,
        default=0.15,
        help="Throttle DOB integrator gain [throttle/(m/s)/s]; 0 disables the DOB",
    )
    p.add_argument(
        "--dob-max",
        type=float,
        default=0.35,
        help="Asymmetric upper clip on the DOB throttle bias (0 = no compensation)",
    )
    p.add_argument(
        "--dob-bleed",
        type=float,
        default=0.5,
        help="Exponential bleed rate of the DOB during MPC braking [1/s]",
    )

    # Analytics
    p.add_argument("--rms-time-start", type=float, default=2.0,
                   help="Start time for RMS calculation, skips startup (s)")
    p.add_argument("--no-plot", action="store_true",
                   help="Skip generating end-of-run plots")
    p.add_argument("--live-plot", action="store_true",
                   help="Open a live matplotlib debug window during the run")
    p.add_argument("--live-plot-every", type=int, default=5,
                   help="Redraw live debug plot every N control steps (default: 5)")
    p.add_argument("--no-csv", action="store_true",
                   help="Skip diagnostic CSV output")
    p.add_argument(
        "--log-tire-csv",
        default=None,
        metavar="PATH",
        help="Append MPC-aligned tire rows (temporal CSV schema) for retraining; "
             "front and rear axle rows are logged per control step with kappa, "
             "bicycle alpha, Fz, steering-rate feature, and mean wheel Fx/Fy "
             "from Chrono when available",
    )
    p.add_argument(
        "--log-rich-tire-csv",
        default=None,
        metavar="PATH",
        help="Append rich sensor-realistic tire rows for retraining. Inputs are "
             "limited to GPS/INS/IMU, steering and wheel encoders, command "
             "signals, fixed-geometry load-transfer estimates, and terrain "
             "estimates; Chrono tire forces are labels only.",
    )
    p.add_argument(
        "--log-scenario-id",
        type=int,
        default=0,
        help="scenario_id column for --log-tire-csv (one run = one scenario)",
    )
    p.add_argument("--plot-dir", default="plots",
                   help="Directory for output plots (default: plots/)")
    p.add_argument(
        "--ax-filter-tau", type=float, default=0.5,
        help="Complementary filter time constant (s) for fusing IMU ax with "
             "model prediction.  Suppresses terrain-induced noise.  0 = no filter.",
    )
    p.add_argument(
        "--vel-filter-tau", type=float, default=0.05,
        help="EMA time constant (s) for smoothing noisy [u, v, omega] before MPC and GP observations. "
             "Addresses σ_v=0.05 m/s noise (SNR 2-10:1 for v); reduces GP target noise by ~1/sqrt(alpha). "
             "0 = no filter (backward compat). Default 0.05s → α≈0.67 at 10 Hz.",
    )

    # Network
    p.add_argument("--sim-host", default="localhost", help="Sim node host")
    p.add_argument("--sim-port", type=int, default=5555, help="Sim state port")
    p.add_argument("--ctrl-port", type=int, default=5556, help="Control command port")

    # Terrain classifier
    p.add_argument("--terrain-classifier", action="store_true",
                   help="Subscribe to terrain classifier estimates (telemetry only unless --use-prediction)")
    p.add_argument("--use-prediction", action="store_true",
                   help="Apply terrain classifier predictions to MPC terrain parameters "
                        "(implies --terrain-classifier; defaults to clay before first prediction)")
    p.add_argument(
        "--prediction-min-confidence",
        type=float,
        default=0.0,
        help="Only apply classifier terrain updates to MPC when confidence is at least this value [0,1]",
    )
    p.add_argument("--tc-port", type=int, default=5557,
                   help="Terrain classifier publish port to subscribe to")

    # Terrain parameter estimator (speed-capability voting, replaces classifier)
    p.add_argument("--terrain-estimator", action="store_true",
                   help="Enable online terrain parameter estimation from speed capability "
                        "and inertial cues (no oracle data). Replaces classifier for "
                        "terrain param updates.")
    p.add_argument("--terrain-estimator-mode", choices=["n"], default="n",
                   help="Select live terrain-estimator output mode. The retained "
                        "paper/runtime estimator is n-only.")
    p.add_argument("--terrain-estimator-backend", choices=["learned", "nn_ukf", "fused"],
                   default="learned",
                   help="Runtime terrain-estimator backend: 'learned' = deployed "
                        "sliding-window MLP; 'nn_ukf' = online Dallas-style "
                        "state-augmented UKF with the whole-vehicle Fy surrogate.")
    p.add_argument("--nn-ukf-q-n", type=float, default=0.01,
                   help="Process-noise std on n for the nn_ukf backend (tracking speed).")
    p.add_argument("--te-window", type=int, default=50,
                   help="Terrain estimator sliding window size in 10 Hz-equivalent samples "
                        "(default 50 -> 5 s)")
    p.add_argument("--te-update-interval", type=int, default=10,
                   help="Run terrain estimation every N accepted 10 Hz-equivalent samples "
                        "(default 10 -> 1 s)")
    p.add_argument("--te-min-excitation", type=float, default=0.3,
                   help="Minimum |ay| (m/s²) to accept observation (excitation gate)")
    p.add_argument("--te-min-confidence", type=float, default=0.3,
                   help="Minimum confidence to apply estimated terrain params to MPC")
    p.add_argument("--learned-terrain-model-dir", default=None,
                   help="Path to the trained terrain_window_mlp/ directory. "
                        "Defaults depend on --terrain-estimator-mode.")
    p.add_argument("--te-verbose", action="store_true",
                   help="Print verbose terrain-estimator predictions (every "
                        "10 observations) for offline parsing/validation.")

    # Planned excitation injection. Adds a small sinusoidal perturbation on
    # the steering channel so the terrain estimator sees slip variance even
    # when the NMPC's nominal trajectory is smooth enough that the
    # estimator would otherwise saturate near the controller prior.
    p.add_argument("--excitation-steer-amp", type=float, default=0.0,
                   help="Amplitude (normalised -1..1) of planned steering "
                        "perturbation. 0 = off; 0.03-0.08 = useful estimator "
                        "excitation without disturbing tracking much.")
    p.add_argument("--excitation-steer-period", type=float, default=1.0,
                   help="Period (seconds) of the planned steering perturbation.")

    args = p.parse_args()
    # --use-prediction implies --terrain-classifier
    if args.use_prediction:
        args.terrain_classifier = True
    run_controller_node(args)


if __name__ == "__main__":
    main()
