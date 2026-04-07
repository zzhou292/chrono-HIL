#!/usr/bin/env python3
"""
ACADOS MPC Controller Node (Decoupled)
========================================

Drop-in ACADOS SQP-RTI MPC controller node using the
solver instead of CasADi+IPOPT.  This should give significant speedup
(5-20×) while maintaining the same dynamics, cost, and constraints.

All helper classes (DelayEstimator, StatePredictor, ControlIntegrator,
TrackingAnalytics, TireHistoryTracker, RateTracker) are imported from
mpc_helpers module to avoid code duplication.

NN tire models are loaded via the unified nn_tire_model.py interface which
auto-detects model type from the checkpoint.

Subscribes: VehicleState from simulation node
Publishes:  ControlCommand to simulation node

Usage:
    python acados_mpc_controller_node.py --nn-model v6_mlp_16_4 --terrain sand --path sinusoidal
    python acados_mpc_controller_node.py --model pacejka --terrain dirt --path lane_change
"""

import argparse
import collections
import csv
import math
import sys
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
    terrain_preset_to_internal, HMMWV_VEHICLE_PARAMS,
    TERRAIN_PRESETS,
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
)

# ACADOS solver + unified NN loader
from acados_mpc_solver import AcadosDallasMPC
from nn_tire_model import load_nn_tire_model

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
_DEFAULT_TERRAIN_CLASS = "dirt"

from path_utils import make_path_function


# =============================================================================
# Main controller loop
# =============================================================================

def run_controller_node(args):
    print("=" * 60)
    print("ACADOS MPC Controller Node (Decoupled)")
    print("=" * 60)

    # ------------------------------------------------------------------
    # Wait for config from sim node
    # ------------------------------------------------------------------
    state_sub = ZMQSubscriber(sim_sub_endpoint(args.sim_host, args.sim_port))
    print(f"  Subscribing to state from {args.sim_host}:{args.sim_port}")

    config = None
    print("  Waiting for sim config...", end="", flush=True)
    for _ in range(200):
        result = state_sub.recv(timeout_ms=100)
        if result is not None:
            topic, msg = result
            if isinstance(msg, SimStatus) and msg.event == "config":
                config = msg.config
                print(" received!")
                break
    if config is None:
        print(" timeout — using CLI arguments for config.")
        vehicle_params = get_vehicle_params_for_demo()
        tp = get_terrain_preset(args.terrain)
        terrain_params = terrain_preset_to_internal(tp)
        config = {
            "vehicle_params": vehicle_params,
            "terrain_params": terrain_params,
            "terrain_preset": args.terrain,
            "path_type": args.path,
            "v_target": args.speed,
            "sim_time": args.time,
            "sine_amplitude": args.sine_amplitude,
            "sine_wavelength": args.sine_wavelength,
            "lead_in": args.lead_in,
        }

    vehicle_params = config["vehicle_params"]
    terrain_params = config["terrain_params"]
    v_target = config.get("v_target", args.speed)
    path_type = config.get("path_type", args.path)
    sine_amp = config.get("sine_amplitude", args.sine_amplitude)
    sine_wl = config.get("sine_wavelength", args.sine_wavelength)
    lead_in = config.get("lead_in", args.lead_in)
    terrain_name = config.get("terrain_preset", args.terrain)

    # When terrain classifier is active, override terrain_params with default
    if args.terrain_classifier:
        default_tp = get_terrain_preset(_DEFAULT_TERRAIN_CLASS)
        terrain_params = terrain_preset_to_internal(default_tp)
        print(f"  Terrain: classifier mode (default={_DEFAULT_TERRAIN_CLASS}, "
              f"GT={terrain_name} hidden from MPC)")
    else:
        print(f"  Terrain: {terrain_name}")
    print(f"  Path: {path_type}, v_target: {v_target} m/s")
    if lead_in > 0:
        print(f"  Lead-in: {lead_in:.0f}m straight before path")

    # ------------------------------------------------------------------
    # Build MPC (ACADOS)
    # ------------------------------------------------------------------
    dt_mpc = 0.1
    N_horizon = 30
    tire_model = args.model

    # Load NN tire model (only for nn mode)
    nn_tire = None
    if tire_model == 'nn':
        base_path = Path(__file__).parent.parent
        model_version = args.nn_model
        model_dir = base_path / "nn_models" / model_version
        if not model_dir.exists():
            print(f"  ERROR: NN model directory not found: {model_dir}")
            sys.exit(1)

        nn_tire = load_nn_tire_model(str(model_dir), terrain_params)
        print(f"  NN model loaded: {nn_tire.model_type} "
              f"(input_dim={nn_tire.input_dim}, params={nn_tire.n_params})")
        # Cache ACADOS codegen/compile per concrete NN model directory.
        # Using only model_type (e.g. static_mlp) causes many different
        # checkpoints to thrash the same cache path.
        safe_model_tag = model_version.replace("/", "_")
        acados_build_dir = Path(f"/tmp/acados_dallas_mpc_{safe_model_tag}")
    else:
        acados_build_dir = Path(f"/tmp/acados_dallas_mpc_{tire_model}")

    mpc = AcadosDallasMPC(
        nn_tire_model=nn_tire,
        dt=dt_mpc,
        N=N_horizon,
        lateral_load_transfer=not args.no_lat_transfer,
        kappa_mode=args.kappa,
        tire_model=tire_model,
        build_dir=acados_build_dir,
    )
    if tire_model == 'nn':
        model_label = f"ACADOS-NN ({nn_tire.model_type})"
    else:
        model_label = f"ACADOS-{tire_model}"
    print(f"  MPC built: {model_label}, N={N_horizon}, dt={dt_mpc}s")

    # Temporal history tracker
    tire_hist = None
    if mpc._temporal_mode:
        K_t = nn_tire.temporal_K
        tire_hist = TireHistoryTracker(K_t)
        # Pre-fill with plausible values so the NN doesn't see OOD zeros
        # (zero Fz/u in history causes near-zero force predictions under RTI)
        L = mpc.Lf + mpc.Lr
        Fz_f0 = mpc.M * 9.81 * mpc.Lr / L / 2.0
        Fz_r0 = mpc.M * 9.81 * mpc.Lf / L / 2.0
        for _ in range(K_t - 1):
            tire_hist.update(0.0, 0.0, 0.5, Fz_f0, 0.0,
                             0.0, 0.0, Fz_r0, 0.0)
        print(f"  Temporal history: K={K_t}, tracking {K_t - 1} past observations per tire")

    # Rate tracker for rate-augmented NN
    rate_tracker = None
    if mpc._rate_mode:
        rate_tracker = RateTracker(dt_mpc)
        print(f"  Rate-augmented NN: tracking dkappa/dt, dalpha/dt, du/dt per axle")

    # Warmup ACADOS solver (first solves trigger JIT)
    print("  Warming up ACADOS solver...", end="", flush=True)
    z0_warm = np.zeros(mpc.nx)
    z0_warm[3] = 5.0
    x_ref_w = np.linspace(0, 20, N_horizon + 1)
    y_ref_w = np.zeros(N_horizon + 1)
    psi_ref_w = np.zeros(N_horizon + 1)
    v_ref_w = 5.0 * np.ones(N_horizon + 1)
    for _ in range(5):
        warm_kwargs = dict(terrain_params=terrain_params)
        if tire_hist is not None:
            warm_kwargs['hist_front'] = tire_hist.front
            warm_kwargs['hist_rear'] = tire_hist.rear
        mpc.solve(z0_warm, x_ref_w, y_ref_w, psi_ref_w, v_ref_w, x_ref_w[-1], 0, 0,
                  **warm_kwargs)
        print(".", end="", flush=True)
    mpc.reset_warmstart()  # force kinematic rollout from real z0
    print(" done!")

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
    )
    path_func = ref_path.get_reference

    # ------------------------------------------------------------------
    # Transport delay compensation
    # ------------------------------------------------------------------
    delay_est = DelayEstimator(initial_delay=args.initial_delay)
    state_predictor = StatePredictor(vehicle_params, dt_prop=0.005)
    control_buffer = collections.deque(maxlen=50)

    # ------------------------------------------------------------------
    # Control integrator
    # ------------------------------------------------------------------
    integrator = ControlIntegrator(mpc, v_target=v_target)

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
    # Steering-rate input to NN (matches 'steering_rate' feature used in training).
    # We don't get measured road-wheel rate from the sim, so we feed the most
    # recent MPC command (delta_dot) as a consistent proxy.
    last_delta_dot_cmd = 0.0
    last_Jx_cmd = 0.0
    if args.terrain_classifier:
        terrain_class_est = _DEFAULT_TERRAIN_CLASS
        n_terrain_est = _TERRAIN_N_LOOKUP[_DEFAULT_TERRAIN_CLASS]
        terrain_params_est = dict(_TERRAIN_PARAMS_LOOKUP[_DEFAULT_TERRAIN_CLASS])
    else:
        terrain_class_est = terrain_name
        n_terrain_est = terrain_params.get("n", 1.1)
        terrain_params_est = {k: terrain_params[k] for k in ("Kphi", "Kc", "n", "c", "phi", "k")}
    terrain_confidence = 0.0 if args.terrain_classifier else 1.0

    # ------------------------------------------------------------------
    # Diagnostic CSV logger
    # ------------------------------------------------------------------
    csv_file = None
    csv_writer = None
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
            "steering", "throttle", "braking", "steering_angle", "acceleration",
            "tau_one_way_ms", "tau_solve_ms", "tau_comp_ms", "solve_time_ms",
            "crosstrack_err", "heading_err_deg", "speed_err",
            "actual_Fy_front", "actual_Fy_rear", "pred_Fy_front", "pred_Fy_rear",
            "actual_Fx_front", "actual_Fx_rear", "pred_Fx_front", "pred_Fx_rear",
            "alpha_f", "alpha_r", "Fz_f_mean", "Fz_r_mean",
        ]
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(csv_header)
        print(f"  Diagnostic CSV: {csv_path}")

    print(f"  Delay compensation: {'ON' if not args.no_delay_comp else 'OFF'} "
          f"(initial τ={args.initial_delay * 1000:.0f}ms)")
    print(f"  Running ACADOS controller loop...")

    running = True
    while running:
        # --- Receive state ---
        result = state_sub.recv(timeout_ms=200)
        if result is None:
            continue

        topic, msg = result

        if isinstance(msg, SimStatus):
            if msg.event == "stop":
                print("  Received stop signal from sim node.")
                running = False
                break
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
                    n_terrain_est = _TERRAIN_N_LOOKUP[tc_class]
                    terrain_params_est = dict(_TERRAIN_PARAMS_LOOKUP[tc_class])

        # Update delay estimate
        delay_est.update_transport(msg.wall_time, recv_time)

        # --- Build state vector ---
        psi = quat_to_yaw(msg.quat_e0, msg.quat_e1, msg.quat_e2, msg.quat_e3)
        Lf = mpc.Lf

        # Transform CG → front axle
        x_fa = msg.x_cg + Lf * np.cos(psi)
        y_fa = msg.y_cg + Lf * np.sin(psi)

        z0_measured = np.array([
            x_fa, y_fa, psi,
            max(msg.u, 0.5),
            msg.v, msg.omega,
            integrator.acceleration,
            integrator.steering_angle,  # δ_prev
            last_Jx_cmd,               # Jx_prev
        ])

        # --- Delay compensation ---
        # StatePredictor expects 8-state [x,y,ψ,u,v,ω,δ,ax].
        # Reorder our 9-state for it, propagate, then reorder back.
        if not args.no_delay_comp:
            tau = delay_est.compensation_delay
            z8 = np.array([
                z0_measured[0], z0_measured[1], z0_measured[2],
                z0_measured[3], z0_measured[4], z0_measured[5],
                z0_measured[7],  # δ_prev → StatePredictor's δ
                z0_measured[6],  # ax
            ])
            z8_pred = state_predictor.propagate(z8, control_buffer, tau)
            z0 = np.array([
                z8_pred[0], z8_pred[1], z8_pred[2],
                z8_pred[3], z8_pred[4], z8_pred[5],
                z8_pred[7],  # ax
                z8_pred[6],  # δ → δ_prev
                z0_measured[8],  # Jx_prev unchanged
            ])
        else:
            z0 = z0_measured
            tau = 0.0

        # --- Generate reference trajectory ---
        x_ref, y_ref, psi_ref, v_ref, x_goal, y_goal, psi_goal = path_func(
            msg.time, z0, mpc.N, mpc.dt
        )

        # --- Compute per-tire operating conditions ---
        u_safe_h = max(abs(msg.u), 0.5)
        alpha_f_h = integrator.steering_angle - math.atan2(
            msg.v + mpc.Lf * msg.omega, u_safe_h)
        alpha_r_h = -math.atan2(
            msg.v - mpc.Lr * msg.omega, u_safe_h)
        g_h = 9.81
        ax_h = integrator.acceleration
        L_h = mpc.Lf + mpc.Lr
        Fz_f_h = (mpc.M * g_h * mpc.Lr - mpc.M * ax_h * mpc.h_cg) / L_h / 2.0
        Fz_r_h = (mpc.M * g_h * mpc.Lf + mpc.M * ax_h * mpc.h_cg) / L_h / 2.0
        # Slip ratio estimate to match the solver's internal convention.
        # This is critical for rate-augmented and temporal models that expect
        # consistent (kappa, dkappa/dt) features.
        if args.kappa == "approx":
            kappa_h = float(np.clip(ax_h / (0.4 * 9.81), -0.3, 0.3))
        else:
            kappa_h = 0.0
        sr_h = 0.0  # no δ̇ to feed as steering rate

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
        delta_cmd, Jx, Z_opt, U_opt = mpc.solve(
            z0, x_ref, y_ref, psi_ref, v_ref,
            x_goal, y_goal, psi_goal,
            **solve_kwargs,
        )
        t_solve = wall_time.time() - t0_solve
        solve_times.append(t_solve)
        delay_est.update_solve(t_solve)

        if Z_opt is None:
            delta_cmd, Jx = integrator.steering_angle, 0.0

        if not np.isfinite(delta_cmd):
            delta_cmd = integrator.steering_angle
        if not np.isfinite(Jx):
            Jx = 0.0

        # Suppress steering during lead-in acceleration phase
        if lead_in > 0 and z0[0] < lead_in and msg.u < 0.8 * v_target:
            delta_cmd = 0.0

        # Compute effective δ̇ for logging (not used for control)
        delta_dot = (delta_cmd - integrator.steering_angle) / max(dt_ctrl, 1e-4)

        last_delta_dot_cmd = float(delta_cmd)  # stores δ_prev for next solve
        last_Jx_cmd = float(Jx)

        # --- Update tire history after solve ---
        if tire_hist is not None:
            tire_hist.update(
                kappa_h, alpha_f_h, u_safe_h, Fz_f_h, sr_h,
                kappa_h, alpha_r_h, Fz_r_h, sr_h,
            )

        # --- Update rate tracker after solve ---
        if rate_tracker is not None:
            # Keep finite-difference rates consistent with actual controller cadence.
            rate_tracker.dt = dt_ctrl
            rate_tracker.update(
                kappa_h, alpha_f_h, u_safe_h,
                kappa_h, alpha_r_h, u_safe_h,
            )

        # --- Apply controls: set δ directly, integrate ax from Jx ---
        integrator.steering_angle = float(np.clip(
            delta_cmd, -mpc.delta_max, mpc.delta_max))
        # Use integrator.update with delta_dot=0 (steering already set)
        # to get throttle/braking from the existing Jx integration logic.
        _saved_delta = integrator.steering_angle
        _, throttle, braking = integrator.update(0.0, Jx, dt_ctrl, msg.u)
        integrator.steering_angle = _saved_delta
        steering = float(np.clip(
            integrator.steering_angle / mpc.delta_max, -1.0, 1.0))

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
            analytics.record(msg.time, true_x_fa, true_y_fa, true_psi, true_u)
        else:
            analytics.record(msg.time, z0_measured[0], z0_measured[1], psi, msg.u)
        analytics.record_control(
            msg.time, steering, throttle, braking,
            integrator.steering_angle, integrator.acceleration,
            t_solve * 1000.0, delay_est.compensation_delay * 1000.0,
        )

        # --- Record tire forces: Chrono actual vs model predicted ---
        if msg.tire_forces is not None:
            tf = msg.tire_forces
            actual_Fy_f = tf.get('front_left_Fy', 0) + tf.get('front_right_Fy', 0)
            actual_Fy_r = tf.get('rear_left_Fy', 0) + tf.get('rear_right_Fy', 0)
            actual_Fx_f = tf.get('front_left_Fx', 0) + tf.get('front_right_Fx', 0)
            actual_Fx_r = tf.get('rear_left_Fx', 0) + tf.get('rear_right_Fx', 0)

            u_safe = max(abs(msg.u), 0.5)
            alpha_f = integrator.steering_angle - math.atan2(
                msg.v + mpc.Lf * msg.omega, u_safe)
            alpha_r = -math.atan2(
                msg.v - mpc.Lr * msg.omega, u_safe)

            g = 9.81
            ax = integrator.acceleration
            L = mpc.Lf + mpc.Lr
            Fz_f_mean = (mpc.M * g * mpc.Lr - mpc.M * ax * mpc.h_cg) / L / 2.0
            Fz_r_mean = (mpc.M * g * mpc.Lf + mpc.M * ax * mpc.h_cg) / L / 2.0

            pred_Fy_f, pred_Fy_r = 0.0, 0.0
            pred_Fx_f, pred_Fx_r = 0.0, 0.0

            if nn_tire is not None:
                hist_f = tire_hist.front if tire_hist is not None else None
                hist_r = tire_hist.rear if tire_hist is not None else None
                rates_f = rate_tracker.front if rate_tracker is not None else None
                rates_r = rate_tracker.rear if rate_tracker is not None else None
                if mpc.lateral_load_transfer:
                    ay = msg.u * msg.omega
                    dFz = mpc.M * ay * mpc.h_cg / mpc.T / 2.0
                    Fz_fo = min(Fz_f_mean + dFz, 1.9 * Fz_f_mean)
                    Fz_fi = max(Fz_f_mean - dFz, 0.1 * Fz_f_mean)
                    Fz_ro = min(Fz_r_mean + dFz, 1.9 * Fz_r_mean)
                    Fz_ri = max(Fz_r_mean - dFz, 0.1 * Fz_r_mean)
                    Fx_fo, Fy_fo = nn_tire.predict_numeric(
                        alpha_f, Fz_fo, u_safe, n_terrain=n_terrain_est,
                        terrain_params=terrain_params_est, hist=hist_f, rates=rates_f)
                    Fx_fi, Fy_fi = nn_tire.predict_numeric(
                        alpha_f, Fz_fi, u_safe, n_terrain=n_terrain_est,
                        terrain_params=terrain_params_est, hist=hist_f, rates=rates_f)
                    Fx_ro, Fy_ro = nn_tire.predict_numeric(
                        alpha_r, Fz_ro, u_safe, n_terrain=n_terrain_est,
                        terrain_params=terrain_params_est, hist=hist_r, rates=rates_r)
                    Fx_ri, Fy_ri = nn_tire.predict_numeric(
                        alpha_r, Fz_ri, u_safe, n_terrain=n_terrain_est,
                        terrain_params=terrain_params_est, hist=hist_r, rates=rates_r)
                    pred_Fy_f = -(Fy_fo + Fy_fi)
                    pred_Fy_r = -(Fy_ro + Fy_ri)
                    pred_Fx_f = Fx_fo + Fx_fi
                    pred_Fx_r = Fx_ro + Fx_ri
                else:
                    Fx_fw, Fy_fw = nn_tire.predict_numeric(
                        alpha_f, Fz_f_mean, u_safe, n_terrain=n_terrain_est,
                        terrain_params=terrain_params_est, hist=hist_f, rates=rates_f)
                    Fx_rw, Fy_rw = nn_tire.predict_numeric(
                        alpha_r, Fz_r_mean, u_safe, n_terrain=n_terrain_est,
                        terrain_params=terrain_params_est, hist=hist_r, rates=rates_r)
                    pred_Fy_f = -2.0 * Fy_fw
                    pred_Fy_r = -2.0 * Fy_rw
                    pred_Fx_f = 2.0 * Fx_fw
                    pred_Fx_r = 2.0 * Fx_rw

            analytics.record_tire_forces(
                msg.time, actual_Fy_f, actual_Fy_r, pred_Fy_f, pred_Fy_r,
                actual_Fx_f, actual_Fx_r, pred_Fx_f, pred_Fx_r)

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
            fx_af = analytics.actual_Fx_front[-1] if analytics.actual_Fx_front else ''
            fx_ar = analytics.actual_Fx_rear[-1] if analytics.actual_Fx_rear else ''
            fx_nf = analytics.pred_Fx_front[-1] if analytics.pred_Fx_front else ''
            fx_nr = analytics.pred_Fx_rear[-1] if analytics.pred_Fx_rear else ''

            u_safe_csv = max(abs(msg.u), 0.5)
            alpha_f_csv = integrator.steering_angle - math.atan2(
                msg.v + mpc.Lf * msg.omega, u_safe_csv)
            alpha_r_csv = -math.atan2(
                msg.v - mpc.Lr * msg.omega, u_safe_csv)
            g_csv = 9.81
            L_csv = mpc.Lf + mpc.Lr
            Fz_f_csv = (mpc.M * g_csv * mpc.Lr - mpc.M * integrator.acceleration * mpc.h_cg) / L_csv / 2.0
            Fz_r_csv = (mpc.M * g_csv * mpc.Lf + mpc.M * integrator.acceleration * mpc.h_cg) / L_csv / 2.0

            mpc_cost = getattr(mpc, 'last_cost', float('nan'))
            solver_status = getattr(mpc, 'last_solver_status', '')
            solver_iters = getattr(mpc, 'last_iter_count', -1)

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
                f"{steering:.6f}", f"{throttle:.4f}", f"{braking:.4f}",
                f"{integrator.steering_angle:.6f}", f"{integrator.acceleration:.4f}",
                f"{delay_est.one_way_delay*1000:.2f}",
                f"{delay_est.solve_time*1000:.2f}",
                f"{delay_est.compensation_delay*1000:.2f}",
                f"{t_solve*1000:.2f}",
                f"{ct_err:.6f}", f"{hd_err:.4f}", f"{sp_err:.4f}",
                fy_af, fy_ar, fy_nf, fy_nr,
                fx_af, fx_ar, fx_nf, fx_nr,
                f"{alpha_f_csv:.6f}", f"{alpha_r_csv:.6f}",
                f"{Fz_f_csv:.1f}", f"{Fz_r_csv:.1f}",
            ])

        seq += 1

        # --- Periodic report ---
        if seq % 20 == 0:
            mean_ms = np.mean(solve_times[-20:]) * 1000
            tau_ms = delay_est.compensation_delay * 1000
            trk = analytics.periodic_summary()
            tc_str = f"  terrain={terrain_class_est}({terrain_confidence:.0%})" if terrain_sub else ""
            print(f"  t={msg.time:.1f}s  solve={mean_ms:.1f}ms  "
                  f"τ_comp={tau_ms:.1f}ms  {trk}  "
                  f"u={msg.u:.2f}m/s{tc_str}")

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

    print(analytics.final_summary())

    # Close CSV
    if csv_file is not None:
        csv_file.close()
        print(f"  Diagnostic CSV written: {csv_path} ({seq} rows)")

    if not args.no_plot:
        analytics.plot_results(
            plot_dir=str(run_dir),
            terrain_name=terrain_name,
            model_label=model_label,
        )

    ctrl_pub.close()
    state_sub.close()
    if terrain_sub is not None:
        terrain_sub.close()


# =============================================================================
# Entry point
# =============================================================================

def main():
    p = argparse.ArgumentParser(description="ACADOS MPC Controller Node (decoupled)")

    # Model (NN or analytical tire model)
    p.add_argument("--model", default="nn",
                   choices=["nn", "pacejka", "tmeasy", "linear"],
                   help="Tire model: nn (neural network), pacejka, tmeasy, or linear")
    p.add_argument("--nn-model", default="v6_mlp_16_4",
                   help="NN model version directory (only used when --model nn)")
    p.add_argument("--kappa", default="zero", choices=["zero", "approx"])
    p.add_argument("--no-lat-transfer", action="store_true",
                   help="Disable lateral load transfer (2 NN calls vs 4)")

    # Path
    p.add_argument("--path", default="lane_change",
                   choices=["lane_change", "double_lane_change", "sinusoidal"])
    p.add_argument("--speed", type=float, default=5.0, help="Target speed (m/s)")
    p.add_argument("--sine-amplitude", type=float, default=2.0)
    p.add_argument("--sine-wavelength", type=float, default=30.0)
    p.add_argument("--lead-in", type=float, default=0.0,
                   help="Straight lead-in distance (m) before path starts")
    p.add_argument("--no-path-reindex", action="store_true")

    # Terrain
    p.add_argument("--terrain", default="sand", choices=["sand", "clay", "dirt"])
    p.add_argument("--time", type=float, default=15.0, help="Expected sim duration")

    # Delay compensation
    p.add_argument("--no-delay-comp", action="store_true",
                   help="Disable transport delay compensation in MPC")
    p.add_argument("--initial-delay", type=float, default=0.02,
                   help="Initial one-way delay estimate (s)")

    # Analytics
    p.add_argument("--rms-time-start", type=float, default=2.0,
                   help="Start time for RMS calculation, skips startup (s)")
    p.add_argument("--no-plot", action="store_true",
                   help="Skip generating end-of-run plots")
    p.add_argument("--no-csv", action="store_true",
                   help="Skip diagnostic CSV output")
    p.add_argument("--plot-dir", default="plots",
                   help="Directory for output plots (default: plots/)")

    # Network
    p.add_argument("--sim-host", default="localhost", help="Sim node host")
    p.add_argument("--sim-port", type=int, default=5555, help="Sim state port")
    p.add_argument("--ctrl-port", type=int, default=5556, help="Control command port")

    # Terrain classifier
    p.add_argument("--terrain-classifier", action="store_true",
                   help="Subscribe to terrain classifier estimates")
    p.add_argument("--tc-port", type=int, default=5557,
                   help="Terrain classifier publish port to subscribe to")

    args = p.parse_args()
    run_controller_node(args)


if __name__ == "__main__":
    main()
