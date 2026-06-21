#!/usr/bin/env python3
"""
Launch script for decoupled Chrono simulation + MPC controller.
================================================================

Starts two processes:
  1. chrono_sim_node.py   — PyChrono HMMWV simulation (publishes state, receives commands)
  2. acados_mpc_controller_node.py — ACADOS MPC controller (receives state, publishes commands)

Usage:
    # Default (NN model, sand terrain, lane change, irrlicht visualization)
    python launch_decoupled.py

    # Sinusoidal path on clay, headless, 30s
    python launch_decoupled.py --path sinusoidal --terrain clay --no-vis --time 30

    # Sensor-only visualization (driver POV camera)
    python launch_decoupled.py --vis-mode sensor

    # Both irrlicht chase cam and sensor driver POV camera
    python launch_decoupled.py --vis-mode both

    # TMeasy MPC tire model
    python launch_decoupled.py --model tmeasy

    # Pacejka MPC tire model with sensor visualization
    python launch_decoupled.py --model pacejka --vis-mode sensor

    # Remote controller (sim on this machine, controller elsewhere)
    python launch_decoupled.py --ctrl-host 192.168.1.50
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


def main():
    p = argparse.ArgumentParser(
        description="Launch decoupled Chrono sim + MPC controller",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --path sinusoidal --terrain clay --time 20
  %(prog)s --model pacejka                   # Pacejka Magic Formula MPC (rigid-terrain params)
  %(prog)s --model pacejka-oracle --terrain clay  # Oracle Pacejka (terrain-fitted, upper bound)
  %(prog)s --model tmeasy                    # TMeasy MPC tire model
  %(prog)s --vis-mode sensor                 # Driver POV via Chrono Sensor
  %(prog)s --vis-mode both                   # Irrlicht + Sensor simultaneously
  %(prog)s --sim-only          # Only start the sim node (controller started separately)
  %(prog)s --ctrl-only         # Only start the controller node
""",
    )

    # Shared args
    p.add_argument("--time", type=float, default=15.0, help="Simulation time (s)")
    p.add_argument("--speed", type=float, default=5.0, help="Target speed (m/s)")
    p.add_argument("--terrain", default="sand", choices=["sand", "clay", "dirt"])
    p.add_argument("--terrain-config", type=str, default=None)
    p.add_argument("--path", default="lane_change",
                   choices=["lane_change", "double_lane_change", "right_left", "sinusoidal"])
    p.add_argument("--sine-amplitude", type=float, default=2.0)
    p.add_argument("--sine-wavelength", type=float, default=30.0)
    p.add_argument("--lead-in", type=float, default=0.0,
                   help="Straight lead-in distance (m) before path starts")
    p.add_argument("--no-vis", action="store_true", help="Headless simulation (alias for --vis-mode none)")
    p.add_argument("--vis-mode", default=None,
                   choices=["irrlicht", "sensor", "both", "none"],
                   help="Visualization mode: irrlicht, sensor (driver POV), both, or none")
    p.add_argument("--irrlicht-window-size", type=int, nargs=2,
                   metavar=("WIDTH", "HEIGHT"), default=[4320, 720],
                   help="Irrlicht window size in pixels")
    p.add_argument("--no-rt",  action="store_true",
                   help="Disable real-time pacing (fast-forward; breaks MPC sync)")
    p.add_argument("--no-noise", action="store_true",
                   help="Disable sensor noise (noise ON by default)")
    p.add_argument("--sim-diag-csv", default="",
                   help="Write sim-side state/control diagnostics to this CSV. "
                        "Useful for manual/HIL rounds where no controller diag exists.")
    p.add_argument("--latency-profile-json", default="",
                   help="JSON profile for time-varying 5G-like one-way latency. "
                        "Forwarded to the sim for control/manual/camera channels.")
    p.add_argument("--latency-profile-log", default="",
                   help="Optional CSV path for logging active latency samples from the sim.")

    # IMU sensor (Chrono sensor module)
    p.add_argument("--no-imu", action="store_true",
                   help="Disable Chrono sensor-module IMU (use analytical ground-truth accel/gyro)")
    p.add_argument("--imu-rate", type=int, default=100,
                   help="IMU update rate in Hz (default 100)")
    p.add_argument("--imu-lag", type=float, default=0.0,
                   help="IMU sensor lag in seconds (default 0)")
    p.add_argument("--imu-acc-stdev", type=float, default=0.015,
                   help="Accelerometer noise stdev in m/s² (default 0.015)")
    p.add_argument("--imu-gyro-stdev", type=float, default=0.001,
                   help="Gyroscope noise stdev in rad/s (default 0.001)")

    # Controller-specific
    p.add_argument("--controller-mode", default="standard",
                   choices=["standard", "mpcc"],
                   help="standard: reference-tracking MPC with curvature-derived v_ref. "
                        "mpcc: Model Predictive Contouring Control — drops the speed "
                        "reference, optimizer picks its own path-progress velocity "
                        "subject to a soft curvature speed cap.")
    p.add_argument("--mpcc-N", type=int, default=20)
    p.add_argument("--mpcc-dt", type=float, default=0.1)
    p.add_argument("--mpcc-w-contour", type=float, default=3000.0)
    p.add_argument("--mpcc-w-lag", type=float, default=2000.0)
    p.add_argument("--mpcc-w-progress", type=float, default=0.5)
    p.add_argument("--mpcc-w-delta-dot", type=float, default=80.0)
    p.add_argument("--mpcc-w-speed-cap", type=float, default=300.0)
    p.add_argument("--mpcc-friction-ellipse", action="store_true")
    p.add_argument("--mpcc-vtheta-max", type=float, default=5.0)
    p.add_argument("--mpcc-diag-csv", default="",
                   help="(MPCC only) path to write per-step diagnostic CSV.")
    p.add_argument("--model", default="nn",
                   choices=["nn", "pacejka", "pacejka-oracle", "tmeasy"],
                   help="MPC tire model: nn, pacejka (rigid-terrain defaults), "
                        "pacejka-oracle (terrain-fitted params, oracle upper bound), "
                        "or tmeasy")
    p.add_argument("--speed-weight", type=float, default=70.0,
                   help="Standard-MPC speed tracking weight. Lower values reduce "
                        "reference-speed chasing in turns.")
    p.add_argument("--speed-cost-mode", choices=["symmetric", "overspeed"],
                   default="symmetric",
                   help="Standard-MPC speed cost: track v_ref symmetrically or "
                        "treat v_ref as an overspeed cap.")
    p.add_argument("--obstacle-weight", type=float, default=5e3,
                   help="Standard-MPC soft obstacle-barrier weight.")
    p.add_argument("--nn-model", default="closed_loop_v2_both_axles_rate_32_16")
    p.add_argument("--kappa", default="measured", choices=["zero", "approx", "measured"])
    p.add_argument("--no-lat-transfer", action="store_true")
    p.add_argument("--no-delay-comp", action="store_true")
    p.add_argument("--no-path-reindex", action="store_true")
    p.add_argument("--no-temporal-staged", action="store_true",
                   help="Disable stage-varying temporal history")
    p.add_argument(
        "--symbolic-rates",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Compute rate features symbolically in MPC dynamics (default: on). "
             "Use --no-symbolic-rates to disable.",
    )
    p.add_argument("--rms-time-start", type=float, default=2.0,
                   help="Start time for RMS calculation (s)")
    p.add_argument("--dob-ki", type=float, default=0.15,
                   help="Throttle DOB integrator gain [throttle/(m/s)/s]; 0 disables DOB")
    p.add_argument("--dob-max", type=float, default=0.35,
                   help="Asymmetric upper clip on the DOB throttle bias")
    p.add_argument("--dob-bleed", type=float, default=0.5,
                   help="Exponential bleed rate of DOB during MPC braking [1/s]")
    p.add_argument("--no-plot", action="store_true",
                   help="Skip generating end-of-run plots")
    p.add_argument("--live-plot", action="store_true",
                   help="Open live matplotlib debug window")
    p.add_argument("--live-plot-every", type=int, default=5,
                   help="Redraw live debug plot every N steps (default: 5)")
    p.add_argument("--no-csv", action="store_true",
                   help="Skip diagnostic CSV output")
    p.add_argument(
        "--log-tire-csv",
        default=None,
        metavar="PATH",
        help="Forward to controller: append MPC-aligned tire training rows (see tire_input_features.py)",
    )
    p.add_argument(
        "--log-rich-tire-csv",
        default=None,
        metavar="PATH",
        help="Forward to controller: append rich sensor-realistic tire training rows.",
    )
    p.add_argument(
        "--log-scenario-id",
        type=int,
        default=0,
        help="scenario_id column when using --log-tire-csv",
    )
    p.add_argument("--plot-dir", default="plots",
                   help="Directory for output plots")

    # Terrain bumpiness
    p.add_argument("--bumpiness", type=int, default=0, choices=range(0, 11),
                   help="Terrain bumpiness level 0 (flat) to 10 (extreme)")

    # Rock obstacles
    p.add_argument("--rocks", type=int, default=0,
                   help="Number of rock obstacles (0 = none)")
    p.add_argument("--rock-zone-x", type=float, nargs=2, default=[-15.0, 50.0])
    p.add_argument("--rock-zone-y", type=float, nargs=2, default=[-10.0, 10.0])
    p.add_argument("--rock-size", type=float, nargs=2, default=[0.5, 3.0])
    p.add_argument("--rock-seed", type=int, default=42)

    # Safety filter
    p.add_argument("--safety-filter", action="store_true",
                   help="Enable safety filter (flavor selected via --safety-flavor)")
    p.add_argument("--safety-flavor", type=str, default="mppi",
                   choices=["mppi", "nmpc", "dob_cbf"],
                   help="mppi (primary), nmpc (ablation), dob_cbf (legacy).")
    p.add_argument("--no-safety-nn", action="store_true",
                   help="Disable NN tire model inside the sim-side safety filter. "
                        "Useful for DOB-CBF NN ablations.")
    p.add_argument("--mppi-samples", type=int, default=384)
    p.add_argument("--mppi-sigma-steer", type=float, default=0.35)
    p.add_argument("--mppi-sigma-alpha", type=float, default=0.35)
    p.add_argument("--mppi-temperature", type=float, default=1.0)
    p.add_argument("--mppi-no-seeds", action="store_true",
                   help="Ablation: disable hand-crafted MPPI seed trajectories.")
    p.add_argument("--shield-no-sigma-gate", action="store_true",
                   help="Ablation: zero out phi_sigma at the sim-node hop (equivalent to mode=off).")
    p.add_argument("--shield-sigma-mode", type=str, default="off",
                   choices=["tighten", "inflate", "both", "off"],
                   help="How shield uses phi_sigma. Default off: shield runs on "
                        "initial terrain (paper Sec. IX-B). tighten/inflate/both "
                        "retained for the sigma_gate_ablation experiment only.")
    p.add_argument("--shield-sigma-buffer-gain", type=float, default=0.05,
                   help="Metres of extra obstacle buffer per degree of phi_sigma.")
    p.add_argument("--nmpc-iter", type=int, default=6)
    p.add_argument("--shield-horizon", type=int, default=12)
    p.add_argument("--mpc-blind-obstacles", action="store_true",
                   help="Make the MPC controller ignore obstacles — safety shield "
                        "becomes the sole collision-avoider.")
    p.add_argument("--cbf-alpha", type=float, default=1.0)
    p.add_argument("--safety-buffer", type=float, default=0.25)
    p.add_argument("--delay-steps", type=int, default=5)
    p.add_argument("--cbf-w-long", type=float, default=0.15)
    p.add_argument("--cbf-w-lat", type=float, default=0.50)
    p.add_argument("--cbf-forward-bias", type=float, default=1.5)
    p.add_argument("--dob-bandwidth", type=float, default=10.0)
    p.add_argument("--cbf-flavor", type=str, default="balance",
                   choices=["balance", "steer_priority", "throttle_priority"])
    p.add_argument("--teleop-delay", type=float, default=0.0,
                   help="Initial one-way teleop delay in seconds (0 = local)")
    p.add_argument("--stale-cmd-timeout", type=float, default=2.0,
                   help="Auto-brake if no command for this many seconds")

    # Network
    p.add_argument("--sim-port", type=int, default=5555)
    p.add_argument("--ctrl-port", type=int, default=5556)
    p.add_argument("--ctrl-host", default="localhost",
                   help="Host running the controller (for sim to subscribe)")

    # Mode
    p.add_argument("--sim-only", action="store_true",
                   help="Only launch the simulation node")
    p.add_argument("--ctrl-only", action="store_true",
                   help="Only launch the controller node")
    p.add_argument("--manual", action="store_true",
                   help="Manual control with G29 steering wheel (no MPC controller)")
    p.add_argument("--wasd", action="store_true",
                   help="Manual control with WASD keyboard (no MPC controller)")
    p.add_argument("--manual-honor-time", action="store_true",
                   help="In manual mode, stop automatically at --time instead of "
                        "requiring the driver to close the window.")
    p.add_argument("--manual-input-delay", type=float, default=0.0,
                   help="Apply a fixed actuation delay to manual steering/throttle/brake inputs.")
    p.add_argument("--camera-input-delay", type=float, default=0.0,
                   help="Apply a fixed lag to the driver POV camera feed (models "
                        "downlink video latency to the operator).")

    # Terrain classifier
    p.add_argument("--terrain-classifier", action="store_true",
                   help="Launch terrain classifier node alongside sim + controller")
    p.add_argument("--use-prediction", action="store_true",
                   help="When terrain classifier is enabled, apply predicted terrain to MPC parameters")
    p.add_argument("--prediction-min-confidence", type=float, default=0.0,
                   help="Controller gate for applying classifier terrain updates to MPC [0,1]")
    p.add_argument("--tc-model", default="terrain_classifier/models/terrain_rf.pkl",
                   help="Path to trained terrain classifier model")
    p.add_argument("--tc-port", type=int, default=5557,
                   help="Port for terrain classifier to publish estimates")
    p.add_argument("--tc-ema-alpha", type=float, default=0.3,
                   help="EMA smoothing for terrain classifier (0=smooth, 1=raw)")

    # Online terrain parameter estimator
    p.add_argument("--terrain-estimator", action="store_true",
                   help="Enable online terrain parameter estimation from speed capability "
                        "and inertial cues (sensor-realistic, replaces classifier for MPC param updates)")
    p.add_argument("--te-window", type=int, default=50)
    p.add_argument("--te-update-interval", type=int, default=10)
    p.add_argument("--te-lr", type=float, default=0.01)
    p.add_argument("--te-steps", type=int, default=20)
    p.add_argument("--te-min-excitation", type=float, default=0.3)
    p.add_argument("--te-min-confidence", type=float, default=0.3)
    p.add_argument("--learned-terrain-model-dir", default=None,
                   help="Path to the retained sliding-window terrain-estimator checkpoint "
                        "(defaults to nn_models/terrain_window_mlp)")
    p.add_argument("--te-verbose", action="store_true",
                   help="Print verbose terrain-estimator predictions in the "
                        "controller (useful for offline log parsing)")

    # Force-level residual (corrects Fy across horizon)
    p.add_argument("--force-residual", action="store_true",
                   help="Enable force-level residual correction (ΔFy per horizon stage)")
    p.add_argument("--force-residual-checkpoint", default=None,
                   help="Path to force_residual_model.pt")
    p.add_argument("--force-residual-online", action="store_true",
                   help="Enable online EMA bias adaptation for force residual (off by default)")
    p.add_argument("--force-residual-clip", type=float, default=500.0,
                   help="Symmetric clip on ΔFy corrections (N)")
    p.add_argument("--force-residual-gain", type=float, default=1.0,
                   help="Output scaling for force residual (<1 = conservative)")
    p.add_argument("--force-residual-online-lr", type=float, default=0.03,
                   help="EMA alpha for force residual online bias adaptation (0.01–0.15)")

    p.add_argument("--gp-max-inducing", type=int, default=200,
                   help="Maximum number of inducing points in sparse GP")
    p.add_argument("--gp-noise-var", type=float, default=0.1,
                   help="GP observation noise variance")

    # Dynamics GP (persistent bicycle model residual learning)
    p.add_argument("--dynamics-gp", action="store_true",
                   help="Enable GP-based persistent dynamics residual learning [Δu̇,Δv̇,Δω̇]")
    p.add_argument("--dynamics-gp-state", default="data/gp_residual/dynamics_gp_state.npz",
                   help="Path to dynamics GP state file. Relative paths are resolved from the project root.")
    p.add_argument("--dynamics-gp-clip", type=float, default=2.0,
                   help="Symmetric clip on dynamics residuals (m/s² or rad/s²)")
    p.add_argument("--dynamics-gp-gain", type=float, default=0.5,
                   help="Output scaling for dynamics GP (<1 = conservative, default: 0.5)")
    p.add_argument("--gp-uncertainty-speed", action="store_true",
                   help="Scale v_ref down proportionally to GP variance; recovers full speed as GP learns")
    p.add_argument("--gp-speed-scale-min", type=float, default=0.7,
                   help="Minimum speed scale when GP is fully uncertain (default: 0.7)")
    p.add_argument("--gp-terrain-gate", nargs="*", default=[],
                   help="Only apply GP corrections when terrain estimate is in this list (e.g. clay)")

    p.add_argument("--ax-filter-tau", type=float, default=0.5,
                   help="Complementary filter time constant (s) for IMU ax (0 = no filter)")
    p.add_argument("--vel-filter-tau", type=float, default=0.05,
                   help="EMA time constant (s) for smoothing noisy [u, v, omega] (0 = off)")

    args = p.parse_args()
    script_dir = Path(__file__).resolve().parent
    project_root = script_dir.parent
    if args.latency_profile_json:
        profile_path = Path(args.latency_profile_json).expanduser()
        args.latency_profile_json = str(profile_path.resolve())
    if args.use_prediction:
        args.terrain_classifier = True
    # Default lead-in for sinusoidal path (cold-start infeasibility without it)
    if args.path == 'sinusoidal' and args.lead_in == 0.0:
        args.lead_in = 0.0
    if args.force_residual and not args.force_residual_checkpoint:
        p.error("--force-residual requires --force-residual-checkpoint")

    if args.dynamics_gp:
        gp_state = Path(args.dynamics_gp_state).expanduser()
        if not gp_state.is_absolute():
            gp_state = project_root / gp_state
        args.dynamics_gp_state = str(gp_state.resolve())

    # Resolve vis mode: --no-vis is shorthand for --vis-mode none
    vis_mode = args.vis_mode
    if vis_mode is None:
        vis_mode = 'none' if args.no_vis else 'irrlicht'

    # ---- Build command lines ----
    sim_cmd = [
        sys.executable, str(script_dir / "chrono_sim_node.py"),
        "--time", str(args.time),
        "--speed", str(args.speed),
        "--terrain", args.terrain,
        "--path", args.path,
        "--sine-amplitude", str(args.sine_amplitude),
        "--sine-wavelength", str(args.sine_wavelength),
        "--lead-in", str(args.lead_in),
        "--sim-port", str(args.sim_port),
        "--ctrl-host", args.ctrl_host,
        "--ctrl-port", str(args.ctrl_port),
        "--bumpiness", str(args.bumpiness),
        "--vis-mode", vis_mode,
        "--irrlicht-window-size", str(args.irrlicht_window_size[0]),
        str(args.irrlicht_window_size[1]),
    ]
    if args.no_rt:
        sim_cmd.append("--no-rt")
    if args.no_noise:
        sim_cmd.append("--no-noise")
    if args.sim_diag_csv:
        sim_cmd.extend(["--sim-diag-csv", args.sim_diag_csv])
    if args.latency_profile_json:
        sim_cmd.extend(["--latency-profile-json", args.latency_profile_json])
    if args.latency_profile_log:
        sim_cmd.extend(["--latency-profile-log", args.latency_profile_log])
    if args.manual:
        sim_cmd.append("--manual")
    if args.wasd:
        sim_cmd.append("--wasd")
    if args.manual_honor_time:
        sim_cmd.append("--manual-honor-time")
    if args.manual_input_delay > 0:
        sim_cmd.extend(["--manual-input-delay", str(args.manual_input_delay)])
    if args.camera_input_delay > 0:
        sim_cmd.extend(["--camera-input-delay", str(args.camera_input_delay)])
    if args.teleop_delay > 0:
        sim_cmd.extend(["--teleop-delay", str(args.teleop_delay)])
        sim_cmd.extend(["--stale-cmd-timeout", str(args.stale_cmd_timeout)])
    if args.terrain_config:
        sim_cmd.extend(["--terrain-config", args.terrain_config])
    # Rock obstacles
    if args.rocks > 0:
        sim_cmd.extend(["--rocks", str(args.rocks)])
        sim_cmd.extend(["--rock-zone-x"] + [str(v) for v in args.rock_zone_x])
        sim_cmd.extend(["--rock-zone-y"] + [str(v) for v in args.rock_zone_y])
        sim_cmd.extend(["--rock-size"] + [str(v) for v in args.rock_size])
        sim_cmd.extend(["--rock-seed", str(args.rock_seed)])
    # Safety filter
    if args.safety_filter:
        sim_cmd.append("--safety-filter")
        sim_cmd.extend(["--safety-flavor", args.safety_flavor])
        if args.no_safety_nn:
            sim_cmd.append("--no-safety-nn")
        sim_cmd.extend(["--safety-buffer", str(args.safety_buffer)])
        sim_cmd.extend(["--shield-horizon", str(args.shield_horizon)])
        if args.safety_flavor == "mppi":
            sim_cmd.extend(["--mppi-samples", str(args.mppi_samples)])
            sim_cmd.extend(["--mppi-sigma-steer", str(args.mppi_sigma_steer)])
            sim_cmd.extend(["--mppi-sigma-alpha", str(args.mppi_sigma_alpha)])
            sim_cmd.extend(["--mppi-temperature", str(args.mppi_temperature)])
            if args.mppi_no_seeds:
                sim_cmd.append("--mppi-no-seeds")
            if args.shield_no_sigma_gate:
                sim_cmd.append("--shield-no-sigma-gate")
            sim_cmd.extend(["--shield-sigma-mode", args.shield_sigma_mode])
            sim_cmd.extend(["--shield-sigma-buffer-gain",
                            str(args.shield_sigma_buffer_gain)])
        elif args.safety_flavor == "nmpc":
            sim_cmd.extend(["--nmpc-iter", str(args.nmpc_iter)])
        else:  # dob_cbf legacy
            sim_cmd.extend(["--cbf-alpha", str(args.cbf_alpha)])
            sim_cmd.extend(["--delay-steps", str(args.delay_steps)])
            sim_cmd.extend(["--cbf-w-long", str(args.cbf_w_long)])
            sim_cmd.extend(["--cbf-w-lat", str(args.cbf_w_lat)])
            sim_cmd.extend(["--cbf-forward-bias", str(args.cbf_forward_bias)])
            sim_cmd.extend(["--dob-bandwidth", str(args.dob_bandwidth)])
            sim_cmd.extend(["--cbf-flavor", args.cbf_flavor])
    # IMU sensor args
    if args.no_imu:
        sim_cmd.append("--no-imu")
    if args.imu_rate != 100:
        sim_cmd.extend(["--imu-rate", str(args.imu_rate)])
    if args.imu_lag > 0:
        sim_cmd.extend(["--imu-lag", str(args.imu_lag)])
    if args.imu_acc_stdev != 0.015:
        sim_cmd.extend(["--imu-acc-stdev", str(args.imu_acc_stdev)])
    if args.imu_gyro_stdev != 0.001:
        sim_cmd.extend(["--imu-gyro-stdev", str(args.imu_gyro_stdev)])

    # Controller selection: standard reference-tracking MPC, or MPCC
    # (Model Predictive Contouring Control) — the path-progress
    # formulation that lets the optimizer pick its own speed.
    use_mpcc = (args.controller_mode == 'mpcc')
    if use_mpcc:
        ctrl_cmd = [
            sys.executable, str(script_dir / "acados_mpcc_controller_node.py"),
            "--nn-model", args.nn_model,
            "--path", args.path,
            "--speed", str(args.speed),
            "--terrain", args.terrain,
            "--sine-amplitude", str(args.sine_amplitude),
            "--sine-wavelength", str(args.sine_wavelength),
            "--lead-in", str(args.lead_in),
            "--sim-port", str(args.sim_port),
            "--ctrl-port", str(args.ctrl_port),
            "--N", str(args.mpcc_N),
            "--dt", str(args.mpcc_dt),
            "--w-contour", str(args.mpcc_w_contour),
            "--w-lag", str(args.mpcc_w_lag),
            "--w-progress", str(args.mpcc_w_progress),
            "--w-delta-dot", str(args.mpcc_w_delta_dot),
            "--w-speed-cap", str(args.mpcc_w_speed_cap),
            "--vtheta-max", str(args.mpcc_vtheta_max),
        ]
        if args.mpcc_friction_ellipse:
            ctrl_cmd.append("--friction-ellipse")
        if args.mpcc_diag_csv:
            ctrl_cmd.extend(["--diag-csv", args.mpcc_diag_csv])
        if args.live_plot:
            ctrl_cmd.append("--live-plot")
            ctrl_cmd.extend(["--live-plot-every", str(args.live_plot_every)])
    else:
        ctrl_cmd = [
            sys.executable, str(script_dir / "acados_mpc_controller_node.py"),
            "--model", args.model,
            "--nn-model", args.nn_model,
            "--kappa", args.kappa,
            "--path", args.path,
            "--speed", str(args.speed),
            "--terrain", args.terrain,
            "--time", str(args.time),
            "--sine-amplitude", str(args.sine_amplitude),
            "--sine-wavelength", str(args.sine_wavelength),
            "--lead-in", str(args.lead_in),
            "--sim-host", "localhost",
            "--sim-port", str(args.sim_port),
            "--ctrl-port", str(args.ctrl_port),
            "--rms-time-start", str(args.rms_time_start),
            "--plot-dir", args.plot_dir,
            "--dob-ki", str(args.dob_ki),
            "--dob-max", str(args.dob_max),
            "--dob-bleed", str(args.dob_bleed),
        ]
    # The flag plumbing below is for the standard MPC node only.  When
    # MPCC mode is selected we skip it entirely (the MPCC node has its
    # own much shorter CLI and doesn't accept DOB / kappa / temporal /
    # GP / terrain-estimator flags).
    if not use_mpcc and args.no_delay_comp:
        ctrl_cmd.append("--no-delay-comp")
    if not use_mpcc and args.no_lat_transfer:
        ctrl_cmd.append("--no-lat-transfer")
    if not use_mpcc and args.no_path_reindex:
        ctrl_cmd.append("--no-path-reindex")
    if not use_mpcc:
        ctrl_cmd.extend(["--speed-weight", str(args.speed_weight)])
        ctrl_cmd.extend(["--speed-cost-mode", args.speed_cost_mode])
        ctrl_cmd.extend(["--obstacle-weight", str(args.obstacle_weight)])
        if args.no_temporal_staged:
            ctrl_cmd.append("--no-temporal-staged")
        if args.symbolic_rates:
            ctrl_cmd.append("--symbolic-rates")
        else:
            ctrl_cmd.append("--no-symbolic-rates")
        if args.no_plot:
            ctrl_cmd.append("--no-plot")
        if args.live_plot:
            ctrl_cmd.append("--live-plot")
            ctrl_cmd.extend(["--live-plot-every", str(args.live_plot_every)])
        if args.no_csv:
            ctrl_cmd.append("--no-csv")
        if args.mpc_blind_obstacles:
            ctrl_cmd.append("--mpc-blind-obstacles")
        if args.log_tire_csv:
            ctrl_cmd.extend(["--log-tire-csv", args.log_tire_csv])
            ctrl_cmd.extend(["--log-scenario-id", str(args.log_scenario_id)])
        if args.log_rich_tire_csv:
            ctrl_cmd.extend(["--log-rich-tire-csv", args.log_rich_tire_csv])
            ctrl_cmd.extend(["--log-scenario-id", str(args.log_scenario_id)])
        if args.terrain_classifier:
            ctrl_cmd.append("--terrain-classifier")
            ctrl_cmd.extend(["--tc-port", str(args.tc_port)])
        if args.use_prediction:
            ctrl_cmd.append("--use-prediction")
        if args.prediction_min_confidence > 0.0:
            ctrl_cmd.extend(["--prediction-min-confidence", str(args.prediction_min_confidence)])
        if args.terrain_estimator:
            ctrl_cmd.append("--terrain-estimator")
            ctrl_cmd.extend(["--te-window", str(args.te_window)])
            ctrl_cmd.extend(["--te-update-interval", str(args.te_update_interval)])
            ctrl_cmd.extend(["--te-lr", str(args.te_lr)])
            ctrl_cmd.extend(["--te-steps", str(args.te_steps)])
            ctrl_cmd.extend(["--te-min-excitation", str(args.te_min_excitation)])
            ctrl_cmd.extend(["--te-min-confidence", str(args.te_min_confidence)])
            if args.learned_terrain_model_dir:
                ctrl_cmd.extend(["--learned-terrain-model-dir",
                                 str(args.learned_terrain_model_dir)])
            if args.te_verbose:
                ctrl_cmd.append("--te-verbose")

        if args.force_residual:
            ctrl_cmd.append("--force-residual")
            ctrl_cmd.extend(["--force-residual-checkpoint", str(args.force_residual_checkpoint)])
            if args.force_residual_online:
                ctrl_cmd.append("--force-residual-online")
            ctrl_cmd.extend(["--force-residual-clip", str(args.force_residual_clip)])
            ctrl_cmd.extend(["--force-residual-gain", str(args.force_residual_gain)])
            ctrl_cmd.extend(["--force-residual-online-lr", str(args.force_residual_online_lr)])

        if args.dynamics_gp:
            ctrl_cmd.append("--dynamics-gp")
            ctrl_cmd.extend(["--dynamics-gp-state", str(args.dynamics_gp_state)])
            ctrl_cmd.extend(["--dynamics-gp-clip", str(args.dynamics_gp_clip)])
            ctrl_cmd.extend(["--dynamics-gp-gain", str(args.dynamics_gp_gain)])
            if args.gp_uncertainty_speed:
                ctrl_cmd.append("--gp-uncertainty-speed")
                ctrl_cmd.extend(["--gp-speed-scale-min", str(args.gp_speed_scale_min)])
            if args.gp_terrain_gate:
                ctrl_cmd.extend(["--gp-terrain-gate"] + args.gp_terrain_gate)

        if args.dynamics_gp:
            ctrl_cmd.extend(["--gp-max-inducing", str(args.gp_max_inducing)])
            ctrl_cmd.extend(["--gp-noise-var", str(args.gp_noise_var)])

        ctrl_cmd.extend(["--ax-filter-tau", str(args.ax_filter_tau)])
        ctrl_cmd.extend(["--vel-filter-tau", str(args.vel_filter_tau)])
    else:
        # MPCC mode: pass through CSV logs if requested.
        if args.log_tire_csv:
            ctrl_cmd.extend(["--log-tire-csv", args.log_tire_csv])
            ctrl_cmd.extend(["--log-scenario-id", str(args.log_scenario_id)])
        if args.log_rich_tire_csv:
            ctrl_cmd.extend(["--log-rich-tire-csv", args.log_rich_tire_csv])
            ctrl_cmd.extend(["--log-scenario-id", str(args.log_scenario_id)])

    # ---- Terrain classifier command ----
    tc_cmd = [
        sys.executable, "-m", "terrain_classifier.classifier_node",
        "--model", args.tc_model,
        "--sim-host", "localhost",
        "--sim-port", str(args.sim_port),
        "--ctrl-host", "localhost",
        "--ctrl-port", str(args.ctrl_port),
        "--pub-port", str(args.tc_port),
        "--ema-alpha", str(args.tc_ema_alpha),
    ]

    # ---- Launch ----
    procs = []

    def cleanup(signum=None, frame=None):
        for proc in procs:
            if proc.poll() is None:
                proc.terminate()
        for proc in procs:
            proc.wait(timeout=5)
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    try:
        if args.ctrl_only:
            print(f"[launch] Starting controller only")
            print(f"  cmd: {' '.join(ctrl_cmd)}")
            proc = subprocess.Popen(ctrl_cmd)
            procs.append(proc)
            proc.wait()
        elif args.sim_only or args.manual or args.wasd:
            if args.wasd:
                mode = "manual (WASD keyboard)"
            elif args.manual:
                mode = "manual (G29)"
            else:
                mode = "simulation only"
            print(f"[launch] Starting {mode}")
            print(f"  cmd: {' '.join(sim_cmd)}")

            # Start terrain classifier if requested (WASD publishes state via ZMQ)
            if args.wasd and args.terrain_classifier:
                print("[launch] Starting terrain classifier...")
                tc_proc = subprocess.Popen(tc_cmd, cwd=str(script_dir))
                procs.append(tc_proc)

            proc = subprocess.Popen(sim_cmd)
            procs.append(proc)
            proc.wait()
        else:
            # Start controller first (it will wait for config from sim)
            print(f"[launch] Starting controller...")
            ctrl_proc = subprocess.Popen(ctrl_cmd)
            procs.append(ctrl_proc)

            # Start terrain classifier if requested
            if args.terrain_classifier:
                print(f"[launch] Starting terrain classifier...")
                tc_proc = subprocess.Popen(tc_cmd, cwd=str(script_dir))
                procs.append(tc_proc)

            # Brief delay, then start simulation.  Wait for first control so ACADOS
            # codegen does not consume --time; controller sends ready-pings until
            # VehicleState arrives (see acados_mpc_controller_node).
            time.sleep(0.5)
            print(f"[launch] Starting simulation...")
            sim_cmd_both = sim_cmd + ["--wait-for-controller", "300"]
            sim_proc = subprocess.Popen(sim_cmd_both)
            procs.append(sim_proc)

            # Wait for simulation to finish
            sim_proc.wait()
            print("[launch] Simulation finished. Waiting for controller...")

            # Give controller a moment to process the stop signal
            ctrl_proc.wait(timeout=10)

    except KeyboardInterrupt:
        pass
    finally:
        cleanup()


if __name__ == "__main__":
    main()
