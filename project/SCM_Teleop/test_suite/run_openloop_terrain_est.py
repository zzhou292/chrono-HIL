#!/usr/bin/env python3
"""
Open-loop sliding-window terrain-estimation test.

Drives the sim with sinusoidal steering + constant throttle while
running the retained sliding-window MLP terrain estimator. No MPC in the loop.

Usage (two terminals):

  Terminal 1 (sim):
    python chrono_sim_node.py --time 45 --speed 5 --terrain dirt \
      --path sinusoidal --vis-mode irrlicht --sim-port 5555 \
      --ctrl-host localhost --ctrl-port 5556 --no-wait-for-controller

  Terminal 2 (this script):
    python run_openloop_terrain_est.py --terrain dirt --time 45

  Or single-terminal headless:
    python run_openloop_terrain_est.py --terrain dirt --time 45 --launch-sim
"""

import argparse
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
SIM_DIR = ROOT / "simulation"
sys.path.insert(0, str(SIM_DIR))

from hil_messages import (
    VehicleState, ControlCommand, SimStatus,
    ZMQPublisher, ZMQSubscriber,
    sim_sub_endpoint, ctrl_pub_endpoint,
)
from param_consistency import terrain_preset_to_internal, get_terrain_preset
from tire_input_features import kappa_from_wheel_speed
from learned_terrain_estimator import LearnedTerrainEstimator


def main():
    p = argparse.ArgumentParser(description="Open-loop terrain estimation")
    p.add_argument("--terrain", default="dirt", choices=["clay", "dirt", "sand"])
    p.add_argument("--time", type=float, default=45.0)
    p.add_argument("--throttle", type=float, default=0.5)
    p.add_argument("--steer-amp", type=float, default=0.5)
    p.add_argument("--steer-period", type=float, default=3.0)
    p.add_argument("--sim-port", type=int, default=5555)
    p.add_argument("--ctrl-port", type=int, default=5556)
    p.add_argument("--nn-model", default="paper_v2_mlp_16_4")
    p.add_argument("--hide-true-wheel-fy", action="store_true",
                   help="Drop true wheel Fy from tire_forces before estimator.observe()")
    p.add_argument("--init-terrain", default="clay",
                   choices=["clay", "dirt", "sand"],
                   help="Initial terrain guess for estimator")
    p.add_argument("--launch-sim", action="store_true",
                   help="Auto-launch sim node headless (for scripted runs)")
    p.add_argument("--estimator-verbose", action="store_true",
                   help="Print per-step learned-estimator diagnostics")
    p.add_argument("--learned-model-dir", default=None,
                   help="Directory with weights.pt + scaler.pkl + config.json "
                        "for the retained sliding-window estimator")
    p.add_argument("--no-plot", action="store_true",
                   help="Suppress saving of the convergence PNG")
    args = p.parse_args()

    sim_proc = None
    if args.launch_sim:
        sim_script = SIM_DIR / "chrono_sim_node.py"
        sim_cmd = [
            sys.executable, str(sim_script),
            "--time", str(args.time), "--speed", "5",
            "--terrain", args.terrain, "--path", "sinusoidal",
            "--vis-mode", "none",
            "--sim-port", str(args.sim_port),
            "--ctrl-host", "localhost",
            "--ctrl-port", str(args.ctrl_port),
            "--no-wait-for-controller",
        ]
        # Optional: inject a custom YAML terrain config for generalization
        # tests.  This env var hook lets the open-loop runner script (which
        # owns the auto-launch path) override the preset without changing
        # the chrono sim node CLI.
        import os
        custom_terrain = os.environ.get("OPENLOOP_TERRAIN_CONFIG")
        if custom_terrain:
            sim_cmd += ["--terrain-config", custom_terrain]
            print(f"[openloop] using custom terrain config: {custom_terrain}")
        print(f"[openloop] Launching sim: {args.terrain}")
        sim_proc = subprocess.Popen(sim_cmd)
        time.sleep(2)

    # ZMQ
    state_sub = ZMQSubscriber(sim_sub_endpoint("localhost", args.sim_port))
    ctrl_pub = ZMQPublisher(ctrl_pub_endpoint(args.ctrl_port))

    # Estimator
    init_terrain = terrain_preset_to_internal(get_terrain_preset(args.init_terrain))
    learned_dir = (Path(args.learned_model_dir).expanduser()
                   if args.learned_model_dir else
                   ROOT / "nn_models" / "terrain_window_mlp_v3_cl")
    estimator = LearnedTerrainEstimator(
        model_dir=str(learned_dir),
        initial_terrain=init_terrain,
        update_interval=1,
        verbose=args.estimator_verbose,
    )
    estimator_kind = "learned"

    true_n = {"clay": 0.5, "dirt": 0.7, "sand": 1.1}[args.terrain]
    init_n = {"clay": 0.5, "dirt": 0.7, "sand": 1.1}[args.init_terrain]

    print(f"[openloop] Terrain: {args.terrain} (true n={true_n})")
    print(f"[openloop] Init: {args.init_terrain} (init n={init_n})")
    print(f"[openloop] Throttle={args.throttle}, steer_amp={args.steer_amp}, "
          f"period={args.steer_period}s")
    print(f"[openloop] Waiting for sim on port {args.sim_port}...")

    seq = 0
    last_print = 0.0
    prev_wheel_angle = None
    prev_t = None
    last_obs_t: float = -1.0
    obs_period = 0.04   # 25 Hz — matches Dallas et al. measurement cadence

    t_history = []
    n_history = []
    conf_history = []

    try:
        while True:
            result = state_sub.recv(timeout_ms=500)
            if result is None:
                continue

            topic, msg = result
            if isinstance(msg, SimStatus):
                if msg.event == "stop":
                    print("[openloop] Sim stopped.")
                    break
                continue
            if not isinstance(msg, VehicleState):
                continue

            t = msg.time

            # Open-loop: sinusoidal steering + constant throttle
            steer = args.steer_amp * math.sin(2 * math.pi * t / args.steer_period)
            cmd = ControlCommand(
                time=t, wall_time=time.time(), seq=seq,
                steering=steer, throttle=args.throttle, braking=0.0,
                delta=0.0, acceleration=0.0,
                delta_dot=0.0, jerk=0.0,
            )
            ctrl_pub.send(cmd)
            seq += 1

            # Default operating-point reconstruction from bicycle kinematics.
            u = max(abs(msg.u), 0.5)
            wheel_angle = msg.steering_angle
            alpha_f = -math.atan2(msg.v + 1.593 * msg.omega, u) + wheel_angle
            alpha_r = -math.atan2(msg.v - 1.709 * msg.omega, u)
            kappa = kappa_from_wheel_speed(
                msg.wheel_omega_fl,
                msg.wheel_omega_fr,
                msg.wheel_omega_rl,
                msg.wheel_omega_rr,
                msg.u,
            )
            kappa_f = kappa
            kappa_r = kappa
            Fz_f = 6500.0
            Fz_r = 6000.0

            tf = msg.tire_forces or {}
            wheel_ops = None
            if tf:
                wheel_ops = {
                    key: float(tf[key])
                    for key in (
                        "front_left_slip_angle",
                        "front_right_slip_angle",
                        "rear_left_slip_angle",
                        "rear_right_slip_angle",
                        "front_left_long_slip",
                        "front_right_long_slip",
                        "rear_left_long_slip",
                        "rear_right_long_slip",
                        "front_left_Fz",
                        "front_right_Fz",
                        "rear_left_Fz",
                        "rear_right_Fz",
                        "front_left_Fy",
                        "front_right_Fy",
                        "rear_left_Fy",
                        "rear_right_Fy",
                    )
                }
                if args.hide_true_wheel_fy:
                    for key in ("front_left_Fy", "front_right_Fy", "rear_left_Fy", "rear_right_Fy"):
                        wheel_ops.pop(key, None)
                # Keep bicycle-model alpha_f/alpha_r (computed above from
                # msg.v, msg.omega, wheel_angle).  Do NOT overwrite with
                # averaged GetSlipAngle() — the per-wheel tire-frame slip
                # angles can have opposite signs for L/R wheels due to track
                # width kinematics, which cancels forces when summed.
                kappa_f = 0.5 * (
                    float(tf["front_left_long_slip"]) +
                    float(tf["front_right_long_slip"])
                )
                kappa_r = 0.5 * (
                    float(tf["rear_left_long_slip"]) +
                    float(tf["rear_right_long_slip"])
                )
                Fz_f = 0.5 * (
                    abs(float(tf["front_left_Fz"])) +
                    abs(float(tf["front_right_Fz"]))
                )
                Fz_r = 0.5 * (
                    abs(float(tf["rear_left_Fz"])) +
                    abs(float(tf["rear_right_Fz"]))
                )

            if prev_wheel_angle is None or prev_t is None or t <= prev_t:
                steering_rate = 0.0
            else:
                steering_rate = (wheel_angle - prev_wheel_angle) / max(t - prev_t, 1e-4)
            prev_wheel_angle = wheel_angle
            prev_t = t

            # Throttle estimator updates to a fixed cadence so the learned
            # sliding-window path sees the same observation cadence across runs.
            run_estimator = (last_obs_t < 0.0) or (t - last_obs_t >= obs_period)
            if not run_estimator:
                continue
            last_obs_t = t

            # Run estimator
            omega_dot = estimator.estimate_omega_dot(msg.omega, t)
            if omega_dot is not None:
                obs_kwargs = dict(
                    kappa=kappa,
                    alpha_f=alpha_f, alpha_r=alpha_r,
                    u=msg.u, Fz_f=Fz_f, Fz_r=Fz_r, sr=steering_rate,
                    ay_imu=msg.ay, omega_dot=omega_dot,
                    omega=msg.omega, v_lateral=msg.v,
                    kappa_f=kappa_f, kappa_r=kappa_r,
                    wheel_ops=wheel_ops,
                    sim_time=t,
                )
                obs_kwargs["wheel_omegas"] = (
                    msg.wheel_omega_fl, msg.wheel_omega_fr,
                    msg.wheel_omega_rl, msg.wheel_omega_rr,
                )
                obs_kwargs["ax_imu"] = msg.ax
                obs_kwargs["throttle_cmd"] = args.throttle
                estimator.observe(**obs_kwargs)

                if estimator.should_update():
                    params, conf = estimator.estimate()
                    n_est = estimator.get_bekker_n()
                    
                    t_history.append(t)
                    n_history.append(n_est)
                    conf_history.append(conf)

                    if t - last_print > 1.0:
                        err = 100 * abs(n_est - true_n) / true_n
                        print(f"  t={t:5.1f}s  {estimator.describe()}  "
                              f"err={err:.1f}%  u={msg.u:.2f}m/s")
                        last_print = t

    except KeyboardInterrupt:
        print("\n[openloop] Interrupted.")
    finally:
        state_sub.close()
        ctrl_pub.close()
        if sim_proc:
            sim_proc.terminate()
            sim_proc.wait(timeout=5)

    if estimator.total_observations > 0:
        n_final = estimator.get_bekker_n()
        err_final = 100 * abs(n_final - true_n) / true_n
        print(f"\n[openloop] Final ({estimator_kind}): "
              f"n={n_final:.3f} (true={true_n}, err={err_final:.1f}%)")

        if not args.no_plot:
            try:
                import matplotlib.pyplot as plt
                plt.figure(figsize=(10, 5))
                plt.plot(t_history, n_history,
                         label=f'Estimated n ({estimator_kind})', linewidth=2)
                plt.axhline(y=true_n, color='g', linestyle='--',
                            label=f'True n ({args.terrain})')
                plt.axhline(y=init_n, color='r', linestyle=':',
                            label=f'Initial n ({args.init_terrain})')
                plt.title(f'{estimator_kind.upper()} Terrain Estimator Convergence')
                plt.xlabel('Time (s)')
                plt.ylabel('Bekker Sinkage Exponent (n)')
                plt.legend()
                plt.grid(True)
                plot_path = f'{estimator_kind}_convergence_{args.terrain}.png'
                plt.savefig(plot_path, dpi=300)
                print(f"[openloop] Saved plot to {plot_path}")
            except ImportError:
                print("[openloop] matplotlib not available for plotting.")


if __name__ == "__main__":
    main()
