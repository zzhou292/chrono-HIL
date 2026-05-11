#!/usr/bin/env python3
"""
Phase 2: Force Prediction Audit

Runs a short sim for each terrain, logs:
1. True per-wheel Fy from SCM (via tire_forces)
2. Axle forces reconstructed from (ay, omega_dot) via rigid-body
3. NN-predicted axle forces at the TRUE n for this terrain
4. NN-predicted axle forces at n=0.5 (clay) for comparison

This directly answers: why does the grid filter pick n=0.5 for all terrains?

Usage:
  conda run -n sim python diag_force_audit.py --terrain dirt --time 15
"""

import argparse
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "simulation"))

from simulation.hil_messages import (
    VehicleState, ControlCommand, SimStatus,
    ZMQPublisher, ZMQSubscriber,
    sim_sub_endpoint, ctrl_pub_endpoint,
)
from simulation.terrain_parameter_estimator import TerrainParameterEstimator
from simulation.param_consistency import (
    terrain_preset_to_internal, get_terrain_preset,
    HMMWV_VEHICLE_PARAMS, TERRAIN_PRESETS,
)
from simulation.tire_input_features import kappa_from_wheel_speed


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--terrain", default="dirt", choices=["clay", "dirt", "sand"])
    p.add_argument("--time", type=float, default=15.0)
    p.add_argument("--sim-port", type=int, default=7890)
    p.add_argument("--ctrl-port", type=int, default=7891)
    args = p.parse_args()

    vp = HMMWV_VEHICLE_PARAMS
    M, Izz, Lf, Lr, L = vp["M"], vp["Izz"], vp["Lf"], vp["Lr"], vp["L"]
    true_n = TERRAIN_PRESETS[args.terrain]["n"]

    # Launch sim
    sim_script = PROJECT_ROOT / "simulation" / "chrono_sim_node.py"
    sim_cmd = [
        sys.executable, str(sim_script),
        "--time", str(args.time), "--speed", "5",
        "--terrain", args.terrain, "--path", "sinusoidal",
        "--vis-mode", "none",
        "--sim-port", str(args.sim_port),
        "--ctrl-host", "localhost", "--ctrl-port", str(args.ctrl_port),
        "--no-wait-for-controller",
    ]
    print(f"Launching sim: {args.terrain} (n={true_n})")
    sim_proc = subprocess.Popen(sim_cmd)
    time.sleep(2)

    state_sub = ZMQSubscriber(sim_sub_endpoint("localhost", args.sim_port))
    ctrl_pub = ZMQPublisher(ctrl_pub_endpoint(args.ctrl_port))

    # NN model for predictions
    model_dir = PROJECT_ROOT / "nn_models" / "paper_v2_mlp_16_4"
    init_terrain = terrain_preset_to_internal(get_terrain_preset("clay"))
    est = TerrainParameterEstimator(
        model_dir=str(model_dir), initial_terrain=init_terrain,
        use_measured_tire_ops=True, update_interval=10,
    )

    records = []
    seq = 0

    try:
        while True:
            result = state_sub.recv(timeout_ms=500)
            if result is None:
                continue
            topic, msg = result
            if isinstance(msg, SimStatus):
                if msg.event == "stop":
                    break
                continue
            if not isinstance(msg, VehicleState):
                continue

            t = msg.time
            u = msg.u

            # Open-loop driving
            steer = 0.5 * math.sin(2 * math.pi * t / 3.0)
            cmd = ControlCommand(
                time=t, wall_time=time.time(), seq=seq,
                steering=steer, throttle=0.5, braking=0.0,
                delta=0.0, acceleration=0.0, delta_dot=0.0, jerk=0.0,
            )
            ctrl_pub.send(cmd)
            seq += 1

            if abs(u) < 1.0:
                continue

            tf = msg.tire_forces or {}
            if not tf:
                continue

            # Extract true per-wheel Fy from SCM
            try:
                true_Fy_fl = float(tf["front_left_Fy"])
                true_Fy_fr = float(tf["front_right_Fy"])
                true_Fy_rl = float(tf["rear_left_Fy"])
                true_Fy_rr = float(tf["rear_right_Fy"])
            except (KeyError, TypeError):
                continue

            true_Fy_f_axle = true_Fy_fl + true_Fy_fr
            true_Fy_r_axle = true_Fy_rl + true_Fy_rr

            # Reconstruct from inertial sensors (what the estimator uses)
            ay = msg.ay
            omega_dot_est = est.estimate_omega_dot(msg.omega, t)
            if omega_dot_est is None:
                continue
            recon_Fy_f = (Izz * omega_dot_est + Lr * M * ay) / L
            recon_Fy_r = (Lf * M * ay - Izz * omega_dot_est) / L

            # Get operating conditions
            alpha_f = 0.5 * (float(tf["front_left_slip_angle"]) + float(tf["front_right_slip_angle"]))
            alpha_r = 0.5 * (float(tf["rear_left_slip_angle"]) + float(tf["rear_right_slip_angle"]))
            kappa_f = 0.5 * (float(tf["front_left_long_slip"]) + float(tf["front_right_long_slip"]))
            kappa_r = 0.5 * (float(tf["rear_left_long_slip"]) + float(tf["rear_right_long_slip"]))
            Fz_f = 0.5 * (abs(float(tf["front_left_Fz"])) + abs(float(tf["front_right_Fz"])))
            Fz_r = 0.5 * (abs(float(tf["rear_left_Fz"])) + abs(float(tf["rear_right_Fz"])))

            est._last_alpha_f = alpha_f
            est._last_alpha_r = alpha_r
            est._last_kappa_f = kappa_f
            est._last_kappa_r = kappa_r
            est._last_Fz_f = Fz_f
            est._last_Fz_r = Fz_r
            est._last_sr = 0.0  # approximate

            # NN predictions at various n values
            nn_forces = {}
            for n_val in [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1]:
                _, Fy_f_pw = est._nn_FxFy(kappa_f, alpha_f, max(abs(u), 0.5), Fz_f, 0.0, n_val)
                _, Fy_r_pw = est._nn_FxFy(kappa_r, alpha_r, max(abs(u), 0.5), Fz_r, 0.0, n_val)
                nn_forces[n_val] = (-2.0 * Fy_f_pw, -2.0 * Fy_r_pw)

            records.append({
                "t": t, "u": u, "ay": ay, "omega_dot": omega_dot_est,
                "alpha_f": alpha_f, "alpha_r": alpha_r,
                "Fz_f": Fz_f, "Fz_r": Fz_r,
                "true_Fy_f": true_Fy_f_axle, "true_Fy_r": true_Fy_r_axle,
                "recon_Fy_f": recon_Fy_f, "recon_Fy_r": recon_Fy_r,
                **{f"nn_Fy_f_n{n:.1f}": f for n, (f, _) in nn_forces.items()},
                **{f"nn_Fy_r_n{n:.1f}": r for n, (_, r) in nn_forces.items()},
            })

    except KeyboardInterrupt:
        pass
    finally:
        state_sub.close()
        ctrl_pub.close()
        sim_proc.terminate()
        sim_proc.wait(timeout=5)

    if not records:
        print("No data collected!")
        return

    # Analysis
    arr = {k: np.array([r[k] for r in records]) for k in records[0]}
    N = len(records)

    print(f"\n{'='*80}")
    print(f"FORCE AUDIT: {args.terrain.upper()} (true n={true_n}), {N} samples")
    print(f"{'='*80}")

    # 1. Compare true SCM forces vs inertial reconstruction
    err_f_recon = arr["recon_Fy_f"] - arr["true_Fy_f"]
    err_r_recon = arr["recon_Fy_r"] - arr["true_Fy_r"]
    print(f"\n--- SCM vs Inertial Reconstruction ---")
    print(f"Front axle Fy: true mean={np.mean(arr['true_Fy_f']):.0f}, "
          f"recon mean={np.mean(arr['recon_Fy_f']):.0f}, "
          f"bias={np.mean(err_f_recon):.0f}, RMSE={np.sqrt(np.mean(err_f_recon**2)):.0f}")
    print(f"Rear  axle Fy: true mean={np.mean(arr['true_Fy_r']):.0f}, "
          f"recon mean={np.mean(arr['recon_Fy_r']):.0f}, "
          f"bias={np.mean(err_r_recon):.0f}, RMSE={np.sqrt(np.mean(err_r_recon**2)):.0f}")

    # 2. Compare NN predictions at true n vs true SCM forces
    true_n_str = f"n{true_n:.1f}"
    err_f_nn = arr[f"nn_Fy_f_{true_n_str}"] - arr["true_Fy_f"]
    err_r_nn = arr[f"nn_Fy_r_{true_n_str}"] - arr["true_Fy_r"]
    print(f"\n--- SCM vs NN at true n={true_n} ---")
    print(f"Front axle Fy: true mean={np.mean(arr['true_Fy_f']):.0f}, "
          f"NN mean={np.mean(arr[f'nn_Fy_f_{true_n_str}']):.0f}, "
          f"bias={np.mean(err_f_nn):.0f}, RMSE={np.sqrt(np.mean(err_f_nn**2)):.0f}")
    print(f"Rear  axle Fy: true mean={np.mean(arr['true_Fy_r']):.0f}, "
          f"NN mean={np.mean(arr[f'nn_Fy_r_{true_n_str}']):.0f}, "
          f"bias={np.mean(err_r_nn):.0f}, RMSE={np.sqrt(np.mean(err_r_nn**2)):.0f}")

    # 3. Compare NN predictions at n=0.5 vs true SCM forces
    err_f_clay = arr["nn_Fy_f_n0.5"] - arr["true_Fy_f"]
    err_r_clay = arr["nn_Fy_r_n0.5"] - arr["true_Fy_r"]
    print(f"\n--- SCM vs NN at n=0.5 (clay) ---")
    print(f"Front axle Fy: true mean={np.mean(arr['true_Fy_f']):.0f}, "
          f"NN mean={np.mean(arr['nn_Fy_f_n0.5']):.0f}, "
          f"bias={np.mean(err_f_clay):.0f}, RMSE={np.sqrt(np.mean(err_f_clay**2)):.0f}")
    print(f"Rear  axle Fy: true mean={np.mean(arr['true_Fy_r']):.0f}, "
          f"NN mean={np.mean(arr['nn_Fy_r_n0.5']):.0f}, "
          f"bias={np.mean(err_r_clay):.0f}, RMSE={np.sqrt(np.mean(err_r_clay**2)):.0f}")

    # 4. Which n has smallest RMSE against RECONSTRUCTION? (this is what the filter uses)
    print(f"\n--- RMSE of NN predictions vs RECONSTRUCTED forces (what the filter compares) ---")
    print(f"{'n':>5} | {'RMSE_Fy_f':>10} | {'RMSE_Fy_r':>10} | {'Total_RMSE':>10}")
    print("-" * 45)
    for n_val in [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1]:
        nn_str = f"n{n_val:.1f}"
        ef = arr[f"nn_Fy_f_{nn_str}"] - arr["recon_Fy_f"]
        er = arr[f"nn_Fy_r_{nn_str}"] - arr["recon_Fy_r"]
        rmse_f = np.sqrt(np.mean(ef**2))
        rmse_r = np.sqrt(np.mean(er**2))
        total = np.sqrt(np.mean(ef**2 + er**2))
        marker = " ← true" if abs(n_val - true_n) < 0.01 else ""
        print(f"{n_val:5.2f} | {rmse_f:10.0f} | {rmse_r:10.0f} | {total:10.0f}{marker}")

    # 5. Which n has smallest RMSE against TRUE SCM forces? (ideal case)
    print(f"\n--- RMSE of NN predictions vs TRUE SCM forces (ideal case) ---")
    print(f"{'n':>5} | {'RMSE_Fy_f':>10} | {'RMSE_Fy_r':>10} | {'Total_RMSE':>10}")
    print("-" * 45)
    for n_val in [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1]:
        nn_str = f"n{n_val:.1f}"
        ef = arr[f"nn_Fy_f_{nn_str}"] - arr["true_Fy_f"]
        er = arr[f"nn_Fy_r_{nn_str}"] - arr["true_Fy_r"]
        rmse_f = np.sqrt(np.mean(ef**2))
        rmse_r = np.sqrt(np.mean(er**2))
        total = np.sqrt(np.mean(ef**2 + er**2))
        marker = " ← true" if abs(n_val - true_n) < 0.01 else ""
        print(f"{n_val:5.2f} | {rmse_f:10.0f} | {rmse_r:10.0f} | {total:10.0f}{marker}")


if __name__ == "__main__":
    main()
