#!/usr/bin/env python3
"""
Definitive per-wheel NN vs SCM force comparison.

For each wheel at each timestep, compare:
  1. True SCM Fy (body frame, from vehicle sim)
  2. NN Fy prediction using EXACT same operating conditions
  3. Identify whether the mismatch is sign, magnitude, or both

This tests the rig→vehicle gap without any axle aggregation.

Usage:
  conda run -n sim python diag_perwheel_match.py --terrain dirt --time 15
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


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--terrain", default="dirt", choices=["clay", "dirt", "sand"])
    p.add_argument("--time", type=float, default=15.0)
    p.add_argument("--sim-port", type=int, default=7890)
    p.add_argument("--ctrl-port", type=int, default=7891)
    args = p.parse_args()

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

    # NN model
    model_dir = PROJECT_ROOT / "nn_models" / "paper_v2_mlp_16_4"
    init_terrain = terrain_preset_to_internal(get_terrain_preset("clay"))
    est = TerrainParameterEstimator(
        model_dir=str(model_dir), initial_terrain=init_terrain,
        use_measured_tire_ops=True, update_interval=10,
    )

    records = []
    seq = 0

    WHEELS = [
        ("front_left", True),
        ("front_right", True),
        ("rear_left", False),
        ("rear_right", False),
    ]

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

            # Open-loop sinusoidal driving
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

            # For each wheel, compare NN vs true SCM
            for wheel_name, is_front in WHEELS:
                try:
                    alpha = float(tf[f"{wheel_name}_slip_angle"])
                    kappa = float(tf[f"{wheel_name}_long_slip"])
                    Fz = abs(float(tf[f"{wheel_name}_Fz"]))
                    Fy_true = float(tf[f"{wheel_name}_Fy"])
                except (KeyError, TypeError):
                    continue

                if Fz < 100:
                    continue

                u_safe = max(abs(u), 0.5)
                # NN prediction at TRUE n
                Fx_nn, Fy_nn = est._nn_FxFy(kappa, alpha, u_safe, Fz, 0.0, true_n)

                records.append({
                    "t": t, "wheel": wheel_name, "is_front": is_front,
                    "alpha": alpha, "kappa": kappa, "Fz": Fz, "u": u,
                    "Fy_true": Fy_true,
                    "Fy_nn": Fy_nn,
                    # Also check with negated convention
                    "Fy_nn_neg": -Fy_nn,
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

    import pandas as pd
    df = pd.DataFrame(records)
    N = len(df)

    print(f"\n{'='*80}")
    print(f"PER-WHEEL FORCE COMPARISON: {args.terrain.upper()} (true n={true_n}), {N} samples")
    print(f"{'='*80}")

    # Overall statistics
    for wheel in ["front_left", "front_right", "rear_left", "rear_right"]:
        wdf = df[df["wheel"] == wheel]
        if len(wdf) == 0:
            continue
        err_pos = wdf["Fy_nn"] - wdf["Fy_true"]
        err_neg = wdf["Fy_nn_neg"] - wdf["Fy_true"]
        corr_pos = wdf["Fy_nn"].corr(wdf["Fy_true"])
        corr_neg = wdf["Fy_nn_neg"].corr(wdf["Fy_true"])
        scale = (wdf["Fy_true"] * wdf["Fy_nn"]).sum() / (wdf["Fy_nn"]**2).sum() if (wdf["Fy_nn"]**2).sum() > 0 else float('nan')
        scale_neg = (wdf["Fy_true"] * wdf["Fy_nn_neg"]).sum() / (wdf["Fy_nn_neg"]**2).sum() if (wdf["Fy_nn_neg"]**2).sum() > 0 else float('nan')

        print(f"\n--- {wheel} ({len(wdf)} samples) ---")
        print(f"  alpha range: [{wdf['alpha'].min():.3f}, {wdf['alpha'].max():.3f}]")
        print(f"  Fz range: [{wdf['Fz'].min():.0f}, {wdf['Fz'].max():.0f}]")
        print(f"  True Fy: mean={wdf['Fy_true'].mean():+.0f}, std={wdf['Fy_true'].std():.0f}")
        print(f"  NN Fy (as-is):  mean={wdf['Fy_nn'].mean():+.0f}, "
              f"corr={corr_pos:.4f}, RMSE={np.sqrt((err_pos**2).mean()):.0f}, "
              f"best_scale={scale:.3f}")
        print(f"  NN Fy (negated): mean={wdf['Fy_nn_neg'].mean():+.0f}, "
              f"corr={corr_neg:.4f}, RMSE={np.sqrt((err_neg**2).mean()):.0f}, "
              f"best_scale={scale_neg:.3f}")

    # Overall: which convention gives better match?
    err_pos_all = df["Fy_nn"] - df["Fy_true"]
    err_neg_all = df["Fy_nn_neg"] - df["Fy_true"]
    rmse_pos = np.sqrt((err_pos_all**2).mean())
    rmse_neg = np.sqrt((err_neg_all**2).mean())
    corr_pos_all = df["Fy_nn"].corr(df["Fy_true"])
    corr_neg_all = df["Fy_nn_neg"].corr(df["Fy_true"])

    print(f"\n{'='*80}")
    print(f"OVERALL (all wheels, {N} samples):")
    print(f"  NN as-is:  RMSE={rmse_pos:.0f}, corr={corr_pos_all:.4f}")
    print(f"  NN negated: RMSE={rmse_neg:.0f}, corr={corr_neg_all:.4f}")

    # Show scatter at specific alpha bins
    print(f"\n--- Per-alpha-bin comparison (front_left wheel, NN as-is) ---")
    wdf = df[df["wheel"] == "front_left"]
    for lo, hi in [(-0.10, -0.07), (-0.05, -0.03), (-0.02, 0.02), (0.03, 0.05), (0.07, 0.10)]:
        mask = wdf["alpha"].between(lo, hi)
        if mask.sum() < 3:
            continue
        sub = wdf[mask]
        print(f"  alpha=[{lo:+.2f},{hi:+.2f}] ({mask.sum()} pts): "
              f"true Fy={sub['Fy_true'].mean():+.0f}±{sub['Fy_true'].std():.0f}, "
              f"NN Fy={sub['Fy_nn'].mean():+.0f}±{sub['Fy_nn'].std():.0f}, "
              f"ratio={sub['Fy_true'].mean()/sub['Fy_nn'].mean():.2f}" if abs(sub['Fy_nn'].mean()) > 1 else "")


if __name__ == "__main__":
    main()
