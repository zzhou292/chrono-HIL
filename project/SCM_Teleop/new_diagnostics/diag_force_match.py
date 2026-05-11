#!/usr/bin/env python3
"""Compare Chrono tire forces against NN predictions under preset terrains."""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "simulation"))

from hil_messages import (  # noqa: E402
    ControlCommand,
    SimStatus,
    VehicleState,
    ZMQPublisher,
    ZMQSubscriber,
    ctrl_pub_endpoint,
    sim_sub_endpoint,
)
from param_consistency import TERRAIN_PRESETS, terrain_preset_to_internal  # noqa: E402
from terrain_parameter_estimator import TerrainParameterEstimator  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("terrain", nargs="?", default="clay", choices=sorted(TERRAIN_PRESETS))
    p.add_argument("--nn-model", default="paper_v2_mlp_16_4")
    p.add_argument("--time", type=float, default=10.0)
    p.add_argument("--speed", type=float, default=5.0)
    p.add_argument("--steer-amp", type=float, default=0.5)
    p.add_argument("--steer-period", type=float, default=3.0)
    p.add_argument("--sim-port", type=int, default=5565)
    p.add_argument("--ctrl-port", type=int, default=5566)
    p.add_argument("--init-terrain", default="dirt", choices=sorted(TERRAIN_PRESETS))
    p.add_argument("--output-csv", default="")
    args = p.parse_args()

    sim_cmd = [
        "/home/kyle/miniconda3/bin/conda",
        "run",
        "--no-capture-output",
        "-n",
        "sim",
        "python",
        "simulation/chrono_sim_node.py",
        "--time",
        str(args.time),
        "--speed",
        str(args.speed),
        "--terrain",
        args.terrain,
        "--path",
        "sinusoidal",
        "--vis-mode",
        "none",
        "--sim-port",
        str(args.sim_port),
        "--ctrl-host",
        "localhost",
        "--ctrl-port",
        str(args.ctrl_port),
        "--no-wait-for-controller",
    ]

    proc = subprocess.Popen(sim_cmd, cwd=str(ROOT))
    time.sleep(2.0)

    sub = ZMQSubscriber(sim_sub_endpoint("localhost", args.sim_port))
    pub = ZMQPublisher(ctrl_pub_endpoint(args.ctrl_port))

    model_dir = str(ROOT / "nn_models" / args.nn_model)
    estimator = TerrainParameterEstimator(
        model_dir=model_dir,
        initial_terrain=terrain_preset_to_internal(TERRAIN_PRESETS[args.init_terrain]),
    )

    stats = {
        name: {
            "front_sq": 0.0,
            "rear_sq": 0.0,
            "front_bias": 0.0,
            "rear_bias": 0.0,
            "count": 0,
        }
        for name in ("clay", "dirt", "sand")
    }
    seq = 0
    csv_file = None
    csv_writer = None
    prev_wheel_angle = None
    prev_t = None

    try:
        if args.output_csv:
            csv_path = Path(args.output_csv)
            csv_path.parent.mkdir(parents=True, exist_ok=True)
            csv_file = csv_path.open("w", newline="")
            fieldnames = [
                "true_terrain",
                "nn_model",
                "time",
                "u",
                "v_body",
                "omega",
                "ay",
                "actual_front_fy",
                "actual_rear_fy",
                "front_left_slip_angle",
                "front_right_slip_angle",
                "rear_left_slip_angle",
                "rear_right_slip_angle",
                "front_left_fy",
                "front_right_fy",
                "rear_left_fy",
                "rear_right_fy",
                "front_left_Fz",
                "front_right_Fz",
                "rear_left_Fz",
                "rear_right_Fz",
                "front_left_long_slip",
                "front_right_long_slip",
                "rear_left_long_slip",
                "rear_right_long_slip",
                "steering_rate",
            ]
            for name in ("clay", "dirt", "sand"):
                fieldnames.extend([f"{name}_pred_front_fy", f"{name}_pred_rear_fy"])
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            csv_writer.writeheader()

        while True:
            result = sub.recv(timeout_ms=1000)
            if result is None:
                continue
            _topic, msg = result
            if isinstance(msg, SimStatus):
                if msg.event == "stop":
                    break
                continue
            if not isinstance(msg, VehicleState):
                continue

            t = float(msg.time)
            steer = args.steer_amp * math.sin(2.0 * math.pi * t / args.steer_period)
            pub.send(
                ControlCommand(
                    time=t,
                    wall_time=time.time(),
                    seq=seq,
                    steering=steer,
                    throttle=0.5,
                    braking=0.0,
                    delta=0.0,
                    acceleration=0.0,
                    delta_dot=0.0,
                    jerk=0.0,
                )
            )
            seq += 1

            tf = msg.tire_forces or {}
            if t < 2.0 or not tf:
                continue

            actual_front = float(tf["front_left_Fy"] + tf["front_right_Fy"])
            actual_rear = float(tf["rear_left_Fy"] + tf["rear_right_Fy"])
            u = max(abs(float(msg.u)), 0.5)
            wheel_angle = float(msg.steering_angle)
            if prev_wheel_angle is None or prev_t is None or t <= prev_t:
                steering_rate = 0.0
            else:
                steering_rate = (wheel_angle - prev_wheel_angle) / max(t - prev_t, 1e-4)
            prev_wheel_angle = wheel_angle
            prev_t = t
            row = None
            if csv_writer is not None:
                row = {
                    "true_terrain": args.terrain,
                    "nn_model": args.nn_model,
                    "time": t,
                    "u": u,
                    "v_body": float(msg.v),
                    "omega": float(msg.omega),
                    "ay": float(msg.ay),
                    "actual_front_fy": actual_front,
                    "actual_rear_fy": actual_rear,
                    "front_left_slip_angle": float(tf["front_left_slip_angle"]),
                    "front_right_slip_angle": float(tf["front_right_slip_angle"]),
                    "rear_left_slip_angle": float(tf["rear_left_slip_angle"]),
                    "rear_right_slip_angle": float(tf["rear_right_slip_angle"]),
                    "front_left_fy": float(tf["front_left_Fy"]),
                    "front_right_fy": float(tf["front_right_Fy"]),
                    "rear_left_fy": float(tf["rear_left_Fy"]),
                    "rear_right_fy": float(tf["rear_right_Fy"]),
                    "front_left_Fz": float(tf["front_left_Fz"]),
                    "front_right_Fz": float(tf["front_right_Fz"]),
                    "rear_left_Fz": float(tf["rear_left_Fz"]),
                    "rear_right_Fz": float(tf["rear_right_Fz"]),
                    "front_left_long_slip": float(tf["front_left_long_slip"]),
                    "front_right_long_slip": float(tf["front_right_long_slip"]),
                    "rear_left_long_slip": float(tf["rear_left_long_slip"]),
                    "rear_right_long_slip": float(tf["rear_right_long_slip"]),
                    "steering_rate": steering_rate,
                }

            for name, preset in TERRAIN_PRESETS.items():
                n_val = terrain_preset_to_internal(preset)["n"]
                pred_front = 0.0
                pred_rear = 0.0
                for side in ("left", "right"):
                    _, fy_front = estimator._nn_FxFy(
                        float(tf[f"front_{side}_long_slip"]),
                        float(tf[f"front_{side}_slip_angle"]),
                        u,
                        abs(float(tf[f"front_{side}_Fz"])),
                        steering_rate,
                        n_val,
                    )
                    _, fy_rear = estimator._nn_FxFy(
                        float(tf[f"rear_{side}_long_slip"]),
                        float(tf[f"rear_{side}_slip_angle"]),
                        u,
                        abs(float(tf[f"rear_{side}_Fz"])),
                        0.0,
                        n_val,
                    )
                    pred_front += -fy_front
                    pred_rear += -fy_rear

                front_err = pred_front - actual_front
                rear_err = pred_rear - actual_rear
                stats[name]["front_sq"] += front_err * front_err
                stats[name]["rear_sq"] += rear_err * rear_err
                stats[name]["front_bias"] += front_err
                stats[name]["rear_bias"] += rear_err
                stats[name]["count"] += 1
                if row is not None:
                    row[f"{name}_pred_front_fy"] = pred_front
                    row[f"{name}_pred_rear_fy"] = pred_rear

            if row is not None:
                csv_writer.writerow(row)
    finally:
        sub.close()
        pub.close()
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except Exception:
            proc.kill()
        if csv_file is not None:
            csv_file.close()

    print(f"terrain={args.terrain}")
    print(f"nn_model={args.nn_model}")
    for name, s in stats.items():
        n = max(s["count"], 1)
        print(
            name,
            f"front_rmse={(s['front_sq'] / n) ** 0.5:.1f}",
            f"rear_rmse={(s['rear_sq'] / n) ** 0.5:.1f}",
            f"front_bias={s['front_bias'] / n:.1f}",
            f"rear_bias={s['rear_bias'] / n:.1f}",
            f"count={s['count']}",
        )
    if args.output_csv:
        print(f"output_csv={args.output_csv}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
