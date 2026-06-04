#!/usr/bin/env python3
"""Collect labeled vehicle-state traces for training a learned terrain estimator.

For each (terrain, throttle, steer_amp, seed) combination we launch
``chrono_sim_node`` headless, drive sinusoidal steering with constant throttle,
and dump the published ``VehicleState`` stream to a CSV in
``../data/terrain_traces/``.  At inference time the learned estimator is
restricted to information actually available on the robot (no oracle tire
forces), so the saved columns mirror exactly what ``run_openloop_terrain_est``
sees: longitudinal/lateral velocity, yaw rate, body-frame accelerations,
wheel encoder speeds, steering sensor angle, and the commanded throttle.
"""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
import time
from itertools import product
from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parent.parent
SIM_DIR = ROOT / "simulation"
sys.path.insert(0, str(SIM_DIR))

from hil_messages import (
    ControlCommand, SimStatus, VehicleState,
    ZMQPublisher, ZMQSubscriber,
    sim_sub_endpoint, ctrl_pub_endpoint,
)


TRUE_N = {"clay": 0.5, "dirt": 0.7, "sand": 1.1}

CSV_HEADER = [
    "t", "terrain", "n_true", "throttle_cmd", "steer_cmd",
    "u", "v", "omega", "ax", "ay",
    "wheel_omega_fl", "wheel_omega_fr",
    "wheel_omega_rl", "wheel_omega_rr",
    "steering_angle",
]


def collect_one(*, terrain: str, throttle: float, steer_amp: float,
                steer_period: float, seed: int, duration: float,
                sim_port: int, ctrl_port: int, out_csv: Path,
                python_exe: str,
                terrain_yaml: Optional[Path] = None,
                n_true_override: Optional[float] = None) -> bool:
    """Run a single sim and stream the trace to CSV.  Returns True on success.

    If ``terrain_yaml`` is supplied the chrono sim is launched with
    ``--terrain-config`` instead of a preset.  ``n_true_override`` is the
    ground-truth n value written into the CSV (necessary because for custom
    YAMLs the global ``TRUE_N`` table doesn't apply)."""

    if out_csv.exists() and out_csv.stat().st_size > 4096:
        print(f"  [skip] {out_csv.name} already populated")
        return True

    out_csv.parent.mkdir(parents=True, exist_ok=True)

    sim_script = SIM_DIR / "chrono_sim_node.py"
    base_args = [
        str(sim_script),
        "--time", str(duration + 5.0), "--speed", "5",
        # The --terrain CLI is required by chrono_sim_node even when a YAML
        # config is provided (it gates a few optional code paths); pass the
        # closest-preset name so logging / NN-CBF lookups still work.
        "--terrain", terrain, "--path", "sinusoidal",
        "--vis-mode", "none",
        "--sim-port", str(sim_port),
        "--ctrl-host", "localhost",
        "--ctrl-port", str(ctrl_port),
        "--no-wait-for-controller",
    ]
    if terrain_yaml is not None:
        base_args += ["--terrain-config", str(terrain_yaml)]
    if python_exe == "conda":
        # ``conda`` is a legacy sentinel; use the interpreter already running
        # this collector rather than a machine-specific hard-coded path.
        sim_cmd = [sys.executable, *base_args]
    else:
        sim_cmd = [python_exe, *base_args]
    print(f"  launching sim: {terrain} thr={throttle} amp={steer_amp} seed={seed}")
    sim_proc = subprocess.Popen(sim_cmd,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)

    state_sub = ZMQSubscriber(sim_sub_endpoint("localhost", sim_port))
    ctrl_pub = ZMQPublisher(ctrl_pub_endpoint(ctrl_port))
    time.sleep(2.0)   # let sim bind sockets

    rows: list[list] = []
    seq = 0
    n_true = (float(n_true_override) if n_true_override is not None
              else TRUE_N[terrain])
    t0_sim: float | None = None
    last_log_t = 0.0
    timeout_count = 0

    try:
        while True:
            res = state_sub.recv(timeout_ms=500)
            if res is None:
                timeout_count += 1
                if timeout_count > 30:
                    print("    [warn] no messages — aborting run")
                    return False
                continue
            timeout_count = 0
            topic, msg = res
            if isinstance(msg, SimStatus) and msg.event == "stop":
                break
            if not isinstance(msg, VehicleState):
                continue

            t = float(msg.time)
            if t0_sim is None:
                t0_sim = t
            t_rel = t - t0_sim

            phase = 2 * math.pi * (t_rel + 0.07 * seed) / steer_period
            steer_cmd = float(steer_amp * math.sin(phase))
            cmd = ControlCommand(
                time=t, wall_time=time.time(), seq=seq,
                steering=steer_cmd, throttle=float(throttle), braking=0.0,
                delta=0.0, acceleration=0.0,
                delta_dot=0.0, jerk=0.0,
            )
            ctrl_pub.send(cmd)
            seq += 1

            rows.append([
                f"{t_rel:.4f}", terrain, f"{n_true:.3f}",
                f"{throttle:.3f}", f"{steer_cmd:.4f}",
                f"{msg.u:.4f}", f"{msg.v:.4f}", f"{msg.omega:.4f}",
                f"{msg.ax:.4f}", f"{msg.ay:.4f}",
                f"{msg.wheel_omega_fl:.4f}", f"{msg.wheel_omega_fr:.4f}",
                f"{msg.wheel_omega_rl:.4f}", f"{msg.wheel_omega_rr:.4f}",
                f"{msg.steering_angle:.4f}",
            ])

            if t_rel - last_log_t > 5.0:
                print(f"    t={t_rel:5.1f}s  u={msg.u:.2f} v={msg.v:+.2f} "
                      f"omega={msg.omega:+.2f}")
                last_log_t = t_rel

            if t_rel >= duration:
                break

    finally:
        state_sub.close()
        ctrl_pub.close()
        sim_proc.terminate()
        try:
            sim_proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            sim_proc.kill()
            sim_proc.wait()

    if len(rows) < 100:
        print(f"    [warn] only {len(rows)} rows captured — discarding")
        return False

    with out_csv.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_HEADER)
        w.writerows(rows)
    print(f"    [ok] saved {len(rows)} rows -> {out_csv.name}")
    return True


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out", default=str(
        Path(__file__).parent.parent / "data" / "terrain_traces"))
    p.add_argument("--duration", type=float, default=30.0)
    p.add_argument("--terrains", nargs="+",
                   default=["clay", "dirt", "sand"])
    p.add_argument("--throttles", type=float, nargs="+",
                   default=[0.35, 0.55, 0.75])
    p.add_argument("--steer-amps", type=float, nargs="+",
                   default=[0.35, 0.6])
    p.add_argument("--steer-period", type=float, default=3.0)
    p.add_argument("--seeds", type=int, nargs="+", default=[0, 1])
    p.add_argument("--sim-port-base", type=int, default=19500)
    p.add_argument("--python-exe", default="conda")
    args = p.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    combos = list(product(args.terrains, args.throttles, args.steer_amps, args.seeds))
    print(f"[collect] {len(combos)} runs total -> {out_dir}")

    port = args.sim_port_base
    for idx, (terr, thr, amp, seed) in enumerate(combos, 1):
        print(f"\n[{idx}/{len(combos)}] terrain={terr} thr={thr} amp={amp} seed={seed}")
        name = f"{terr}_thr{int(round(thr*100)):02d}_amp{int(round(amp*100)):02d}_seed{seed}.csv"
        out = out_dir / name
        ok = collect_one(
            terrain=terr, throttle=thr, steer_amp=amp,
            steer_period=args.steer_period, seed=seed,
            duration=args.duration,
            sim_port=port, ctrl_port=port + 1,
            out_csv=out, python_exe=args.python_exe,
        )
        port += 4
        if port > args.sim_port_base + 200:
            port = args.sim_port_base
        if not ok:
            print(f"    run failed — continuing")
        time.sleep(0.5)

    print("\n[collect] done")


if __name__ == "__main__":
    main()
