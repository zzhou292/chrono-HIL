#!/usr/bin/env python3
"""Collect labeled vehicle-state traces *with the MPC closing the loop*.

The original ``collect_terrain_traces.py`` drives the vehicle with an
open-loop sinusoidal steer command + constant throttle.  The learned
terrain estimator trained on that data then jitters in closed-loop MPC
because the windowed-feature distribution is different (MPC modulates
throttle to track speed, steering tracks the path rather than tracing a
pure sine, lateral excitation is shape-tied to the path, etc.).

This script fixes the train/eval gap by:

1.  Launching ``chrono_sim_node`` headless (with a custom YAML or preset
    terrain), exactly as the open-loop collector does;
2.  Launching ``acados_mpc_controller_node`` as a separate subprocess so
    it generates real MPC commands;
3.  Subscribing to the sim's ``VehicleState`` topic *and* the controller's
    ``ControlCommand`` topic, joining them on simulation time, and
    writing a CSV in the same column layout the trainer already
    consumes.

The written CSVs land in ``data/terrain_traces_closedloop/`` so they
remain easy to mix-or-match with the open-loop set during retraining.
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
from typing import Optional, Dict

ROOT = Path(__file__).resolve().parent.parent
SIM_DIR = ROOT / "simulation"
sys.path.insert(0, str(SIM_DIR))

from hil_messages import (
    ControlCommand, SimStatus, VehicleState,
    ZMQSubscriber,
    sim_sub_endpoint, ctrl_sub_endpoint,
)


TRUE_N = {"clay": 0.5, "dirt": 0.7, "sand": 1.1}

CSV_HEADER = [
    "t", "terrain", "n_true", "throttle_cmd", "steer_cmd",
    "u", "v", "omega", "ax", "ay",
    "wheel_omega_fl", "wheel_omega_fr",
    "wheel_omega_rl", "wheel_omega_rr",
    "steering_angle",
]


CONDA_BIN = "/home/kyle/miniconda3/bin/conda"


def _conda_run(args: list[str]) -> list[str]:
    """Wrap a python invocation in `conda run -n sim --no-capture-output`."""
    return [CONDA_BIN, "run", "--no-capture-output", "-n", "sim", "python", *args]


def collect_one_closed_loop(
    *,
    terrain: str,                 # preset name (clay/dirt/sand) — controls
                                  # MPC initial NN-CBF lookup; the actual
                                  # SCM physics is set by the YAML if given
    n_true: float,                # ground-truth n written into the CSV
    throttle_cap: float,          # passed through as MPC v_target proxy
    duration: float,
    sim_port: int,
    ctrl_port: int,
    out_csv: Path,
    terrain_yaml: Optional[Path] = None,
    speed: float = 5.0,
    path: str = "sinusoidal",
    sine_amplitude: float = 2.0,
    sine_wavelength: float = 30.0,
    seed: int = 0,
    nn_model: str = "paper_v2_mlp_16_4",
) -> bool:

    if out_csv.exists() and out_csv.stat().st_size > 4096:
        print(f"  [skip] {out_csv.name} already populated")
        return True
    out_csv.parent.mkdir(parents=True, exist_ok=True)

    sim_script = SIM_DIR / "chrono_sim_node.py"
    ctrl_script = SIM_DIR / "acados_mpc_controller_node.py"

    sim_args = [
        str(sim_script),
        "--time", str(duration + 8.0),
        "--speed", str(speed),
        "--terrain", terrain, "--path", path,
        "--vis-mode", "none",
        "--sim-port", str(sim_port),
        "--ctrl-host", "localhost",
        "--ctrl-port", str(ctrl_port),
    ]
    if terrain_yaml is not None:
        sim_args += ["--terrain-config", str(terrain_yaml)]
    if path == "sinusoidal":
        sim_args += ["--sine-amplitude", str(sine_amplitude),
                     "--sine-wavelength", str(sine_wavelength)]

    ctrl_args = [
        str(ctrl_script),
        "--time", str(duration + 8.0),
        "--speed", str(speed),
        "--terrain", terrain, "--path", path,
        "--model", "nn", "--nn-model", nn_model,
        "--sim-host", "localhost",
        "--sim-port", str(sim_port),
        "--ctrl-port", str(ctrl_port),
        "--no-csv", "--no-plot",
    ]
    if path == "sinusoidal":
        ctrl_args += ["--sine-amplitude", str(sine_amplitude),
                      "--sine-wavelength", str(sine_wavelength)]

    print(f"  launching sim+ctrl: terrain={terrain}  n_true={n_true:.2f}  "
          f"speed={speed}  yaml={terrain_yaml}")
    sim_proc = subprocess.Popen(_conda_run(sim_args),
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
    time.sleep(2.0)   # let sim bind sockets
    ctrl_proc = subprocess.Popen(_conda_run(ctrl_args),
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)

    state_sub = ZMQSubscriber(sim_sub_endpoint("localhost", sim_port))
    cmd_sub = ZMQSubscriber(ctrl_sub_endpoint("localhost", ctrl_port))
    time.sleep(1.5)

    rows: list[list] = []
    last_cmd: Optional[ControlCommand] = None
    t0_sim: Optional[float] = None
    last_log_t = 0.0
    timeout_count = 0

    try:
        # Drive a short busy loop pulling messages from both sockets.
        while True:
            # Drain any pending control commands first (cheap).
            while True:
                cmd_res = cmd_sub.recv(timeout_ms=0)
                if cmd_res is None:
                    break
                _, cmsg = cmd_res
                if isinstance(cmsg, ControlCommand):
                    last_cmd = cmsg

            res = state_sub.recv(timeout_ms=500)
            if res is None:
                timeout_count += 1
                if timeout_count > 30:
                    print("    [warn] no state messages — aborting run")
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

            thr = float(last_cmd.throttle) if last_cmd is not None else 0.0
            steer = float(last_cmd.steering) if last_cmd is not None else 0.0

            rows.append([
                f"{t_rel:.4f}", terrain, f"{n_true:.3f}",
                f"{thr:.3f}", f"{steer:.4f}",
                f"{msg.u:.4f}", f"{msg.v:.4f}", f"{msg.omega:.4f}",
                f"{msg.ax:.4f}", f"{msg.ay:.4f}",
                f"{msg.wheel_omega_fl:.4f}", f"{msg.wheel_omega_fr:.4f}",
                f"{msg.wheel_omega_rl:.4f}", f"{msg.wheel_omega_rr:.4f}",
                f"{msg.steering_angle:.4f}",
            ])

            if t_rel - last_log_t > 5.0:
                print(f"    t={t_rel:5.1f}s  u={msg.u:.2f} v={msg.v:+.2f} "
                      f"omega={msg.omega:+.2f}  thr_cmd={thr:.2f}  "
                      f"steer_cmd={steer:+.2f}")
                last_log_t = t_rel

            if t_rel >= duration:
                break

    finally:
        state_sub.close()
        cmd_sub.close()
        for proc, name in ((ctrl_proc, "ctrl"), (sim_proc, "sim")):
            proc.terminate()
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()

    if len(rows) < 100:
        print(f"    [warn] only {len(rows)} rows captured — discarding")
        return False
    # Drop rows from before the controller actually started publishing
    # (steer_cmd will be exactly 0.0000 in that prefix).
    while rows and rows[0][4] == "0.0000" and rows[0][3] == "0.000":
        rows.pop(0)
    if len(rows) < 100:
        print(f"    [warn] only {len(rows)} rows after ctrl-warmup trim — discarding")
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
        Path(__file__).parent.parent / "data" / "terrain_traces_closedloop"))
    p.add_argument("--terrains", nargs="+",
                   default=["clay", "dirt", "sand"],
                   help="Preset terrains to run (also map to ground truth n)")
    p.add_argument("--speeds", type=float, nargs="+", default=[4.0, 5.5, 7.0],
                   help="MPC target speeds (m/s)")
    p.add_argument("--sine-amplitudes", type=float, nargs="+",
                   default=[1.5, 2.5])
    p.add_argument("--sine-wavelengths", type=float, nargs="+",
                   default=[25.0, 35.0])
    p.add_argument("--duration", type=float, default=25.0)
    p.add_argument("--seeds", type=int, nargs="+", default=[0])
    p.add_argument("--sim-port-base", type=int, default=31000)
    p.add_argument("--nn-model", default="paper_v2_mlp_16_4")
    args = p.parse_args()

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    combos = list(product(args.terrains, args.speeds,
                          args.sine_amplitudes, args.sine_wavelengths,
                          args.seeds))
    print(f"[cl-collect] {len(combos)} runs total -> {out_dir}")

    port = args.sim_port_base
    for idx, (terr, sp, amp, wl, seed) in enumerate(combos, 1):
        n_true = TRUE_N[terr]
        name = (f"cl_{terr}_v{int(round(sp*10)):03d}"
                f"_amp{int(round(amp*10)):02d}"
                f"_wl{int(round(wl)):02d}_seed{seed}.csv")
        out = out_dir / name
        print(f"\n[{idx}/{len(combos)}] {name}")
        ok = collect_one_closed_loop(
            terrain=terr, n_true=n_true,
            throttle_cap=0.65,
            duration=args.duration,
            sim_port=port, ctrl_port=port + 1,
            out_csv=out,
            speed=sp,
            sine_amplitude=amp,
            sine_wavelength=wl,
            seed=seed,
            nn_model=args.nn_model,
        )
        port += 4
        if port > args.sim_port_base + 200:
            port = args.sim_port_base
        if not ok:
            print("    run failed — continuing")
        time.sleep(1.0)

    print("\n[cl-collect] done")


if __name__ == "__main__":
    main()
