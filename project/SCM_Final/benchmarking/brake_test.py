#!/usr/bin/env python3
"""Validate the collision-warning brake-decel predictions against actual
Chrono SCM stopping behavior.

Each test:
  * t < 5 s: cruise-up to a target speed using a simple P throttle controller
  * t = 5 s: apply full brake (throttle=0, brake=1) and zero steering
  * t > 5 s: log until vehicle is below 0.2 m/s OR 20 s timeout

Then we compute, from the recorded chassis state:
  * u_initial    : speed at the moment brake was applied (m/s)
  * a_peak       : maximum |dv/dt| seen during the stop (m/s^2)
  * a_mean       : average decel over the stopping window
  * d_stop       : distance traveled from brake-apply to stop (m)
  * t_stop       : time taken to stop (s)

…and compare those against the rig-NN's predicted a_brake(n) and the
implied d_stop_pred = u^2 / (2 * a_brake).

Sweep: terrain ∈ {clay, dirt, sand}, target_speed ∈ {3, 5, 7} m/s, 3 seeds.
"""
from __future__ import annotations

import csv
import math
import random
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parent.parent
SIM = REPO / "simulation"
sys.path.insert(0, str(SIM))

from hil_messages import (   # noqa: E402
    ControlCommand, SimStatus, VehicleState,
    ZMQPublisher, ZMQSubscriber,
    sim_sub_endpoint, ctrl_pub_endpoint,
)

TERRAINS = ("clay", "dirt", "sand")
SPEEDS   = (3.0, 5.0, 7.0)
SEEDS    = (0, 1, 2)

BRAKE_T  = 5.0     # apply brake at this sim time
TIMEOUT  = 20.0    # max sim time
STOP_SPD = 0.2     # consider stopped below this


@dataclass(frozen=True)
class Scenario:
    idx: int
    terrain: str
    target_speed: float
    seed: int
    sim_port: int
    ctrl_port: int
    out_dir: str
    duration: float = TIMEOUT


def _throttle_for(t_rel: float, u: float, target_speed: float,
                   was_braking: bool) -> tuple[float, float, float]:
    """Pre-brake: simple P throttle for cruise. Post-brake: full brake."""
    if t_rel < BRAKE_T:
        # simple P controller
        err = target_speed - u
        thr = max(0.0, min(0.8, 0.2 + 0.10 * err))
        return thr, 0.0, 0.0
    return 0.0, 0.0, 1.0


def _run_one(task: Scenario) -> dict:
    Path(task.out_dir).mkdir(parents=True, exist_ok=True)
    sim_log = Path(task.out_dir) / "sim.log"
    trace_csv = Path(task.out_dir) / "trace.csv"

    sim_cmd = [
        sys.executable, "-u", str(SIM / "chrono_sim_node.py"),
        "--time", str(task.duration + 6.0),
        "--speed", "5",                       # init kinetic placeholder
        "--terrain", task.terrain, "--path", "sinusoidal",
        "--vis-mode", "none",
        "--sim-port", str(task.sim_port),
        "--ctrl-host", "localhost",
        "--ctrl-port", str(task.ctrl_port),
        "--no-wait-for-controller",
        "--bumpiness", "0",
        "--lead-in", "0.0",
    ]
    log_f = open(sim_log, "w")
    proc = subprocess.Popen(sim_cmd, stdout=log_f,
                             stderr=subprocess.STDOUT)
    time.sleep(2.0)

    sub = ZMQSubscriber(sim_sub_endpoint("localhost", task.sim_port))
    pub = ZMQPublisher(ctrl_pub_endpoint(task.ctrl_port))

    rows = []
    seq = 0
    t0_sim = None
    timeout_count = 0
    was_braking = False
    try:
        while True:
            res = sub.recv(timeout_ms=500)
            if res is None:
                timeout_count += 1
                if timeout_count > 30: break
                continue
            timeout_count = 0
            _, msg = res
            if isinstance(msg, SimStatus) and msg.event == "stop":
                break
            if not isinstance(msg, VehicleState):
                continue
            t = float(msg.time)
            if t0_sim is None:
                t0_sim = t
            t_rel = t - t0_sim
            thr, steer, brake = _throttle_for(t_rel, msg.u, task.target_speed,
                                               was_braking)
            if brake > 0.5:
                was_braking = True
            cmd = ControlCommand(
                time=t, wall_time=time.time(), seq=seq,
                steering=steer, throttle=thr, braking=brake,
                delta=0.0, acceleration=0.0,
                delta_dot=0.0, jerk=0.0,
            )
            pub.send(cmd)
            seq += 1
            rows.append({
                "t": t_rel,
                "x": float(msg.x_cg), "y": float(msg.y_cg),
                "u": float(msg.u), "ax": float(msg.ax),
                "throttle": thr, "brake": brake,
            })
            # Stop early once we're at rest after braking
            if was_braking and abs(msg.u) < STOP_SPD and t_rel > BRAKE_T + 0.5:
                break
            if t_rel >= task.duration:
                break
    finally:
        sub.close(); pub.close()
        proc.terminate()
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.kill(); proc.wait()
        log_f.close()

    if not rows:
        return dict(idx=task.idx, terrain=task.terrain,
                     target_speed=task.target_speed, seed=task.seed,
                     ok=False)

    df = pd.DataFrame(rows)
    df.to_csv(trace_csv, index=False)

    # Extract the brake-onset speed + post-brake metrics
    brake_mask = df["brake"] > 0.5
    if not brake_mask.any():
        return dict(idx=task.idx, terrain=task.terrain,
                     target_speed=task.target_speed, seed=task.seed,
                     ok=False)
    t_brake = float(df.loc[brake_mask].iloc[0].t)
    u0 = float(df.loc[brake_mask].iloc[0].u)
    x0 = float(df.loc[brake_mask].iloc[0].x)
    y0 = float(df.loc[brake_mask].iloc[0].y)
    post = df.loc[brake_mask].reset_index(drop=True)
    stop_idx = (post["u"].abs() < STOP_SPD).idxmax() if (post["u"].abs() < STOP_SPD).any() else (len(post) - 1)
    post_stop = post.iloc[: stop_idx + 1]
    t_stop = float(post_stop.iloc[-1].t) - t_brake
    d_stop = math.sqrt(
        (post_stop.iloc[-1].x - x0) ** 2 + (post_stop.iloc[-1].y - y0) ** 2
    )
    # Differentiate speed → decel
    dt = np.diff(post_stop["t"].to_numpy())
    du = np.diff(post_stop["u"].to_numpy())
    a = -du / np.maximum(dt, 1e-6)         # positive = deceleration
    a_peak = float(np.max(a)) if a.size else math.nan
    a_mean = (u0 - float(post_stop.iloc[-1].u)) / max(t_stop, 1e-6)

    return dict(idx=task.idx, terrain=task.terrain,
                 target_speed=task.target_speed, seed=task.seed,
                 u_initial=u0, a_peak=a_peak, a_mean=a_mean,
                 d_stop=d_stop, t_stop=t_stop, ok=True)


def main():
    out_root = REPO / "benchmarking" / "results" / \
        f"brake_test_{time.strftime('%Y%m%d_%H%M%S')}"
    out_root.mkdir(parents=True, exist_ok=True)

    tasks = []
    port = 58000
    for terr in TERRAINS:
        for v in SPEEDS:
            for seed in SEEDS:
                tasks.append(Scenario(
                    idx=len(tasks), terrain=terr, target_speed=v, seed=seed,
                    sim_port=port, ctrl_port=port + 1,
                    out_dir=str(out_root / f"{terr}_v{int(v)}_s{seed}"),
                ))
                port += 4

    print(f"Running {len(tasks)} brake tests "
          f"({len(TERRAINS)} terrains × {len(SPEEDS)} speeds × {len(SEEDS)} seeds)")
    results = []
    with ProcessPoolExecutor(max_workers=4) as ex:
        futs = {ex.submit(_run_one, t): t for t in tasks}
        for fut in as_completed(futs):
            r = fut.result()
            results.append(r)
            if r.get("ok"):
                print(f"  {r['terrain']:>4s} v={r['target_speed']:.0f} "
                      f"s={r['seed']}  u0={r['u_initial']:.2f}m/s  "
                      f"a_peak={r['a_peak']:.2f}  a_mean={r['a_mean']:.2f}  "
                      f"d_stop={r['d_stop']:.2f}m  t_stop={r['t_stop']:.2f}s",
                      flush=True)
            else:
                print(f"  {r['terrain']:>4s} v={r['target_speed']:.0f} "
                      f"s={r['seed']}  FAILED", flush=True)

    df = pd.DataFrame(results)
    df.to_csv(out_root / "results.csv", index=False)
    print(f"\nWrote {out_root / 'results.csv'}")


if __name__ == "__main__":
    main()
