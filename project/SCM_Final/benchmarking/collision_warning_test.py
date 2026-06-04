#!/usr/bin/env python3
"""Test the modular collision warning system under terrain × latency sweeps.

Drives the HMMWV blindly straight at a fixed rock obstacle with a
constant throttle (no MPC, no safety filter) and a passive collision
warning system running in parallel. Records:

  * t_first_warn[severity] — earliest sim time the warning hit each
    severity level (YELLOW/ORANGE/RED).
  * t_collision           — actual sim time the chassis impacted the rock,
    or NaN if the run finished without contact.
  * stopping distance d_stop estimated by the warning system at t_first_warn[RED].

Sweeps:
  * terrain  ∈ {clay, dirt, sand} — soft → hard
  * latency  ∈ {0, 150 ms, 300 ms} — operator-side delay piped to the
    warning system via ``set_teleop_delay``.

The warning is considered *effective* if a RED warning fires before the
collision with enough lead time to stop:
    lead_time = t_collision - t_red
    expected_lead = d_stop / v_at_warning
The benchmark reports lead_time and expected_lead per (terrain, latency).

This script does NOT use a controller — it talks directly to chrono_sim_node
via ZMQ, supplying its own constant-throttle commands.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import random
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, asdict
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SIM = REPO / "simulation"
sys.path.insert(0, str(SIM))

from hil_messages import (   # noqa: E402
    ControlCommand, SimStatus, VehicleState,
    ZMQPublisher, ZMQSubscriber,
    sim_sub_endpoint, ctrl_pub_endpoint,
)
from safety.collision_warning import (  # noqa: E402
    CollisionWarningSystem, make_collision_warning_system,
)

TERRAINS = ("clay", "dirt", "sand")
DEFAULT_LATENCIES_S = (0.0, 0.15, 0.30)


@dataclass(frozen=True)
class Scenario:
    idx: int
    terrain: str
    latency_s: float
    sim_port: int
    ctrl_port: int
    rock_seed: int
    out_dir: str
    duration: float = 18.0
    throttle: float = 0.6
    # Place the rock ~25 m straight ahead in the path of the vehicle.
    rock_zone_x: tuple = (25.0, 26.0)
    rock_zone_y: tuple = (-0.8, 0.8)


def _quat_yaw(msg) -> float:
    return math.atan2(
        2 * (msg.quat_e0 * msg.quat_e3 + msg.quat_e1 * msg.quat_e2),
        1 - 2 * (msg.quat_e2 ** 2 + msg.quat_e3 ** 2),
    )


def _terrain_n_for(terr: str) -> float:
    # Mirrors TERRAIN_PRESETS["<terr>"]["n"] — kept hard-coded here so we
    # don't have to import param_consistency in this test.
    return {"clay": 0.50, "dirt": 0.70, "sand": 1.10}.get(terr, 1.0)


def _run_one(task: Scenario) -> dict:
    run_dir = Path(task.out_dir)
    run_dir.mkdir(parents=True, exist_ok=True)
    sim_log = run_dir / "sim.log"

    sim_cmd = [
        sys.executable, "-u", str(SIM / "chrono_sim_node.py"),
        "--time", str(task.duration + 5.0),
        "--speed", str(task.throttle * 12.0),
        "--terrain", task.terrain, "--path", "sinusoidal",
        "--vis-mode", "none",
        "--sim-port", str(task.sim_port),
        "--ctrl-host", "localhost",
        "--ctrl-port", str(task.ctrl_port),
        "--no-wait-for-controller",
        "--rocks", "1",
        "--rock-seed", str(task.rock_seed),
        "--rock-zone-x", str(task.rock_zone_x[0]), str(task.rock_zone_x[1]),
        "--rock-zone-y", str(task.rock_zone_y[0]), str(task.rock_zone_y[1]),
        "--rock-size", "1.2", "1.2",
        "--lead-in", "0.0",
    ]
    log_f = open(sim_log, "w")
    proc = subprocess.Popen(sim_cmd, stdout=log_f, stderr=subprocess.STDOUT)
    time.sleep(2.0)

    sub = ZMQSubscriber(sim_sub_endpoint("localhost", task.sim_port))
    pub = ZMQPublisher(ctrl_pub_endpoint(task.ctrl_port))

    warn_sys: CollisionWarningSystem = make_collision_warning_system(
        flavor="ttc", verbose=False,
        tire_model_dir=str(REPO / "nn_models" / "rig_rate_64_32"),
    )
    warn_sys.set_teleop_delay(task.latency_s)

    seq = 0
    t0_sim = None
    t_first = {1: math.nan, 2: math.nan, 3: math.nan}  # YELLOW/ORANGE/RED
    t_collision = math.nan
    last_severity = 0
    last_warn = None
    rows: list[dict] = []
    rock_xyz: tuple[float, float, float] | None = None
    timeout_count = 0
    eta_rng = random.Random(task.idx)

    try:
        while True:
            res = sub.recv(timeout_ms=500)
            if res is None:
                timeout_count += 1
                if timeout_count > 30:
                    break
                continue
            timeout_count = 0
            topic, msg = res

            if isinstance(msg, SimStatus) and msg.event == "stop":
                break
            if not isinstance(msg, VehicleState):
                continue
            # Obstacle list rides on VehicleState (flat [x,y,r,...]).
            if rock_xyz is None and msg.obstacles and len(msg.obstacles) >= 3:
                rock_xyz = (
                    float(msg.obstacles[0]),
                    float(msg.obstacles[1]),
                    float(msg.obstacles[2]),
                )

            t = float(msg.time)
            if t0_sim is None:
                t0_sim = t
            t_rel = t - t0_sim

            # Simulated remote-driver command path with the configured
            # one-way delay: stamp now-tau on the wall_time so the
            # warning system's update_command_age sees the delay.
            wall_now = time.time() - task.latency_s
            # Constant low throttle, zero steering: drive blindly forward.
            cmd = ControlCommand(
                time=t, wall_time=wall_now, seq=seq,
                steering=0.0, throttle=task.throttle, braking=0.0,
                delta=0.0, acceleration=0.0,
                delta_dot=0.0, jerk=0.0,
            )
            pub.send(cmd)
            seq += 1

            # Feed jitter (uniform ±20 ms × eta_rng) so the warning
            # system's jitter estimator has something to chew on.
            jitter = eta_rng.uniform(-0.02, 0.02)
            warn_sys.update_command_age(wall_now + jitter)

            # Build inputs for the warning system.
            yaw = _quat_yaw(msg)
            vehicle_state = {
                "x": float(msg.x_cg), "y": float(msg.y_cg),
                "psi": yaw, "u": float(msg.u),
            }
            obstacles = []
            if rock_xyz is not None:
                obstacles.append(rock_xyz)
            warning = warn_sys.evaluate(
                vehicle_state, obstacles,
                terrain_n=_terrain_n_for(task.terrain),
            )

            for sev in (1, 2, 3):
                if warning.severity >= sev and math.isnan(t_first[sev]):
                    t_first[sev] = t_rel

            # Detect a real chassis-rock collision via Euclidean distance
            if rock_xyz is not None:
                dx = rock_xyz[0] - msg.x_cg
                dy = rock_xyz[1] - msg.y_cg
                d = math.sqrt(dx * dx + dy * dy)
                # Vehicle CG to rock surface (rock radius + ~1m vehicle half-length)
                if math.isnan(t_collision) and d - rock_xyz[2] < 1.0:
                    t_collision = t_rel

            rows.append(dict(
                t=t_rel,
                u=float(msg.u),
                severity=int(warning.severity),
                ttc=float(warning.ttc) if math.isfinite(warning.ttc) else -1.0,
                clearance=float(warning.clearance) if math.isfinite(warning.clearance) else -1.0,
                stopping_distance=float(warning.stopping_distance),
                margin=float(warning.margin) if math.isfinite(warning.margin) else -1.0,
                latency_inflation_m=float(warning.latency_inflation_m),
            ))
            last_severity = warning.severity
            last_warn = warning

            if t_rel >= task.duration:
                break
            # Stop early if we already collided to save sim time
            if math.isfinite(t_collision) and t_rel - t_collision > 1.0:
                break
    finally:
        sub.close(); pub.close()
        proc.terminate()
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.kill(); proc.wait()
        log_f.close()

    # Write per-tick trace
    trace_csv = run_dir / "trace.csv"
    if rows:
        with open(trace_csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            for r in rows:
                w.writerow(r)

    return dict(
        idx=task.idx,
        terrain=task.terrain,
        latency_s=task.latency_s,
        t_first_yellow=t_first[1],
        t_first_orange=t_first[2],
        t_first_red=t_first[3],
        t_collision=t_collision,
        lead_red_s=(t_collision - t_first[3]) if (math.isfinite(t_first[3])
                                                  and math.isfinite(t_collision))
                                              else math.nan,
        last_severity=last_severity,
        last_message=(last_warn.message if last_warn else ""),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--terrains", nargs="+", default=list(TERRAINS),
                    choices=TERRAINS)
    ap.add_argument("--latencies", nargs="+", type=float,
                    default=list(DEFAULT_LATENCIES_S))
    ap.add_argument("--rock-seeds", nargs="+", type=int, default=[7])
    ap.add_argument("--out", type=Path,
                    default=REPO / "benchmarking" / "results" /
                            f"collision_warning_{time.strftime('%Y%m%d_%H%M%S')}")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--base-port", type=int, default=51000)
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    tasks: list[Scenario] = []
    idx = 0
    for terr in args.terrains:
        for lat in args.latencies:
            for seed in args.rock_seeds:
                run_dir = args.out / f"{terr}_lat{int(round(lat*1000)):03d}_s{seed}"
                tasks.append(Scenario(
                    idx=idx, terrain=terr, latency_s=float(lat),
                    sim_port=args.base_port + 4 * idx,
                    ctrl_port=args.base_port + 4 * idx + 1,
                    rock_seed=int(seed),
                    out_dir=str(run_dir),
                ))
                idx += 1
    print(f"[cw-test] {len(tasks)} scenarios → {args.out}")

    results = []
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = {ex.submit(_run_one, t): t for t in tasks}
        for fut in as_completed(futs):
            r = fut.result()
            results.append(r)
            print(f"  {r['terrain']:5s}  lat={r['latency_s']*1000:4.0f}ms  "
                  f"t_yellow={r['t_first_yellow']:5.2f}s  "
                  f"t_orange={r['t_first_orange']:5.2f}s  "
                  f"t_red={r['t_first_red']:5.2f}s  "
                  f"t_collision={r['t_collision']:5.2f}s  "
                  f"lead_red={r['lead_red_s']:+.2f}s",
                  flush=True)

    # Save summary
    summary = args.out / "results.csv"
    with open(summary, "w", newline="") as f:
        if results:
            w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
            w.writeheader()
            for r in sorted(results, key=lambda x: (x["terrain"], x["latency_s"])):
                w.writerow(r)
    print(f"\n[cw-test] saved {summary}")


if __name__ == "__main__":
    main()
