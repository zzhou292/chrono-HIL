#!/usr/bin/env python3
"""Collect per-wheel tire forces WITH ground-truth SCM sinkage.

Motivation
----------
The whole-vehicle surrogate's *rear*-axle force prediction is stubborn: adding
every causal dynamic/load-transfer feature improves the FRONT axle by ~25 %
(Fx and Fy) but the REAR by only ~3 %. The leading hypothesis is the
multi-pass effect -- the rear wheels run on soil the front wheels just
compacted/rutted, so the rear's force depends on a soil state no current
feature carries. A 4 s probe confirms it directly: on dirt the rear axle sinks
~0.058 m vs the front's ~0.037 m at the same instant.

This collector logs the ground-truth per-wheel sinkage (init terrain height -
current height under each spindle) alongside the usual operating point, so we
can train baseline vs. +sinkage on *identical* data and measure the rear-Fy
effect. Sinkage is used here as the ORACLE upper bound -- if even ground-truth
sinkage doesn't help the rear, the multi-pass hypothesis is wrong. If it does,
the deployable version derives sinkage from the estimated soil + measured load
(Bekker pressure-sinkage), which stays inference-legal.

Excitation is open-loop scripted (sinusoid / fast-sinusoid / chirp / step) over
uniform-LHS terrains, so coverage is broad and unbiased (CLAUDE.md rule 8) and
the front-then-rear track overlap that creates multi-pass is naturally present.
Parallelized per terrain (CLAUDE.md rule 7).
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
import traceback
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List

import numpy as np

_sim_dir = str(Path(__file__).resolve().parent.parent / "simulation")
if _sim_dir not in sys.path:
    sys.path.insert(0, _sim_dir)

REPO = Path(__file__).resolve().parent.parent


# ─── steering / throttle excitation (broad, unbiased) ────────────────────

def _sinusoidal(t, amp, period):
    return amp * math.sin(2 * math.pi * t / period)


def _chirp(t, amp, p0, p1, dur):
    f0, f1 = 1.0 / p0, 1.0 / p1
    f = f0 + (f1 - f0) * t / max(dur, 1e-6)
    return amp * math.sin(2 * math.pi * f * t)


def _step(t, amp, every):
    return amp if int(t / every) % 2 == 0 else -amp


class TrajectorySet:
    """Four sequential steering excitations + slow throttle modulation.

    A per-scenario steer bias and phase shift diversify the runs so the rear
    axle sees sustained-slip cornering (not just symmetric sweeps).
    """

    def __init__(self, total_time: float, seed: int = 0):
        self._total = total_time
        self._seg = total_time / 4.0
        rng = np.random.default_rng(seed)
        self._bias = float(rng.uniform(-0.18, 0.18))      # sustained-slip offset
        self._phase = float(rng.uniform(0.0, 2 * math.pi))
        self._amp = float(rng.uniform(0.35, 0.6))
        self._thr_base = float(rng.uniform(0.4, 0.6))

    def steering(self, t: float) -> float:
        seg = self._seg
        a = self._amp
        if t < seg:
            s = _sinusoidal(t + self._phase, a, 6.0)
        elif t < 2 * seg:
            s = _sinusoidal((t - seg) + self._phase, a * 0.7, 2.0)
        elif t < 3 * seg:
            s = _chirp(t - 2 * seg, a * 0.85, 8.0, 1.5, seg)
        else:
            s = _step(t - 3 * seg, a * 0.85, 3.0)
        return float(np.clip(s + self._bias, -1.0, 1.0))

    def throttle(self, t: float) -> float:
        base = self._thr_base
        if t > self._total * 0.75:
            base *= 0.7
        return float(np.clip(base + 0.1 * math.sin(2 * math.pi * t / 20.0), 0.0, 1.0))


WHEEL_MAP = [(0, "LEFT", 0), (0, "RIGHT", 1), (1, "LEFT", 0), (1, "RIGHT", 1)]

CSV_HEADER = [
    "scenario_id", "timestep", "axle_id", "side",
    "slip_ratio", "slip_angle", "velocity", "vertical_load",
    "steering_rate", "steering_angle",
    "u_body", "v_body", "yaw_rate", "throttle_cmd",
    "sinkage", "sinkage_front_same_side",
    "bekker_Kphi", "bekker_Kc", "bekker_n",
    "mohr_cohesion", "mohr_friction", "janosi_shear",
    "Fx", "Fy",
]


@dataclass(frozen=True)
class Task:
    """Pickle-friendly per-terrain run description."""
    scenario_id: int
    terrain_config: Dict
    out_csv: str
    total_time: float = 25.0
    step_size: float = 5e-4
    log_interval: float = 0.02
    settle_time: float = 2.0
    seed: int = 0


def _run_one(task: Task) -> Dict:
    """Worker: run one scripted scenario on one terrain, log per-wheel rows."""
    t0 = time.time()
    try:
        import pychrono as chrono
        import pychrono.vehicle as veh
        from chrono_setup import setup_chrono_vehicle, setup_scm_terrain

        SIDE = {"LEFT": veh.LEFT, "RIGHT": veh.RIGHT}

        system, vehicle = setup_chrono_vehicle(visualize=False)
        terrain, tp = setup_scm_terrain(
            system, vehicle=vehicle, visualize=False,
            terrain_config=task.terrain_config,
        )
        vo = vehicle.GetVehicle()
        driver = veh.ChDriver(vo)
        traj = TrajectorySet(task.total_time, seed=task.seed)

        terr_cols = [
            tp["Kphi"], tp["Kc"], tp["n"], tp["c"],
            math.radians(tp["phi"]), tp["k"],
        ]

        rows: List[List] = []
        last_log = -1.0
        prev_steer = 0.0
        prev_t = 0.0
        step = task.step_size
        log_n = 0
        t = 0.0
        n_steps = int((task.total_time + task.settle_time) / step)

        for i in range(n_steps):
            t = i * step
            active = t > task.settle_time
            te = max(t - task.settle_time, 0.0)
            steer_cmd = traj.steering(te) if active else 0.0
            thr_cmd = traj.throttle(te) if active else 0.3

            di = veh.DriverInputs()
            di.m_steering = float(np.clip(steer_cmd, -1.0, 1.0))
            di.m_throttle = float(np.clip(thr_cmd, 0.0, 1.0))
            di.m_braking = 0.0

            driver.Synchronize(t)
            terrain.Synchronize(t)
            vehicle.Synchronize(t, di, terrain)
            driver.Advance(step)
            terrain.Advance(step)
            vehicle.Advance(step)

            ch = vehicle.GetChassisBody()
            vel_loc = ch.GetRot().RotateBack(ch.GetPosDt())
            u_body, v_body = vel_loc.x, vel_loc.y
            yaw_rate = ch.GetAngVelLocal().z

            dt = t - prev_t if t > prev_t else step
            steering_rate = (steer_cmd - prev_steer) / max(dt, 1e-6)
            prev_steer, prev_t = steer_cmd, t

            if active and (t - last_log) >= task.log_interval:
                if abs(u_body) > 1.0:
                    # per-wheel sinkage first (need front for the rear's context)
                    sink = {}
                    for ax, sd, side_id in WHEEL_MAP:
                        sp = vo.GetSpindlePos(ax, SIDE[sd])
                        loc = chrono.ChVector3d(sp.x, sp.y, 0.0)
                        sink[(ax, side_id)] = terrain.GetInitHeight(loc) - terrain.GetHeight(loc)
                    for ax, sd, side_id in WHEEL_MAP:
                        sdx = SIDE[sd]
                        tire = vo.GetTire(ax, sdx)
                        fg = tire.ReportTireForce(terrain)
                        f_tire = vo.GetSpindleRot(ax, sdx).RotateBack(fg.force)
                        front_same = sink[(0, side_id)]
                        rows.append([
                            task.scenario_id, log_n, ax, side_id,
                            float(tire.GetLongitudinalSlip()),
                            float(tire.GetSlipAngle()),
                            float(max(abs(u_body), 0.5)),
                            float(abs(f_tire.z)),
                            float(steering_rate), float(steer_cmd),
                            float(u_body), float(v_body), float(yaw_rate),
                            float(di.m_throttle),
                            float(sink[(ax, side_id)]), float(front_same),
                            *terr_cols,
                            float(f_tire.x), float(f_tire.y),
                        ])
                    log_n += 1
                last_log = t

        with open(task.out_csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(CSV_HEADER)
            w.writerows(rows)

        return {"scenario_id": task.scenario_id, "ok": True,
                "rows": len(rows), "wall_s": time.time() - t0,
                "csv": task.out_csv}
    except Exception as e:  # noqa: BLE001  (report, don't take down the pool)
        return {"scenario_id": task.scenario_id, "ok": False,
                "rows": 0, "wall_s": time.time() - t0,
                "err": f"{e}\n{traceback.format_exc()}", "csv": task.out_csv}


def build_tasks(out_dir: Path, n_lhs: int, lhs_seed: int, preset_reps: int,
                total_time: float, step_size: float) -> List[Task]:
    from param_consistency import get_terrain_preset, generate_lhs_terrain_yaml_dicts
    tasks: List[Task] = []
    sid = 0
    # canonical presets, repeated with different excitation seeds
    for name in ("clay", "dirt", "sand"):
        cfg = get_terrain_preset(name)
        for rep in range(preset_reps):
            tasks.append(Task(
                scenario_id=sid, terrain_config=dict(cfg),
                out_csv=str(out_dir / f"scn_{sid:04d}_{name}.csv"),
                total_time=total_time, step_size=step_size, seed=1000 + sid,
            ))
            sid += 1
    # uniform-LHS terrains
    if n_lhs > 0:
        for cfg in generate_lhs_terrain_yaml_dicts(n_lhs, seed=lhs_seed):
            tasks.append(Task(
                scenario_id=sid, terrain_config=dict(cfg),
                out_csv=str(out_dir / f"scn_{sid:04d}_lhs.csv"),
                total_time=total_time, step_size=step_size, seed=1000 + sid,
            ))
            sid += 1
    return tasks


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--lhs", type=int, default=45, help="number of LHS terrains")
    p.add_argument("--lhs-seed", type=int, default=7)
    p.add_argument("--preset-reps", type=int, default=2,
                   help="repeats of each canonical preset (different excitation)")
    p.add_argument("--time", type=float, default=25.0, help="sim seconds per run")
    p.add_argument("--step-size", type=float, default=5e-4)
    p.add_argument("--workers", type=int, default=6)
    p.add_argument("--output-dir", default=None)
    args = p.parse_args()

    out_dir = Path(args.output_dir) if args.output_dir else (
        REPO / "data" / "whole_vehicle" / "sinkage")
    out_dir.mkdir(parents=True, exist_ok=True)

    tasks = build_tasks(out_dir, args.lhs, args.lhs_seed, args.preset_reps,
                        args.time, args.step_size)
    print(f"[sinkage] {len(tasks)} scenarios -> {out_dir}  (workers={args.workers})")

    results: List[Dict] = []
    # solo-prewarm task 0 (Chrono/JIT warmup, matches CLAUDE.md guidance)
    r0 = _run_one(tasks[0])
    results.append(r0)
    print(f"  [{r0['scenario_id']:04d}] ok={r0['ok']} rows={r0['rows']} "
          f"wall={r0['wall_s']:.0f}s")
    if not r0["ok"]:
        print(r0.get("err", ""))

    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = {ex.submit(_run_one, t): t for t in tasks[1:]}
        for fut in as_completed(futs):
            r = fut.result()
            results.append(r)
            tag = "ok" if r["ok"] else "FAIL"
            print(f"  [{r['scenario_id']:04d}] {tag} rows={r['rows']} "
                  f"wall={r['wall_s']:.0f}s")
            if not r["ok"]:
                print("   ", r.get("err", "").splitlines()[-1] if r.get("err") else "")

    ok = [r for r in results if r["ok"]]
    total_rows = sum(r["rows"] for r in ok)
    print(f"\n[sinkage] done: {len(ok)}/{len(tasks)} ok, {total_rows} rows -> {out_dir}")

    # aggregate into one CSV the trainer can read
    agg = out_dir / "training_data_sinkage.csv"
    import pandas as pd
    frames = []
    for r in ok:
        try:
            frames.append(pd.read_csv(r["csv"]))
        except Exception:
            pass
    if frames:
        df = pd.concat(frames, ignore_index=True)
        df.to_csv(agg, index=False)
        print(f"[sinkage] aggregated {len(df)} rows -> {agg}")
    print("SINKAGE_COLLECT_DONE")


if __name__ == "__main__":
    main()
