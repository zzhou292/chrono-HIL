#!/usr/bin/env python3
"""Collect CLEAN (DOB-off) NMPC prediction logs for motion-resistance calibration.

The longitudinal channel is u_dot = ax + du_dot_resid with du_dot_resid currently
0, so with the throttle DOB also OFF the prediction-vs-actual speed drift is the
pure unmodelled sinkage drag (no DOB throttle to confound it). We log predictions
(LOG_MPC_PREDICTIONS=1) over clay/dirt/sand x speeds x seeds, terrain KNOWN (no
estimator) so the calibration is per-n. Feed the output to
calibrate_motion_resistance.py.
"""
from __future__ import annotations
import argparse, os, sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import DEFAULT_NN_MODEL, launch_and_collect, timestamped_result_dir  # noqa: E402

TERRAINS = ("clay", "dirt", "sand")


@dataclass(frozen=True)
class _Task:
    idx: int
    terrain: str
    speed: float
    seed: int
    run_dir_str: str
    sim_port: int
    ctrl_port: int
    sim_time: float
    timeout: float


def _run_one(task: _Task):
    os.environ["LOG_MPC_PREDICTIONS"] = "1"   # gate the controller's npz dump
    res = launch_and_collect(
        experiment="drag_calib", variant="dob_off",
        controller_mode="standard", mpc_model="nn", nn_model=DEFAULT_NN_MODEL,
        terrain=task.terrain, path="sinusoidal", speed=task.speed,
        bumpiness=0, seed=task.seed, run_dir=Path(task.run_dir_str),
        sim_port=task.sim_port, ctrl_port=task.ctrl_port,
        sim_time=task.sim_time, timeout=task.timeout, rocks=0, lead_in=5.0,
        extra_args=["--dob-ki", "0.0", "--dob-max", "0.0"],
    )
    return task.terrain, task.speed, task.seed, res.status, res.mean_speed_mps, res.speed_ratio


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speeds", nargs="+", type=float, default=[5.0, 7.0])
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--time", type=float, default=18.0)
    ap.add_argument("--timeout", type=float, default=220.0)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--base-port", type=int, default=8400)
    args = ap.parse_args()

    out_dir = timestamped_result_dir("drag_calib")
    print(f"Output: {out_dir}")
    tasks, idx = [], 0
    for terr in TERRAINS:
        for sp in args.speeds:
            for si in range(args.seeds):
                port = args.base_port + 2 * idx
                rd = out_dir / "raw" / f"{idx:03d}_{terr}_v{sp:g}_s{si}"
                tasks.append(_Task(idx, terr, sp, 700 + si, str(rd),
                                   port, port + 1, args.time, args.timeout))
                idx += 1
    print(f"{len(tasks)} DOB-off runs (LOG_MPC_PREDICTIONS=1)")
    r0 = _run_one(tasks[0])
    print("  warmup:", r0)
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = {ex.submit(_run_one, t): t for t in tasks[1:]}
        for fut in as_completed(futs):
            print("  ", fut.result())
    print(f"DRAG_CALIB_LOGS_DONE {out_dir}")


if __name__ == "__main__":
    main()
