#!/usr/bin/env python3
"""Can a calibrated feedforward throttle map REPLACE the integral throttle DOB?

The asymmetric velocity-error DOB converges to a per-terrain throttle offset
(throttle - a_x/a_x_max): clay/dirt ~+0.24, sand ~+0.07. That offset is what the
naive linear actuation map omits on deformable soil, and it is smooth in the
sinkage exponent n -- so it can be applied as a FEEDFORWARD d_ff(n_hat) with no
integral. Variants (sensor noise on):

  dob     : reactive integral throttle DOB (baseline).
  off     : no DOB, no feedforward (raw deficit).
  ffwd    : feedforward throttle map, DOB off (--ff-throttle --dob-ki 0) -- the
            reactive-DOB replacement.
  ffwd+dob: feedforward bulk + small integral residual.

If ffwd matches dob on speed_ratio without inflating rms_cte, the DOB can be
replaced by a static terrain-aware actuation map.
"""
from __future__ import annotations
import argparse, sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (  # noqa: E402
    DEFAULT_NN_MODEL, launch_and_collect, summarize_by_variant,
    timestamped_result_dir, write_results_csv, RunResult,
)

VARIANTS = {
    "dob":      [],
    "off":      ["--dob-ki", "0.0", "--dob-max", "0.0"],
    "ffwd":     ["--dob-ki", "0.0", "--dob-max", "0.0", "--ff-throttle"],
    "ffwd+dob": ["--ff-throttle"],
}


@dataclass(frozen=True)
class _Task:
    idx: int
    variant: str
    extra: tuple
    terrain: str
    speed: float
    seed: int
    run_dir_str: str
    sim_port: int
    ctrl_port: int
    sim_time: float
    timeout: float


def _run_one(task: _Task) -> RunResult:
    return launch_and_collect(
        experiment="ff_throttle_ablation", variant=task.variant,
        controller_mode="standard", mpc_model="nn", nn_model=DEFAULT_NN_MODEL,
        terrain=task.terrain, path="sinusoidal", speed=task.speed,
        bumpiness=0, seed=task.seed, run_dir=Path(task.run_dir_str),
        sim_port=task.sim_port, ctrl_port=task.ctrl_port,
        sim_time=task.sim_time, timeout=task.timeout, rocks=0, lead_in=5.0,
        extra_args=list(task.extra),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--terrains", nargs="+", default=["clay", "dirt", "sand"])
    ap.add_argument("--speeds", nargs="+", type=float, default=[5.0, 7.0])
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--time", type=float, default=18.0)
    ap.add_argument("--timeout", type=float, default=220.0)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--base-port", type=int, default=8800)
    args = ap.parse_args()

    out_dir = timestamped_result_dir("ff_throttle_ablation")
    print(f"Output: {out_dir}")
    tasks, idx = [], 0
    for variant, extra in VARIANTS.items():
        for terr in args.terrains:
            for sp in args.speeds:
                for si in range(args.seeds):
                    port = args.base_port + 2 * idx
                    rd = out_dir / "raw" / f"{idx:03d}_{variant}_{terr}_v{sp:g}_s{si}"
                    tasks.append(_Task(idx, variant, tuple(extra), terr, sp,
                                       900 + si, str(rd), port, port + 1,
                                       args.time, args.timeout))
                    idx += 1
    print(f"{len(tasks)} runs across {len(VARIANTS)} variants")

    results = [_run_one(tasks[0])]
    print(f"  warmup {tasks[0].variant}/{tasks[0].terrain}: "
          f"cte={results[0].rms_cte_m:.3f} sr={results[0].speed_ratio:.2f}")
    write_results_csv(out_dir / "results.csv", results)
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = {ex.submit(_run_one, t): t for t in tasks[1:]}
        for fut in as_completed(futs):
            results.append(fut.result())
            write_results_csv(out_dir / "results.csv", results)

    write_results_csv(out_dir / "results.csv", results)
    summarize_by_variant(results, ["rms_cte_m", "speed_ratio", "mean_speed_mps"]).to_csv(
        out_dir / "summary_by_variant.csv", index=False)
    import pandas as pd
    ok = pd.read_csv(out_dir / "results.csv")
    ok = ok[ok["status"] == "ok"]
    for metric in ["mean_speed_mps", "speed_ratio", "rms_cte_m"]:
        print(f"\n=== {metric} (terrain x variant) ===")
        p = ok.pivot_table(index="terrain", columns="variant", values=metric, aggfunc="mean")
        cols = [c for c in ["off", "ffwd", "dob", "ffwd+dob"] if c in p.columns]
        print(p[cols].round(3).to_string())
    p = ok.pivot_table(index="terrain", columns="variant", values="mean_speed_mps", aggfunc="mean")
    p.to_csv(out_dir / "mean_speed_by_terrain.csv")
    print(f"\nFF_THROTTLE_ABLATION_DONE {out_dir}")


if __name__ == "__main__":
    main()
