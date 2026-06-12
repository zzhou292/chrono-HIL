#!/usr/bin/env python3
"""Can a calibrated feedforward sinkage-drag term replace the reactive throttle DOB?

The NMPC longitudinal channel is kinematic (u_dot = ax + du_dot_resid). The
feedforward drag sets du_dot_resid = -c_drag(n_hat), calibrated from DOB-off
rollout drift; the fit is non-monotonic and effectively sand-specific (only firm
soil over-predicts speed). Four variants, sensor noise on:

  dob          : default reactive throttle DOB (baseline).
  off          : no DOB, no drag (the raw deficit).
  ffdrag       : feedforward drag only, DOB off (the model-based replacement).
  ffdrag+dob   : both.

We report rms_cte, speed_ratio, mean_speed per variant x terrain so the sand
column (where drag is active) shows whether ffdrag recovers the DOB's speed.
"""
from __future__ import annotations
import argparse, os, sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (  # noqa: E402
    DEFAULT_NN_MODEL, launch_and_collect, summarize_by_variant,
    timestamped_result_dir, write_results_csv, RunResult,
)

VARIANTS = {
    "dob":        [],
    "off":        ["--dob-ki", "0.0", "--dob-max", "0.0"],
    "ffdrag":     ["--dob-ki", "0.0", "--dob-max", "0.0", "--ff-drag"],
    "ffdrag+dob": ["--ff-drag"],
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
    res = launch_and_collect(
        experiment="ff_drag_ablation", variant=task.variant,
        controller_mode="standard", mpc_model="nn", nn_model=DEFAULT_NN_MODEL,
        terrain=task.terrain, path="sinusoidal", speed=task.speed,
        bumpiness=0, seed=task.seed, run_dir=Path(task.run_dir_str),
        sim_port=task.sim_port, ctrl_port=task.ctrl_port,
        sim_time=task.sim_time, timeout=task.timeout, rocks=0, lead_in=5.0,
        extra_args=list(task.extra),
    )
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--terrains", nargs="+", default=["clay", "dirt", "sand"])
    ap.add_argument("--speeds", nargs="+", type=float, default=[5.0, 7.0])
    ap.add_argument("--seeds", type=int, default=2)
    ap.add_argument("--time", type=float, default=18.0)
    ap.add_argument("--timeout", type=float, default=220.0)
    ap.add_argument("--workers", type=int, default=8)
    ap.add_argument("--base-port", type=int, default=8600)
    args = ap.parse_args()

    out_dir = timestamped_result_dir("ff_drag_ablation")
    print(f"Output: {out_dir}")
    tasks, idx = [], 0
    for variant, extra in VARIANTS.items():
        for terr in args.terrains:
            for sp in args.speeds:
                for si in range(args.seeds):
                    port = args.base_port + 2 * idx
                    rd = out_dir / "raw" / f"{idx:03d}_{variant}_{terr}_v{sp:g}_s{si}"
                    tasks.append(_Task(idx, variant, tuple(extra), terr, sp,
                                       800 + si, str(rd), port, port + 1,
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
            r = fut.result(); results.append(r)
            write_results_csv(out_dir / "results.csv", results)

    write_results_csv(out_dir / "results.csv", results)
    summ = summarize_by_variant(results, ["rms_cte_m", "speed_ratio", "mean_speed_mps"])
    summ.to_csv(out_dir / "summary_by_variant.csv", index=False)
    # per terrain x variant
    import pandas as pd
    df = pd.read_csv(out_dir / "results.csv")
    ok = df[df["status"] == "ok"]
    piv_sr = ok.pivot_table(index="terrain", columns="variant", values="speed_ratio", aggfunc="mean")
    piv_cte = ok.pivot_table(index="terrain", columns="variant", values="rms_cte_m", aggfunc="mean")
    print("\n=== speed_ratio by terrain x variant ===")
    print(piv_sr.round(3).to_string())
    print("\n=== rms_cte (m) by terrain x variant ===")
    print(piv_cte.round(3).to_string())
    piv_sr.to_csv(out_dir / "speed_ratio_by_terrain.csv")
    piv_cte.to_csv(out_dir / "rms_cte_by_terrain.csv")
    print(f"\nFF_DRAG_ABLATION_DONE {out_dir}")


if __name__ == "__main__":
    main()
