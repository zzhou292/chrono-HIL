#!/usr/bin/env python3
"""Paper experiment: MPPI seed-trajectory ablation.

One thing tested: how much of the MPPI shield's collision-rate improvement
comes from the hand-crafted seed trajectories (passthrough, brake, evade
left/right, brake-while-turn) versus pure Gaussian sampling around the
operator command. Sensor noise is enabled in every run.

This sweep runs the MPPI shield on the same obstacle scenarios as
``safety_filter_sweep.py`` (planner-blind MPC so the shield is the sole
collision avoider), once with seeds enabled and once with
``--mppi-no-seeds`` disabling them.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (  # noqa: E402
    BUMPS,
    DEFAULT_NN_MODEL,
    PATHS,
    SPEEDS,
    TERRAINS,
    RunResult,
    launch_and_collect,
    plot_metric_distribution_grid,
    plot_trajectory_overlays,
    save_summary_markdown,
    summarize_by_variant,
    timestamped_result_dir,
    write_manifest,
    write_results_csv,
)


VARIANTS = {
    "mppi_with_seeds": dict(
        extra=[],
        note="Default MPPI shield with all hand-crafted seed trajectories enabled.",
    ),
    "mppi_no_seeds": dict(
        extra=["--mppi-no-seeds"],
        note="MPPI shield with seed trajectories disabled; pure Gaussian sampling around operator command.",
    ),
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--variants", nargs="+", default=list(VARIANTS),
                   choices=list(VARIANTS))
    p.add_argument("--terrains", nargs="+", default=list(TERRAINS), choices=list(TERRAINS))
    p.add_argument("--paths", nargs="+", default=list(PATHS))
    p.add_argument("--speeds", nargs="+", type=float, default=list(SPEEDS))
    p.add_argument("--bumpiness", nargs="+", type=int, default=list(BUMPS))
    p.add_argument("--seeds", type=int, default=5)
    p.add_argument("--base-seed", type=int, default=850)
    p.add_argument("--time", type=float, default=12.0)
    p.add_argument("--lead-in", type=float, default=5.0)
    p.add_argument("--rocks", type=int, default=5)
    p.add_argument("--shield-horizon", type=int, default=18)
    p.add_argument("--mppi-samples", type=int, default=384)
    p.add_argument("--safety-buffer", type=float, default=0.50)
    p.add_argument("--timeout", type=float, default=200.0)
    p.add_argument("--base-port", type=int, default=8500)
    p.add_argument("--workers", type=int, default=6,
                   help="Parallel worker processes. Worker 0 runs solo first "
                        "to warm the acados/CasADi codegen cache.")
    p.add_argument("--quick", action="store_true",
                   help="Tiny smoke matrix: clay/sinusoidal/v5/b0, one seed.")
    return p.parse_args()


def shield_args(variant: str, args: argparse.Namespace) -> list[str]:
    extra = [
        "--safety-filter", "--safety-flavor", "mppi",
        "--shield-horizon", str(args.shield_horizon),
        "--safety-buffer", str(args.safety_buffer),
        "--mppi-samples", str(args.mppi_samples),
        "--mpc-blind-obstacles",
    ]
    extra += VARIANTS[variant]["extra"]
    return extra


@dataclass(frozen=True)
class _Task:
    """Pickle-friendly description of one closed-loop run."""
    idx: int
    variant: str
    extra: tuple[str, ...]
    note: str
    terrain: str
    path: str
    speed: float
    bumpiness: int
    seed: int
    rocks: int
    run_dir_str: str
    sim_port: int
    ctrl_port: int
    sim_time: float
    timeout: float
    lead_in: float


def _run_one(task: _Task) -> RunResult:
    """ProcessPool worker. Each call runs one launch_decoupled closed-loop."""
    os.environ.setdefault("ACADOS_UNIQUE_BUILD_DIR", "1")
    res = launch_and_collect(
        experiment="mppi_seed_ablation",
        variant=task.variant,
        controller_mode="standard",
        mpc_model="nn",
        nn_model=DEFAULT_NN_MODEL,
        terrain=task.terrain, path=task.path,
        speed=task.speed, bumpiness=task.bumpiness, seed=task.seed,
        run_dir=Path(task.run_dir_str),
        sim_port=task.sim_port, ctrl_port=task.ctrl_port,
        sim_time=task.sim_time, timeout=task.timeout,
        rocks=task.rocks, lead_in=task.lead_in,
        extra_args=list(task.extra),
    )
    res.notes = task.note
    return res


def plot_figures(results_csv: Path, out_dir: Path) -> None:
    df = pd.read_csv(results_csv)
    ok = df[df["status"] == "ok"].copy()
    if ok.empty:
        return
    fig_dir = out_dir / "figures"

    summary = ok.groupby("variant", sort=False).agg(
        collisions=("collisions", "mean"),
        collisions_std=("collisions", "std"),
        near=("near_misses", "mean"),
        clearance=("min_clearance_m", "mean"),
        intervention=("intervention_rate_pct", "mean"),
        rms_cte=("rms_cte_m", "mean"),
    ).reset_index()

    fig, axes = plt.subplots(2, 2, figsize=(12, 7.6))
    specs = [
        ("collisions", "collisions_std", "Unique obstacles hit", "Collisions"),
        ("clearance", None, "Minimum clearance (m)", "Clearance"),
        ("intervention", None, "Intervention rate (%)", "Invasiveness"),
        ("rms_cte", None, "RMS CTE (m)", "Tracking error"),
    ]
    x = np.arange(len(summary))
    for ax, (mean_key, std_key, ylabel, title) in zip(axes.flat, specs):
        err = summary[std_key].fillna(0.0) if std_key else None
        ax.bar(x, summary[mean_key], yerr=err, capsize=3)
        ax.set_xticks(x)
        ax.set_xticklabels(summary["variant"], rotation=15, ha="right")
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.3)
    fig.suptitle("MPPI seed-trajectory ablation")
    fig.tight_layout()
    fig.savefig(fig_dir / "mppi_seed_ablation_summary.png", dpi=220)
    plt.close(fig)

    ok["scenario"] = (
        ok["terrain"] + "/" + ok["path"] + "/v" + ok["speed_mps"].astype(str)
        + "/b" + ok["bumpiness"].astype(str)
    )
    pivot = ok.pivot_table(index="scenario", columns="variant",
                           values="collisions", aggfunc="mean")
    fig, ax = plt.subplots(figsize=(1.4 * len(pivot.columns) + 4,
                                    0.42 * len(pivot.index) + 2.5))
    im = ax.imshow(pivot.values, aspect="auto", cmap="RdYlGn_r")
    ax.set_xticks(range(len(pivot.columns)))
    ax.set_xticklabels(pivot.columns, rotation=25, ha="right")
    ax.set_yticks(range(len(pivot.index)))
    ax.set_yticklabels(pivot.index, fontsize=8)
    for i in range(pivot.shape[0]):
        for j in range(pivot.shape[1]):
            v = pivot.values[i, j]
            if math.isfinite(v):
                ax.text(j, i, f"{v:.1f}", ha="center", va="center", fontsize=8)
    ax.set_title("Mean unique obstacles hit by scenario (lower is better)")
    fig.colorbar(im, ax=ax, fraction=0.035)
    fig.tight_layout()
    fig.savefig(fig_dir / "mppi_seed_ablation_collision_heatmap.png", dpi=220)
    plt.close(fig)

    plot_metric_distribution_grid(
        results_csv,
        out_dir,
        [
            ("collisions", "Unique obstacles hit", "Collisions"),
            ("min_clearance_m", "Minimum clearance (m)", "Clearance"),
            ("intervention_rate_pct", "Intervention rate (%)", "Invasiveness"),
            ("rms_cte_m", "RMS CTE (m)", "Tracking error"),
        ],
        "mppi_seed_ablation_metric_distributions.png",
        "MPPI seed-trajectory ablation",
    )
    plot_trajectory_overlays(
        results_csv,
        out_dir,
        filename_prefix="mppi_seed_ablation_trajectory_overlay",
        max_scenarios=4,
    )


def main() -> None:
    args = parse_args()
    if args.quick:
        args.terrains = ["clay"]
        args.paths = ["sinusoidal"]
        args.speeds = [5.0]
        args.bumpiness = [0]
        args.seeds = 1
        args.time = min(args.time, 8.0)

    out_dir = timestamped_result_dir("mppi_seed_ablation")
    write_manifest(out_dir, args,
                   "MPPI seed-trajectory ablation with sensor noise enabled.")
    print(f"Output: {out_dir}")

    tasks: list[_Task] = []
    idx = 0
    for variant in args.variants:
        for terrain in args.terrains:
            for path in args.paths:
                for speed in args.speeds:
                    for bump in args.bumpiness:
                        for seed_i in range(args.seeds):
                            seed = args.base_seed + seed_i
                            sim_port = args.base_port + 2 * idx
                            ctrl_port = sim_port + 1
                            run_dir = out_dir / "raw" / (
                                f"{idx:04d}_{variant}_{terrain}_{path}_v{speed:g}_b{bump}_s{seed}"
                            )
                            tasks.append(_Task(
                                idx=idx, variant=variant,
                                extra=tuple(shield_args(variant, args)),
                                note=VARIANTS[variant]["note"],
                                terrain=terrain, path=path, speed=speed,
                                bumpiness=bump, seed=seed,
                                rocks=args.rocks,
                                run_dir_str=str(run_dir),
                                sim_port=sim_port, ctrl_port=ctrl_port,
                                sim_time=args.time, timeout=args.timeout,
                                lead_in=args.lead_in,
                            ))
                            idx += 1

    total = len(tasks)
    results: list[RunResult] = []

    print(f"[1/{total}] (warmup) {tasks[0].variant} {tasks[0].terrain}/{tasks[0].path} "
          f"v={tasks[0].speed:g} b={tasks[0].bumpiness} seed={tasks[0].seed}")
    first = _run_one(tasks[0])
    results.append(first)
    write_results_csv(out_dir / "results.csv", results)
    print(f"    {first.status}: collisions={first.collisions} "
          f"near={first.near_misses} clearance={first.min_clearance_m}")

    if len(tasks) > 1:
        completed = 1
        with ProcessPoolExecutor(max_workers=max(1, args.workers)) as ex:
            futs = {ex.submit(_run_one, t): t for t in tasks[1:]}
            for fut in as_completed(futs):
                t = futs[fut]
                res = fut.result()
                results.append(res)
                completed += 1
                write_results_csv(out_dir / "results.csv", results)
                print(f"[{completed}/{total}] {t.variant} {t.terrain}/{t.path} "
                      f"v={t.speed:g} b={t.bumpiness} seed={t.seed}")
                print(f"    {res.status}: collisions={res.collisions} "
                      f"near={res.near_misses} clearance={res.min_clearance_m}")

    write_results_csv(out_dir / "results.csv", results)
    summary = summarize_by_variant(
        results,
        ["collisions", "near_misses", "min_clearance_m", "intervention_rate_pct",
         "mean_abs_dsteer", "mean_abs_dthrottle", "rms_cte_m"],
    )
    summary.to_csv(out_dir / "summary_by_variant.csv", index=False)
    save_summary_markdown(
        out_dir,
        "MPPI Seed-Trajectory Ablation",
        summary,
        [
            "Noise policy: sensor noise enabled in every run.",
            f"Safety buffer: {args.safety_buffer:g} m beyond the hard collision footprint.",
            "Planner-blind MPC: shield is the sole collision avoider.",
            "Variant notes: " + "; ".join(f"{k}: {VARIANTS[k]['note']}" for k in args.variants),
        ],
    )
    plot_figures(out_dir / "results.csv", out_dir)
    print(f"Done: {out_dir}")


if __name__ == "__main__":
    main()
