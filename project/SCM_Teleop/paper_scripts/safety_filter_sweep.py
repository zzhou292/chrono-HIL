#!/usr/bin/env python3
"""Paper experiment: safety-filter obstacle avoidance comparison.

One thing tested: obstacle avoidance performance of no filter, DOB-CBF,
MPPI, and gradient NMPC safety filters. The optional blind-MPC mode makes
the shield the sole avoider. Sensor noise is enabled in every run.
"""

from __future__ import annotations

import argparse
import math
import sys
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


FLAVORS = ("none", "dob_cbf", "mppi", "nmpc")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--flavors", nargs="+", default=list(FLAVORS), choices=list(FLAVORS))
    p.add_argument("--terrains", nargs="+", default=list(TERRAINS), choices=list(TERRAINS))
    p.add_argument("--paths", nargs="+", default=list(PATHS))
    p.add_argument("--speeds", nargs="+", type=float, default=list(SPEEDS))
    p.add_argument("--bumpiness", nargs="+", type=int, default=list(BUMPS))
    p.add_argument("--seeds", type=int, default=5)
    p.add_argument("--base-seed", type=int, default=45)
    p.add_argument("--time", type=float, default=12.0)
    p.add_argument("--lead-in", type=float, default=5.0)
    p.add_argument("--rocks", type=int, default=5)
    p.add_argument("--blind-and-aware", action="store_true",
                   help="Run both planner-aware and planner-blind obstacle cases.")
    p.add_argument("--shield-horizon", type=int, default=18)
    p.add_argument("--mppi-samples", type=int, default=384)
    p.add_argument("--nmpc-iter", type=int, default=6)
    p.add_argument("--safety-buffer", type=float, default=0.50,
                   help="Extra obstacle clearance buffer passed to safety filters. "
                        "Default 0.50 m gives the predictive shields a practical "
                        "guard band beyond the collision logger's hard footprint.")
    p.add_argument("--timeout", type=float, default=200.0)
    p.add_argument("--base-port", type=int, default=7800)
    p.add_argument("--quick", action="store_true",
                   help="Tiny smoke matrix: clay/sinusoidal/v5/b0, one seed, blind mode.")
    p.add_argument("--output-suffix", type=str, default="",
                   help="Tag appended to the results-folder prefix so multiple invocations "
                        "(e.g. planner-blind vs planner-aware) do not collide.")
    return p.parse_args()


def extra_args_for(flavor: str, blind: bool, args: argparse.Namespace) -> list[str]:
    extra: list[str] = []
    if flavor != "none":
        extra += [
            "--safety-filter", "--safety-flavor", flavor,
            "--shield-horizon", str(args.shield_horizon),
            "--safety-buffer", str(args.safety_buffer),
        ]
        if flavor == "mppi":
            extra += ["--mppi-samples", str(args.mppi_samples)]
        if flavor == "nmpc":
            extra += ["--nmpc-iter", str(args.nmpc_iter)]
    if blind:
        extra.append("--mpc-blind-obstacles")
    return extra


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
        dsteer=("mean_abs_dsteer", "mean"),
        dthr=("mean_abs_dthrottle", "mean"),
        rt=("rt_factor", "mean"),
    ).reset_index()

    x = np.arange(len(summary))
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    specs = [
        ("collisions", "collisions_std", "Unique obstacles hit", "Collisions"),
        ("clearance", None, "Minimum clearance (m)", "Clearance"),
        ("intervention", None, "Intervention rate (%)", "Invasiveness"),
        ("rt", None, "RT factor", "Runtime"),
    ]
    for ax, (mean_key, std_key, ylabel, title) in zip(axes.flat, specs):
        err = summary[std_key].fillna(0.0) if std_key else None
        ax.bar(x, summary[mean_key], yerr=err, capsize=3)
        ax.set_xticks(x)
        ax.set_xticklabels(summary["variant"], rotation=25, ha="right")
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(fig_dir / "safety_filter_summary.png", dpi=220)
    plt.close(fig)

    ok["scenario"] = ok["terrain"] + "/" + ok["path"] + "/v" + ok["speed_mps"].astype(str)
    pivot = ok.pivot_table(index="scenario", columns="variant", values="collisions", aggfunc="mean")
    fig, ax = plt.subplots(figsize=(1.45 * len(pivot.columns) + 4, 0.48 * len(pivot.index) + 2.5))
    im = ax.imshow(pivot.values, aspect="auto", cmap="RdYlGn_r")
    ax.set_xticks(range(len(pivot.columns)))
    ax.set_xticklabels(pivot.columns, rotation=30, ha="right")
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
    fig.savefig(fig_dir / "safety_collision_heatmap.png", dpi=220)
    plt.close(fig)

    plot_metric_distribution_grid(
        results_csv,
        out_dir,
        [
            ("collisions", "Unique obstacles hit", "Collisions"),
            ("min_clearance_m", "Minimum clearance (m)", "Clearance"),
            ("intervention_rate_pct", "Intervention rate (%)", "Invasiveness"),
            ("rms_cte_m", "RMS CTE (m)", "Tracking error"),
            ("rt_factor", "RT factor", "Runtime"),
        ],
        "safety_filter_metric_distributions.png",
        "Safety filter obstacle-avoidance sweep",
    )
    plot_trajectory_overlays(
        results_csv,
        out_dir,
        filename_prefix="safety_filter_trajectory_overlay",
        max_scenarios=4,
    )


def main() -> None:
    args = parse_args()
    if args.quick:
        args.flavors = list(FLAVORS)
        args.terrains = ["clay"]
        args.paths = ["sinusoidal"]
        args.speeds = [5.0]
        args.bumpiness = [0]
        args.seeds = 1
        args.time = min(args.time, 8.0)
        args.blind_and_aware = False

    prefix = "safety_filter_sweep"
    if args.output_suffix:
        prefix = f"{prefix}_{args.output_suffix}"
    out_dir = timestamped_result_dir(prefix)
    write_manifest(out_dir, args, "Safety filter obstacle-avoidance sweep with sensor noise enabled.")
    print(f"Output: {out_dir}")

    awareness_modes = [True, False] if args.blind_and_aware else [True]
    results: list[RunResult] = []
    idx = 0
    total = (len(args.flavors) * len(awareness_modes) * len(args.terrains)
             * len(args.paths) * len(args.speeds) * len(args.bumpiness) * args.seeds)
    for flavor in args.flavors:
        for blind in awareness_modes:
            variant = f"{flavor}_{'blind' if blind else 'aware'}"
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
                                idx += 1
                                print(f"[{idx}/{total}] {variant} {terrain}/{path} v={speed:g} b={bump} seed={seed}")
                                res = launch_and_collect(
                                    experiment="safety_filter_sweep",
                                    variant=variant,
                                    controller_mode="standard",
                                    mpc_model="nn",
                                    nn_model=DEFAULT_NN_MODEL,
                                    terrain=terrain,
                                    path=path,
                                    speed=speed,
                                    bumpiness=bump,
                                    seed=seed,
                                    run_dir=run_dir,
                                    sim_port=sim_port,
                                    ctrl_port=ctrl_port,
                                    sim_time=args.time,
                                    timeout=args.timeout,
                                    rocks=args.rocks,
                                    lead_in=args.lead_in,
                                    extra_args=extra_args_for(flavor, blind, args),
                                )
                                results.append(res)
                                write_results_csv(out_dir / "results.csv", results)
                                print(f"    {res.status}: collisions={res.collisions} near={res.near_misses} "
                                      f"interv={res.intervention_rate_pct:.1f}%")

    write_results_csv(out_dir / "results.csv", results)
    summary = summarize_by_variant(
        results,
        ["collisions", "near_misses", "min_clearance_m", "intervention_rate_pct",
         "mean_abs_dsteer", "mean_abs_dthrottle", "rt_factor", "rms_cte_m"],
    )
    summary.to_csv(out_dir / "summary_by_filter.csv", index=False)
    save_summary_markdown(
        out_dir,
        "Safety Filter Sweep",
        summary,
        [
            "Noise policy: sensor noise enabled in every run.",
            f"Safety buffer: {args.safety_buffer:g} m beyond the hard collision footprint.",
            "NMPC diagnostic angle: if NMPC loses to DOB-CBF, inspect raw `nmpc_shield_log.csv` and trajectories for local-minimum behavior, short-horizon braking, or insufficient geometric steering commitment. DOB-CBF has an explicit reactive steering layer that can win on head-on rocks even without long-horizon optimization.",
        ],
    )
    plot_figures(out_dir / "results.csv", out_dir)
    print(f"Done: {out_dir}")


if __name__ == "__main__":
    main()
