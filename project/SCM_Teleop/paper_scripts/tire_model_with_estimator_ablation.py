#!/usr/bin/env python3
"""Paper experiment: live terrain estimator on/off for tire-model comparison.

The abstract claims the NN-NMPC stack with the online terrain estimator
"improves closed-loop tracking by an order of magnitude over Pacejka and
TMeasy."  The default ``mpc_tire_model_sweep`` runs every model with
static terrain parameters, so it cannot speak to that claim.  This sweep
re-runs the same tire models with ``--terrain-estimator`` enabled so the
live-conditioned advantage (or its absence) is measurable.

Sensor noise is enabled in every run.
"""

from __future__ import annotations

import argparse
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
    save_summary_markdown,
    summarize_by_variant,
    timestamped_result_dir,
    write_manifest,
    write_results_csv,
)


# Subset of tire models actually wired into the orchestrator's pilot tier.
# ``estimator`` is True when this row should carry --terrain-estimator.
VARIANTS = {
    "pacejka_static":           dict(mpc="pacejka", nn=DEFAULT_NN_MODEL, estimator=False),
    "tmeasy_static":            dict(mpc="tmeasy",  nn=DEFAULT_NN_MODEL, estimator=False),
    "nn_v3_static":             dict(mpc="nn", nn="closed_loop_v3_axle_rate_64_32", estimator=False),
    "nn_v3_estimator":          dict(mpc="nn", nn="closed_loop_v3_axle_rate_64_32", estimator=True),
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
    p.add_argument("--base-seed", type=int, default=400)
    p.add_argument("--time", type=float, default=20.0,
                   help="Longer than the static sweep so the estimator has time to converge.")
    p.add_argument("--lead-in", type=float, default=5.0)
    p.add_argument("--metric-start", type=float, default=8.0,
                   help="Start KPI window after the estimator has had time to settle.")
    p.add_argument("--timeout", type=float, default=240.0)
    p.add_argument("--base-port", type=int, default=9000)
    p.add_argument("--quick", action="store_true")
    return p.parse_args()


def plot_figures(results_csv: Path, out_dir: Path) -> None:
    df = pd.read_csv(results_csv)
    ok = df[df["status"] == "ok"].copy()
    if ok.empty:
        return
    fig_dir = out_dir / "figures"

    summary = ok.groupby("variant", sort=False).agg(
        rms_cte=("rms_cte_m", "mean"),
        rms_cte_std=("rms_cte_m", "std"),
        speed_ratio=("speed_ratio", "mean"),
        speed_ratio_std=("speed_ratio", "std"),
        solve_ms=("mean_solve_ms", "mean"),
    ).reset_index()
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.2))
    x = np.arange(len(summary))
    for ax, mean_key, std_key, ylabel in [
        (axes[0], "rms_cte", "rms_cte_std", "RMS CTE (m)"),
        (axes[1], "speed_ratio", "speed_ratio_std", "mean speed / target"),
        (axes[2], "solve_ms", None, "Mean solve time (ms)"),
    ]:
        err = summary[std_key].fillna(0.0) if std_key else None
        ax.bar(x, summary[mean_key], yerr=err, capsize=3)
        ax.set_xticks(x)
        ax.set_xticklabels(summary["variant"], rotation=20, ha="right")
        ax.set_ylabel(ylabel)
        ax.grid(axis="y", alpha=0.3)
    fig.suptitle("Tire model x live terrain estimator")
    fig.tight_layout()
    fig.savefig(fig_dir / "tire_estimator_summary.png", dpi=220)
    plt.close(fig)

    pivot = ok.pivot_table(index=["terrain","speed_mps"], columns="variant",
                           values="rms_cte_m", aggfunc="mean")
    fig, ax = plt.subplots(figsize=(1.5 * len(pivot.columns) + 4,
                                    0.5 * len(pivot.index) + 3))
    im = ax.imshow(pivot.values, aspect="auto", cmap="viridis_r")
    ax.set_xticks(range(len(pivot.columns)))
    ax.set_xticklabels(pivot.columns, rotation=25, ha="right")
    ax.set_yticks(range(len(pivot.index)))
    ax.set_yticklabels([f"{t}/v{v}" for t,v in pivot.index], fontsize=8)
    for i in range(pivot.shape[0]):
        for j in range(pivot.shape[1]):
            v = pivot.values[i, j]
            if np.isfinite(v):
                ax.text(j, i, f"{v:.2f}", ha="center", va="center", fontsize=8,
                        color="white" if v > pivot.values.mean() else "black")
    ax.set_title("RMS CTE (m) by (terrain, speed)")
    fig.colorbar(im, ax=ax, fraction=0.035)
    fig.tight_layout()
    fig.savefig(fig_dir / "tire_estimator_rms_cte_heatmap.png", dpi=220)
    plt.close(fig)

    plot_metric_distribution_grid(
        results_csv,
        out_dir,
        [
            ("rms_cte_m", "RMS CTE (m)", "Tracking error"),
            ("speed_ratio", "mean speed / target", "Speed retention"),
            ("mean_solve_ms", "Mean solve (ms)", "Runtime"),
        ],
        "tire_estimator_metric_distributions.png",
        "Tire model x live terrain estimator",
    )


def main() -> None:
    args = parse_args()
    if args.quick:
        args.terrains = ["clay"]
        args.paths = ["sinusoidal"]
        args.speeds = [5.0]
        args.bumpiness = [0]
        args.seeds = 1
        args.time = min(args.time, 12.0)

    out_dir = timestamped_result_dir("tire_model_with_estimator_ablation")
    write_manifest(out_dir, args,
                   "Tire model sweep with the live terrain estimator on, "
                   "to test the order-of-magnitude tracking claim.")
    print(f"Output: {out_dir}")

    results: list[RunResult] = []
    idx = 0
    total = (len(args.variants) * len(args.terrains) * len(args.paths)
             * len(args.speeds) * len(args.bumpiness) * args.seeds)
    for variant in args.variants:
        spec = VARIANTS[variant]
        extra = []
        if spec["estimator"]:
            extra.append("--terrain-estimator")
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
                            print(f"[{idx}/{total}] {variant} {terrain}/{path} "
                                  f"v={speed:g} b={bump} seed={seed}")
                            res = launch_and_collect(
                                experiment="tire_model_with_estimator_ablation",
                                variant=variant,
                                controller_mode="standard",
                                mpc_model=spec["mpc"],
                                nn_model=spec["nn"],
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
                                rocks=0,
                                lead_in=args.lead_in,
                                extra_args=extra,
                                metric_start=args.metric_start,
                            )
                            results.append(res)
                            write_results_csv(out_dir / "results.csv", results)
                            print(f"    {res.status}: rms_cte={res.rms_cte_m:.3f} "
                                  f"speed_ratio={res.speed_ratio:.2f}")

    write_results_csv(out_dir / "results.csv", results)
    summary = summarize_by_variant(
        results,
        ["rms_cte_m","speed_ratio","mean_speed_mps","mean_solve_ms"],
    )
    summary.to_csv(out_dir / "summary_by_variant.csv", index=False)
    save_summary_markdown(
        out_dir,
        "Tire model x live terrain estimator",
        summary,
        [
            "Noise policy: sensor noise enabled in every run.",
            "Estimator-off variants use static terrain params; estimator-on adds "
            "--terrain-estimator so n_terrain is re-conditioned online from IMU + "
            "wheel-speed signals.",
            "Metric window starts after the estimator has had time to settle.",
        ],
    )
    plot_figures(out_dir / "results.csv", out_dir)
    print(f"Done: {out_dir}")


if __name__ == "__main__":
    main()
