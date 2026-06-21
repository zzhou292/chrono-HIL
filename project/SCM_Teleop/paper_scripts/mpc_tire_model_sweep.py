#!/usr/bin/env python3
"""Paper experiment: standard MPC tracking versus tire model choice.

One thing tested: how the tire model inside the standard acados MPC changes
closed-loop speed/tracking performance across paths, terrains, speeds, and
bumpiness. Sensor noise is ON because launch_decoupled.py defaults to noisy
measurements and this script never passes --no-noise.
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
    plot_force_prediction_figures,
    plot_metric_distribution_grid,
    plot_trajectory_overlays,
    save_summary_markdown,
    summarize_by_variant,
    timestamped_result_dir,
    write_manifest,
    write_results_csv,
)


# Only the tire models actually used by the paper sweeps are kept here.
# The deprecated paper_v1/paper_v2 rig series, factored ResNet, temporal,
# and small-MLP variants were archived 2026-05-16 to
# archive/2026-05-16_nn_models/ and are no longer selectable.
MODEL_SPECS = {
    "pacejka": dict(mpc_model="pacejka", nn_model=DEFAULT_NN_MODEL, extra=[]),
    "tmeasy": dict(mpc_model="tmeasy", nn_model=DEFAULT_NN_MODEL, extra=[]),
    "closed_loop_mlp": dict(mpc_model="nn", nn_model="closed_loop_v1_mlp_32_16", extra=[]),
    "closed_loop_v2_rate_mlp": dict(mpc_model="nn", nn_model="closed_loop_v2_both_axles_rate_32_16", extra=[]),
    "closed_loop_v3_axle_rate_mlp": dict(mpc_model="nn", nn_model="closed_loop_v3_axle_rate_64_32", extra=[]),
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--models", nargs="+", default=[
        "pacejka", "tmeasy", "closed_loop_v2_rate_mlp", "closed_loop_v3_axle_rate_mlp",
    ], choices=list(MODEL_SPECS))
    p.add_argument("--terrains", nargs="+", default=list(TERRAINS), choices=list(TERRAINS))
    p.add_argument("--paths", nargs="+", default=list(PATHS))
    p.add_argument("--speeds", nargs="+", type=float, default=list(SPEEDS))
    p.add_argument("--bumpiness", nargs="+", type=int, default=list(BUMPS))
    p.add_argument("--seeds", type=int, default=5)
    p.add_argument("--base-seed", type=int, default=100)
    p.add_argument("--time", type=float, default=15.0)
    p.add_argument("--lead-in", type=float, default=5.0)
    p.add_argument("--timeout", type=float, default=180.0)
    p.add_argument("--base-port", type=int, default=6200)
    p.add_argument("--quick", action="store_true",
                   help="Tiny smoke matrix: clay/sinusoidal, one speed, one bumpiness, one seed.")
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
        solve_ms_std=("mean_solve_ms", "std"),
    ).reset_index()

    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2))
    specs = [
        ("rms_cte", "rms_cte_std", "RMS CTE (m)", "Tracking Error"),
        ("speed_ratio", "speed_ratio_std", "mean speed / target", "Speed Retention"),
        ("solve_ms", "solve_ms_std", "Mean solve time (ms)", "Runtime"),
    ]
    x = np.arange(len(summary))
    for ax, (mean_key, std_key, ylabel, title) in zip(axes, specs):
        ax.bar(x, summary[mean_key], yerr=summary[std_key].fillna(0.0), capsize=3)
        ax.set_xticks(x)
        ax.set_xticklabels(summary["variant"], rotation=25, ha="right")
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(fig_dir / "tire_model_summary.png", dpi=220)
    plt.close(fig)

    ok["scenario"] = (
        ok["terrain"] + "/" + ok["path"] + "/v" + ok["speed_mps"].astype(str)
        + "/b" + ok["bumpiness"].astype(str)
    )
    pivot = ok.pivot_table(index="scenario", columns="variant", values="rms_cte_m", aggfunc="mean")
    fig, ax = plt.subplots(figsize=(1.5 * len(pivot.columns) + 4, 0.42 * len(pivot.index) + 2.5))
    im = ax.imshow(pivot.values, aspect="auto", cmap="viridis_r")
    ax.set_xticks(range(len(pivot.columns)))
    ax.set_xticklabels(pivot.columns, rotation=30, ha="right")
    ax.set_yticks(range(len(pivot.index)))
    ax.set_yticklabels(pivot.index, fontsize=8)
    for i in range(pivot.shape[0]):
        for j in range(pivot.shape[1]):
            v = pivot.values[i, j]
            if math.isfinite(v):
                ax.text(j, i, f"{v:.2f}", ha="center", va="center", fontsize=7, color="white")
    ax.set_title("RMS CTE by tire model and scenario (m; lower is better)")
    fig.colorbar(im, ax=ax, fraction=0.035)
    fig.tight_layout()
    fig.savefig(fig_dir / "tire_model_rms_cte_heatmap.png", dpi=220)
    plt.close(fig)

    plot_metric_distribution_grid(
        results_csv,
        out_dir,
        [
            ("rms_cte_m", "RMS CTE (m)", "Tracking error"),
            ("speed_ratio", "mean speed / target", "Speed retention"),
            ("mean_solve_ms", "Mean solve time (ms)", "Runtime"),
            ("progress_m", "Progress (m)", "Distance traveled"),
        ],
        "tire_model_metric_distributions.png",
        "Standard MPC tire-model sweep",
    )
    plot_trajectory_overlays(
        results_csv,
        out_dir,
        filename_prefix="tire_model_trajectory_overlay",
        max_scenarios=4,
    )
    plot_force_prediction_figures(results_csv, out_dir)


def main() -> None:
    args = parse_args()
    if args.quick:
        args.models = ["pacejka", "tmeasy", "closed_loop_mlp"]
        args.terrains = ["clay"]
        args.paths = ["sinusoidal"]
        args.speeds = [5.0]
        args.bumpiness = [0]
        args.seeds = 1
        args.time = min(args.time, 8.0)

    out_dir = timestamped_result_dir("mpc_tire_model_sweep")
    write_manifest(out_dir, args, "Standard MPC tire-model sweep with sensor noise enabled.")
    print(f"Output: {out_dir}")

    results: list[RunResult] = []
    idx = 0
    total = (len(args.models) * len(args.terrains) * len(args.paths)
             * len(args.speeds) * len(args.bumpiness) * args.seeds)
    for model_key in args.models:
        spec = MODEL_SPECS[model_key]
        for terrain in args.terrains:
            for path in args.paths:
                for speed in args.speeds:
                    for bump in args.bumpiness:
                        for seed_i in range(args.seeds):
                            seed = args.base_seed + seed_i
                            sim_port = args.base_port + 2 * idx
                            ctrl_port = sim_port + 1
                            run_dir = out_dir / "raw" / (
                                f"{idx:04d}_{model_key}_{terrain}_{path}_v{speed:g}_b{bump}_s{seed}"
                            )
                            idx += 1
                            print(f"[{idx}/{total}] {model_key} {terrain}/{path} v={speed:g} b={bump} seed={seed}")
                            res = launch_and_collect(
                                experiment="mpc_tire_model_sweep",
                                variant=model_key,
                                controller_mode="standard",
                                mpc_model=spec["mpc_model"],
                                nn_model=spec["nn_model"],
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
                                extra_args=spec["extra"],
                            )
                            results.append(res)
                            write_results_csv(out_dir / "results.csv", results)
                            print(f"    {res.status}: rms_cte={res.rms_cte_m:.3f} speed_ratio={res.speed_ratio:.2f}")

    write_results_csv(out_dir / "results.csv", results)
    summary = summarize_by_variant(
        results,
        ["rms_cte_m", "mean_abs_cte_m", "speed_ratio", "mean_speed_mps", "mean_solve_ms", "progress_m"],
    )
    summary.to_csv(out_dir / "summary_by_model.csv", index=False)
    save_summary_markdown(
        out_dir,
        "Standard MPC Tire-Model Sweep",
        summary,
        [
            "Noise policy: sensor noise enabled in every run.",
            "Raw per-run logs and diagnostic CSV files are under `raw/`.",
        ],
    )
    plot_figures(out_dir / "results.csv", out_dir)
    print(f"Done: {out_dir}")


if __name__ == "__main__":
    main()
