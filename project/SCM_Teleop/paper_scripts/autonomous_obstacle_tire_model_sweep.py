#!/usr/bin/env python3
"""Paper experiment: autonomous obstacle avoidance versus tire model.

One thing tested: how tire-model choice changes autonomous obstacle avoidance,
tracking, speed retention, and runtime.  By default this uses only the standard
MPC's in-horizon obstacle barrier; an optional fixed downstream safety filter
can be enabled for the whole sweep when evaluating the full autonomy stack.
Sensor noise is on.
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
from mpc_tire_model_sweep import MODEL_SPECS  # noqa: E402


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--models", nargs="+", default=[
        "pacejka", "tmeasy", "closed_loop_mlp", "rate_mlp", "resnet",
    ], choices=list(MODEL_SPECS))
    p.add_argument("--terrains", nargs="+", default=list(TERRAINS), choices=list(TERRAINS))
    p.add_argument("--paths", nargs="+", default=list(PATHS))
    p.add_argument("--speeds", nargs="+", type=float, default=list(SPEEDS))
    p.add_argument("--bumpiness", nargs="+", type=int, default=list(BUMPS))
    p.add_argument("--seeds", type=int, default=5)
    p.add_argument("--base-seed", type=int, default=510)
    p.add_argument("--time", type=float, default=12.0)
    p.add_argument("--lead-in", type=float, default=5.0)
    p.add_argument("--rocks", type=int, default=5)
    p.add_argument("--speed-weight", type=float, default=15.0,
                   help="Use the softened standard-MPC speed cost by default.")
    p.add_argument("--speed-cost-mode", choices=["symmetric", "overspeed"], default="symmetric",
                   help="Use 'overspeed' to treat v_ref as a cap instead of a command.")
    p.add_argument("--obstacle-weight", type=float, default=5e3,
                   help="Standard-MPC soft obstacle-barrier weight.")
    p.add_argument("--safety-flavor", choices=["none", "dob_cbf", "mppi", "nmpc"], default="none",
                   help="Optional fixed downstream safety filter used for every tire model.")
    p.add_argument("--shield-horizon", type=int, default=18)
    p.add_argument("--mppi-samples", type=int, default=384)
    p.add_argument("--nmpc-iter", type=int, default=8)
    p.add_argument("--safety-buffer", type=float, default=0.50)
    p.add_argument("--mpc-blind-obstacles", action="store_true",
                   help="Hide rocks from the MPC so the fixed safety filter is the sole avoider.")
    p.add_argument("--timeout", type=float, default=220.0)
    p.add_argument("--base-port", type=int, default=8600)
    p.add_argument("--quick", action="store_true",
                   help="Tiny smoke matrix: clay/sinusoidal/v5/b0, one seed.")
    return p.parse_args()


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
        rms_cte=("rms_cte_m", "mean"),
        speed_ratio=("speed_ratio", "mean"),
        solve_ms=("mean_solve_ms", "mean"),
    ).reset_index()

    x = np.arange(len(summary))
    fig, axes = plt.subplots(2, 3, figsize=(15, 7.5))
    specs = [
        ("collisions", "collisions_std", "Unique obstacles hit"),
        ("clearance", None, "Minimum clearance (m)"),
        ("near", None, "Near-miss count"),
        ("rms_cte", None, "RMS CTE (m)"),
        ("speed_ratio", None, "Mean speed / target"),
        ("solve_ms", None, "Mean solve time (ms)"),
    ]
    for ax, (mean_key, std_key, ylabel) in zip(axes.flat, specs):
        err = summary[std_key].fillna(0.0) if std_key else None
        ax.bar(x, summary[mean_key], yerr=err, capsize=3)
        ax.set_xticks(x)
        ax.set_xticklabels(summary["variant"], rotation=25, ha="right")
        ax.set_ylabel(ylabel)
        ax.grid(axis="y", alpha=0.3)
    fig.suptitle("Autonomous MPC obstacle avoidance by tire model")
    fig.tight_layout()
    fig.savefig(fig_dir / "autonomous_obstacle_tire_model_summary.png", dpi=220)
    plt.close(fig)

    ok["scenario"] = (
        ok["terrain"] + "/" + ok["path"] + "/v" + ok["speed_mps"].astype(str)
        + "/b" + ok["bumpiness"].astype(str)
    )
    for value, fname, title, cmap in [
        ("collisions", "autonomous_obstacle_collision_heatmap.png",
         "Mean unique obstacles hit (lower is better)", "RdYlGn_r"),
        ("min_clearance_m", "autonomous_obstacle_clearance_heatmap.png",
         "Mean minimum clearance, m (higher is better)", "RdYlGn"),
    ]:
        pivot = ok.pivot_table(index="scenario", columns="variant", values=value, aggfunc="mean")
        fig, ax = plt.subplots(figsize=(1.45 * len(pivot.columns) + 4, 0.46 * len(pivot.index) + 2.5))
        im = ax.imshow(pivot.values, aspect="auto", cmap=cmap)
        ax.set_xticks(range(len(pivot.columns)))
        ax.set_xticklabels(pivot.columns, rotation=30, ha="right")
        ax.set_yticks(range(len(pivot.index)))
        ax.set_yticklabels(pivot.index, fontsize=8)
        for i in range(pivot.shape[0]):
            for j in range(pivot.shape[1]):
                v = pivot.values[i, j]
                if math.isfinite(v):
                    ax.text(j, i, f"{v:.2f}", ha="center", va="center", fontsize=7)
        ax.set_title(title)
        fig.colorbar(im, ax=ax, fraction=0.035)
        fig.tight_layout()
        fig.savefig(fig_dir / fname, dpi=220)
        plt.close(fig)

    plot_metric_distribution_grid(
        results_csv,
        out_dir,
        [
            ("collisions", "Unique obstacles hit", "Collisions"),
            ("min_clearance_m", "Minimum clearance (m)", "Clearance"),
            ("rms_cte_m", "RMS CTE (m)", "Tracking error"),
            ("speed_ratio", "mean speed / target", "Speed retention"),
            ("intervention_rate_pct", "Intervention rate (%)", "Shield activity"),
            ("mean_solve_ms", "Mean solve time (ms)", "Runtime"),
        ],
        "autonomous_obstacle_metric_distributions.png",
        "Autonomous obstacle avoidance by tire model",
    )
    plot_trajectory_overlays(
        results_csv,
        out_dir,
        filename_prefix="autonomous_obstacle_trajectory_overlay",
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

    safety_extra: list[str] = []
    safety_label = "barrier_only"
    if args.safety_flavor != "none":
        safety_label = args.safety_flavor
        safety_extra = [
            "--safety-filter",
            "--safety-flavor", args.safety_flavor,
            "--shield-horizon", str(args.shield_horizon),
            "--safety-buffer", str(args.safety_buffer),
        ]
        if args.safety_flavor == "mppi":
            safety_extra += ["--mppi-samples", str(args.mppi_samples)]
        elif args.safety_flavor == "nmpc":
            safety_extra += ["--nmpc-iter", str(args.nmpc_iter)]
    if args.mpc_blind_obstacles:
        safety_label += "_mpc_blind"
        safety_extra.append("--mpc-blind-obstacles")

    out_dir = timestamped_result_dir(f"autonomous_obstacle_tire_model_sweep_{safety_label}")
    write_manifest(out_dir, args, "Autonomous obstacle-aware MPC tire-model sweep with sensor noise enabled.")
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
                                experiment="autonomous_obstacle_tire_model_sweep",
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
                                rocks=args.rocks,
                                lead_in=args.lead_in,
                                extra_args=[
                                    "--speed-weight", str(args.speed_weight),
                                    "--speed-cost-mode", args.speed_cost_mode,
                                    "--obstacle-weight", str(args.obstacle_weight),
                                ] + safety_extra + list(spec["extra"]),
                            )
                            results.append(res)
                            write_results_csv(out_dir / "results.csv", results)
                            print(f"    {res.status}: collisions={res.collisions} "
                                  f"clearance={res.min_clearance_m:.2f} rms_cte={res.rms_cte_m:.3f}")

    write_results_csv(out_dir / "results.csv", results)
    summary = summarize_by_variant(
        results,
        ["collisions", "near_misses", "min_clearance_m", "rms_cte_m",
         "speed_ratio", "mean_solve_ms", "progress_m"],
    )
    summary.to_csv(out_dir / "summary_by_model.csv", index=False)
    save_summary_markdown(
        out_dir,
        "Autonomous Obstacle Avoidance by Tire Model",
        summary,
        [
            "Noise policy: sensor noise enabled in every run.",
            ("No downstream safety filter is used; the standard MPC's obstacle barrier is the only autonomous obstacle-avoidance mechanism."
             if args.safety_flavor == "none" else
             f"Fixed downstream safety filter: {args.safety_flavor}; tire model is the swept variable."),
            f"Standard-MPC speed-weight is {args.speed_weight:g} so obstacle avoidance is not confounded by aggressive v_ref chasing in turns.",
            f"Standard-MPC speed-cost-mode is {args.speed_cost_mode}.",
            f"Standard-MPC obstacle-weight is {args.obstacle_weight:g}.",
            f"MPC blind to obstacles: {bool(args.mpc_blind_obstacles)}.",
            f"Safety buffer: {args.safety_buffer:g} m.",
        ],
    )
    plot_figures(out_dir / "results.csv", out_dir)
    print(f"Done: {out_dir}")


if __name__ == "__main__":
    main()
