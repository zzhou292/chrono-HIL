#!/usr/bin/env python3
"""Paper experiment: MPCC versus standard MPC speed/tracking tradeoff.

One thing tested: whether MPCC's path-progress formulation improves the
speed-versus-tracking Pareto point relative to standard MPC, and which MPCC
knobs explain failures. Sensor noise is enabled in every run.
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


# MPCC's CasADi solver is wired to the 11-input static MLP only (see
# AGENT.md and acados_mpcc_solver.py:272). The rate-augmented v2/v3 MLPs
# expect a 12th `rates` vector and crash at solver build. Pin MPCC variants
# to the known-good static MLP until the rate variant is ported.
MPCC_NN_MODEL = "closed_loop_v1_mlp_32_16"


VARIANTS = {
    "standard_mpc": dict(
        controller_mode="standard", mpc_model="nn", nn_model=DEFAULT_NN_MODEL, extra=[],
        note="Baseline reference-tracking MPC with the closed-loop NN surrogate.",
    ),
    "standard_mpc_soft_speed": dict(
        controller_mode="standard", mpc_model="nn", nn_model=DEFAULT_NN_MODEL,
        extra=["--speed-weight", "15.0"],
        note="Same MPC with weaker speed tracking so turns prioritize path tracking over v_ref recovery.",
    ),
    "standard_mpc_overspeed_cap": dict(
        controller_mode="standard", mpc_model="nn", nn_model=DEFAULT_NN_MODEL,
        extra=["--speed-weight", "70.0", "--speed-cost-mode", "overspeed"],
        note="Treats v_ref as a speed cap instead of a command, avoiding acceleration just to erase underspeed.",
    ),
    "standard_mpc_no_speed": dict(
        controller_mode="standard", mpc_model="nn", nn_model=DEFAULT_NN_MODEL,
        extra=["--speed-weight", "0.0"],
        note="Ablates speed tracking entirely; useful to bound whether v_ref chasing is the failure mode.",
    ),
    "mpcc_default": dict(
        controller_mode="mpcc", mpc_model="nn", nn_model=MPCC_NN_MODEL, extra=[],
        note="Current MPCC defaults with the static-MLP NN surrogate.",
    ),
    "mpcc_less_speed_cap": dict(
        controller_mode="mpcc", mpc_model="nn", nn_model=MPCC_NN_MODEL,
        extra=["--mpcc-vtheta-max", "8.0", "--mpcc-w-speed-cap", "80.0"],
        note="Tests whether MPCC is being boxed in by vtheta_max and the soft curvature speed cap.",
    ),
    "mpcc_more_progress": dict(
        controller_mode="mpcc", mpc_model="nn", nn_model=MPCC_NN_MODEL,
        extra=["--mpcc-vtheta-max", "8.0", "--mpcc-w-progress", "2.0", "--mpcc-w-speed-cap", "150.0"],
        note="Tests whether the progress reward is too weak relative to lag/contour costs.",
    ),
    "mpcc_tight_tracking": dict(
        controller_mode="mpcc", mpc_model="nn", nn_model=MPCC_NN_MODEL,
        extra=[
            "--mpcc-w-contour", "9000.0",
            "--mpcc-w-lag", "6000.0",
            "--mpcc-w-progress", "0.15",
            "--mpcc-w-speed-cap", "600.0",
        ],
        note="Conservative MPCC tuning that prioritizes path adherence; tests whether default contour/lag weights are too weak.",
    ),
    "mpcc_balanced_tracking": dict(
        controller_mode="mpcc", mpc_model="nn", nn_model=MPCC_NN_MODEL,
        extra=[
            "--mpcc-w-contour", "6000.0",
            "--mpcc-w-lag", "4000.0",
            "--mpcc-w-progress", "0.25",
            "--mpcc-w-speed-cap", "450.0",
        ],
        note="Middle-ground MPCC tuning between the fast default and the tight-tracking variant.",
    ),
    "mpcc_low_steer_reg": dict(
        controller_mode="mpcc", mpc_model="nn", nn_model=MPCC_NN_MODEL,
        extra=["--mpcc-w-delta-dot", "30.0"],
        note="Tests whether steering-rate regularization is too conservative for fast tracking.",
    ),
    "mpcc_friction_ellipse": dict(
        controller_mode="mpcc", mpc_model="nn", nn_model=MPCC_NN_MODEL,
        extra=["--mpcc-friction-ellipse"],
        note="Tests the disabled hard friction ellipse; expected to expose NN peak-Fy infeasibility if it fails.",
    ),
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--variants", nargs="+", default=[
        "standard_mpc", "mpcc_default", "mpcc_less_speed_cap", "mpcc_more_progress",
    ], choices=list(VARIANTS))
    p.add_argument("--terrains", nargs="+", default=list(TERRAINS), choices=list(TERRAINS))
    p.add_argument("--paths", nargs="+", default=list(PATHS))
    p.add_argument("--speeds", nargs="+", type=float, default=list(SPEEDS))
    p.add_argument("--bumpiness", nargs="+", type=int, default=list(BUMPS))
    p.add_argument("--seeds", type=int, default=5)
    p.add_argument("--base-seed", type=int, default=200)
    p.add_argument("--time", type=float, default=15.0)
    p.add_argument("--lead-in", type=float, default=5.0)
    p.add_argument("--timeout", type=float, default=180.0)
    p.add_argument("--base-port", type=int, default=7000)
    p.add_argument("--quick", action="store_true",
                   help="Tiny smoke matrix: clay/sinusoidal/v5/b0, one seed.")
    return p.parse_args()


def plot_figures(results_csv: Path, out_dir: Path) -> None:
    df = pd.read_csv(results_csv)
    ok = df[df["status"] == "ok"].copy()
    if ok.empty:
        return
    fig_dir = out_dir / "figures"

    fig, ax = plt.subplots(figsize=(7.2, 5.2))
    for variant, sub in ok.groupby("variant", sort=False):
        ax.scatter(sub["rms_cte_m"], sub["speed_ratio"], s=42, alpha=0.65, label=variant)
        ax.scatter([sub["rms_cte_m"].mean()], [sub["speed_ratio"].mean()],
                   s=130, marker="X", edgecolor="black")
    ax.set_xlabel("RMS CTE (m), lower is better")
    ax.set_ylabel("mean speed / target, higher is better")
    ax.set_title("Speed-tracking Pareto view across all scenarios")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(fig_dir / "mpcc_speed_tracking_pareto.png", dpi=220)
    plt.close(fig)

    summary = ok.groupby("variant", sort=False).agg(
        rms_cte=("rms_cte_m", "mean"),
        rms_cte_std=("rms_cte_m", "std"),
        speed_ratio=("speed_ratio", "mean"),
        speed_ratio_std=("speed_ratio", "std"),
        solve_ms=("mean_solve_ms", "mean"),
        progress=("progress_m", "mean"),
    ).reset_index()
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.2))
    x = np.arange(len(summary))
    for ax, mean_key, std_key, ylabel in [
        (axes[0], "rms_cte", "rms_cte_std", "RMS CTE (m)"),
        (axes[1], "speed_ratio", "speed_ratio_std", "mean speed / target"),
        (axes[2], "solve_ms", None, "Mean solve time (ms)"),
    ]:
        err = summary[std_key].fillna(0.0) if std_key else None
        ax.bar(x, summary[mean_key], yerr=err, capsize=3)
        ax.set_xticks(x)
        ax.set_xticklabels(summary["variant"], rotation=25, ha="right")
        ax.set_ylabel(ylabel)
        ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(fig_dir / "mpcc_variant_summary.png", dpi=220)
    plt.close(fig)

    ok["scenario"] = ok["terrain"] + "/" + ok["path"] + "/v" + ok["speed_mps"].astype(str)
    pivot = ok.pivot_table(index="scenario", columns="variant", values="speed_ratio", aggfunc="mean")
    fig, ax = plt.subplots(figsize=(1.5 * len(pivot.columns) + 4, 0.45 * len(pivot.index) + 2.5))
    im = ax.imshow(pivot.values, aspect="auto", cmap="RdYlGn", vmin=0.0, vmax=max(1.2, np.nanmax(pivot.values)))
    ax.set_xticks(range(len(pivot.columns)))
    ax.set_xticklabels(pivot.columns, rotation=30, ha="right")
    ax.set_yticks(range(len(pivot.index)))
    ax.set_yticklabels(pivot.index, fontsize=8)
    for i in range(pivot.shape[0]):
        for j in range(pivot.shape[1]):
            v = pivot.values[i, j]
            if math.isfinite(v):
                ax.text(j, i, f"{v:.2f}", ha="center", va="center", fontsize=8)
    ax.set_title("Mean speed / target by scenario")
    fig.colorbar(im, ax=ax, fraction=0.035)
    fig.tight_layout()
    fig.savefig(fig_dir / "mpcc_speed_ratio_heatmap.png", dpi=220)
    plt.close(fig)

    plot_metric_distribution_grid(
        results_csv,
        out_dir,
        [
            ("rms_cte_m", "RMS CTE (m)", "Tracking error"),
            ("speed_ratio", "mean speed / target", "Speed retention"),
            ("progress_m", "Progress (m)", "Distance traveled"),
            ("mean_solve_ms", "Mean solve time (ms)", "Runtime"),
        ],
        "mpcc_metric_distributions.png",
        "MPCC vs standard MPC speed/tracking",
    )
    plot_trajectory_overlays(
        results_csv,
        out_dir,
        filename_prefix="mpcc_trajectory_overlay",
        max_scenarios=4,
    )


def main() -> None:
    args = parse_args()
    if args.quick:
        args.variants = ["standard_mpc", "standard_mpc_soft_speed", "mpcc_default", "mpcc_less_speed_cap"]
        args.terrains = ["clay"]
        args.paths = ["sinusoidal"]
        args.speeds = [5.0]
        args.bumpiness = [0]
        args.seeds = 1
        args.time = min(args.time, 8.0)

    out_dir = timestamped_result_dir("mpcc_vs_mpc_speed_tracking")
    write_manifest(out_dir, args, "MPCC vs standard MPC speed/tracking sweep with sensor noise enabled.")
    print(f"Output: {out_dir}")

    results: list[RunResult] = []
    idx = 0
    total = (len(args.variants) * len(args.terrains) * len(args.paths)
             * len(args.speeds) * len(args.bumpiness) * args.seeds)
    for variant in args.variants:
        spec = VARIANTS[variant]
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
                                experiment="mpcc_vs_mpc_speed_tracking",
                                variant=variant,
                                controller_mode=spec["controller_mode"],
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
                            res.notes = spec["note"]
                            results.append(res)
                            write_results_csv(out_dir / "results.csv", results)
                            print(f"    {res.status}: rms_cte={res.rms_cte_m:.3f} speed_ratio={res.speed_ratio:.2f}")

    write_results_csv(out_dir / "results.csv", results)
    summary = summarize_by_variant(results, ["rms_cte_m", "speed_ratio", "mean_speed_mps", "mean_solve_ms", "progress_m"])
    summary.to_csv(out_dir / "summary_by_variant.csv", index=False)
    save_summary_markdown(
        out_dir,
        "MPCC vs Standard MPC Speed/Tracking",
        summary,
        [
            "Noise policy: sensor noise enabled in every run.",
            "MPC speed-weight interpretation: if `standard_mpc_soft_speed` reduces CTE without a large speed-ratio loss, the paper baseline should use the softer speed cost rather than force the tracker to chase v_ref in turns.",
            "Troubleshooting interpretation: if relaxed MPCC variants do not improve speed ratio without CTE growth, the current MPCC is limited by missing baseline features/model mismatch rather than just conservative vtheta/cap settings.",
            "Variant notes: " + "; ".join(f"{k}: {VARIANTS[k]['note']}" for k in args.variants),
        ],
    )
    plot_figures(out_dir / "results.csv", out_dir)
    print(f"Done: {out_dir}")


if __name__ == "__main__":
    main()
