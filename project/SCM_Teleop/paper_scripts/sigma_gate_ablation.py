#!/usr/bin/env python3
"""Paper experiment: estimator sigma-gating of the MPPI safety shield.

The abstract claims the shield's friction-cone bound is tightened by the
terrain estimator's disagreement on phi.  Until the loop fix, no caller
ever forwarded a non-zero ``phi_uncertainty_deg`` to ``MPPIShield``, so
the gate was a no-op.  This sweep compares the now-wired gate against
two ablations (gate disabled; live terrain off entirely) on a fixed
obstacle scenario.  Sensor noise is enabled in every run.
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


# Each variant runs the same MPPI shield against rocks under planner-blind
# MPC; the only thing that changes is how the shield reacts to the live
# (n, phi, sigma_phi) estimate the controller publishes.
VARIANTS = {
    "sigma_inflate": dict(
        extra=["--terrain-estimator",
               "--shield-sigma-mode", "inflate"],
        note="Canonical: sigma_phi inflates the obstacle clearance buffer.",
    ),
    "sigma_tighten": dict(
        extra=["--terrain-estimator",
               "--shield-sigma-mode", "tighten"],
        note="Legacy: sigma_phi tightens the friction cone (phi - sigma).",
    ),
    "sigma_both": dict(
        extra=["--terrain-estimator",
               "--shield-sigma-mode", "both"],
        note="Both gates: friction cone tightened AND obstacle buffer inflated.",
    ),
    "sigma_off": dict(
        extra=["--terrain-estimator",
               "--shield-sigma-mode", "off"],
        note="Live terrain (n, phi) but sigma ignored.",
    ),
    "no_live_terrain": dict(
        extra=[],
        note="Baseline: no live terrain at all; shield uses static initial terrain.",
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
    p.add_argument("--base-seed", type=int, default=950)
    p.add_argument("--time", type=float, default=12.0)
    p.add_argument("--lead-in", type=float, default=5.0)
    p.add_argument("--rocks", type=int, default=5)
    p.add_argument("--shield-horizon", type=int, default=18)
    p.add_argument("--mppi-samples", type=int, default=384)
    p.add_argument("--safety-buffer", type=float, default=0.50)
    p.add_argument("--timeout", type=float, default=240.0)
    p.add_argument("--base-port", type=int, default=9500)
    p.add_argument("--quick", action="store_true")
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
    fig.suptitle("MPPI shield sigma-gate ablation")
    fig.tight_layout()
    fig.savefig(fig_dir / "sigma_gate_ablation_summary.png", dpi=220)
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
    ax.set_xticklabels(pivot.columns, rotation=20, ha="right")
    ax.set_yticks(range(len(pivot.index)))
    ax.set_yticklabels(pivot.index, fontsize=8)
    for i in range(pivot.shape[0]):
        for j in range(pivot.shape[1]):
            v = pivot.values[i, j]
            if math.isfinite(v):
                ax.text(j, i, f"{v:.1f}", ha="center", va="center", fontsize=8)
    ax.set_title("Mean collisions per scenario (lower is better)")
    fig.colorbar(im, ax=ax, fraction=0.035)
    fig.tight_layout()
    fig.savefig(fig_dir / "sigma_gate_ablation_collision_heatmap.png", dpi=220)
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
        "sigma_gate_ablation_metric_distributions.png",
        "MPPI shield sigma-gate ablation",
    )
    plot_trajectory_overlays(
        results_csv,
        out_dir,
        filename_prefix="sigma_gate_ablation_trajectory_overlay",
        max_scenarios=4,
    )


def main() -> None:
    args = parse_args()
    if args.quick:
        args.variants = ["sigma_gate_on", "sigma_gate_off"]
        args.terrains = ["clay"]
        args.paths = ["sinusoidal"]
        args.speeds = [5.0]
        args.bumpiness = [0]
        args.seeds = 1
        args.time = min(args.time, 8.0)

    out_dir = timestamped_result_dir("sigma_gate_ablation")
    write_manifest(out_dir, args,
                   "MPPI shield sigma-gating ablation with sensor noise enabled.")
    print(f"Output: {out_dir}")

    results: list[RunResult] = []
    idx = 0
    total = (len(args.variants) * len(args.terrains) * len(args.paths)
             * len(args.speeds) * len(args.bumpiness) * args.seeds)
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
                            idx += 1
                            print(f"[{idx}/{total}] {variant} {terrain}/{path} "
                                  f"v={speed:g} b={bump} seed={seed}")
                            res = launch_and_collect(
                                experiment="sigma_gate_ablation",
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
                                extra_args=shield_args(variant, args),
                            )
                            res.notes = VARIANTS[variant]["note"]
                            results.append(res)
                            write_results_csv(out_dir / "results.csv", results)
                            print(f"    {res.status}: collisions={res.collisions} "
                                  f"clearance={res.min_clearance_m:.2f} "
                                  f"interv={res.intervention_rate_pct:.1f}%")

    write_results_csv(out_dir / "results.csv", results)
    summary = summarize_by_variant(
        results,
        ["collisions", "near_misses", "min_clearance_m", "intervention_rate_pct",
         "mean_abs_dsteer", "mean_abs_dthrottle", "rms_cte_m"],
    )
    summary.to_csv(out_dir / "summary_by_variant.csv", index=False)
    save_summary_markdown(
        out_dir,
        "MPPI Shield Sigma-Gate Ablation",
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
