#!/usr/bin/env python3
"""Paper experiment: NN contribution inside the DOB-CBF safety filter.

One thing tested: DOB-CBF with the NN tire model enabled versus the same
DOB-CBF code forced to use its kinematic fallback. Sensor noise is enabled in
every run. This is the ablation to answer whether the NN is actually helping
the legacy DOB-CBF obstacle-avoidance filter.
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
    save_summary_markdown,
    summarize_by_variant,
    timestamped_result_dir,
    write_manifest,
    write_results_csv,
)


VARIANTS = {
    "no_filter": [],
    "dob_cbf_nn": ["--safety-filter", "--safety-flavor", "dob_cbf"],
    "dob_cbf_no_nn": ["--safety-filter", "--safety-flavor", "dob_cbf", "--no-safety-nn"],
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--variants", nargs="+", default=list(VARIANTS), choices=list(VARIANTS))
    p.add_argument("--terrains", nargs="+", default=list(TERRAINS), choices=list(TERRAINS))
    p.add_argument("--paths", nargs="+", default=list(PATHS))
    p.add_argument("--speeds", nargs="+", type=float, default=list(SPEEDS))
    p.add_argument("--bumpiness", nargs="+", type=int, default=list(BUMPS))
    p.add_argument("--seeds", type=int, default=5)
    p.add_argument("--base-seed", type=int, default=45)
    p.add_argument("--time", type=float, default=12.0)
    p.add_argument("--lead-in", type=float, default=5.0)
    p.add_argument("--rocks", type=int, default=5)
    p.add_argument("--aware", action="store_true",
                   help="Let the planning MPC see rocks. Default is blind MPC so DOB-CBF is the sole avoider.")
    p.add_argument("--timeout", type=float, default=200.0)
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
        intervention=("intervention_rate_pct", "mean"),
        dsteer=("mean_abs_dsteer", "mean"),
        dthr=("mean_abs_dthrottle", "mean"),
        rms_cte=("rms_cte_m", "mean"),
    ).reset_index()

    x = np.arange(len(summary))
    fig, axes = plt.subplots(1, 4, figsize=(15, 4.2))
    for ax, mean_key, std_key, ylabel in [
        (axes[0], "collisions", "collisions_std", "Unique obstacles hit"),
        (axes[1], "clearance", None, "Min clearance (m)"),
        (axes[2], "intervention", None, "Intervention rate (%)"),
        (axes[3], "rms_cte", None, "RMS CTE (m)"),
    ]:
        err = summary[std_key].fillna(0.0) if std_key else None
        ax.bar(x, summary[mean_key], yerr=err, capsize=3)
        ax.set_xticks(x)
        ax.set_xticklabels(summary["variant"], rotation=25, ha="right")
        ax.set_ylabel(ylabel)
        ax.grid(axis="y", alpha=0.3)
    fig.suptitle("DOB-CBF NN ablation")
    fig.tight_layout()
    fig.savefig(fig_dir / "dob_cbf_nn_ablation_summary.png", dpi=220)
    plt.close(fig)

    ok["scenario"] = ok["terrain"] + "/" + ok["path"] + "/v" + ok["speed_mps"].astype(str)
    pivot = ok.pivot_table(index="scenario", columns="variant", values="collisions", aggfunc="mean")
    fig, ax = plt.subplots(figsize=(1.5 * len(pivot.columns) + 4, 0.48 * len(pivot.index) + 2.5))
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
    ax.set_title("DOB-CBF NN ablation: mean obstacles hit")
    fig.colorbar(im, ax=ax, fraction=0.035)
    fig.tight_layout()
    fig.savefig(fig_dir / "dob_cbf_nn_ablation_heatmap.png", dpi=220)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    if args.quick:
        args.variants = list(VARIANTS)
        args.terrains = ["clay"]
        args.paths = ["sinusoidal"]
        args.speeds = [5.0]
        args.bumpiness = [0]
        args.seeds = 1
        args.time = min(args.time, 8.0)

    out_dir = timestamped_result_dir("dob_cbf_nn_ablation")
    write_manifest(out_dir, args, "DOB-CBF NN-on versus NN-off ablation with sensor noise enabled.")
    print(f"Output: {out_dir}")

    results: list[RunResult] = []
    idx = 0
    total = (len(args.variants) * len(args.terrains) * len(args.paths)
             * len(args.speeds) * len(args.bumpiness) * args.seeds)
    for variant in args.variants:
        base_extra = list(VARIANTS[variant])
        if not args.aware:
            base_extra.append("--mpc-blind-obstacles")
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
                                experiment="dob_cbf_nn_ablation",
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
                                extra_args=base_extra,
                            )
                            results.append(res)
                            write_results_csv(out_dir / "results.csv", results)
                            print(f"    {res.status}: collisions={res.collisions} near={res.near_misses} "
                                  f"interv={res.intervention_rate_pct:.1f}%")

    write_results_csv(out_dir / "results.csv", results)
    summary = summarize_by_variant(
        results,
        ["collisions", "near_misses", "min_clearance_m", "intervention_rate_pct",
         "mean_abs_dsteer", "mean_abs_dthrottle", "rms_cte_m", "speed_ratio"],
    )
    summary.to_csv(out_dir / "summary_by_variant.csv", index=False)
    save_summary_markdown(
        out_dir,
        "DOB-CBF NN Ablation",
        summary,
        [
            "Noise policy: sensor noise enabled in every run.",
            "Default planner policy: MPC is blind to rocks, so the filter is the sole obstacle avoider. Pass `--aware` to benchmark the easier combined planner+filter stack.",
        ],
    )
    plot_figures(out_dir / "results.csv", out_dir)
    print(f"Done: {out_dir}")


if __name__ == "__main__":
    main()
