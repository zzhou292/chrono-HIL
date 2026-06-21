#!/usr/bin/env python3
"""Paper experiment: learned terrain estimator in-distribution vs OOD.

One thing tested: how accurately the online learned estimator recovers Bekker n
from noisy closed-loop driving on canonical terrains and randomly sampled
out-of-distribution SCM soils. Each run starts the estimator from the same
neutral dirt initialization used by the controller.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
from dataclasses import asdict
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
    PROJECT_ROOT,
    SPEEDS,
    TERRAINS,
    RunResult,
    launch_and_collect,
    save_summary_markdown,
    timestamped_result_dir,
    write_manifest,
)


TRUE_N = {"clay": 0.5, "dirt": 0.7, "sand": 1.1}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--distributions", nargs="+", default=["id", "ood"], choices=["id", "ood"])
    p.add_argument("--terrains", nargs="+", default=list(TERRAINS), choices=list(TERRAINS),
                   help="Canonical in-distribution terrains.")
    p.add_argument("--paths", nargs="+", default=["sinusoidal"], choices=list(PATHS),
                   help="Sinusoidal is the default paper setting because the estimator needs excitation.")
    p.add_argument("--speeds", nargs="+", type=float, default=[5.0, 7.0])
    p.add_argument("--bumpiness", nargs="+", type=int, default=list(BUMPS))
    p.add_argument("--seeds", type=int, default=5)
    p.add_argument("--base-seed", type=int, default=710)
    p.add_argument("--ood-terrains", type=int, default=6)
    p.add_argument("--time", type=float, default=20.0)
    p.add_argument("--lead-in", type=float, default=5.0)
    p.add_argument("--metric-start", type=float, default=8.0)
    p.add_argument("--speed-weight", type=float, default=15.0)
    p.add_argument("--timeout", type=float, default=240.0)
    p.add_argument("--base-port", type=int, default=9400)
    p.add_argument("--quick", action="store_true",
                   help="Tiny smoke matrix: one ID run and one OOD run.")
    return p.parse_args()


def generate_ood_terrains(out_dir: Path, n_terrains: int, seed: int) -> pd.DataFrame:
    terrain_dir = out_dir / "raw" / "ood_terrains"
    cmd = [
        sys.executable, "-u", str(PROJECT_ROOT / "utilities" / "generate_random_terrains.py"),
        "--out-dir", str(terrain_dir),
        "--n-terrains", str(n_terrains),
        "--seed", str(seed),
    ]
    log_path = terrain_dir / "generate.log"
    terrain_dir.mkdir(parents=True, exist_ok=True)
    with log_path.open("w") as f:
        subprocess.run(cmd, cwd=str(PROJECT_ROOT), stdout=f, stderr=subprocess.STDOUT, check=True)
    return pd.read_csv(terrain_dir / "manifest.csv")


def estimator_metrics(diag_csv: str, true_n: float, metric_start: float) -> dict[str, float]:
    if not diag_csv:
        return {
            "n_est_final": math.nan,
            "n_est_mean_tail": math.nan,
            "n_abs_err_final": math.nan,
            "n_abs_err_tail": math.nan,
            "terrain_update_count": 0,
            "first_update_time_s": math.nan,
        }
    df = pd.read_csv(diag_csv)
    if df.empty or "n_terrain_est" not in df.columns:
        return {
            "n_est_final": math.nan,
            "n_est_mean_tail": math.nan,
            "n_abs_err_final": math.nan,
            "n_abs_err_tail": math.nan,
            "terrain_update_count": 0,
            "first_update_time_s": math.nan,
        }
    t = pd.to_numeric(df.get("sim_time", pd.Series(np.arange(len(df)))), errors="coerce")
    n_est = pd.to_numeric(df["n_terrain_est"], errors="coerce")
    finite = np.isfinite(n_est)
    tail = finite & np.isfinite(t) & (t >= metric_start)
    if not tail.any():
        tail = finite
    updates = pd.to_numeric(df.get("terrain_update_applied", pd.Series(np.zeros(len(df)))), errors="coerce").fillna(0)
    update_mask = updates > 0
    first_update = float(t[update_mask].iloc[0]) if update_mask.any() else math.nan
    final_est = float(n_est[finite].iloc[-1]) if finite.any() else math.nan
    mean_tail = float(n_est[tail].mean()) if tail.any() else math.nan
    return {
        "n_est_final": final_est,
        "n_est_mean_tail": mean_tail,
        "n_abs_err_final": abs(final_est - true_n) if math.isfinite(final_est) else math.nan,
        "n_abs_err_tail": abs(mean_tail - true_n) if math.isfinite(mean_tail) else math.nan,
        "terrain_update_count": int(update_mask.sum()),
        "first_update_time_s": first_update,
    }


def result_row(res: RunResult, distribution: str, case_label: str, true_n: float,
               terrain_config: str, metric_start: float) -> dict:
    row = asdict(res)
    row.pop("extra", None)
    row.update({
        "distribution": distribution,
        "case_label": case_label,
        "true_n": true_n,
        "terrain_config": terrain_config,
    })
    row.update(estimator_metrics(res.diag_csv, true_n, metric_start))
    return row


def plot_figures(results_csv: Path, out_dir: Path) -> None:
    df = pd.read_csv(results_csv)
    ok = df[df["status"] == "ok"].copy()
    if ok.empty:
        return
    fig_dir = out_dir / "figures"

    summary = ok.groupby("distribution", sort=False).agg(
        n_err=("n_abs_err_tail", "mean"),
        n_err_std=("n_abs_err_tail", "std"),
        cte=("rms_cte_m", "mean"),
        updates=("terrain_update_count", "mean"),
        first_update=("first_update_time_s", "mean"),
    ).reset_index()
    x = np.arange(len(summary))
    fig, axes = plt.subplots(1, 3, figsize=(12.5, 4.0))
    specs = [
        ("n_err", "n_err_std", "Tail mean |n error|"),
        ("cte", None, "RMS CTE (m)"),
        ("first_update", None, "First accepted update (s)"),
    ]
    for ax, (mean_key, std_key, ylabel) in zip(axes, specs):
        err = summary[std_key].fillna(0.0) if std_key else None
        ax.bar(x, summary[mean_key], yerr=err, capsize=3)
        ax.set_xticks(x)
        ax.set_xticklabels(summary["distribution"])
        ax.set_ylabel(ylabel)
        ax.grid(axis="y", alpha=0.3)
    fig.suptitle("Online terrain estimator: ID vs OOD")
    fig.tight_layout()
    fig.savefig(fig_dir / "terrain_estimator_summary.png", dpi=220)
    plt.close(fig)

    ok["scenario"] = ok["case_label"] + "/" + ok["path"] + "/v" + ok["speed_mps"].astype(str)
    pivot = ok.pivot_table(index="scenario", columns="distribution", values="n_abs_err_tail", aggfunc="mean")
    fig, ax = plt.subplots(figsize=(6.5, 0.46 * len(pivot.index) + 2.5))
    im = ax.imshow(pivot.values, aspect="auto", cmap="viridis_r")
    ax.set_xticks(range(len(pivot.columns)))
    ax.set_xticklabels(pivot.columns)
    ax.set_yticks(range(len(pivot.index)))
    ax.set_yticklabels(pivot.index, fontsize=8)
    for i in range(pivot.shape[0]):
        for j in range(pivot.shape[1]):
            v = pivot.values[i, j]
            if math.isfinite(v):
                ax.text(j, i, f"{v:.3f}", ha="center", va="center", fontsize=8, color="white")
    ax.set_title("Tail mean |n error| by case")
    fig.colorbar(im, ax=ax, fraction=0.045)
    fig.tight_layout()
    fig.savefig(fig_dir / "terrain_estimator_error_heatmap.png", dpi=220)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.0, 4.5))
    for distribution, sub in ok.groupby("distribution", sort=False):
        ax.scatter(sub["true_n"], sub["n_est_mean_tail"], label=distribution, alpha=0.75, s=46)
    lims = [
        min(ok["true_n"].min(), ok["n_est_mean_tail"].min()) - 0.05,
        max(ok["true_n"].max(), ok["n_est_mean_tail"].max()) + 0.05,
    ]
    ax.plot(lims, lims, "k--", linewidth=1)
    ax.set_xlim(lims)
    ax.set_ylim(lims)
    ax.set_xlabel("True Bekker n")
    ax.set_ylabel("Estimated tail-mean n")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(fig_dir / "terrain_estimator_true_vs_estimated.png", dpi=220)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    if args.quick:
        args.distributions = ["id", "ood"]
        args.terrains = ["clay"]
        args.paths = ["sinusoidal"]
        args.speeds = [5.0]
        args.bumpiness = [0]
        args.seeds = 1
        args.ood_terrains = 1
        args.time = min(args.time, 8.0)

    out_dir = timestamped_result_dir("terrain_estimator_benchmark")
    write_manifest(out_dir, args, "Learned terrain estimator ID/OOD benchmark with sensor noise enabled.")
    print(f"Output: {out_dir}")

    ood_manifest = pd.DataFrame()
    if "ood" in args.distributions:
        ood_manifest = generate_ood_terrains(out_dir, args.ood_terrains, args.base_seed)

    rows: list[dict] = []
    idx = 0
    id_cases = [
        dict(distribution="id", terrain=t, case_label=t, true_n=TRUE_N[t], terrain_config="")
        for t in args.terrains
    ]
    ood_cases = []
    for _, r in ood_manifest.iterrows():
        ood_cases.append(dict(
            distribution="ood",
            terrain=str(r["preset_proxy"]),
            case_label=str(r["label"]),
            true_n=float(r["true_n"]),
            terrain_config=str(r["yaml"]),
        ))
    cases = []
    if "id" in args.distributions:
        cases.extend(id_cases)
    if "ood" in args.distributions:
        cases.extend(ood_cases)

    total = len(cases) * len(args.paths) * len(args.speeds) * len(args.bumpiness) * args.seeds
    for case in cases:
        for path in args.paths:
            for speed in args.speeds:
                for bump in args.bumpiness:
                    for seed_i in range(args.seeds):
                        seed = args.base_seed + seed_i
                        sim_port = args.base_port + 2 * idx
                        ctrl_port = sim_port + 1
                        run_dir = out_dir / "raw" / (
                            f"{idx:04d}_{case['distribution']}_{case['case_label']}_{path}_v{speed:g}_b{bump}_s{seed}"
                        )
                        idx += 1
                        extra = [
                            "--terrain-estimator",
                            "--te-verbose",
                            "--speed-weight", str(args.speed_weight),
                        ]
                        if case["terrain_config"]:
                            extra += ["--terrain-config", case["terrain_config"]]
                        print(f"[{idx}/{total}] {case['distribution']}:{case['case_label']} "
                              f"{path} v={speed:g} b={bump} seed={seed}")
                        res = launch_and_collect(
                            experiment="terrain_estimator_benchmark",
                            variant=case["distribution"],
                            controller_mode="standard",
                            mpc_model="nn",
                            nn_model=DEFAULT_NN_MODEL,
                            terrain=case["terrain"],
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
                        row = result_row(
                            res, case["distribution"], case["case_label"],
                            case["true_n"], case["terrain_config"], args.metric_start,
                        )
                        rows.append(row)
                        pd.DataFrame(rows).to_csv(out_dir / "results.csv", index=False)
                        print(f"    {res.status}: n_tail={row['n_est_mean_tail']:.3f} "
                              f"|err|={row['n_abs_err_tail']:.3f} rms_cte={res.rms_cte_m:.3f}")

    results_csv = out_dir / "results.csv"
    pd.DataFrame(rows).to_csv(results_csv, index=False)
    summary = pd.DataFrame(rows).groupby("distribution", sort=False).agg(
        n_runs=("variant", "count"),
        n_ok=("status", lambda s: int((s == "ok").sum())),
        n_abs_err_tail_mean=("n_abs_err_tail", "mean"),
        n_abs_err_tail_std=("n_abs_err_tail", "std"),
        rms_cte_m_mean=("rms_cte_m", "mean"),
        first_update_time_s_mean=("first_update_time_s", "mean"),
    ).reset_index()
    summary.to_csv(out_dir / "summary_by_distribution.csv", index=False)
    save_summary_markdown(
        out_dir,
        "Terrain Estimator Benchmark",
        summary,
        [
            "Noise policy: sensor noise enabled in every run.",
            "The estimator starts from neutral dirt/n=0.7 in every run; OOD terrains are generated into this result directory.",
            f"Estimator tail metrics are averaged after t={args.metric_start:g}s, or over all finite estimates if a quick run is shorter.",
        ],
    )
    plot_figures(results_csv, out_dir)
    print(f"Done: {out_dir}")


if __name__ == "__main__":
    main()
