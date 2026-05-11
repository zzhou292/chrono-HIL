#!/usr/bin/env python3
"""
Feature benchmarks for paper: obstacle avoidance, terrain estimator, safety filter.

Produces CSV + publication-quality plots for each feature:

  Suite A — Obstacle Avoidance:
    Compares MPC tracking with/without obstacle field across terrains × speeds.
    Key metrics: collision count, near-miss count, CTE, deviation from obstacle-free.

  Suite B — Terrain Estimator:
    Evaluates online terrain estimation across all terrains × paths × speeds.
    Key metrics: estimated n vs true n, estimated phi vs true phi, CTE.

  Suite C — Safety Filter (CBF):
    Tests DOB-CBF-QP with/without obstacles and with/without teleop delay.
    Key metrics: collision count, QP success rate, CTE, CBF interventions.

Usage:
    # All suites
    python run_feature_benchmarks.py -j 4

    # Single suite
    python run_feature_benchmarks.py --suite obstacle -j 2
    python run_feature_benchmarks.py --suite terrain-estimator -j 2
    python run_feature_benchmarks.py --suite safety-filter -j 2
"""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

# Force unbuffered output
os.environ["PYTHONUNBUFFERED"] = "1"
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(line_buffering=True)

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

UTIL_DIR = Path(__file__).parent
PROJECT_ROOT = UTIL_DIR.parent
SIM_DIR = PROJECT_ROOT / "simulation"
PLOTS_DIR = SIM_DIR / "plots"
PAPER_FIGS = PROJECT_ROOT / "my_paper" / "paper_figures"

TERRAINS = ["clay", "dirt", "sand"]
PATHS = ["sinusoidal", "lane_change", "double_lane_change"]
SPEEDS = [5.0, 8.0]

# Terrain ground-truth params for estimator comparison
TERRAIN_TRUTH = {
    "clay": {"n": 0.50, "phi": 13.0, "mu": 0.231},
    "dirt": {"n": 0.70, "phi": 29.0, "mu": 0.554},
    "sand": {"n": 1.10, "phi": 30.0, "mu": 0.577},
}

BASE_PORT = 6100  # ZMQ ports start here (avoid collision with interactive runs)


# ═══════════════════════════════════════════════════════════════════════
# Shared helpers
# ═══════════════════════════════════════════════════════════════════════

def _run_sim(cmd_extra, terrain, path, speed, sim_time, plot_dir,
             sim_port, ctrl_port):
    """Run launch_decoupled.py with given args, return (wall_time, diag_csv_path)."""
    cmd = [
        sys.executable,
        str(SIM_DIR / "launch_decoupled.py"),
        "--model", "nn",
        "--nn-model", "paper_v2_mlp_16_4",
        "--terrain", terrain,
        "--path", path,
        "--speed", str(speed),
        "--time", str(sim_time),
        "--no-vis",
        "--no-plot",
        "--sim-port", str(sim_port),
        "--ctrl-port", str(ctrl_port),
        "--plot-dir", str(plot_dir),
    ]
    if path == "sinusoidal":
        cmd += ["--sine-amplitude", "2.0", "--sine-wavelength", "30.0"]

    cmd += cmd_extra

    t0 = time.time()
    try:
        proc = subprocess.run(cmd, timeout=300, cwd=str(SIM_DIR),
                              capture_output=True, text=True)
        wall = time.time() - t0
    except subprocess.TimeoutExpired:
        return time.time() - t0, None

    # Find diagnostic CSV
    if not Path(plot_dir).is_dir():
        return wall, None
    diag_csvs = sorted(Path(plot_dir).glob("**/diag_*.csv"))
    return wall, str(diag_csvs[0]) if diag_csvs else None


def _parse_diag(csv_path, t_start=2.0):
    """Parse a diagnostic CSV, return dict of metrics."""
    import pandas as pd
    df = pd.read_csv(csv_path)
    df_ss = df[df["sim_time"] >= t_start]

    cte = df_ss["crosstrack_err"].abs()
    metrics = {
        "n_solves": len(df),
        "avg_cte_m": float(cte.mean()) if len(cte) else float("nan"),
        "rms_cte_m": float(np.sqrt((cte ** 2).mean())) if len(cte) else float("nan"),
        "max_cte_m": float(cte.max()) if len(cte) else float("nan"),
        "mean_speed": float(df_ss["u_meas"].mean()) if "u_meas" in df.columns and len(df_ss) else float("nan"),
    }

    # Solver success
    if "solver_status" in df.columns:
        st = df["solver_status"]
        metrics["solver_success_pct"] = 100.0 * ((st == 0) | (st == 2)).sum() / max(len(st), 1)

    # Solve time
    if "solve_time_ms" in df.columns:
        metrics["mean_solve_ms"] = float(df["solve_time_ms"].mean())

    # Terrain estimator output
    if "terrain_class_est" in df.columns and "n_terrain_est" in df.columns:
        est_rows = df_ss[df_ss["n_terrain_est"].notna()]
        if len(est_rows):
            metrics["est_n_mean"] = float(est_rows["n_terrain_est"].mean())
            metrics["est_n_min"] = float(est_rows["n_terrain_est"].min())
            metrics["est_n_max"] = float(est_rows["n_terrain_est"].max())
        if "terrain_confidence" in df.columns:
            metrics["est_confidence_mean"] = float(est_rows["terrain_confidence"].mean())

    return metrics


def _collision_from_stdout(stdout_text):
    """Extract collision count from sim stdout."""
    collisions = 0
    near_misses = 0
    for line in (stdout_text or "").split("\n"):
        m = re.search(r"(\d+)\s+collision", line, re.IGNORECASE)
        if m:
            collisions = int(m.group(1))
        m = re.search(r"(\d+)\s+near.miss", line, re.IGNORECASE)
        if m:
            near_misses = int(m.group(1))
    return collisions, near_misses


# ═══════════════════════════════════════════════════════════════════════
# Suite A — Obstacle Avoidance Benchmark
# ═══════════════════════════════════════════════════════════════════════

def build_obstacle_jobs(base_port_offset=0):
    """Build job list for obstacle avoidance benchmark."""
    jobs = []
    idx = 0
    for terrain in TERRAINS:
        for speed in SPEEDS:
            for n_rocks in [0, 4, 8]:
                port_base = BASE_PORT + base_port_offset + idx * 2
                tag = f"obstacle_{terrain}_v{int(speed)}_r{n_rocks}"
                jobs.append({
                    "tag": tag,
                    "terrain": terrain,
                    "path": "sinusoidal",
                    "speed": speed,
                    "sim_time": 20.0,
                    "n_rocks": n_rocks,
                    "sim_port": port_base,
                    "ctrl_port": port_base + 1,
                    "idx": idx,
                })
                idx += 1
    return jobs


def run_obstacle_job(job):
    """Run one obstacle avoidance benchmark."""
    tag = job["tag"]
    plot_dir = PLOTS_DIR / "bench_obstacle" / tag
    extra = []
    if job["n_rocks"] > 0:
        extra += ["--rocks", str(job["n_rocks"]),
                   "--rock-seed", "42"]
    wall, diag_csv = _run_sim(
        extra, job["terrain"], job["path"], job["speed"], job["sim_time"],
        str(plot_dir), job["sim_port"], job["ctrl_port"])

    result = {"tag": tag, "terrain": job["terrain"], "speed": job["speed"],
              "n_rocks": job["n_rocks"], "wall_s": wall, "status": "ok"}

    if diag_csv:
        result.update(_parse_diag(diag_csv))
    else:
        result["status"] = "no_csv"
    return result


def plot_obstacle_results(results, out_dir):
    """Generate obstacle avoidance paper figure."""
    import pandas as pd
    df = pd.DataFrame(results)
    df.to_csv(out_dir / "obstacle_avoidance_results.csv", index=False)

    ok = df[df["status"] == "ok"]
    if len(ok) == 0:
        print("  No successful obstacle runs to plot.")
        return

    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5), sharey=True)
    colors = {"clay": "#D35400", "dirt": "#8B4513", "sand": "#DAA520"}
    rock_counts = sorted(ok["n_rocks"].unique())

    for ax_i, speed in enumerate(sorted(ok["speed"].unique())):
        ax = axes[ax_i] if len(axes.shape) == 0 else axes[ax_i]
        sub = ok[ok["speed"] == speed]
        x = np.arange(len(rock_counts))
        w = 0.25
        for t_i, terrain in enumerate(TERRAINS):
            tsub = sub[sub["terrain"] == terrain]
            vals = [tsub[tsub["n_rocks"] == r]["rms_cte_m"].values for r in rock_counts]
            means = [v[0] if len(v) else 0 for v in vals]
            ax.bar(x + t_i * w, means, w, label=terrain, color=colors[terrain], alpha=0.85)
        ax.set_xticks(x + w)
        ax.set_xticklabels([f"{r} rocks" for r in rock_counts])
        ax.set_title(f"v = {speed:.0f} m/s")
        ax.set_ylabel("RMS CTE (m)" if ax_i == 0 else "")
        if ax_i == 0:
            ax.legend()

    # Handle case where only 2 speeds
    if len(axes) > len(ok["speed"].unique()):
        for i in range(len(ok["speed"].unique()), len(axes)):
            axes[i].set_visible(False)

    fig.suptitle("Obstacle Avoidance: Tracking Error vs Rock Count", fontsize=13)
    fig.tight_layout()
    path = out_dir / "obstacle_avoidance.png"
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  Saved {path}")


# ═══════════════════════════════════════════════════════════════════════
# Suite B — Terrain Estimator Benchmark
# ═══════════════════════════════════════════════════════════════════════

def build_terrain_estimator_jobs(base_port_offset=200):
    jobs = []
    idx = 0
    for terrain in TERRAINS:
        for path in PATHS:
            for speed in SPEEDS:
                port_base = BASE_PORT + base_port_offset + idx * 2
                tag = f"te_{terrain}_{path}_v{int(speed)}"
                jobs.append({
                    "tag": tag,
                    "terrain": terrain,
                    "path": path,
                    "speed": speed,
                    "sim_time": 20.0,
                    "sim_port": port_base,
                    "ctrl_port": port_base + 1,
                    "idx": idx,
                })
                idx += 1
    return jobs


def run_terrain_estimator_job(job):
    tag = job["tag"]
    plot_dir = PLOTS_DIR / "bench_terrain_est" / tag
    extra = ["--terrain-estimator"]
    wall, diag_csv = _run_sim(
        extra, job["terrain"], job["path"], job["speed"], job["sim_time"],
        str(plot_dir), job["sim_port"], job["ctrl_port"])

    truth = TERRAIN_TRUTH[job["terrain"]]
    result = {"tag": tag, "terrain": job["terrain"], "path": job["path"],
              "speed": job["speed"], "true_n": truth["n"], "true_phi": truth["phi"],
              "wall_s": wall, "status": "ok"}

    if diag_csv:
        metrics = _parse_diag(diag_csv)
        result.update(metrics)
        # Compute estimation error
        if "est_n_mean" in metrics:
            result["n_error"] = abs(metrics["est_n_mean"] - truth["n"])
            result["n_error_pct"] = 100.0 * result["n_error"] / truth["n"]
    else:
        result["status"] = "no_csv"
    return result


def plot_terrain_estimator_results(results, out_dir):
    import pandas as pd
    df = pd.DataFrame(results)
    df.to_csv(out_dir / "terrain_estimator_results.csv", index=False)

    ok = df[df["status"] == "ok"]
    if len(ok) == 0:
        print("  No successful terrain estimator runs to plot.")
        return

    # Figure 1: Estimated n vs true n for each terrain
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5))
    colors = {"clay": "#D35400", "dirt": "#8B4513", "sand": "#DAA520"}

    for ax_i, terrain in enumerate(TERRAINS):
        ax = axes[ax_i]
        tsub = ok[ok["terrain"] == terrain]
        truth_n = TERRAIN_TRUTH[terrain]["n"]

        if "est_n_mean" in tsub.columns and tsub["est_n_mean"].notna().any():
            # Box plot across paths and speeds
            data_by_path = []
            labels = []
            for path in PATHS:
                psub = tsub[tsub["path"] == path]
                vals = psub["est_n_mean"].dropna().values
                if len(vals):
                    data_by_path.append(vals)
                    labels.append(path.replace("_", "\n"))

            if data_by_path:
                bp = ax.boxplot(data_by_path, labels=labels, patch_artist=True)
                for patch in bp["boxes"]:
                    patch.set_facecolor(colors[terrain])
                    patch.set_alpha(0.6)

        ax.axhline(truth_n, color="red", linestyle="--", linewidth=2, label=f"True n={truth_n}")
        ax.set_title(f"{terrain.capitalize()} (φ={TERRAIN_TRUTH[terrain]['phi']}°)")
        ax.set_ylabel("Estimated n" if ax_i == 0 else "")
        ax.set_ylim(0.3, 1.3)
        ax.legend(fontsize=8)

    fig.suptitle("Online Terrain Parameter Estimation: Bekker n", fontsize=13)
    fig.tight_layout()
    path_fig = out_dir / "terrain_estimator_n.png"
    fig.savefig(path_fig, dpi=200)
    plt.close(fig)
    print(f"  Saved {path_fig}")

    # Figure 2: CTE comparison with/without estimator
    fig, ax = plt.subplots(figsize=(8, 5))
    x = np.arange(len(TERRAINS))
    w = 0.35
    for s_i, speed in enumerate(sorted(ok["speed"].unique())):
        vals = []
        for terrain in TERRAINS:
            sub = ok[(ok["terrain"] == terrain) & (ok["speed"] == speed)]
            vals.append(sub["rms_cte_m"].mean() if len(sub) else 0)
        ax.bar(x + s_i * w, vals, w, label=f"v={speed:.0f} m/s", alpha=0.8)
    ax.set_xticks(x + w / 2)
    ax.set_xticklabels([t.capitalize() for t in TERRAINS])
    ax.set_ylabel("RMS CTE (m)")
    ax.set_title("Tracking with Online Terrain Estimation")
    ax.legend()
    fig.tight_layout()
    path_fig = out_dir / "terrain_estimator_cte.png"
    fig.savefig(path_fig, dpi=200)
    plt.close(fig)
    print(f"  Saved {path_fig}")


# ═══════════════════════════════════════════════════════════════════════
# Suite C — Safety Filter (CBF) Benchmark
# ═══════════════════════════════════════════════════════════════════════

def build_safety_filter_jobs(base_port_offset=400):
    jobs = []
    idx = 0
    configs = [
        # (label_suffix, extra_args)
        ("no_cbf_no_rocks",   []),
        ("cbf_no_rocks",      ["--safety-filter"]),
        ("cbf_4rocks",        ["--safety-filter", "--rocks", "4", "--rock-seed", "42"]),
        ("cbf_8rocks",        ["--safety-filter", "--rocks", "8", "--rock-seed", "42"]),
        ("cbf_4rocks_delay",  ["--safety-filter", "--rocks", "4", "--rock-seed", "42",
                               "--teleop-delay", "0.15"]),
    ]
    for terrain in TERRAINS:
        for speed in SPEEDS:
            for label_sfx, extra in configs:
                port_base = BASE_PORT + base_port_offset + idx * 2
                tag = f"safety_{terrain}_v{int(speed)}_{label_sfx}"
                jobs.append({
                    "tag": tag,
                    "terrain": terrain,
                    "path": "sinusoidal",
                    "speed": speed,
                    "sim_time": 20.0,
                    "extra_args": extra,
                    "label": label_sfx,
                    "sim_port": port_base,
                    "ctrl_port": port_base + 1,
                    "idx": idx,
                })
                idx += 1
    return jobs


def run_safety_filter_job(job):
    tag = job["tag"]
    plot_dir = PLOTS_DIR / "bench_safety" / tag
    wall, diag_csv = _run_sim(
        job["extra_args"], job["terrain"], job["path"], job["speed"],
        job["sim_time"], str(plot_dir), job["sim_port"], job["ctrl_port"])

    result = {"tag": tag, "terrain": job["terrain"], "speed": job["speed"],
              "config": job["label"], "wall_s": wall, "status": "ok"}

    if diag_csv:
        result.update(_parse_diag(diag_csv))
    else:
        result["status"] = "no_csv"
    return result


def plot_safety_filter_results(results, out_dir):
    import pandas as pd
    df = pd.DataFrame(results)
    df.to_csv(out_dir / "safety_filter_results.csv", index=False)

    ok = df[df["status"] == "ok"]
    if len(ok) == 0:
        print("  No successful safety filter runs to plot.")
        return

    configs_ordered = ["no_cbf_no_rocks", "cbf_no_rocks", "cbf_4rocks",
                       "cbf_8rocks", "cbf_4rocks_delay"]
    config_labels = ["No CBF\n0 rocks", "CBF\n0 rocks", "CBF\n4 rocks",
                     "CBF\n8 rocks", "CBF+delay\n4 rocks"]

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    colors = {"clay": "#D35400", "dirt": "#8B4513", "sand": "#DAA520"}

    # Left: RMS CTE by configuration (grouped by terrain)
    ax = axes[0]
    x = np.arange(len(configs_ordered))
    w = 0.25
    for t_i, terrain in enumerate(TERRAINS):
        vals = []
        for cfg in configs_ordered:
            sub = ok[(ok["terrain"] == terrain) & (ok["config"] == cfg)]
            vals.append(sub["rms_cte_m"].mean() if len(sub) else 0)
        ax.bar(x + t_i * w, vals, w, label=terrain, color=colors[terrain], alpha=0.85)
    ax.set_xticks(x + w)
    ax.set_xticklabels(config_labels, fontsize=8)
    ax.set_ylabel("RMS CTE (m)")
    ax.set_title("Tracking Error by Safety Configuration")
    ax.legend()

    # Right: Solver success rate
    ax = axes[1]
    for t_i, terrain in enumerate(TERRAINS):
        vals = []
        for cfg in configs_ordered:
            sub = ok[(ok["terrain"] == terrain) & (ok["config"] == cfg)]
            vals.append(sub["solver_success_pct"].mean() if "solver_success_pct" in sub.columns and len(sub) else 100)
        ax.bar(x + t_i * w, vals, w, label=terrain, color=colors[terrain], alpha=0.85)
    ax.set_xticks(x + w)
    ax.set_xticklabels(config_labels, fontsize=8)
    ax.set_ylabel("Solver Success (%)")
    ax.set_title("ACADOS Solver Success Rate")
    ax.set_ylim(80, 101)
    ax.legend()

    fig.suptitle("DOB-CBF Safety Filter Benchmark", fontsize=13)
    fig.tight_layout()
    path_fig = out_dir / "safety_filter_benchmark.png"
    fig.savefig(path_fig, dpi=200)
    plt.close(fig)
    print(f"  Saved {path_fig}")


# ═══════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════

def run_suite(suite_name, jobs, run_fn, plot_fn, max_workers):
    print(f"\n{'=' * 60}")
    print(f"  Suite: {suite_name}  ({len(jobs)} runs)")
    print(f"{'=' * 60}")

    results = []
    if max_workers <= 1:
        for job in jobs:
            print(f"  [{job['idx'] + 1}/{len(jobs)}] {job['tag']}")
            results.append(run_fn(job))
    else:
        with concurrent.futures.ThreadPoolExecutor(max_workers=max_workers) as pool:
            futures = {pool.submit(run_fn, j): j for j in jobs}
            for future in concurrent.futures.as_completed(futures):
                r = future.result()
                results.append(r)
                status = r.get("status", "?")
                cte = r.get("rms_cte_m", float("nan"))
                print(f"  [done] {r['tag']}  status={status}  rms_cte={cte:.4f}m")

    PAPER_FIGS.mkdir(parents=True, exist_ok=True)
    plot_fn(results, PAPER_FIGS)
    return results


def main():
    p = argparse.ArgumentParser(description="Feature benchmarks for paper figures")
    p.add_argument("-j", "--workers", type=int, default=1,
                   help="Parallel workers (default 1 = sequential)")
    p.add_argument("--suite", default="all",
                   choices=["all", "obstacle", "terrain-estimator", "safety-filter"],
                   help="Which suite to run")
    args = p.parse_args()

    all_results = {}

    if args.suite in ("all", "obstacle"):
        jobs = build_obstacle_jobs()
        all_results["obstacle"] = run_suite(
            "Obstacle Avoidance", jobs,
            run_obstacle_job, plot_obstacle_results,
            args.workers)

    if args.suite in ("all", "terrain-estimator"):
        jobs = build_terrain_estimator_jobs()
        all_results["terrain_estimator"] = run_suite(
            "Terrain Estimator", jobs,
            run_terrain_estimator_job, plot_terrain_estimator_results,
            args.workers)

    if args.suite in ("all", "safety-filter"):
        jobs = build_safety_filter_jobs()
        all_results["safety_filter"] = run_suite(
            "Safety Filter (CBF)", jobs,
            run_safety_filter_job, plot_safety_filter_results,
            args.workers)

    # Summary
    print(f"\n{'=' * 60}")
    print("  All benchmarks complete")
    print(f"{'=' * 60}")
    for suite, results in all_results.items():
        ok = sum(1 for r in results if r.get("status") == "ok")
        print(f"  {suite}: {ok}/{len(results)} successful")
    print(f"  Figures saved to: {PAPER_FIGS}")


if __name__ == "__main__":
    main()
