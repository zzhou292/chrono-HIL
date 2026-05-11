#!/usr/bin/env python3
"""
Unified paper benchmark suite.

Runs three comparisons across all terrains × paths:
  1. Analytical models (pacejka, tmeasy) vs best NN
  2. All working static NN models (MLPs + ResNets) — paper_v2
  3. All NN model types including rate (solver success comparison) — paper_v2

Each (model, terrain, path, speed) combination runs once (default speeds:
5 and 8 m/s).  Results are collected from per-run diagnostic CSVs and
written to a single JSON + summary CSV.

Usage:
    # Full suite (all 3 comparisons, all terrains × paths × default speeds)
    python run_paper_benchmarks.py -j 4

    # Only comparison 1 (NN vs analytical)
    python run_paper_benchmarks.py --suite analytical -j 4

    # Only comparison 2 (static NN scaling)
    python run_paper_benchmarks.py --suite nn-static -j 4

    # Only comparison 3 (solver success across all types)
    python run_paper_benchmarks.py --suite solver-success -j 4

    # Single speed (overrides default multi-speed sweep)
    python run_paper_benchmarks.py --speed 7.0 -j 4

    # Explicit speed list
    python run_paper_benchmarks.py --speeds 5 6 8 -j 4
"""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import json
import math
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

UTIL_DIR = Path(__file__).parent
PROJECT_ROOT = UTIL_DIR.parent
SIM_DIR = PROJECT_ROOT / "simulation"
PLOTS_DIR = SIM_DIR / "plots"
BENCH_ROOT = PLOTS_DIR / "paper_benchmark"
CONDA_BIN = "/home/kyle/miniconda3/bin/conda"

# Default target speeds (m/s) for the full grid sweep
DEFAULT_BENCHMARK_SPEEDS_MPS = (5.0, 8.0)

# ── Terrains ──────────────────────────────────────────────────────────
TERRAINS = ["sand", "clay", "dirt"]

# ── Paths (name → extra CLI args) ────────────────────────────────────
PATHS = {
    "lane_change":        {"path": "lane_change"},
    "double_lane_change": {"path": "double_lane_change"},
    "right_left":         {"path": "right_left"},
    "sinusoidal":         {"path": "sinusoidal",
                           "sine_amplitude": "2.0",
                           "sine_wavelength": "30.0"},
}

# ── Model sets ────────────────────────────────────────────────────────
# Comparison 1: analytical vs best NN
ANALYTICAL_MODELS = {
    "pacejka":  {"model": "pacejka"},
    "tmeasy":   {"model": "tmeasy"},
}

# Comparison 2: static NN architectures (all work in MPC)
STATIC_NN_MODELS = {
    "mlp_12_2":      "paper_v2_mlp_12_2",
    "mlp_16_4":      "paper_v2_mlp_16_4",
    "mlp_16_8":      "paper_v2_mlp_16_8",
    "mlp_24_12":     "paper_v2_mlp_24_12",
    "mlp_32_16":     "paper_v2_mlp_32_16",
    "resnet_h8_b2":  "paper_v2_resnet_h8_b2",
    "resnet_h16_b2": "paper_v2_resnet_h16_b2",
    "resnet_h16_b4": "paper_v2_resnet_h16_b4",
    "resnet_h32_b2": "paper_v2_resnet_h32_b2",
}

# Comparison 3: ALL v2 types including rate (solver success table)
ALL_NN_MODELS = {
    # Static MLP
    "mlp_12_2":             "paper_v2_mlp_12_2",
    "mlp_16_4":             "paper_v2_mlp_16_4",
    "mlp_16_8":             "paper_v2_mlp_16_8",
    "mlp_24_12":            "paper_v2_mlp_24_12",
    "mlp_32_16":            "paper_v2_mlp_32_16",
    # Static ResNet
    "resnet_h8_b2":         "paper_v2_resnet_h8_b2",
    "resnet_h16_b2":        "paper_v2_resnet_h16_b2",
    "resnet_h16_b4":        "paper_v2_resnet_h16_b4",
    "resnet_h32_b2":        "paper_v2_resnet_h32_b2",
    # Rate MLP
    "rate_mlp_12_2":        "paper_v2_mlp_rate_12_2",
    "rate_mlp_16_4":        "paper_v2_mlp_rate_16_4",
    "rate_mlp_16_8":        "paper_v2_mlp_rate_16_8",
    "rate_mlp_24_12":       "paper_v2_mlp_rate_24_12",
    "rate_mlp_32_16":       "paper_v2_mlp_rate_32_16",
    # Rate ResNet
    "rate_resnet_h8_b2":    "paper_v2_resnet_rate_h8_b2",
    "rate_resnet_h16_b2":   "paper_v2_resnet_rate_h16_b2",
    "rate_resnet_h16_b4":   "paper_v2_resnet_rate_h16_b4",
    "rate_resnet_h32_b2":   "paper_v2_resnet_rate_h32_b2",
}


# ── Path extent for stall detection ──────────────────────────────────
def reference_path_x_end(path_type, lead_in=10.0, sine_wl=30.0):
    if path_type == "lane_change":
        return 25.0 + lead_in + 40.0
    if path_type == "double_lane_change":
        return 38.0 + lead_in + 40.0
    if path_type == "right_left":
        return 48.0 + lead_in + 40.0
    if path_type == "sinusoidal":
        return lead_in + 5.0 * sine_wl
    return 200.0


# ── CSV parsing ───────────────────────────────────────────────────────
def _float_col(rows, key):
    out = []
    for r in rows:
        v = r.get(key, "")
        if v is None or v == "":
            out.append(np.nan)
        else:
            try:
                out.append(float(v))
            except ValueError:
                out.append(np.nan)
    return np.asarray(out, dtype=float)


def parse_diagnostic_csv(csv_path, common):
    """Parse a diagnostic CSV and compute benchmark metrics."""
    rows = []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    if not rows:
        return {}

    lead_in = float(common.get("lead_in", "10.0"))
    sine_wl = float(common.get("sine_wavelength", "30.0"))
    path_type = common["path"]

    cte = np.array([float(r.get("crosstrack_err", 0)) for r in rows])
    solver_status = [r.get("solver_status", "") for r in rows]
    solve_times = np.array(
        [float(r.get("solve_time_ms", 0)) / 1000.0
         for r in rows if r.get("solve_time_ms")])
    sim_times = np.array([float(r.get("sim_time", 0)) for r in rows])

    mask = sim_times >= 2.0
    cte_m = cte[mask] if mask.any() else cte

    total = len(solver_status)
    success_count = sum(1 for s in solver_status if s in ("0", "2"))
    maxiter_count = sum(1 for s in solver_status if s == "2")

    metrics = {
        "n_solves": total,
        "avg_cte_m": float(np.mean(np.abs(cte_m))),
        "max_cte_m": float(np.max(np.abs(cte_m))) if len(cte_m) else 0.0,
        "rms_cte_m": float(np.sqrt(np.mean(cte_m ** 2))),
        "success_pct": 100.0 * success_count / max(total, 1),
        "maxiter_pct": 100.0 * maxiter_count / max(total, 1),
    }

    if len(solve_times) > 0:
        metrics["mean_solve_ms"] = float(np.mean(solve_times) * 1000)

    u_true = _float_col(rows, "u_true")
    v_ref0 = _float_col(rows, "v_ref_0")
    heading_err = _float_col(rows, "heading_err_deg")
    if mask.any():
        u_m = u_true[mask]
        he_m = heading_err[mask]
    else:
        u_m = u_true
        he_m = heading_err

    if np.any(np.isfinite(u_m)):
        mu = float(np.nanmean(u_m))
        metrics["mean_speed_mps"] = mu
        vref = float(v_ref0[0]) if len(v_ref0) and np.isfinite(v_ref0[0]) else np.nan
        if np.isfinite(vref) and vref > 1e-6:
            metrics["speed_ratio"] = float(mu / vref)

    if np.any(np.isfinite(he_m)):
        metrics["rms_heading_deg"] = float(np.sqrt(np.nanmean(he_m ** 2)))

    # Stall detection
    x_true = _float_col(rows, "x_fa_true")
    x_end = reference_path_x_end(path_type, lead_in, sine_wl)
    if len(x_true) and np.any(np.isfinite(x_true)):
        final_x = float(x_true[-1]) if np.isfinite(x_true[-1]) else float(np.nanmax(x_true))
    else:
        final_x = 0.0
    progress = final_x / x_end if x_end > 1e-6 else 0.0
    last_t = float(sim_times[-1]) if len(sim_times) else 0.0
    stalled = last_t >= 8.0 and progress < 0.48
    metrics["progress_frac"] = progress
    metrics["stalled"] = stalled

    return metrics


# ── Find run output directory ─────────────────────────────────────────
def find_run_dir(created_after, search_root, terrain, path_type):
    if not search_root.is_dir():
        return None
    candidates = sorted(
        [d for d in search_root.iterdir()
         if d.is_dir()
         and terrain in d.name
         and path_type in d.name
         and d.stat().st_mtime > created_after],
        key=lambda d: d.stat().st_mtime, reverse=True)
    return candidates[0] if candidates else None


def _speed_dir_tag(speed: float) -> str:
    """Folder-safe tag for plot output (e.g. v5, v8, v6p5)."""
    if abs(speed - round(speed)) < 1e-6:
        return f"v{int(round(speed))}"
    return "v" + repr(float(speed)).replace(".", "p")


# ── Run a single simulation ───────────────────────────────────────────
def run_one(job):
    """Launch one sim + controller, parse diagnostic CSV, return metrics."""
    label = job["label"]
    terrain = job["terrain"]
    path_name = job["path_name"]
    path_args = job["path_args"]
    model_cfg = job["model_cfg"]
    sim_port = job["sim_port"]
    ctrl_port = job["ctrl_port"]
    speed = job["speed"]
    sim_time = job["sim_time"]
    lead_in = job["lead_in"]
    idx = job["idx"]
    total = job["total"]

    tag = f"{label}__{terrain}__{path_name}__{_speed_dir_tag(speed)}"
    plot_dir = BENCH_ROOT / tag

    print(f"[{idx + 1}/{total}] {tag}  (ports {sim_port}/{ctrl_port})")

    cmd = [
        CONDA_BIN, "run", "--no-capture-output", "-n", "sim", "python",
        str(SIM_DIR / "launch_decoupled.py"),
        "--path", path_args["path"],
        "--terrain", terrain,
        "--time", str(sim_time),
        "--speed", str(speed),
        "--lead-in", str(lead_in),
        "--no-vis",
        "--sim-port", str(sim_port),
        "--ctrl-port", str(ctrl_port),
        "--plot-dir", str(plot_dir),
    ]

    # Sinusoidal-specific args
    if "sine_amplitude" in path_args:
        cmd += ["--sine-amplitude", path_args["sine_amplitude"]]
    if "sine_wavelength" in path_args:
        cmd += ["--sine-wavelength", path_args["sine_wavelength"]]

    # Model type
    model_type = model_cfg.get("model", "nn")
    cmd += ["--model", model_type]
    if model_type == "nn":
        cmd += ["--nn-model", model_cfg["nn_model"]]

    t0 = time.time()
    try:
        proc = subprocess.run(cmd, timeout=600, cwd=str(SIM_DIR),
                              capture_output=True, text=True)
        wall = time.time() - t0
    except subprocess.TimeoutExpired:
        print(f"  TIMEOUT: {tag}")
        return {"label": label, "terrain": terrain, "path": path_name,
                "target_speed_mps": speed, "status": "timeout"}

    if proc.returncode != 0:
        # Check for common failure patterns in stderr
        stderr_tail = (proc.stderr or "")[-500:]
        print(f"  FAILED (exit {proc.returncode}): {tag}")
        if stderr_tail:
            print(f"    stderr: {stderr_tail[:200]}")
        return {"label": label, "terrain": terrain, "path": path_name,
                "target_speed_mps": speed, "status": f"exit_{proc.returncode}"}

    # Find diagnostic CSV
    run_dir = find_run_dir(t0, plot_dir, terrain, path_args["path"])
    if not run_dir:
        stderr_tail = (proc.stderr or "")[-300:].strip()
        print(f"  NO OUTPUT: {tag}  (wall={wall:.0f}s)")
        if stderr_tail:
            print(f"    stderr: {stderr_tail[:200]}")
        return {"label": label, "terrain": terrain, "path": path_name,
                "target_speed_mps": speed, "status": "no_output", "wall_s": round(wall, 1)}

    diag_csvs = sorted(run_dir.glob("diag_*.csv"))
    if not diag_csvs:
        print(f"  NO CSV: {tag}")
        return {"label": label, "terrain": terrain, "path": path_name,
                "target_speed_mps": speed, "status": "no_csv"}

    common = {
        "path": path_args["path"],
        "lead_in": str(lead_in),
        "sine_wavelength": path_args.get("sine_wavelength", "30.0"),
    }
    metrics = parse_diagnostic_csv(diag_csvs[-1], common)

    result = {
        "label": label,
        "terrain": terrain,
        "path": path_name,
        "target_speed_mps": speed,
        "status": "ok",
        "wall_s": round(wall, 1),
        "run_dir": str(run_dir),
    }
    result.update(metrics)

    rms = metrics.get("rms_cte_m")
    ok = metrics.get("success_pct")
    ms = metrics.get("mean_solve_ms")
    rms_s = f"{rms:.3f}" if rms is not None else "?"
    ok_s = f"{ok:.0f}" if ok is not None else "?"
    ms_s = f"{ms:.1f}" if ms is not None else "?"
    print(f"  OK: RMS_CTE={rms_s}m  Solver={ok_s}%  Solve={ms_s}ms  ({wall:.0f}s)")
    return result


# ── Build job list ────────────────────────────────────────────────────
def build_jobs(suites, speeds, sim_time, lead_in, base_port):
    """Build flat list of jobs for the requested suites."""
    jobs = []
    seen = set()

    def _add(label, terrain, path_name, path_args, model_cfg, speed):
        key = (label, terrain, path_name, speed)
        if key in seen:
            return
        seen.add(key)
        jobs.append({
            "label": label,
            "terrain": terrain,
            "path_name": path_name,
            "path_args": path_args,
            "model_cfg": model_cfg,
            "speed": speed,
            "sim_time": sim_time,
            "lead_in": lead_in,
        })

    if "analytical" in suites or "all" in suites:
        for name, cfg in ANALYTICAL_MODELS.items():
            for terrain in TERRAINS:
                for pname, pargs in PATHS.items():
                    for spd in speeds:
                        _add(name, terrain, pname, pargs, cfg, spd)

    if "nn-static" in suites or "all" in suites:
        for name, nn_model in STATIC_NN_MODELS.items():
            cfg = {"model": "nn", "nn_model": nn_model}
            for terrain in TERRAINS:
                for pname, pargs in PATHS.items():
                    for spd in speeds:
                        _add(name, terrain, pname, pargs, cfg, spd)

    if "solver-success" in suites or "all" in suites:
        for name, nn_model in ALL_NN_MODELS.items():
            cfg = {"model": "nn", "nn_model": nn_model}
            for terrain in TERRAINS:
                for pname, pargs in PATHS.items():
                    for spd in speeds:
                        _add(name, terrain, pname, pargs, cfg, spd)

    # Assign ports and indices
    for i, job in enumerate(jobs):
        job["idx"] = i
        job["total"] = len(jobs)
        job["sim_port"] = base_port + i * 10
        job["ctrl_port"] = base_port + i * 10 + 1

    return jobs


# ── Summary tables ────────────────────────────────────────────────────
def write_summary_csv(results, out_path):
    """Write results to CSV for easy import into LaTeX / pandas."""
    if not results:
        return
    fields = [
        "label", "terrain", "path", "target_speed_mps", "status",
        "rms_cte_m", "avg_cte_m", "max_cte_m",
        "rms_heading_deg", "mean_speed_mps", "speed_ratio",
        "success_pct", "maxiter_pct", "mean_solve_ms",
        "progress_frac", "stalled", "wall_s",
    ]
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for r in sorted(results, key=lambda x: (
                x["label"], x["terrain"], x["path"],
                x.get("target_speed_mps", 0) or 0)):
            w.writerow(r)
    print(f"\nCSV summary → {out_path}")


def print_comparison_table(results, title, group_by="label"):
    """Print a formatted comparison table to stdout."""
    ok = [r for r in results if r.get("status") == "ok"]
    if not ok:
        print(f"\n{title}: no successful runs")
        return

    print(f"\n{'=' * 80}")
    print(f"  {title}")
    print(f"{'=' * 80}")

    # Group by model, then by (terrain, path)
    from collections import defaultdict
    grouped = defaultdict(list)
    for r in ok:
        grouped[r[group_by]].append(r)

    header = (
        f"{'Model':<22s} {'Terrain':<6s} {'Path':<18s} {'v':>4s} "
        f"{'RMS CTE':>8s} {'Hdg°':>6s} {'Solver%':>8s} {'ms':>6s} {'Spd%':>5s}"
    )
    print(header)
    print("-" * len(header))

    for model in sorted(grouped.keys()):
        runs = sorted(grouped[model], key=lambda x: (
            x["terrain"], x["path"], x.get("target_speed_mps", 0) or 0))
        for r in runs:
            rms = f"{r.get('rms_cte_m', 0):.3f}" if "rms_cte_m" in r else "---"
            hdg = f"{r.get('rms_heading_deg', 0):.1f}" if "rms_heading_deg" in r else "---"
            sol = f"{r.get('success_pct', 0):.0f}" if "success_pct" in r else "---"
            ms = f"{r.get('mean_solve_ms', 0):.1f}" if "mean_solve_ms" in r else "---"
            spd = f"{r.get('speed_ratio', 0) * 100:.0f}" if "speed_ratio" in r else "---"
            stall = " STALL" if r.get("stalled") else ""
            vm = r.get("target_speed_mps")
            vs = f"{vm:g}" if vm is not None else "--"
            print(f"{model:<22s} {r['terrain']:<6s} {r['path']:<18s} {vs:>4s} "
                  f"{rms:>8s} {hdg:>6s} {sol:>8s} {ms:>6s} {spd:>5s}{stall}")


# ── Main ──────────────────────────────────────────────────────────────
def main():
    p = argparse.ArgumentParser(
        description="Unified paper benchmark suite",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--suite", nargs="+",
                   default=["all"],
                   choices=["all", "analytical", "nn-static", "solver-success"],
                   help="Which comparison(s) to run")
    p.add_argument("-j", "--workers", type=int, default=1,
                   help="Parallel workers (default 1 = sequential)")
    p.add_argument(
        "--speeds",
        type=float,
        nargs="+",
        default=None,
        metavar="MPS",
        help="Target speeds (m/s); one job per speed × terrain × path × model "
             f"(default: {list(DEFAULT_BENCHMARK_SPEEDS_MPS)})",
    )
    p.add_argument(
        "--speed",
        type=float,
        default=None,
        help="If set, run only this target speed (overrides --speeds)",
    )
    p.add_argument("--time", type=float, default=30.0,
                   help="Simulation time in seconds")
    p.add_argument("--lead-in", type=float, default=10.0)
    p.add_argument("--base-port", type=int, default=6600,
                   help="Base ZMQ port (each job uses port + i*10)")
    p.add_argument("--dry-run", action="store_true",
                   help="Print job list without running")
    args = p.parse_args()

    if args.speed is not None:
        speeds = [float(args.speed)]
    elif args.speeds is not None:
        speeds = list(args.speeds)
    else:
        speeds = list(DEFAULT_BENCHMARK_SPEEDS_MPS)
    jobs = build_jobs(args.suite, speeds, args.time, args.lead_in,
                      args.base_port)

    print(f"Paper benchmark: {len(jobs)} simulations")
    print(f"  Suites:   {args.suite}")
    print(f"  Terrains: {TERRAINS}")
    print(f"  Paths:    {list(PATHS.keys())}")
    print(f"  Speeds:   {speeds} m/s")
    print(f"  Lead-in:  {args.lead_in} m")
    print(f"  Workers:  {args.workers}")
    print()

    if args.dry_run:
        for j in jobs:
            print(f"  {j['label']:22s}  {j['terrain']:6s}  {j['path_name']:18s}  "
                  f"v={j['speed']:g}m/s  ports {j['sim_port']}/{j['ctrl_port']}")
        print(f"\nTotal: {len(jobs)} simulations")
        return

    BENCH_ROOT.mkdir(parents=True, exist_ok=True)
    t_start = time.time()

    # Run simulations
    results = []
    if args.workers <= 1:
        for job in jobs:
            results.append(run_one(job))
    else:
        with concurrent.futures.ProcessPoolExecutor(
                max_workers=args.workers) as pool:
            futures = {pool.submit(run_one, j): j for j in jobs}
            for fut in concurrent.futures.as_completed(futures):
                try:
                    results.append(fut.result())
                except Exception as e:
                    job = futures[fut]
                    print(f"  EXCEPTION {job['label']}/{job['terrain']}/{job['path_name']}/"
                          f"v={job['speed']}: {e}")
                    results.append({
                        "label": job["label"],
                        "terrain": job["terrain"],
                        "path": job["path_name"],
                        "target_speed_mps": job["speed"],
                        "status": "exception",
                    })

    elapsed = time.time() - t_start
    ok_count = sum(1 for r in results if r.get("status") == "ok")
    print(f"\nCompleted {ok_count}/{len(results)} in {elapsed / 60:.1f} min")

    # Write outputs
    json_path = BENCH_ROOT / "paper_benchmark_results.json"
    with open(json_path, "w") as f:
        json.dump(results, f, indent=2, default=str)
    print(f"JSON → {json_path}")

    csv_path = BENCH_ROOT / "paper_benchmark_results.csv"
    write_summary_csv(results, csv_path)

    # Print comparison tables — only for suites that were actually requested
    suites_run = set(args.suite)

    analytical_labels = set(ANALYTICAL_MODELS.keys())
    static_labels = set(STATIC_NN_MODELS.keys())
    all_labels = set(ALL_NN_MODELS.keys())

    if "analytical" in suites_run or "all" in suites_run:
        analytical_results = [r for r in results if r["label"] in analytical_labels]
        if analytical_results:
            print_comparison_table(analytical_results,
                                   "Comparison 1: Analytical vs NN")

    if "nn-static" in suites_run or "all" in suites_run:
        static_results = [r for r in results if r["label"] in static_labels]
        if static_results:
            print_comparison_table(static_results,
                                   "Comparison 2: Static NN Architecture Scaling")

    if "solver-success" in suites_run or "all" in suites_run:
        solver_results = [r for r in results if r["label"] in all_labels]
        if solver_results:
            print_comparison_table(solver_results,
                                   "Comparison 3: Solver Success (All Input Types)")


if __name__ == "__main__":
    main()
