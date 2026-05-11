#!/usr/bin/env python3
"""Benchmark sweep: compare NN-only vs NN+force-residual across scenarios.

Runs each terrain × path × speed combination twice (with and without
force-residual compensation), collects results, and prints a comparison
table.

Usage:
    python sweep_benchmark.py --residual-checkpoint logs/force_residual/force_residual_model.pt
    python sweep_benchmark.py --workers 4 --time 20 --residual-checkpoint <path>
"""

import argparse
import csv
import glob
import itertools
import json
import subprocess
import sys
import os
import re
import numpy as np
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime

ROOT = Path(__file__).resolve().parent.parent
SIM_DIR = ROOT / "simulation"
CONDA_BIN = "/home/kyle/miniconda3/bin/conda"

TERRAINS = ["clay", "dirt", "sand"]
PATHS = ["lane_change", "double_lane_change", "right_left", "sinusoidal"]
SPEEDS = [5.0, 8.0]


def run_one(config: dict) -> dict:
    """Run a single simulation. Returns result dict with metrics."""
    cmd = [
        CONDA_BIN, "run", "--no-capture-output", "-n", "sim",
        "python", str(config["script"]),
        "--terrain", config["terrain"],
        "--path", config["path"],
        "--speed", str(config["speed"]),
        "--time", str(config["time"]),
        "--no-vis",
        "--no-plot",
        "--model", "nn",
        "--nn-model", config["nn_model"],
        "--sim-port", str(config["sim_port"]),
        "--ctrl-port", str(config["ctrl_port"]),
        "--plot-dir", config["plot_dir"],
    ]
    if config.get("residual_checkpoint"):
        cmd += [
            "--force-residual",
            "--force-residual-checkpoint", config["residual_checkpoint"],
            "--force-residual-gain", str(config.get("residual_gain", 1.0)),
            "--force-residual-clip", str(config.get("residual_clip", 500.0)),
        ]

    label = f"{config['terrain']}_{config['path']}_v{config['speed']}_{config['mode']}"
    replica = config.get("replica", 0)
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=config["timeout"],
            cwd=str(config["script"].parent),
        )
        if result.returncode != 0:
            return {
                "label": label, "mode": config["mode"],
                "terrain": config["terrain"], "path": config["path"],
                "speed": config["speed"], "replica": replica, "success": False,
                "error": result.stderr[-300:] if result.stderr else "unknown",
            }
        # Parse metrics from the newest CSV in the output directory
        metrics = _extract_metrics(config["plot_dir"], config["terrain"],
                                   config["path"], config.get("rms_time_start", 2.0))
        metrics.update({
            "label": label, "mode": config["mode"],
            "terrain": config["terrain"], "path": config["path"],
            "speed": config["speed"], "replica": replica, "success": True,
        })
        return metrics
    except subprocess.TimeoutExpired:
        return {"label": label, "mode": config["mode"], "success": False, "error": "TIMEOUT",
                "terrain": config["terrain"], "path": config["path"], "speed": config["speed"],
                "replica": replica}
    except Exception as e:
        return {"label": label, "mode": config["mode"], "success": False, "error": str(e),
                "terrain": config["terrain"], "path": config["path"], "speed": config["speed"],
                "replica": replica}


def _extract_metrics(plot_dir: str, terrain: str, path: str,
                     rms_time_start: float = 2.0) -> dict:
    """Extract tracking metrics from the most recent matching CSV."""
    pattern = f"{plot_dir}/*_{terrain}_{path}_*/diag_*.csv"
    csvs = sorted(glob.glob(pattern))
    if not csvs:
        return {"rms_cte": float("nan"), "mean_speed": float("nan"),
                "solver_ok_pct": float("nan")}
    csv_path = csvs[-1]  # most recent
    try:
        import pandas as pd
        df = pd.read_csv(csv_path)
        mask = df["sim_time"] >= rms_time_start
        d = df[mask]
        rms_cte = float(np.sqrt(np.mean(d["crosstrack_err"].values ** 2)))
        mean_speed = float(d["u_meas"].mean())
        solver_ok = float(100.0 * (d["solver_status"] == 0).mean())
        heading_err = float(np.sqrt(np.mean(d["heading_err_deg"].values ** 2)))
        speed_err = float(np.sqrt(np.mean(d["speed_err"].values ** 2)))
        return {
            "rms_cte": rms_cte, "mean_speed": mean_speed,
            "solver_ok_pct": solver_ok, "rms_heading_err": heading_err,
            "rms_speed_err": speed_err, "csv": csv_path,
        }
    except Exception as e:
        return {"rms_cte": float("nan"), "mean_speed": float("nan"),
                "solver_ok_pct": float("nan"), "error": str(e)}


def main():
    parser = argparse.ArgumentParser(description="Benchmark: NN vs NN+residual")
    parser.add_argument("--residual-checkpoint", required=True,
                        help="Path to force-residual model checkpoint")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--time", type=float, default=20.0)
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--nn-model", default="paper_v2_mlp_rate_16_8")
    parser.add_argument("--terrains", nargs="+", default=TERRAINS)
    parser.add_argument("--paths", nargs="+", default=PATHS)
    parser.add_argument("--speeds", nargs="+", type=float, default=SPEEDS)
    parser.add_argument("--base-port", type=int, default=7000)
    parser.add_argument("--force-residual-gain", type=float, default=1.0,
                        help="Output scaling for force residual")
    parser.add_argument("--force-residual-clip", type=float, default=500.0,
                        help="Symmetric clip on ΔFy corrections (N)")
    parser.add_argument("--replicas", type=int, default=1,
                        help="Number of replicas per scenario (for statistical significance)")
    parser.add_argument("--output", default=None,
                        help="JSON file for results (default: benchmark_results_<timestamp>.json)")
    args = parser.parse_args()

    if not Path(args.residual_checkpoint).exists():
        print(f"ERROR: checkpoint not found: {args.residual_checkpoint}", file=sys.stderr)
        sys.exit(1)

    script = SIM_DIR / "launch_decoupled.py"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    # Build all configurations: each scenario × 2 modes × N replicas
    combos = list(itertools.product(args.terrains, args.paths, args.speeds))
    configs = []
    for rep in range(args.replicas):
        rep_suffix = f"_r{rep}" if args.replicas > 1 else ""
        plot_dir_nn = f"benchmark_{timestamp}_nn{rep_suffix}"
        plot_dir_resid = f"benchmark_{timestamp}_nn_resid{rep_suffix}"
        for i, (terrain, path, speed) in enumerate(combos):
            # 4 ports per combo per replica: nn_sim, nn_ctrl, resid_sim, resid_ctrl
            port_offset = (rep * len(combos) + i) * 4
            base_cfg = {
                "script": script,
                "terrain": terrain,
                "path": path,
                "speed": speed,
                "time": args.time,
                "timeout": args.timeout,
                "nn_model": args.nn_model,
                "replica": rep,
            }
            # NN-only
            cfg_nn = {**base_cfg,
                      "mode": "nn",
                      "plot_dir": plot_dir_nn,
                      "residual_checkpoint": None,
                      "sim_port": args.base_port + port_offset,
                      "ctrl_port": args.base_port + port_offset + 1}
            configs.append(cfg_nn)
            # NN + residual
            cfg_resid = {**base_cfg,
                         "mode": "nn+resid",
                         "plot_dir": plot_dir_resid,
                         "residual_checkpoint": args.residual_checkpoint,
                         "residual_gain": args.force_residual_gain,
                         "residual_clip": args.force_residual_clip,
                         "sim_port": args.base_port + port_offset + 2,
                         "ctrl_port": args.base_port + port_offset + 3}
            configs.append(cfg_resid)

    n_replicas = args.replicas
    print(f"Benchmark: {len(configs)} runs ({len(combos)} scenarios × 2 modes × {n_replicas} replicas)")
    print(f"  Workers: {args.workers}")
    print(f"  Checkpoint: {args.residual_checkpoint}")
    print(f"  Gain: {args.force_residual_gain}, Clip: {args.force_residual_clip}")
    print()

    # Create all plot dirs
    for cfg in configs:
        Path(cfg["plot_dir"]).mkdir(parents=True, exist_ok=True)

    results = []
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures = {pool.submit(run_one, cfg): cfg for cfg in configs}
        for future in as_completed(futures):
            r = future.result()
            status = "OK" if r.get("success") else "FAIL"
            cte_str = f"CTE={r.get('rms_cte', 'N/A'):.3f}" if r.get("success") else r.get("error", "?")[:40]
            rep_str = f"r{r.get('replica', 0)}" if n_replicas > 1 else ""
            print(f"  [{status}] {r['label']:50s} {rep_str:>3s} {cte_str}")
            results.append(r)

    # --- Aggregate results across replicas ---
    from collections import defaultdict
    nn_by_scenario = defaultdict(list)   # (terrain, path, speed) -> [rms_cte, ...]
    resid_by_scenario = defaultdict(list)

    for r in results:
        if not r.get("success"):
            continue
        key = (r["terrain"], r["path"], r["speed"])
        cte = r.get("rms_cte", float("nan"))
        if not np.isfinite(cte):
            continue
        if r["mode"] == "nn":
            nn_by_scenario[key].append(r)
        else:
            resid_by_scenario[key].append(r)

    # Print comparison table
    W = 120
    print("\n" + "=" * W)
    if n_replicas > 1:
        print(f"{'Scenario':<35s} {'NN CTE':>10s} {'Resid CTE':>12s} {'Δ%':>8s} "
              f"{'p-val':>7s} {'n':>3s} {'NN Spd':>7s} {'R Spd':>7s}")
    else:
        print(f"{'Scenario':<35s} {'NN CTE':>8s} {'Resid CTE':>10s} {'Δ%':>7s} "
              f"{'NN Spd':>7s} {'R Spd':>7s} {'NN Sol%':>7s} {'R Sol%':>7s}")
    print("-" * W)

    all_keys = sorted(set(list(nn_by_scenario.keys()) + list(resid_by_scenario.keys())))
    all_nn_ctes, all_resid_ctes = [], []
    paired_nn, paired_resid = [], []

    for key in all_keys:
        nn_list = nn_by_scenario.get(key, [])
        rs_list = resid_by_scenario.get(key, [])
        label = f"{key[0]}_{key[1]}_v{key[2]}"

        nn_ctes = [r["rms_cte"] for r in nn_list]
        rs_ctes = [r["rms_cte"] for r in rs_list]

        if not nn_ctes or not rs_ctes:
            print(f"{label:<35s} {'INCOMPLETE':>10s}")
            continue

        nn_mean = np.mean(nn_ctes)
        rs_mean = np.mean(rs_ctes)
        delta_pct = 100.0 * (rs_mean - nn_mean) / nn_mean if nn_mean > 0 else float("nan")

        all_nn_ctes.extend(nn_ctes)
        all_resid_ctes.extend(rs_ctes)
        paired_nn.append(nn_mean)
        paired_resid.append(rs_mean)

        if n_replicas > 1:
            nn_std = np.std(nn_ctes, ddof=1) if len(nn_ctes) > 1 else 0
            rs_std = np.std(rs_ctes, ddof=1) if len(rs_ctes) > 1 else 0
            # Paired t-test (or Wilcoxon) if enough replicas
            n_pairs = min(len(nn_ctes), len(rs_ctes))
            if n_pairs >= 3:
                from scipy import stats
                t_stat, p_val = stats.ttest_rel(nn_ctes[:n_pairs], rs_ctes[:n_pairs])
            elif n_pairs >= 2:
                from scipy import stats
                t_stat, p_val = stats.ttest_ind(nn_ctes, rs_ctes)
            else:
                p_val = float("nan")
            nn_spd = np.mean([r.get("mean_speed", float("nan")) for r in nn_list])
            rs_spd = np.mean([r.get("mean_speed", float("nan")) for r in rs_list])
            p_str = f"{p_val:.3f}" if np.isfinite(p_val) else "N/A"
            sig = "*" if np.isfinite(p_val) and p_val < 0.05 else ""
            print(f"{label:<35s} {nn_mean:7.3f}±{nn_std:.3f} {rs_mean:8.3f}±{rs_std:.3f} "
                  f"{delta_pct:+7.1f}{sig} {p_str:>7s} {n_pairs:3d} {nn_spd:7.2f} {rs_spd:7.2f}")
        else:
            nn_r = nn_list[0]
            rs_r = rs_list[0]
            nn_spd = nn_r.get("mean_speed", float("nan"))
            rs_spd = rs_r.get("mean_speed", float("nan"))
            nn_sol = nn_r.get("solver_ok_pct", float("nan"))
            rs_sol = rs_r.get("solver_ok_pct", float("nan"))
            print(f"{label:<35s} {nn_mean:8.3f} {rs_mean:10.3f} {delta_pct:+7.1f} "
                  f"{nn_spd:7.2f} {rs_spd:7.2f} {nn_sol:7.1f} {rs_sol:7.1f}")

    if paired_nn:
        avg_nn = np.mean(paired_nn)
        avg_rs = np.mean(paired_resid)
        avg_delta = 100.0 * (avg_rs - avg_nn) / avg_nn
        print("-" * W)
        if n_replicas > 1 and len(paired_nn) >= 3:
            from scipy import stats
            t_stat, p_val = stats.ttest_rel(paired_nn, paired_resid)
            sig = "*" if p_val < 0.05 else ""
            print(f"{'MEAN ACROSS SCENARIOS':<35s} {avg_nn:10.3f} {avg_rs:12.3f} "
                  f"{avg_delta:+7.1f}{sig} p={p_val:.3f}")
        else:
            print(f"{'AVERAGE':<35s} {avg_nn:8.3f} {avg_rs:10.3f} {avg_delta:+7.1f}")
    print("=" * W)

    # Save results
    output_path = args.output or f"benchmark_results_{timestamp}.json"
    with open(output_path, "w") as f:
        # Convert nan to null for JSON
        def clean(v):
            if isinstance(v, float) and not np.isfinite(v):
                return None
            return v
        json.dump([{k: clean(v) for k, v in r.items()} for r in results],
                  f, indent=2)
    print(f"\nResults saved to {output_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
