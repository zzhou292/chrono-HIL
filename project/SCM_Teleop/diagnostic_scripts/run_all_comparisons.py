#!/usr/bin/env python3
"""
Batch runner: simulate every (terrain x path) combo, collect force logs,
and generate comparison plots for each.

Usage:
    python diagnostic_scripts/run_all_comparisons.py [--time 15] [--model ...]
"""

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

TERRAINS = ["clay", "sand", "dirt"]
PATHS = ["lane_change", "double_lane_change", "sinusoidal"]

DIAG_DIR = Path(__file__).resolve().parent
SCM_TELEOP = DIAG_DIR.parent
SIM_SCRIPT = SCM_TELEOP / "simulation" / "dallas_chrono_demo.py"
CMP_SCRIPT = DIAG_DIR / "compare_forces.py"

FORCE_LOG = DIAG_DIR / "tire_force_log.json"
STATE_LOG = DIAG_DIR / "state_history_log.json"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--time", type=float, default=15.0)
    parser.add_argument("--model", default=str(SCM_TELEOP / "nn_models/v6_sweep_16_4/best_terrain_nn.pt"))
    parser.add_argument("--scaler", default=str(SCM_TELEOP / "nn_models/v6_sweep_16_4/scalers.pkl"))
    parser.add_argument("--terrains", nargs="+", default=TERRAINS, choices=TERRAINS)
    parser.add_argument("--paths", nargs="+", default=PATHS, choices=PATHS)
    args = parser.parse_args()

    combos = [(t, p) for t in args.terrains for p in args.paths]
    n = len(combos)
    results = {}

    print(f"Running {n} combos: {len(args.terrains)} terrains x {len(args.paths)} paths")
    print(f"Sim time: {args.time}s  Model: {Path(args.model).name}\n")

    for idx, (terrain, path) in enumerate(combos, 1):
        tag = f"{terrain}_{path}"
        print(f"\n{'='*70}")
        print(f"[{idx}/{n}] terrain={terrain}  path={path}")
        print(f"{'='*70}")

        # --- 1. Run simulation ---
        sim_cmd = [
            sys.executable, str(SIM_SCRIPT),
            "--nn", "--terrain", terrain, "--path", path,
            "--time", str(args.time), "--no-vis", "--debug",
        ]
        ret = subprocess.run(sim_cmd, cwd=str(SCM_TELEOP))
        if ret.returncode != 0:
            print(f"  *** Simulation FAILED for {tag}, skipping ***")
            results[tag] = "FAILED"
            continue

        # --- 2. Copy logs ---
        log_dir = DIAG_DIR / "logs" / tag
        log_dir.mkdir(parents=True, exist_ok=True)
        for src in [FORCE_LOG, STATE_LOG]:
            if src.exists():
                shutil.copy2(src, log_dir / src.name)

        # --- 3. Run compare_forces ---
        save_dir = DIAG_DIR / "plots" / tag
        cmp_cmd = [
            sys.executable, str(CMP_SCRIPT),
            "--model", args.model, "--scaler", args.scaler,
            "--terrain", terrain,
            "--force-log", str(log_dir / "tire_force_log.json"),
            "--state-log", str(log_dir / "state_history_log.json"),
            "--save-dir", str(save_dir),
        ]
        ret2 = subprocess.run(cmp_cmd, cwd=str(SCM_TELEOP), capture_output=True, text=True)
        print(ret2.stdout)
        if ret2.stderr:
            print(ret2.stderr, file=sys.stderr)

        # Extract RMS from simulation output
        results[tag] = "OK"

    # --- Summary ---
    print(f"\n{'='*70}")
    print("BATCH SUMMARY")
    print(f"{'='*70}")
    for tag, status in results.items():
        parts = tag.split("_", 1)
        print(f"  {parts[0]:>8s} / {parts[1]:<20s} : {status}")
    print(f"\nAll plots in: {DIAG_DIR / 'plots'}/")


if __name__ == "__main__":
    main()
