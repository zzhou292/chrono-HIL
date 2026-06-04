#!/usr/bin/env python3
"""
Automated Training Data Collection
====================================

Launches multiple chrono_sim_node.py + collect_data.py runs across terrain
presets (clay, sand, dirt), paths (lane_change, double_lane_change, sinusoidal),
and bumpiness levels to build a diverse training dataset.

Usage:
    python -m terrain_classifier.launch_data_collection --runs-per-combo 3 --time 20

This produces one merged CSV: terrain_classifier/data/training_data.csv
"""

import argparse
import itertools
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


TERRAINS = ["clay", "sand", "dirt"]
PATHS = ["lane_change", "double_lane_change", "sinusoidal"]
BUMPINESS_LEVELS = [0, 3, 6]   # flat, mild, rough
SPEEDS = [5.0, 8.0]            # m/s


def main():
    p = argparse.ArgumentParser(description="Automated terrain classification data collection")
    p.add_argument("--time", type=float, default=20.0, help="Sim duration per run (s)")
    p.add_argument("--runs-per-combo", type=int, default=1,
                   help="Runs per (terrain, path, bumpiness, speed) combo")
    p.add_argument("--output-dir", default="terrain_classifier/data",
                   help="Directory for per-run CSVs")
    p.add_argument("--merged-output", default="terrain_classifier/data/training_data.csv",
                   help="Path for merged training CSV")
    p.add_argument("--sim-port", type=int, default=5555)
    p.add_argument("--ctrl-port", type=int, default=5556)
    p.add_argument("--terrains", nargs="+", default=TERRAINS, choices=TERRAINS)
    p.add_argument("--paths", nargs="+", default=PATHS, choices=PATHS)
    p.add_argument("--bumpiness", nargs="+", type=int, default=BUMPINESS_LEVELS)
    p.add_argument("--speeds", nargs="+", type=float, default=SPEEDS)
    p.add_argument("--no-noise", action="store_true", help="Disable sensor noise")
    p.add_argument("--model", default="nn",
                   choices=["nn", "pacejka", "tmeasy"],
                   help="MPC tire model (must match runtime model)")
    p.add_argument("--nn-model", default=None,
                   help="NN model version directory (only used when --model nn)")
    args = p.parse_args()

    script_dir = Path(__file__).resolve().parent.parent
    out_dir = script_dir / args.output_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    combos = list(itertools.product(
        args.terrains, args.paths, args.bumpiness, args.speeds))
    total = len(combos) * args.runs_per_combo

    print("=" * 60)
    print("Terrain Classifier — Automated Data Collection")
    print("=" * 60)
    print(f"  Combos: {len(combos)} × {args.runs_per_combo} runs = {total} total")
    print(f"  Duration per run: {args.time}s")
    print(f"  Output: {out_dir}/")

    run_csvs = []
    run_idx = 0

    for terrain, path, bump, speed in combos:
        for rep in range(args.runs_per_combo):
            run_idx += 1
            tag = f"{terrain}_{path}_b{bump}_v{speed:.0f}_r{rep}"
            csv_path = out_dir / f"{tag}.csv"
            run_csvs.append(csv_path)

            print(f"\n--- Run {run_idx}/{total}: {tag} ---")

            # Build sim command (headless, no MPC needed — use manual excitation)
            sim_cmd = [
                sys.executable, str(script_dir / "chrono_sim_node.py"),
                "--time", str(args.time),
                "--speed", str(speed),
                "--terrain", terrain,
                "--path", path,
                "--bumpiness", str(bump),
                "--sim-port", str(args.sim_port),
                "--ctrl-host", "localhost",
                "--ctrl-port", str(args.ctrl_port),
                "--vis-mode", "none",
                "--state-rate", "100",
            ]
            if args.no_noise:
                sim_cmd.append("--no-noise")

            # Build MPC controller (provides steering excitation for diverse data)
            ctrl_cmd = [
                sys.executable, str(script_dir / "acados_mpc_controller_node.py"),
                "--model", args.model,
                "--path", path,
                "--speed", str(speed),
                "--terrain", terrain,
                "--time", str(args.time),
                "--sim-host", "localhost",
                "--sim-port", str(args.sim_port),
                "--ctrl-port", str(args.ctrl_port),
                "--no-plot",
                "--no-csv",
            ]
            if args.model == "nn" and args.nn_model:
                ctrl_cmd.extend(["--nn-model", args.nn_model])

            # Build collector command
            collect_cmd = [
                sys.executable, "-m", "terrain_classifier.collect_data",
                "--sim-host", "localhost",
                "--sim-port", str(args.sim_port),
                "--ctrl-host", "localhost",
                "--ctrl-port", str(args.ctrl_port),
                "--output", str(csv_path),
                "--terrain-label", terrain,
            ]

            procs = []
            try:
                # Start collector first
                collector = subprocess.Popen(collect_cmd, cwd=str(script_dir))
                procs.append(collector)

                # Start controller
                controller = subprocess.Popen(ctrl_cmd, cwd=str(script_dir))
                procs.append(controller)
                time.sleep(0.5)

                # Start sim
                sim = subprocess.Popen(sim_cmd, cwd=str(script_dir))
                procs.append(sim)

                # Wait for sim to finish
                sim.wait(timeout=args.time + 30)

                # Give controller/collector a moment to wrap up
                time.sleep(2.0)

            except subprocess.TimeoutExpired:
                print(f"  WARNING: Run timed out")
            except KeyboardInterrupt:
                print("\n  Interrupted — cleaning up...")
                for proc in procs:
                    if proc.poll() is None:
                        proc.terminate()
                sys.exit(1)
            finally:
                for proc in procs:
                    if proc.poll() is None:
                        proc.terminate()
                for proc in procs:
                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        proc.kill()

            if csv_path.exists():
                import csv as csv_mod
                with open(csv_path) as f:
                    n = sum(1 for _ in f) - 1  # subtract header
                print(f"  Collected {n} samples → {csv_path.name}")
            else:
                print(f"  WARNING: No output file produced")

    # ---- Merge all per-run CSVs ----
    merged_path = script_dir / args.merged_output
    print(f"\n{'=' * 60}")
    print(f"Merging {len(run_csvs)} files → {merged_path}")

    import csv as csv_mod
    header_written = False
    total_samples = 0

    with open(merged_path, "w", newline="") as fout:
        writer = None
        for csv_path in run_csvs:
            if not csv_path.exists():
                continue
            with open(csv_path) as fin:
                reader = csv_mod.reader(fin)
                header = next(reader, None)
                if header is None:
                    continue
                if not header_written:
                    writer = csv_mod.writer(fout)
                    writer.writerow(header)
                    header_written = True
                for row in reader:
                    writer.writerow(row)
                    total_samples += 1

    print(f"  Total samples: {total_samples}")
    print(f"  Output: {merged_path}")
    print("Done!")


if __name__ == "__main__":
    main()
