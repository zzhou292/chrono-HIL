#!/usr/bin/env python3
"""Parallel data-collection sweep for force-residual training.

Runs launch_decoupled.py across terrain × path × speed combinations
with unique ZMQ ports per worker.  All output goes to a shared --plot-dir
(timestamped subdirs avoid collisions).

Usage:
    python sweep_collect_data.py                        # defaults
    python sweep_collect_data.py --workers 4 --time 20  # custom
    python sweep_collect_data.py --plot-dir sweep_data  # custom output dir
"""

import argparse
import itertools
import subprocess
import sys
import os
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed

TERRAINS = ["clay", "dirt", "sand"]
PATHS = ["lane_change", "double_lane_change", "right_left", "sinusoidal"]
SPEEDS = [3.0, 5.0, 8.0]


def run_one(config: dict) -> dict:
    """Run a single simulation. Returns result dict."""
    cmd = [
        sys.executable, str(config["script"]),
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
    label = f"{config['terrain']}_{config['path']}_v{config['speed']}"
    try:
        result = subprocess.run(
            cmd, capture_output=True, text=True, timeout=config["timeout"],
            cwd=str(config["script"].parent),
        )
        return {
            "label": label,
            "returncode": result.returncode,
            "stdout_tail": result.stdout[-500:] if result.stdout else "",
            "stderr_tail": result.stderr[-500:] if result.stderr else "",
        }
    except subprocess.TimeoutExpired:
        return {"label": label, "returncode": -1, "stdout_tail": "TIMEOUT", "stderr_tail": ""}
    except Exception as e:
        return {"label": label, "returncode": -2, "stdout_tail": str(e), "stderr_tail": ""}


def main():
    parser = argparse.ArgumentParser(description="Parallel data-collection sweep")
    parser.add_argument("--workers", type=int, default=4,
                        help="Max parallel simulations")
    parser.add_argument("--time", type=float, default=20.0,
                        help="Simulation duration per run (s)")
    parser.add_argument("--timeout", type=float, default=180.0,
                        help="Wall-time timeout per run (s)")
    parser.add_argument("--plot-dir", default="sweep_data",
                        help="Shared output directory for diagnostic CSVs")
    parser.add_argument("--nn-model", default="paper_v1_mlp_rate_16_8",
                        help="NN tire model to use")
    parser.add_argument("--terrains", nargs="+", default=TERRAINS)
    parser.add_argument("--paths", nargs="+", default=PATHS)
    parser.add_argument("--speeds", nargs="+", type=float, default=SPEEDS)
    parser.add_argument("--base-port", type=int, default=6000,
                        help="Base ZMQ port (each worker uses 2 ports)")
    args = parser.parse_args()

    script = Path(__file__).parent / "launch_decoupled.py"
    if not script.exists():
        print(f"ERROR: {script} not found", file=sys.stderr)
        sys.exit(1)

    # Build all configurations
    combos = list(itertools.product(args.terrains, args.paths, args.speeds))
    configs = []
    for i, (terrain, path, speed) in enumerate(combos):
        configs.append({
            "script": script,
            "terrain": terrain,
            "path": path,
            "speed": speed,
            "time": args.time,
            "timeout": args.timeout,
            "plot_dir": args.plot_dir,
            "nn_model": args.nn_model,
            "sim_port": args.base_port + i * 2,
            "ctrl_port": args.base_port + i * 2 + 1,
        })

    print(f"Sweep: {len(configs)} runs, {args.workers} parallel workers")
    print(f"  Terrains: {args.terrains}")
    print(f"  Paths: {args.paths}")
    print(f"  Speeds: {args.speeds}")
    print(f"  Output: {args.plot_dir}/")
    print()

    # Ensure output dir exists
    Path(args.plot_dir).mkdir(parents=True, exist_ok=True)

    succeeded = 0
    failed = 0
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures = {pool.submit(run_one, cfg): cfg for cfg in configs}
        for future in as_completed(futures):
            result = future.result()
            status = "OK" if result["returncode"] == 0 else f"FAIL({result['returncode']})"
            print(f"  [{status}] {result['label']}")
            if result["returncode"] != 0:
                failed += 1
                if result["stderr_tail"]:
                    print(f"         stderr: {result['stderr_tail'][-200:]}")
            else:
                succeeded += 1

    print(f"\nDone: {succeeded}/{len(configs)} succeeded, {failed} failed")
    print(f"CSVs in: {args.plot_dir}/")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
