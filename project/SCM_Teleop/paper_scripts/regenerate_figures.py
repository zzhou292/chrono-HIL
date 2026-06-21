#!/usr/bin/env python3
"""Regenerate paper-script figures from existing result folders.

This does not rerun Chrono.  It reuses each folder's ``results.csv`` plus the
raw diagnostic CSVs under ``raw/`` to rebuild summary, trajectory, and force
diagnostic figures after plotting code changes.
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path
from typing import Callable


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))

from autonomous_obstacle_tire_model_sweep import plot_figures as plot_autonomous  # noqa: E402
from dob_cbf_nn_ablation import plot_figures as plot_dob_ablation  # noqa: E402
from mpc_tire_model_sweep import plot_figures as plot_tire_models  # noqa: E402
from mpcc_vs_mpc_speed_tracking import plot_figures as plot_mpcc  # noqa: E402
from safety_filter_sweep import plot_figures as plot_safety  # noqa: E402
from terrain_estimator_benchmark import plot_figures as plot_terrain  # noqa: E402


PLOTTERS: list[tuple[str, Callable[[Path, Path], None]]] = [
    ("autonomous_obstacle_tire_model_sweep", plot_autonomous),
    ("mpc_tire_model_sweep", plot_tire_models),
    ("mpcc_vs_mpc_speed_tracking", plot_mpcc),
    ("safety_filter_sweep", plot_safety),
    ("dob_cbf_nn_ablation", plot_dob_ablation),
    ("terrain_estimator_benchmark", plot_terrain),
]


def find_plotter(result_dir: Path) -> Callable[[Path, Path], None] | None:
    name = result_dir.name
    for prefix, plotter in PLOTTERS:
        if name.startswith(prefix):
            return plotter
    return None


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("result_dirs", nargs="+", type=Path,
                   help="Result folders, or the paper_scripts/results root with --all.")
    p.add_argument("--all", action="store_true",
                   help="Treat each input directory as a results root and regenerate every known child folder.")
    p.add_argument("--clean", action="store_true",
                   help="Remove each figures/ directory before regenerating.")
    return p.parse_args()


def expand_inputs(paths: list[Path], include_all: bool) -> list[Path]:
    out: list[Path] = []
    for path in paths:
        path = path.expanduser().resolve()
        if include_all:
            out.extend(sorted(p for p in path.iterdir() if p.is_dir() and (p / "results.csv").exists()))
        else:
            out.append(path)
    return out


def main() -> None:
    args = parse_args()
    result_dirs = expand_inputs(args.result_dirs, args.all)
    if not result_dirs:
        raise SystemExit("No result folders found.")

    for result_dir in result_dirs:
        results_csv = result_dir / "results.csv"
        if not results_csv.exists():
            print(f"[skip] {result_dir}: missing results.csv")
            continue
        plotter = find_plotter(result_dir)
        if plotter is None:
            print(f"[skip] {result_dir}: no known plotter")
            continue
        fig_dir = result_dir / "figures"
        if args.clean and fig_dir.exists():
            shutil.rmtree(fig_dir)
        fig_dir.mkdir(exist_ok=True)
        print(f"[plot] {result_dir}")
        plotter(results_csv, result_dir)


if __name__ == "__main__":
    main()
