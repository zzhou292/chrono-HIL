#!/usr/bin/env python3
"""Paper figures: learned terrain-estimator true-vs-estimated scatters.

Produces, straight into ``my_paper/paper_figures/`` with the exact names
``paper.tex`` expects:

* ``terrain_estimator_scatter.png``  -- closed-loop NMPC benchmark
  (Fig.~\\ref{fig:estimator_scatter}); read from the newest
  ``terrain_estimator_benchmark_*`` result folder.
* ``open_loop_estimator_diagnostic.png`` -- scripted open-loop diagnostic
  (Fig.~\\ref{fig:estimator_open_loop}); read from the newest
  ``open_loop_terrain_estimator_benchmark_*`` result folder.

This replaces the legacy ``make_fig_terrain_est.py`` aggregator, whose
hard-coded ``runs_v7_random/`` and ``runs/ood_terrains/`` inputs were
removed in the 2026-05 cleanup. Both figures are now regenerated directly
from the benchmark ``results.csv`` files that ``run.py`` / the open-loop
benchmark produce, so they are fully reproducible from ``benchmarking/``.
"""
from __future__ import annotations

import glob
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "benchmarking" / "results"
OUT_DIR = ROOT / "my_paper" / "paper_figures"


def _latest(prefix: str) -> Path:
    hits = sorted(RESULTS.glob(f"{prefix}_*/results.csv"),
                  key=lambda p: p.stat().st_mtime)
    if not hits:
        raise FileNotFoundError(
            f"no {prefix}_* results under {RESULTS} -- run the benchmark first")
    return hits[-1]


def _scatter(csv_path: Path, out_name: str, title: str) -> None:
    df = pd.read_csv(csv_path)
    df = df[df["n_est_mean_tail"].notna()]
    fig, ax = plt.subplots(figsize=(7.0, 4.6))
    lo, hi = 0.45, 1.25
    grid = np.linspace(lo, hi, 100)
    ax.fill_between(grid, grid - 0.10, grid + 0.10, color="#888", alpha=0.08,
                    label=r"$\pm 0.10$ band")
    ax.fill_between(grid, grid - 0.05, grid + 0.05, color="#888", alpha=0.14,
                    label=r"$\pm 0.05$ band")
    ax.plot([lo, hi], [lo, hi], "k--", lw=1.0, label="$y=x$")
    for dist, color in (("id", "#1f77b4"), ("ood", "#ff7f0e")):
        sub = df[df["distribution"] == dist]
        if len(sub):
            ax.scatter(sub["true_n"], sub["n_est_mean_tail"], s=55, alpha=0.75,
                       color=color, edgecolor="none", label=dist)
    e = (df["n_est_mean_tail"] - df["true_n"]).abs()
    within05 = 100.0 * (e <= 0.05).mean()
    within10 = 100.0 * (e <= 0.10).mean()
    ax.set_xlabel("True Bekker n")
    ax.set_ylabel("Estimated tail-mean n")
    ax.set_title(f"{title}\n"
                 f"$\\pm0.05$/$\\pm0.10$ bands contain "
                 f"{within05:.0f}\\%/{within10:.0f}\\% of points (N={len(df)})")
    ax.set_xlim(lo, hi)
    ax.set_ylim(lo, hi)
    ax.grid(alpha=0.3)
    ax.legend(loc="upper left", fontsize=9, framealpha=0.92)
    fig.tight_layout()
    out = OUT_DIR / out_name
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"Wrote {out}  ({within05:.0f}%/{within10:.0f}% within bands, N={len(df)})")


def main() -> int:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    _scatter(_latest("terrain_estimator_benchmark"),
             "terrain_estimator_scatter.png",
             "Learned terrain estimator (closed-loop NMPC, sinusoidal)")
    _scatter(_latest("open_loop_terrain_estimator_benchmark"),
             "open_loop_estimator_diagnostic.png",
             "Learned terrain estimator (scripted open-loop diagnostic)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
