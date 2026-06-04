#!/usr/bin/env python3
"""Paper figure: MPPI seed-trajectory ablation.

Replaces the prior collision heatmap. Bar-chart contrast of headline
metrics from Table~\\ref{tab:mppi_seeds} (64 runs).
"""
from __future__ import annotations
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "my_paper" / "paper_figures" / "mppi_seed_ablation_collision_heatmap.png"

# values from tab:mppi_seeds
VARIANTS = ["MPPI\nno seeds", "MPPI\nwith seeds"]
COLLISIONS = [1.28, 0.03]
CLEARANCE  = [-0.50, +0.20]
INTERV_PCT = [63.0, 63.0]
CTE_M      = [0.77, 1.08]
COLOR      = ["#aaaaaa", "#1f77b4"]


def _bar_panel(ax, values, title, ylabel, fmt, *, lower_better=True, bold_idx=None):
    x = np.arange(len(VARIANTS))
    ax.bar(x, values, color=COLOR, edgecolor="black", linewidth=0.5)
    ax.set_xticks(x); ax.set_xticklabels(VARIANTS, fontsize=9)
    ax.set_ylabel(ylabel, fontsize=10)
    ax.set_title(title, fontsize=10.5)
    ax.grid(axis="y", alpha=0.3)
    for i, v in enumerate(values):
        ax.text(i, v, f" {fmt(v)}",
                ha="center", va="bottom" if v >= 0 else "top",
                fontsize=10,
                fontweight="bold" if i == bold_idx else "normal")
    if min(values) < 0:
        ax.axhline(0, color="black", lw=0.6)


def main():
    fig, axes = plt.subplots(2, 2, figsize=(7.0, 5.2))

    _bar_panel(axes[0, 0], COLLISIONS,
               "Unique obstacles hit per run (lower better)",
               "Collisions", lambda v: f"{v:.2f}", bold_idx=1)
    _bar_panel(axes[0, 1], CLEARANCE,
               "Minimum clearance (higher better)",
               "Clearance (m)", lambda v: f"{v:+.2f}", lower_better=False, bold_idx=1)
    _bar_panel(axes[1, 0], INTERV_PCT,
               "Intervention rate",
               "Intervention (%)", lambda v: f"{v:.0f}", lower_better=False)
    _bar_panel(axes[1, 1], CTE_M,
               "RMS CTE",
               "RMS CTE (m)", lambda v: f"{v:.2f}", lower_better=False)

    fig.suptitle("MPPI seed-trajectory ablation (64 runs); removing hand-crafted seeds increases collisions ~40×",
                 fontsize=11, y=1.00)
    fig.tight_layout()
    fig.savefig(OUT, dpi=200, bbox_inches="tight")
    print(f"Wrote {OUT}")


if __name__ == "__main__":
    main()
