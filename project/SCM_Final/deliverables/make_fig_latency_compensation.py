#!/usr/bin/env python3
"""Paper figure: 5G-profile latency compensation sweep.

Replaces the prior collision heatmap. Bar chart of headline metrics
from Table~\\ref{tab:latency_comp} (96 runs).
"""
from __future__ import annotations
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "my_paper" / "paper_figures" / "latency_compensation_collision_heatmap.png"

VARIANTS = ["No filter", "DOB-CBF", "MPPI"]
COLLISIONS = [1.84, 0.22, 0.19]
CLEARANCE  = [-1.54, +0.52, +0.23]
INTERV_PCT = [0.0, 54.0, 67.0]
COLOR      = ["#aaaaaa", "#1f77b4", "#2ca02c"]


def _bar_panel(ax, values, title, ylabel, fmt, *, lower_better=True, bold=None):
    x = np.arange(len(VARIANTS))
    ax.bar(x, values, color=COLOR, edgecolor="black", linewidth=0.5)
    ax.set_xticks(x); ax.set_xticklabels(VARIANTS, fontsize=10)
    ax.set_ylabel(ylabel, fontsize=10.5)
    ax.set_title(title, fontsize=10.5)
    ax.grid(axis="y", alpha=0.3)
    for i, v in enumerate(values):
        ax.text(i, v, f" {fmt(v)}",
                ha="center", va="bottom" if v >= 0 else "top",
                fontsize=10.5,
                fontweight="bold" if i in (bold or []) else "normal")
    if min(values) < 0:
        ax.axhline(0, color="black", lw=0.6)


def main():
    fig, axes = plt.subplots(1, 3, figsize=(10.5, 3.6))
    _bar_panel(axes[0], COLLISIONS,
               "Collisions (lower better)",
               "Mean unique obstacles hit",
               lambda v: f"{v:.2f}", bold=[2])
    _bar_panel(axes[1], CLEARANCE,
               "Clearance (higher better)",
               "Min clearance (m)",
               lambda v: f"{v:+.2f}", lower_better=False, bold=[1])
    _bar_panel(axes[2], INTERV_PCT,
               "Shield intrusiveness",
               "Intervention rate (%)",
               lambda v: f"{v:.0f}", lower_better=False)

    fig.suptitle(
        "Closed-loop performance under the learned N-HiTS-5G uplink profile "
        "(96 runs; mean ctrl/cam delay $\\approx 57/90\\,$ms, bursts to $660\\,$ms)",
        fontsize=11, y=1.02)
    fig.tight_layout()
    fig.savefig(OUT, dpi=200, bbox_inches="tight")
    print(f"Wrote {OUT}")


if __name__ == "__main__":
    main()
