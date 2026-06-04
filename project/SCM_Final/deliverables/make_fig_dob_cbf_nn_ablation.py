#!/usr/bin/env python3
"""Paper figure: DOB-CBF neural-surrogate ablation.

Replaces the prior heatmap (which suffered from a single 583-collision
outlier compressing the colormap). The figure now reports the four
headline KPIs as a 2x2 panel so the reviewer can see the magnitude of
each ablation effect at a glance. Source values come from
Table~\\ref{tab:dobcbf_nn} in `my_paper/paper.tex` (96 closed-loop runs).
"""
from __future__ import annotations
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "my_paper" / "paper_figures" / "dob_cbf_nn_ablation_heatmap.png"

# values from tab:dobcbf_nn
VARIANTS = ["No filter", "DOB-CBF\nno NN", "DOB-CBF\nwith NN"]
COLLISIONS  = [2.00, 1.00, 0.00]
CLEARANCE   = [-1.64, -0.20, +0.52]
INTERV_PCT  = [0.0, 47.0, 58.0]   # No filter has no intervention concept
CTE_M       = [0.19, 2.11, 2.83]
COLOR       = ["#aaaaaa", "#d4a017", "#1f77b4"]


def _bar_panel(ax, values, title, ylabel, fmt, *, baseline_idx=None, lower_better=True):
    x = np.arange(len(VARIANTS))
    ax.bar(x, values, color=COLOR, edgecolor="black", linewidth=0.5)
    ax.set_xticks(x); ax.set_xticklabels(VARIANTS, fontsize=9)
    ax.set_ylabel(ylabel, fontsize=10)
    ax.set_title(title, fontsize=10.5)
    ax.grid(axis="y", alpha=0.3)
    for i, v in enumerate(values):
        ax.text(i, v, f" {fmt(v)}",
                ha="center",
                va="bottom" if v >= 0 else "top",
                fontsize=9.5,
                fontweight="bold" if i == 2 else "normal")
    if min(values) < 0:
        ax.axhline(0, color="black", lw=0.6)


def main():
    fig, axes = plt.subplots(2, 2, figsize=(8.0, 5.4))

    _bar_panel(axes[0, 0], COLLISIONS,
               "Unique obstacles hit per run (lower better)",
               "Collisions", lambda v: f"{v:.2f}")
    _bar_panel(axes[0, 1], CLEARANCE,
               "Minimum clearance to nearest rock (higher better)",
               "Clearance (m)", lambda v: f"{v:+.2f}", lower_better=False)
    _bar_panel(axes[1, 0], INTERV_PCT,
               "Intervention rate (shield command)",
               "Intervention (%)", lambda v: f"{v:.0f}", lower_better=False)
    _bar_panel(axes[1, 1], CTE_M,
               "RMS CTE from obstacle-blind reference",
               "RMS CTE (m)", lambda v: f"{v:.2f}", lower_better=False)

    fig.suptitle("DOB-CBF NN ablation (96 runs, clay+sand × 2 paths × 2 speeds × 2 bumps × 3 seeds)",
                 fontsize=11.5, y=1.00)
    fig.tight_layout()
    fig.savefig(OUT, dpi=200, bbox_inches="tight")
    print(f"Wrote {OUT}")


if __name__ == "__main__":
    main()
