#!/usr/bin/env python3
"""Paper figure: Asymmetric throttle disturbance observer ablation.

Replaces the prior speed-retention heatmap. Bar pair from
Table~\\ref{tab:throttledob} (64 runs per variant).
"""
from __future__ import annotations
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "my_paper" / "paper_figures" / "throttle_dob_ablation_speed_heatmap.png"

VARIANTS = ["DOB off", "DOB on"]
CTE_MEAN  = [0.091, 0.087]
CTE_STD   = [0.027, 0.024]
SPEED_REF = [0.78, 0.81]
UBAR_MS   = [3.52, 3.92]
COLOR     = ["#aaaaaa", "#1f77b4"]


def main():
    fig, axes = plt.subplots(1, 3, figsize=(10.0, 3.6))
    x = np.arange(len(VARIANTS))

    # RMS CTE
    axes[0].bar(x, CTE_MEAN, yerr=CTE_STD, capsize=4,
                color=COLOR, edgecolor="black", linewidth=0.5)
    axes[0].set_xticks(x); axes[0].set_xticklabels(VARIANTS, fontsize=10.5)
    axes[0].set_ylabel("RMS CTE (m)", fontsize=10.5)
    axes[0].set_title("Tracking error (no measurable change)", fontsize=10.5)
    axes[0].grid(axis="y", alpha=0.3)
    for i, (m, s) in enumerate(zip(CTE_MEAN, CTE_STD)):
        axes[0].text(i, m + s + 0.003, f"{m:.3f} ± {s:.3f}",
                     ha="center", va="bottom", fontsize=9.5)

    # Speed ratio vs profile
    axes[1].bar(x, SPEED_REF, color=COLOR, edgecolor="black", linewidth=0.5)
    axes[1].set_xticks(x); axes[1].set_xticklabels(VARIANTS, fontsize=10.5)
    axes[1].set_ylabel("Speed ratio vs profile", fontsize=10.5)
    axes[1].set_title("Speed retention", fontsize=10.5)
    axes[1].grid(axis="y", alpha=0.3)
    axes[1].set_ylim(0, 1.0)
    for i, v in enumerate(SPEED_REF):
        axes[1].text(i, v + 0.01, f"{v:.2f}",
                     ha="center", va="bottom", fontsize=10.5,
                     fontweight="bold" if i == 1 else "normal")

    # Achieved mean speed
    axes[2].bar(x, UBAR_MS, color=COLOR, edgecolor="black", linewidth=0.5)
    axes[2].set_xticks(x); axes[2].set_xticklabels(VARIANTS, fontsize=10.5)
    axes[2].set_ylabel("Mean achieved $\\bar u$ (m/s)", fontsize=10.5)
    axes[2].set_title("Achieved speed", fontsize=10.5)
    axes[2].grid(axis="y", alpha=0.3)
    for i, v in enumerate(UBAR_MS):
        axes[2].text(i, v + 0.04, f"{v:.2f} m/s",
                     ha="center", va="bottom", fontsize=10.5,
                     fontweight="bold" if i == 1 else "normal")

    fig.suptitle(
        "Throttle DOB ablation: $K_i, d_{\\max}\\!\\to\\!0$ vs nominal (64 runs/variant). "
        "DOB lifts speed by ~11% with no tracking cost.",
        fontsize=11, y=1.02)
    fig.tight_layout()
    fig.savefig(OUT, dpi=200, bbox_inches="tight")
    print(f"Wrote {OUT}")


if __name__ == "__main__":
    main()
