#!/usr/bin/env python3
"""Paper figure: live-estimator tire-model benchmark, RMS CTE distributions.

Replaces the older `tire_model_with_estimator_rms_cte_heatmap.png` which
displayed dozens of small per-scenario cells in a dense heatmap. A
per-variant box plot with terrain-stratified jitter is a clearer view
of the distribution that the paper text actually describes.

Source: my_paper/paper_figures/tire_model_with_estimator_results.csv
"""
from __future__ import annotations
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
SRC_CSV = ROOT / "my_paper" / "paper_figures" / "tire_model_with_estimator_results.csv"
OUT_DIR = ROOT / "my_paper" / "paper_figures"

VARIANT_ORDER = ["pacejka_static", "tmeasy_static", "nn_static", "nn_estimator"]
VARIANT_LABEL = {
    "pacejka_static":  "Pacejka\n(static)",
    "tmeasy_static":   "TMeasy\n(static)",
    "nn_static":       "NN axle-rate\n(static)",
    "nn_estimator":    "NN axle-rate\n+ estimator",
}
TERRAIN_COLOR = {"clay": "#1f77b4", "dirt": "#8c564b", "sand": "#d4a017"}


def main():
    df = pd.read_csv(SRC_CSV)
    df = df[df["status"] == "ok"]

    fig, ax = plt.subplots(figsize=(7.6, 4.6))
    positions = np.arange(len(VARIANT_ORDER))

    # Box plot per variant
    box_data = [df[df.variant == v]["rms_cte_m"].dropna().values for v in VARIANT_ORDER]
    bp = ax.boxplot(box_data, positions=positions, widths=0.55,
                    patch_artist=True, showfliers=False, zorder=2,
                    medianprops=dict(color="black", lw=1.2))
    for patch in bp["boxes"]:
        patch.set_facecolor("#dddddd")
        patch.set_alpha(0.7)
        patch.set_edgecolor("black")

    # Jittered points per terrain for each variant
    rng = np.random.default_rng(0)
    for i, v in enumerate(VARIANT_ORDER):
        sub = df[df.variant == v]
        for t, c in TERRAIN_COLOR.items():
            pts = sub[sub.terrain == t]["rms_cte_m"].dropna().values
            xj = positions[i] + rng.normal(0, 0.06, size=len(pts))
            label = t if i == 0 else None
            ax.scatter(xj, pts, color=c, edgecolor="black", linewidth=0.4,
                       s=22, alpha=0.85, zorder=3, label=label)

    # Annotate mean above max value (or near box top if outliers extend far)
    y_top = max(df["rms_cte_m"].max(), 2.3)
    for i, v in enumerate(VARIANT_ORDER):
        m = df[df.variant == v]["rms_cte_m"].mean()
        ax.text(positions[i], y_top * 1.02, f"$\\mu$={m:.2f} m",
                ha="center", va="bottom", fontsize=9.5,
                fontweight="bold", color="#222")

    ax.set_xticks(positions)
    ax.set_xticklabels([VARIANT_LABEL[v] for v in VARIANT_ORDER], fontsize=9.5)
    ax.set_ylabel("RMS crosstrack error (m), $t \\geq 8\\,$s window", fontsize=11)
    ax.set_title(
        "Live-estimator tire-model benchmark "
        "(405 runs/variant: 3 terrains × 3 paths × 3 speeds × 3 bumpiness × 5 seeds)",
        fontsize=10.5)
    ax.legend(title="Terrain", loc="upper right", framealpha=0.95, fontsize=9)
    ax.grid(axis="y", alpha=0.3)
    ax.set_ylim(0, y_top * 1.15)

    fig.tight_layout()
    out = OUT_DIR / "tire_model_with_estimator_rms_cte_heatmap.png"  # keep name for paper.tex
    fig.savefig(out, dpi=200, bbox_inches="tight")
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
