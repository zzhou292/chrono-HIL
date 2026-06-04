#!/usr/bin/env python3
"""Paper figure: autonomous obstacle avoidance, planner tire model sweep
under a fixed MPPI shield. Replaces the old heatmap, which carried
a stale 583-style outlier and missing dirt row.

Grouped bar chart over (tire_model, terrain) with mean ± std collisions
across 8 cells per (variant, terrain) — 2 paths × 2 speeds × 2 seeds.
Source: my_paper/paper_figures/autonomous_obstacle_results.csv.
"""
from __future__ import annotations
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
SRC_CSV = ROOT / "my_paper" / "paper_figures" / "autonomous_obstacle_results.csv"
OUT_DIR = ROOT / "my_paper" / "paper_figures"

VARIANT_ORDER = ["pacejka", "tmeasy", "closed_loop_v2_rate_mlp", "closed_loop_v3_axle_rate_mlp"]
VARIANT_LABEL = {
    "pacejka": "Pacejka",
    "tmeasy": "TMeasy",
    "closed_loop_v2_rate_mlp": "NN rate-MLP",
    "closed_loop_v3_axle_rate_mlp": "NN axle-rate-MLP",
}
VARIANT_COLOR = {
    "pacejka":                       "#aaaaaa",
    "tmeasy":                        "#d4a017",
    "closed_loop_v2_rate_mlp":       "#1f77b4",
    "closed_loop_v3_axle_rate_mlp":  "#2ca02c",
}


def main():
    df = pd.read_csv(SRC_CSV)
    df = df[df["status"] == "ok"].copy()
    terrains = sorted(df["terrain"].unique())
    n_runs = len(df)

    fig, ax = plt.subplots(figsize=(7.0, 4.0))
    width = 0.20
    x = np.arange(len(terrains))

    for i, variant in enumerate(VARIANT_ORDER):
        means, stds = [], []
        for t in terrains:
            sub = df[(df.variant == variant) & (df.terrain == t)]["collisions"]
            means.append(sub.mean() if len(sub) else np.nan)
            stds.append(sub.std() if len(sub) > 1 else 0.0)
        offset = (i - (len(VARIANT_ORDER) - 1) / 2) * width
        ax.bar(x + offset, means, width,
               yerr=stds, capsize=3,
               color=VARIANT_COLOR[variant],
               edgecolor="black", linewidth=0.5,
               label=VARIANT_LABEL[variant])
        for j, m in enumerate(means):
            if np.isfinite(m):
                top = m + (stds[j] if np.isfinite(stds[j]) else 0)
                ax.text(x[j] + offset, top + 0.02, f"{m:.2f}",
                        ha="center", va="bottom", fontsize=8)

    ax.set_xticks(x); ax.set_xticklabels(terrains)
    ax.set_xlabel("Terrain", fontsize=11)
    ax.set_ylabel("Mean unique obstacles hit per run\n(lower is better)", fontsize=11)
    ax.set_title("Autonomous obstacle avoidance: planner tire model × terrain\n"
                 f"(fixed MPPI shield, n={n_runs} runs)", fontsize=10.5)
    ax.legend(title="Planner tire model", loc="upper right",
              framealpha=0.95, fontsize=9)
    ax.grid(axis="y", alpha=0.3)
    ax.set_ylim(0, max(0.6, ax.get_ylim()[1] * 1.1))

    fig.tight_layout()
    out = OUT_DIR / "autonomous_obstacle_collision_heatmap.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
