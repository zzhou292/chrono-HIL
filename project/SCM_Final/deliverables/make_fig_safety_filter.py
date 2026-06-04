#!/usr/bin/env python3
"""Paper figure: safety-filter shield-only validation (planner blind).

Replaces the old `safety_filter_collision_heatmap.png` which (1) had a
runaway 178-collision outlier compressing the colormap and (2) was
missing the dirt row entirely.

Builds a grouped bar chart of mean unique obstacles hit per filter
across terrain × path, with whisker error bars (std across seeds).
Source: my_paper/paper_figures/safety_filter_planner_aware_results.csv,
restricted to the four planner-blind variants.
"""
from __future__ import annotations
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
SRC_CSV = ROOT / "my_paper" / "paper_figures" / "safety_filter_planner_aware_results.csv"
OUT_DIR = ROOT / "my_paper" / "paper_figures"

FILTER_ORDER = ["none_blind", "dob_cbf_blind", "mppi_blind", "nmpc_blind"]
FILTER_LABEL = {
    "none_blind":    "None",
    "dob_cbf_blind": "DOB-CBF",
    "mppi_blind":    "MPPI",
    "nmpc_blind":    "NMPC",
}
FILTER_COLOR = {
    "none_blind":    "#aaaaaa",
    "dob_cbf_blind": "#1f77b4",
    "mppi_blind":    "#2ca02c",
    "nmpc_blind":    "#d62728",
}


def main():
    df = pd.read_csv(SRC_CSV)
    df = df[df["status"] == "ok"]
    df = df[df["variant"].isin(FILTER_ORDER)].copy()

    terrains = sorted(df["terrain"].unique())   # clay, sand
    n_runs = len(df)

    fig, ax = plt.subplots(figsize=(7.6, 4.4))
    width = 0.20
    x = np.arange(len(terrains))

    for i, variant in enumerate(FILTER_ORDER):
        means, stds = [], []
        for t in terrains:
            sub = df[(df["variant"] == variant) & (df["terrain"] == t)]["collisions"]
            means.append(sub.mean() if len(sub) else np.nan)
            stds.append(sub.std() if len(sub) > 1 else 0.0)
        offset = (i - (len(FILTER_ORDER) - 1) / 2) * width
        bars = ax.bar(x + offset, means, width,
                      yerr=stds, capsize=3,
                      color=FILTER_COLOR[variant],
                      edgecolor="black", linewidth=0.5,
                      label=FILTER_LABEL[variant])
        for j, m in enumerate(means):
            if np.isfinite(m):
                ax.text(x[j] + offset, m + (stds[j] if np.isfinite(stds[j]) else 0) + 0.05,
                        f"{m:.2f}", ha="center", va="bottom", fontsize=8)

    ax.set_xticks(x)
    ax.set_xticklabels(terrains)
    ax.set_ylabel("Mean unique obstacles hit per run\n(lower is better)", fontsize=11)
    ax.set_xlabel("Terrain", fontsize=11)
    ax.set_title(
        "Shield-only validation: collisions by terrain × filter\n"
        f"(planner blind to obstacles, n={n_runs} runs across "
        f"2 paths × 2 speeds × 2 bumpiness × 4 seeds)", fontsize=10.5)
    ax.legend(title="Safety filter", loc="upper right",
              framealpha=0.95, fontsize=9)
    ax.grid(axis="y", alpha=0.3)
    ax.set_ylim(0, max(3.5, ax.get_ylim()[1]))

    fig.tight_layout()
    out = OUT_DIR / "safety_filter_collision_heatmap.png"  # keep name for paper.tex
    fig.savefig(out, dpi=200, bbox_inches="tight")
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
