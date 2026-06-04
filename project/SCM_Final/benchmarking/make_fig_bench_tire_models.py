#!/usr/bin/env python3
"""Paper figure: static-terrain tire-model benchmark, RMS CTE distributions.

Replaces the old dense heatmap (5-column × 24-row grid with missing
Vehicle-NN cells). Source: the fresh `mpc_tire_model_sweep` run that
covers 3 terrains × 3 paths × 3 speeds × 3 bumpiness × 5 seeds = 1215
runs across the deployable models (Pacejka, TMeasy, Vehicle NN
rate-MLP).
"""
from __future__ import annotations
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
SRC_CSV = sorted((ROOT / "benchmarking" / "results").glob("mpc_tire_model_sweep_*/results.csv"),
                 key=lambda p: p.stat().st_mtime)[-1]
OUT = ROOT / "my_paper" / "paper_figures" / "bench_tire_models_heatmap.png"

VARIANT_ORDER = ["pacejka", "tmeasy", "vehicle_rate"]
VARIANT_LABEL = {
    "pacejka": "Pacejka",
    "tmeasy": "TMeasy",
    "vehicle_rate": "Vehicle NN\nrate-MLP",
}
TERRAIN_COLOR = {"clay": "#1f77b4", "dirt": "#8c564b", "sand": "#d4a017"}


def main():
    df = pd.read_csv(SRC_CSV)
    df = df[df["status"] == "ok"]

    fig, ax = plt.subplots(figsize=(6.8, 4.6))
    positions = np.arange(len(VARIANT_ORDER))

    box_data = [df[df.variant == v]["rms_cte_m"].dropna().values for v in VARIANT_ORDER]
    bp = ax.boxplot(box_data, positions=positions, widths=0.55,
                    patch_artist=True, showfliers=False, zorder=2,
                    medianprops=dict(color="black", lw=1.2))
    for patch in bp["boxes"]:
        patch.set_facecolor("#dddddd"); patch.set_alpha(0.7); patch.set_edgecolor("black")

    rng = np.random.default_rng(0)
    for i, v in enumerate(VARIANT_ORDER):
        sub = df[df.variant == v]
        for t, c in TERRAIN_COLOR.items():
            pts = sub[sub.terrain == t]["rms_cte_m"].dropna().values
            xj = positions[i] + rng.normal(0, 0.05, size=len(pts))
            label = t if i == 0 else None
            ax.scatter(xj, pts, color=c, edgecolor="black", linewidth=0.3,
                       s=14, alpha=0.55, zorder=3, label=label)

    y_top = max(2.6, df["rms_cte_m"].max() * 0.5)
    for i, v in enumerate(VARIANT_ORDER):
        m = df[df.variant == v]["rms_cte_m"].mean()
        ax.text(positions[i], y_top * 1.02, f"$\\mu$={m:.2f} m",
                ha="center", va="bottom", fontsize=9.5,
                fontweight="bold", color="#222")

    ax.set_xticks(positions)
    ax.set_xticklabels([VARIANT_LABEL[v] for v in VARIANT_ORDER], fontsize=10)
    ax.set_ylabel("RMS crosstrack error (m)", fontsize=11)
    ax.set_title(
        "Static-terrain tire-model RMS CTE distribution "
        f"(n={len(df)} runs, 3 terrains × 3 paths × 3 speeds × 3 bumpiness × 5 seeds)",
        fontsize=10.5)
    ax.legend(title="Terrain", loc="upper right", framealpha=0.95, fontsize=9)
    ax.grid(axis="y", alpha=0.3)
    ax.set_ylim(0, y_top * 1.18)

    fig.tight_layout()
    fig.savefig(OUT, dpi=200, bbox_inches="tight")
    print(f"Wrote {OUT}")


if __name__ == "__main__":
    main()
