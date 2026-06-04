#!/usr/bin/env python3
"""Paper figure: one comprehensive RMS CTE heatmap.

Per-(terrain, path) cell shows the mean RMS CTE across speeds, bumpiness,
and seeds for three controllers: Pacejka (analytical), TMeasy (analytical),
and the deployed Vehicle NN rate-MLP. The figure is intentionally a
single 9-row × 3-column heatmap so the analytical-vs-NN gap is visible at
a glance across the full terrain × path matrix.

Source: 1215-run `mpc_tire_model_sweep` (3 terrains × 3 paths × 3 speeds
× 3 bumpiness × 5 seeds × 3 variants).
"""
from __future__ import annotations
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
# Newest mpc_tire_model_sweep results (the reproduced full 1215-run matrix).
SRC = sorted((ROOT / "benchmarking" / "results").glob("mpc_tire_model_sweep_*/results.csv"),
             key=lambda p: p.stat().st_mtime)[-1]
OUT = ROOT / "my_paper" / "paper_figures" / "cte_master_heatmap.png"

VARIANT_ORDER = ["pacejka", "tmeasy", "vehicle_rate"]
VARIANT_LABEL = {
    "pacejka": "Pacejka\n(analytical)",
    "tmeasy": "TMeasy\n(analytical)",
    "vehicle_rate": "Vehicle NN\nrate-MLP",
}
TERRAINS = ["clay", "dirt", "sand"]
PATHS = ["sinusoidal", "lane_change", "right_left"]
PATH_LABEL = {"sinusoidal": "sinusoidal", "lane_change": "lane change",
              "right_left": "right-left"}


def main():
    df = pd.read_csv(SRC)
    df = df[df["status"] == "ok"].copy()

    # Build the 9×3 mean-RMS-CTE matrix
    rows = [(t, p) for t in TERRAINS for p in PATHS]
    mat = np.full((len(rows), len(VARIANT_ORDER)), np.nan)
    counts = np.zeros_like(mat, dtype=int)
    for i, (t, p) in enumerate(rows):
        for j, v in enumerate(VARIANT_ORDER):
            sub = df[(df.terrain == t) & (df.path == p) & (df.variant == v)]["rms_cte_m"].dropna()
            if len(sub):
                mat[i, j] = sub.mean()
                counts[i, j] = len(sub)

    fig, ax = plt.subplots(figsize=(6.6, 5.4))
    vmax = np.nanpercentile(mat, 95)        # ceiling at 95th pct so outliers don't compress scale
    im = ax.imshow(mat, aspect="auto", cmap="RdYlGn_r", vmin=0, vmax=vmax)

    ax.set_xticks(np.arange(len(VARIANT_ORDER)))
    ax.set_xticklabels([VARIANT_LABEL[v] for v in VARIANT_ORDER], fontsize=10)
    ax.set_yticks(np.arange(len(rows)))
    ax.set_yticklabels([f"{t} / {PATH_LABEL[p]}" for (t, p) in rows], fontsize=10)

    # Annotate cells with mean ± value; text is white on saturated red cells, black otherwise
    for i in range(mat.shape[0]):
        for j in range(mat.shape[1]):
            v = mat[i, j]
            if np.isnan(v):
                continue
            txtcol = "white" if v > 0.55 * vmax else "black"
            ax.text(j, i, f"{v:.2f}", ha="center", va="center",
                    fontsize=10, color=txtcol, fontweight="bold")

    # Terrain grouping dividers
    for t_end in (3, 6):
        ax.axhline(t_end - 0.5, color="black", lw=1.0)

    cb = fig.colorbar(im, ax=ax, fraction=0.04, pad=0.03)
    cb.set_label("Mean RMS CTE (m), n=15 runs per cell", fontsize=10)

    ax.set_title(
        "Closed-loop tracking error by terrain × path × tire model\n"
        "(1215 runs total, averaged over 3 speeds × 3 bumpiness × 5 seeds per cell)",
        fontsize=10.5)
    fig.tight_layout()
    fig.savefig(OUT, dpi=200, bbox_inches="tight")
    print(f"Wrote {OUT}")


if __name__ == "__main__":
    main()
