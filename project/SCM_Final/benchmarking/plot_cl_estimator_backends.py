#!/usr/bin/env python3
"""Regenerate the closed-loop estimator-backend comparison figure
(``closed_loop_estimator_backends.png``, paper Fig. cl_estimator_backends)
from the committed summary CSV
``benchmarking/closed_loop_estimator_fused_summary.csv``.

The CSV is produced by ``closed_loop_estimator_compare_fused.py`` (3 backends
x clay/dirt/sand x {5,7} m/s x 3 seeds, tail-window in-loop |dn|). Re-plotting
from the CSV needs no Chrono re-sim.
"""
from __future__ import annotations
import sys
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
CSV = ROOT / "benchmarking" / "closed_loop_estimator_all_summary.csv"
OUT = ROOT / "my_paper" / "paper_figures" / "closed_loop_estimator_backends.png"

# display label + colour per CSV backend row (matched by prefix)
STYLE = [
    ("MLP",         "MLP (window)",            "#4c78a8"),
    ("Bekker-UKF",  "Bekker-UKF (force)",      "#b07aa1"),
    ("NN-UKF",      "NN-UKF (force)",          "#dd8452"),
    ("Fused-UKF",   "Fused-UKF (deployed)",    "#59a14f"),
]
GROUPS = [("clay", "Clay"), ("dirt", "Dirt"), ("sand", "Sand"), ("ALL", "Overall")]


def plot_figures(csv_path: Path = CSV, out_path: Path = OUT) -> Path:
    df = pd.read_csv(csv_path)
    rows = []
    for key, label, colour in STYLE:
        m = df[df["backend"].str.startswith(key)]
        if m.empty:
            continue
        r = m.iloc[0]
        rows.append((label, colour, [float(r[g]) for g, _ in GROUPS]))

    x = np.arange(len(GROUPS))
    nb = len(rows)
    w = 0.8 / nb
    fig, ax = plt.subplots(figsize=(7.4, 4.3))
    for i, (label, colour, vals) in enumerate(rows):
        xb = x - 0.4 + w * (i + 0.5)
        bars = ax.bar(xb, vals, w, label=label, color=colour)
        for xi, v in zip(xb, vals):
            ax.text(xi, v + 0.006, f"{v:.3f}", ha="center", va="bottom",
                    fontsize=7.5)
    ax.axhline(0.10, color="0.5", lw=0.8, ls="--")
    ax.text(len(GROUPS) - 0.5, 0.104, "0.10", color="0.4", fontsize=7,
            ha="right", va="bottom")
    ax.set_xticks(x)
    ax.set_xticklabels([g[1] for g in GROUPS])
    ax.set_ylabel(r"closed-loop tail $|\Delta n|$")
    ax.set_ylim(0, max(0.5, max(max(v) for _, _, v in rows) * 1.12))
    ax.set_title("Live in-loop terrain estimation, four estimators\n"
                 "(force-only UKFs fail firm sand; the deployed force+proprioceptive Fused-UKF tracks all soils)",
                 fontsize=10.5)
    ax.legend(loc="upper left", fontsize=9, framealpha=0.92)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    plt.close(fig)
    return out_path


if __name__ == "__main__":
    p = plot_figures()
    print(f"Wrote {p}")
