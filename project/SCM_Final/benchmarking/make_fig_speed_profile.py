#!/usr/bin/env python3
"""Paper figure: terrain-aware g-g speed profile vs the curvature heuristic.

Source: ``my_paper/paper_figures/speed_profile_ablation.csv`` -- a closed-loop
A/B over clay/dirt/sand x {sinusoidal, double_lane_change} x {5, 7} m/s
(2 seeds, averaged), comparing the legacy geometric curvature-limited speed
reference (``--legacy-speed-ref``) against the deployed friction-circle (g-g)
profile whose grip limits come live from the tyre surrogate at the estimated
terrain n_hat.

Panel A: per-cell RMS CTE, curvature vs g-g (log-log, y=x diagonal). The g-g
profile pulls the worst cells (high-speed sinusoid on soft soil, where the
geometric heuristic overspeeds the corner) down to the trackable cluster, and
is neutral-to-marginally-conservative on the already-easy cells.

Panel B: the speed/accuracy Pareto -- each cell's curvature operating point
(open) and g-g operating point (filled) joined by an arrow. g-g trades a small
amount of mean speed for a large CTE reduction on the demanding cells.
"""
from __future__ import annotations
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
SRC_CSV = ROOT / "my_paper" / "paper_figures" / "speed_profile_ablation.csv"
OUT = ROOT / "my_paper" / "paper_figures" / "speed_profile_gg.png"

TERRAIN_COLOR = {"clay": "#1f77b4", "dirt": "#8c564b", "sand": "#d4a017"}
PATH_MARKER = {"sinusoidal": "o", "double_lane_change": "s"}
PATH_LABEL = {"sinusoidal": "sinusoidal", "double_lane_change": "double lane change"}


def main():
    df = pd.read_csv(SRC_CSV)

    fig, (axA, axB) = plt.subplots(1, 2, figsize=(9.2, 4.2))

    # --- Panel A: curvature vs g-g RMS CTE (log-log) ---
    lim_lo, lim_hi = 0.03, 0.8
    axA.plot([lim_lo, lim_hi], [lim_lo, lim_hi], color="0.5", lw=1.0, ls="--",
             zorder=1, label="y = x (no change)")
    for _, r in df.iterrows():
        axA.scatter(r["cte_baseline"], r["cte_terrain"],
                    color=TERRAIN_COLOR[r["terrain"]],
                    marker=PATH_MARKER[r["path"]], s=46, alpha=0.85,
                    edgecolor="black", linewidth=0.4, zorder=3)
    # annotate the two worst curvature cells (the headline wins)
    worst = df.nlargest(2, "cte_baseline")
    for _, r in worst.iterrows():
        axA.annotate(f"{r['terrain']} {PATH_LABEL[r['path']]}\n{r['speed']:.0f} m/s",
                     (r["cte_baseline"], r["cte_terrain"]),
                     textcoords="offset points", xytext=(-6, 8), fontsize=7.0,
                     ha="right", color="0.2")
    axA.set_xscale("log"); axA.set_yscale("log")
    axA.set_xlim(lim_lo, lim_hi); axA.set_ylim(lim_lo, lim_hi)
    axA.set_xlabel("RMS CTE, curvature reference (m)")
    axA.set_ylabel("RMS CTE, g-g profile (m)")
    axA.set_title("(a) Per-cell tracking error")
    axA.grid(True, which="both", alpha=0.25)
    # below-diagonal region note
    axA.text(0.5, 0.06, "below line: g-g better", fontsize=7.5, color="0.35",
             ha="center")

    # legend: terrain colors + path markers
    terrain_handles = [plt.Line2D([0], [0], marker="o", ls="", color=c,
                                  markeredgecolor="black", label=t)
                       for t, c in TERRAIN_COLOR.items()]
    path_handles = [plt.Line2D([0], [0], marker=m, ls="", color="0.5",
                               markeredgecolor="black", label=PATH_LABEL[p])
                    for p, m in PATH_MARKER.items()]
    axA.legend(handles=terrain_handles + path_handles, fontsize=7.0,
               loc="upper left", framealpha=0.9)

    # --- Panel B: speed/accuracy Pareto, per cell (arrows curvature -> g-g) ---
    for _, r in df.iterrows():
        c = TERRAIN_COLOR[r["terrain"]]
        axB.annotate("", xy=(r["speed_terrain"], r["cte_terrain"]),
                     xytext=(r["speed_baseline"], r["cte_baseline"]),
                     arrowprops=dict(arrowstyle="->", color=c, alpha=0.55, lw=1.0))
        axB.scatter(r["speed_baseline"], r["cte_baseline"], facecolor="white",
                    edgecolor=c, marker=PATH_MARKER[r["path"]], s=42,
                    linewidth=1.1, zorder=3)
        axB.scatter(r["speed_terrain"], r["cte_terrain"], color=c,
                    marker=PATH_MARKER[r["path"]], s=42, edgecolor="black",
                    linewidth=0.4, zorder=4)
    axB.set_xlabel("Mean forward speed (m/s)")
    axB.set_ylabel("RMS CTE (m)")
    axB.set_title("(b) Speed / accuracy trade")
    axB.grid(True, alpha=0.25)
    axB.text(0.97, 0.95, "open = curvature\nfilled = g-g",
             transform=axB.transAxes, fontsize=7.5, va="top", ha="right",
             color="0.25", bbox=dict(boxstyle="round", fc="white", ec="0.7",
                                     alpha=0.9))

    fig.tight_layout()
    fig.savefig(OUT, dpi=220)
    print(f"wrote {OUT}")

    # --- summary stats for the paper text ---
    mb, mt = df["cte_baseline"].mean(), df["cte_terrain"].mean()
    sb, st = df["speed_baseline"].mean(), df["speed_terrain"].mean()
    print(f"mean RMS CTE: curvature {mb:.3f} m  ->  g-g {mt:.3f} m  "
          f"({100*(mt-mb)/mb:+.0f}%)")
    print(f"mean speed:   curvature {sb:.2f} m/s ->  g-g {st:.2f} m/s "
          f"({100*(st-sb)/sb:+.0f}%)")
    for _, r in df.nlargest(2, "cte_baseline").iterrows():
        print(f"  worst cell {r['terrain']:>4} {r['path']:>18} v{r['speed']:.0f}: "
              f"{r['cte_baseline']:.3f} -> {r['cte_terrain']:.3f} m "
              f"({100*(r['cte_terrain']-r['cte_baseline'])/r['cte_baseline']:+.0f}%)")


if __name__ == "__main__":
    main()
