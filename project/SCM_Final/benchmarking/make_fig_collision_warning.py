#!/usr/bin/env python3
"""Render two figures for the collision-warning section.

  fig_cw_timeline.png      — severity escalation over time for one scenario
                             per terrain (latency = 150 ms), with the
                             actual chassis-rock impact time annotated.
  fig_cw_lead_vs_terrain.png — bar chart of RED-warning lead time
                             (t_collision − t_red) per (terrain, latency).
"""
from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
RESULTS_DIR = sorted((ROOT / "benchmarking" / "results").glob(
    "collision_warning_*"))[-1]
OUT_DIR = ROOT / "my_paper" / "paper_figures"
OUT_DIR.mkdir(parents=True, exist_ok=True)

TERRAINS = ("clay", "dirt", "sand")
TRUE_N = {"clay": 0.50, "dirt": 0.70, "sand": 1.10}
COLORS = {"clay": "#1f77b4", "dirt": "#8c564b", "sand": "#d4a017"}
SEVERITY_LABELS = {0: "GREEN", 1: "YELLOW", 2: "ORANGE", 3: "RED"}
SEVERITY_BG = {0: "#dfeede", 1: "#fff3cd", 2: "#ffd6a8", 3: "#f8c7c4"}


def _load_run(terr, lat_ms):
    p = RESULTS_DIR / f"{terr}_lat{lat_ms:03d}_s7" / "trace.csv"
    if not p.exists():
        return None
    return pd.read_csv(p)


def _load_summary():
    return pd.read_csv(RESULTS_DIR / "results.csv")


def render_timeline():
    # Wide 1x3 (clay/dirt/sand side by side) so the figure is page-friendly
    # full-width rather than a tall single-column stack.
    fig, axes = plt.subplots(1, 3, figsize=(13.8, 4.0), sharey=True)
    summary = _load_summary()
    for ax, terr in zip(axes, TERRAINS):
        df = _load_run(terr, 150)
        if df is None or df.empty:
            ax.text(0.5, 0.5, f"no data for {terr}",
                    transform=ax.transAxes, ha="center")
            continue
        t = df["t"].to_numpy()
        sev = df["severity"].to_numpy()
        # Background shading: severity bands as colored rectangles
        for i in range(len(t) - 1):
            ax.axvspan(t[i], t[i + 1], color=SEVERITY_BG[int(sev[i])], alpha=0.5)
        # Stop distance + clearance
        ax.plot(t, df["clearance"].to_numpy(), color="#222", lw=1.8,
                label="clearance to rock (m)")
        ax.plot(t, df["stopping_distance"].to_numpy(), color="#d62728",
                lw=1.5, ls="--", label="required stop dist (m)")
        # Annotate collision and RED warning fire
        row = summary[(summary.terrain == terr) & (summary.latency_s == 0.15)]
        if not row.empty:
            t_red = float(row.iloc[0].t_first_red)
            t_col = float(row.iloc[0].t_collision)
            if np.isfinite(t_red):
                ax.axvline(t_red, color="#d62728", lw=1.5, ls=":",
                           label=f"first RED  {t_red:.2f}s")
            if np.isfinite(t_col):
                ax.axvline(t_col, color="black", lw=1.5,
                           label=f"impact  {t_col:.2f}s")
        ax.set_title(f"{terr} (n={TRUE_N[terr]:.2f})", fontsize=10)
        ax.set_ylim(-1, 28)
        ax.set_xlim(0, 9.0)
        ax.grid(alpha=0.3)
        ax.set_xlabel("sim time (s)", fontsize=10)
        ax.legend(loc="upper right", fontsize=7.5, framealpha=0.95)
    axes[0].set_ylabel("clearance / required stop distance (m)", fontsize=10)
    fig.suptitle("Collision-warning timeline (latency = 150 ms); "
                 "background shading = severity GREEN→YELLOW→ORANGE→RED",
                 fontsize=10.5)
    fig.tight_layout()
    out = OUT_DIR / "cw_timeline.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    print(f"Wrote {out}")


def render_lead_vs_terrain():
    df = _load_summary().copy()
    df["lat_ms"] = (df["latency_s"] * 1000).round().astype(int)
    pivot = df.pivot_table(
        index="terrain", columns="lat_ms",
        values="lead_red_s", aggfunc="mean",
    ).reindex(TERRAINS)

    fig, ax = plt.subplots(figsize=(7.5, 4.6))
    lat_levels = list(pivot.columns)
    x = np.arange(len(TERRAINS))
    width = 0.25
    for i, lat_ms in enumerate(lat_levels):
        bars = ax.bar(x + (i - 1) * width, pivot[lat_ms].values, width,
                      color=plt.cm.viridis((i + 1) / (len(lat_levels) + 1)),
                      label=f"latency = {lat_ms} ms")
        for b, val in zip(bars, pivot[lat_ms].values):
            if np.isfinite(val):
                ax.text(b.get_x() + b.get_width() / 2, val + 0.03,
                        f"{val:.2f}", ha="center", va="bottom", fontsize=9)
    ax.set_xticks(x)
    ax.set_xticklabels([f"{t}\n(n={TRUE_N[t]:.2f})" for t in TERRAINS])
    ax.set_ylabel("RED-warning lead time before impact (s)", fontsize=11)
    ax.set_title("Forward collision warning lead time — "
                 "terrain × latency sweep\n"
                 "soft soil and high jitter both fire the warning earlier",
                 fontsize=11)
    ax.axhline(0, color="#444", lw=0.5)
    ax.grid(axis="y", alpha=0.3)
    ax.legend(loc="upper right", fontsize=10, framealpha=0.95)
    fig.tight_layout()
    out = OUT_DIR / "cw_lead_vs_terrain.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    print(f"Wrote {out}")


def main():
    print(f"Reading from {RESULTS_DIR}")
    render_lead_vs_terrain()
    render_timeline()


if __name__ == "__main__":
    main()
