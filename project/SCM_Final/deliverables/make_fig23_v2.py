#!/usr/bin/env python3
"""Figures 2 and 3 v2 — seed 7, 25-second sim so the safety filters and
NMPC actually have time to clear the rock cluster and return toward the
sinusoidal reference.

Adds a speed-vs-time inset to Fig 2 so we can see *why* DOB-CBF and MPPI
differ (DOB-CBF keeps moving; MPPI brakes hard).
"""

import json
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
RUNS = Path(__file__).parent / "runs"
OUT = Path(__file__).parent / "figures"
OUT.mkdir(parents=True, exist_ok=True)


def load(tag):
    return pd.read_csv(RUNS / tag / "sim_diag.csv")


def load_ref():
    for tag in sorted(RUNS.iterdir()):
        if tag.is_dir():
            for ref in tag.rglob("reference_path_sinusoidal.csv"):
                df = pd.read_csv(ref)
                xc = next((c for c in df.columns if c.lower() in ("x", "x_ref")), df.columns[0])
                yc = next((c for c in df.columns if c.lower() in ("y", "y_ref")), df.columns[1])
                return df[xc].to_numpy(), df[yc].to_numpy()
    return np.array([]), np.array([])


def load_rocks_seed7():
    with open(RUNS / "rocks.json") as f:
        return json.load(f)


def draw_rocks(ax, rocks):
    for x, y, d in rocks:
        ax.add_patch(mpatches.Circle((x, y), d / 2.0, facecolor="#dadada",
                                     edgecolor="#444", linewidth=1.0, alpha=0.85,
                                     zorder=2))
    ax.add_patch(mpatches.Circle((-1000, -1000), 0.5,
                                 facecolor="#dadada", edgecolor="#444",
                                 linewidth=1.0, label="Rock obstacle"))


def collisions_in(tag):
    log = (RUNS / tag / "run.log").read_text(errors="replace")
    m = re.search(r"Total collision events:\s*(\d+)", log)
    if m:
        return int(m.group(1))
    return log.count("[COLLISION]")


def trim(df):
    return df[df["time"] >= 5.0].reset_index(drop=True)


def make_fig2():
    rocks = load_rocks_seed7()
    ref_x, ref_y = load_ref()
    cases = [
        ("s7_dob_cbf_blind", "DOB-CBF (intent-preserving)", "#1f78b4"),
        ("s7_mppi_blind",    "MPPI (predictive)",           "#d95f02"),
    ]

    fig = plt.figure(figsize=(13.0, 6.6))
    gs = fig.add_gridspec(2, 2, width_ratios=[3.6, 1.0], hspace=0.32, wspace=0.18)
    for row, (tag, label, color) in enumerate(cases):
        df = trim(load(tag))
        ax = fig.add_subplot(gs[row, 0])
        draw_rocks(ax, rocks)
        if ref_x.size:
            ax.plot(ref_x, ref_y, color="#888", lw=1.1, ls="--",
                    label="Reference (passes through rock field)", zorder=3)
        ax.plot(df["x"].to_numpy(), df["y"].to_numpy(),
                color=color, lw=2.0, label=f"{label} actual path", zorder=5)
        coll = int(df["collisions"].iloc[-1])
        end_x = float(df["x"].iloc[-1])
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        title = label.split(" ")[0]
        ax.set_title(f"NMPC blind, shield = {title}   collisions={coll}, end x = {end_x:.1f} m",
                     fontsize=10)
        ax.set_aspect("equal")
        ax.grid(alpha=0.3)
        ax.set_xlim(-2, 75)
        ax.set_ylim(-9, 9)
        ax.legend(loc="upper right", fontsize=8, framealpha=0.9)

        # Speed-over-time inset on the right column
        ax2 = fig.add_subplot(gs[row, 1])
        ax2.plot(df["time"].to_numpy(), df["speed"].to_numpy(), color=color, lw=1.8)
        ax2.axhline(5.0, color="#888", lw=1.0, ls="--", label="cmd 5 m/s")
        ax2.set_xlabel("t (s)")
        ax2.set_ylabel("u (m/s)")
        ax2.set_title("speed", fontsize=10)
        ax2.set_ylim(-0.5, 7.5)
        ax2.grid(alpha=0.3)
        ax2.legend(fontsize=7, loc="lower right")

    fig.suptitle(
        "Figure 2 — Shield-only autonomous avoidance (NMPC blind to rocks)\n"
        "Seed 7 places 4 of 5 rocks on/near the reference; 25 s sim lets the shield finish the maneuver.",
        fontsize=11, y=0.995,
    )
    out = OUT / "fig2_shield_obstacle_avoidance.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    print(f"Wrote {out}")


def make_fig3():
    rocks = load_rocks_seed7()
    ref_x, ref_y = load_ref()
    cases = [
        ("s7_pacejka_aware", "Pacejka (rigid-terrain analytical)", "#b2182b"),
        ("s7_rig_aware",     "Tire-rig NN surrogate", "#1f78b4"),
        ("s7_vehicle_aware", "Whole-vehicle NN surrogate", "#1b7837"),
    ]

    fig, ax = plt.subplots(figsize=(12.5, 5.6))
    draw_rocks(ax, rocks)
    if ref_x.size:
        ax.plot(ref_x, ref_y, color="#888", lw=1.1, ls="--",
                label="Reference (passes through rock field)", zorder=3)
    for tag, label, color in cases:
        df = trim(load(tag))
        coll = int(df["collisions"].iloc[-1])
        ax.plot(df["x"].to_numpy(), df["y"].to_numpy(),
                color=color, lw=2.0,
                label=f"{label}  (collisions: {coll})", zorder=5)

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(
        "Figure 3 — Planner-aware obstacle avoidance, no downstream shield\n"
        "Same seed-7 rock layout; NMPC sees rocks via the in-horizon softplus barrier.",
        fontsize=10,
    )
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlim(-2, 95)
    ax.set_ylim(-9, 9)
    ax.legend(loc="upper right", fontsize=9, framealpha=0.9)
    fig.tight_layout()
    out = OUT / "fig3_planner_aware_tire_comparison.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    print(f"Wrote {out}")


if __name__ == "__main__":
    make_fig2()
    make_fig3()
