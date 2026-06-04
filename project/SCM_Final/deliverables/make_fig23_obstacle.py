#!/usr/bin/env python3
"""Render figures 2 and 3: obstacle-avoidance trajectories.

Figure 2: NMPC blind to obstacles, safety filter does the dodging.
          Compares DOB-CBF vs MPPI.
Figure 3: NMPC aware of obstacles, no safety filter.
          Compares Pacejka analytical vs rig NN vs whole-vehicle NN
          tire models.
"""

import json
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


def load_trajectory(tag: str) -> pd.DataFrame:
    csv = RUNS / tag / "sim_diag.csv"
    return pd.read_csv(csv)


def load_reference() -> tuple[np.ndarray, np.ndarray]:
    """Reference sinusoidal path used by the runs."""
    # Find the reference CSV any run wrote out
    for tag in sorted(RUNS.iterdir()):
        if not tag.is_dir():
            continue
        for ref in tag.rglob("reference_path_sinusoidal.csv"):
            df = pd.read_csv(ref)
            # Columns are typically x_ref, y_ref, psi_ref, v_ref or similar
            xcol = next((c for c in df.columns if c.lower() in ("x", "x_ref")), df.columns[0])
            ycol = next((c for c in df.columns if c.lower() in ("y", "y_ref")), df.columns[1])
            return df[xcol].to_numpy(), df[ycol].to_numpy()
    return np.array([]), np.array([])


def load_rocks() -> list[tuple[float, float, float]]:
    with open(RUNS / "rocks.json") as f:
        return json.load(f)


def draw_rocks(ax, rocks):
    for x, y, d in rocks:
        # Physical footprint at SCM ground level
        circle = mpatches.Circle((x, y), d / 2.0, facecolor="#dadada",
                                 edgecolor="#444", linewidth=1.0, alpha=0.85,
                                 zorder=2)
        ax.add_patch(circle)
    # Once, for legend
    ax.add_patch(mpatches.Circle((-1000, -1000), 0.5,
                                 facecolor="#dadada", edgecolor="#444",
                                 linewidth=1.0, label="Rock obstacle"))


def style_axes(ax, title, *, ymax=13.0):
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(title, fontsize=10)
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlim(-2, 55)
    ax.set_ylim(-ymax, ymax)


def collisions_in(tag: str) -> int:
    log = (RUNS / tag / "run.log").read_text(errors="replace")
    # "Total collisions: N" or fallback to counting collision events
    import re
    m = re.search(r"Total collision events:\s*(\d+)", log)
    if m:
        return int(m.group(1))
    m = re.search(r"Total collisions:\s*(\d+)", log)
    if m:
        return int(m.group(1))
    return log.count("[COLLISION]")


def trim(df: pd.DataFrame) -> pd.DataFrame:
    # Drop the 5 s lead-in so the figure focuses on the rock-field traversal.
    return df[df["time"] >= 5.0].reset_index(drop=True)


# ---------------- Figure 2 ----------------------------------------------------

def make_fig2():
    rocks = load_rocks()
    ref_x, ref_y = load_reference()

    cases = [
        ("fig2_dob_cbf_blind", "DOB-CBF", "#1f78b4"),
        ("fig2_mppi_blind",    "MPPI",    "#d95f02"),
    ]

    fig, axes = plt.subplots(2, 1, figsize=(11.5, 7.4), sharex=True)
    for ax, (tag, label, color) in zip(axes, cases):
        df = trim(load_trajectory(tag))
        draw_rocks(ax, rocks)
        if ref_x.size:
            ax.plot(ref_x, ref_y, color="#888", lw=1.1, ls="--",
                    label="Reference (through rocks)", zorder=3)
        ax.plot(df["x"].to_numpy(), df["y"].to_numpy(),
                color=color, lw=2.0, label=f"{label} actual path",
                zorder=5)
        coll = collisions_in(tag)
        style_axes(ax, f"NMPC blind, shield = {label}  (collision events: {coll})")
        ax.legend(loc="upper right", fontsize=8, framealpha=0.9, ncol=1)

    fig.suptitle("Figure 2 — Shield-only obstacle avoidance (NMPC blind to rocks)",
                 fontsize=12, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    out = OUT / "fig2_shield_obstacle_avoidance.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    print(f"Wrote {out}")


# ---------------- Figure 3 ----------------------------------------------------

def make_fig3():
    rocks = load_rocks()
    ref_x, ref_y = load_reference()

    cases = [
        ("fig3_pacejka_aware", "Pacejka (analytical)", "#b2182b"),
        ("fig3_rig_aware",     "Tire-rig NN surrogate", "#1f78b4"),
        ("fig3_vehicle_aware", "Whole-vehicle NN surrogate", "#1b7837"),
    ]

    fig, ax = plt.subplots(figsize=(11.5, 5.4))
    draw_rocks(ax, rocks)
    if ref_x.size:
        ax.plot(ref_x, ref_y, color="#888", lw=1.1, ls="--",
                label="Reference (through rocks)", zorder=3)
    for tag, label, color in cases:
        df = trim(load_trajectory(tag))
        coll = collisions_in(tag)
        ax.plot(df["x"].to_numpy(), df["y"].to_numpy(),
                color=color, lw=2.0,
                label=f"{label}  (collisions: {coll})",
                zorder=5)
    style_axes(ax, "Figure 3 — Planner-aware obstacle avoidance, no downstream shield")
    ax.legend(loc="upper right", fontsize=9, framealpha=0.9, ncol=1)

    fig.tight_layout()
    out = OUT / "fig3_planner_aware_tire_comparison.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    print(f"Wrote {out}")


if __name__ == "__main__":
    make_fig2()
    make_fig3()
