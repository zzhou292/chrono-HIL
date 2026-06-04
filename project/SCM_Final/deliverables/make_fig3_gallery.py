#!/usr/bin/env python3
"""Render a Fig 3 variant for each of the 8 seeds in the chassis-collision
sweep. Saves to deliverables/figures/fig3_seed{1..8}.png so the user can
pick which seed to keep as the canonical figure."""

import json
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# Reuse helpers from make_fig23_v3.
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from make_fig23_v3 import (  # noqa: E402
    VEHICLE_R, NEAR_MISS_MARGIN, draw_rocks, episode_collision_counts,
    metrics, rock_positions, load_ref, trim,
)

RUNS = HERE / "runs"
OUT_DIR = HERE / "figures"
OUT_DIR.mkdir(parents=True, exist_ok=True)

CASES_TEMPLATE = [
    ("pacejka", "Pacejka (rigid-terrain analytical)",   "#b2182b"),
    ("tmeasy",  "TMeasy (rigid-terrain analytical)",    "#e08214"),
    ("rig",     "Tire-rig NN surrogate (SCM-trained)",  "#1f78b4"),
    ("vehicle", "Whole-vehicle NN surrogate (LHS)",     "#1b7837"),
]


def load(tag):
    return pd.read_csv(RUNS / tag / "sim_diag.csv")


def render_one(seed: int):
    ref_x, ref_y = load_ref()
    rock_list = rock_positions(seed)

    fig, ax = plt.subplots(figsize=(13.0, 6.4))
    draw_rocks(ax, rock_list)
    if ref_x.size:
        ax.plot(ref_x, ref_y, color="#888", lw=1.1, ls="--",
                label="Reference path (through rock field)", zorder=3)

    summary_rows = []
    for short, label, color in CASES_TEMPLATE:
        tag = f"sweep_s{seed}_{short}"
        df = trim(load(tag))
        end_x = float(df["x"].iloc[-1])
        max_u = float(df["speed"].max())
        m = metrics(df, ref_x, ref_y)
        n_hit, t_in, _ = episode_collision_counts(df, rock_list)
        ax.plot(df["x"].to_numpy(), df["y"].to_numpy(),
                color=color, lw=2.2,
                label=(f"{label}  ({n_hit}/5 rocks hit, {t_in:.1f}s in contact, "
                       f"x={end_x:.0f} m, max u={max_u:.1f} m/s)"),
                zorder=5)
        summary_rows.append((short, n_hit, t_in, end_x, max_u, m["mean_u"], m["rms_cte"]))

    ax.set_xlabel("x (m)", labelpad=4)
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlim(-2, 100)
    ax.set_ylim(-7, 8)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.22),
              ncol=2, fontsize=9, framealpha=0.95)
    ax.set_title(
        f"Fig 3 candidate — SEED {seed}  "
        f"(clay terrain, v_cmd = 7 m/s, obstacle-w = 3500, chassis collision ON)",
        fontsize=10,
    )
    fig.subplots_adjust(left=0.06, right=0.98, bottom=0.30, top=0.91)
    out = OUT_DIR / f"fig3_seed{seed}.png"
    fig.savefig(out, dpi=170, bbox_inches="tight")
    plt.close(fig)

    print(f"seed {seed}: {[(s, h, f'{t:.1f}s') for s,h,t,_,_,_,_ in summary_rows]}")
    return out


def main():
    for seed in range(1, 9):
        render_one(seed)


if __name__ == "__main__":
    main()
