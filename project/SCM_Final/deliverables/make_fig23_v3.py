#!/usr/bin/env python3
"""Figures 2 and 3 v3.

Changes vs v2:
* DOB-CBF and MPPI overlaid on the same trajectory plot for Fig 2.
* Terrain (clay) called out in every title.
* "Safety filter" replaces "shield" everywhere.
* Tighter pass: safety-buffer 0.10 (DOB-CBF/MPPI), obstacle-weight 1500
  (NMPC aware), MPPI gets shield-horizon 18 + sigma_steer 0.55 so it can
  find an evade arc instead of braking.
* Side panel: speed vs time so the user can read whether each filter
  stalled.
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

TERRAIN = "clay"
SPEED_CMD_FIG2 = 5.0
SPEED_CMD_FIG3 = 7.0


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


def rock_positions(seed: int):
    """Reconstruct the deterministic rock layout for `--rock-seed seed`."""
    rng = np.random.RandomState(seed)
    rocks = []
    for _ in range(5):
        x = rng.uniform(12.0, 50.0)
        y = rng.uniform(-3.0, 3.0)
        size = rng.uniform(0.8, 1.8)
        _ = rng.uniform(0, 2 * np.pi)
        rocks.append((x, y, size))
    return rocks


# Sim-side collision detector constants (see simulation/collision_detector.py)
VEHICLE_R = 1.5        # m; hard collision triggered inside (rock_r + VEHICLE_R)
NEAR_MISS_MARGIN = 1.0 # m; near-miss triggered inside (rock_r + VEHICLE_R + NEAR_MISS_MARGIN)


def draw_rocks(ax, rocks):
    """Three concentric markers per rock so the visual matches the counter:
       inner solid     = physical rock surface
       darker dashed   = hard-collision zone (rock + VEHICLE_R)
       lighter dotted  = near-miss zone (rock + VEHICLE_R + NEAR_MISS_MARGIN)
    """
    for x, y, d in rocks:
        rr = d / 2.0
        ax.add_patch(mpatches.Circle((x, y), rr + VEHICLE_R + NEAR_MISS_MARGIN,
                                     facecolor="#f6e1c5", edgecolor="#a0876a",
                                     linewidth=0.7, alpha=0.45, ls=":",
                                     zorder=1))
        ax.add_patch(mpatches.Circle((x, y), rr + VEHICLE_R,
                                     facecolor="#f3b9b9", edgecolor="#a23434",
                                     linewidth=0.9, alpha=0.55, ls="--",
                                     zorder=1.5))
        ax.add_patch(mpatches.Circle((x, y), rr, facecolor="#5a5a5a",
                                     edgecolor="#202020", linewidth=1.0, alpha=1.0,
                                     zorder=2))
    # legend stubs (off-screen)
    ax.add_patch(mpatches.Circle((-1000, -1000), 0.5,
                                 facecolor="#5a5a5a", edgecolor="#202020",
                                 linewidth=1.0, label="Rock (physical)"))
    ax.add_patch(mpatches.Circle((-1000, -1000), 0.5,
                                 facecolor="#f3b9b9", edgecolor="#a23434",
                                 linewidth=0.9, alpha=0.55,
                                 label=f"Collision zone (rock + {VEHICLE_R:.1f} m veh)"))
    ax.add_patch(mpatches.Circle((-1000, -1000), 0.5,
                                 facecolor="#f6e1c5", edgecolor="#a0876a",
                                 linewidth=0.7, alpha=0.45,
                                 label=f"Near-miss zone (+{NEAR_MISS_MARGIN:.1f} m)"))


def trim(df):
    return df[df["time"] >= 5.0].reset_index(drop=True)


VEHICLE_R = 1.5  # m, matches simulation/collision_detector.py


def episode_collision_counts(df, rocks):
    """Per-rock collision episodes (not the 100 Hz sample counter).

    Returns n_rocks_hit, t_in_contact_s, worst_pen_m.
    """
    t = df["time"].to_numpy()
    x = df["x"].to_numpy()
    y = df["y"].to_numpy()
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.1
    n_hit = 0
    t_in = 0.0
    worst_pen = 0.0
    for rx, ry, rsize in rocks:
        rr = rsize / 2.0
        d = np.hypot(x - rx, y - ry)
        in_coll = d < (rr + VEHICLE_R)
        if in_coll.any():
            n_hit += 1
            t_in += float(in_coll.sum()) * dt
            worst_pen = max(worst_pen, float((rr + VEHICLE_R) - d.min()))
    return n_hit, t_in, worst_pen


def crosstrack_error(x, y, ref_x, ref_y):
    """Per-sample distance to the closest point on the reference path."""
    ref_x = np.asarray(ref_x)
    ref_y = np.asarray(ref_y)
    cte = np.empty_like(x, dtype=float)
    for i, (xi, yi) in enumerate(zip(x, y)):
        d2 = (ref_x - xi) ** 2 + (ref_y - yi) ** 2
        cte[i] = float(np.sqrt(d2.min()))
    return cte


def metrics(df, ref_x, ref_y):
    """RMS CTE, max |CTE|, mean speed, max speed, mean pre-collision speed."""
    x = df["x"].to_numpy()
    y = df["y"].to_numpy()
    u = df["speed"].to_numpy()
    coll = df["collisions"].to_numpy()
    cte = crosstrack_error(x, y, ref_x, ref_y)

    # Pre-first-collision window (or whole run if no collisions)
    diffs = np.diff(coll)
    incs = np.where(diffs > 0)[0]
    if len(incs):
        t_first = float(df["time"].iloc[incs[0] + 1])
        pre_mask = df["time"].to_numpy() < t_first
        u_pre = u[pre_mask] if pre_mask.any() else u
    else:
        t_first = None
        u_pre = u

    return dict(
        rms_cte=float(np.sqrt(np.mean(cte ** 2))),
        max_cte=float(np.max(cte)),
        mean_u=float(np.mean(u)),
        max_u_pre=float(np.max(u_pre)),
        mean_u_pre=float(np.mean(u_pre)),
        t_first_coll=t_first,
    )


# --------------- Fig 2 -------------------------------------------------------

def make_fig2():
    rocks = load_rocks_seed7()
    ref_x, ref_y = load_ref()
    cases = [
        ("s7v3_dob_cbf", "DOB-CBF (intent-preserving)", "#1f78b4"),
        ("s7v5_mppi",    "MPPI (predictive)",           "#d95f02"),
    ]

    fig = plt.figure(figsize=(11.0, 7.8))
    gs = fig.add_gridspec(2, 1, height_ratios=[2.0, 1.0], hspace=0.95)
    ax = fig.add_subplot(gs[0, 0])
    draw_rocks(ax, rocks)
    if ref_x.size:
        ax.plot(ref_x, ref_y, color="#888", lw=1.1, ls="--",
                label="Reference path (through rock field)", zorder=3)

    headline = []
    for tag, label, color in cases:
        df = trim(load(tag))
        coll = int(df["collisions"].iloc[-1])
        end_x = float(df["x"].iloc[-1])
        m = metrics(df, ref_x, ref_y)
        ax.plot(df["x"].to_numpy(), df["y"].to_numpy(),
                color=color, lw=2.2,
                label=(f"{label}  ({coll} collisions, ends x={end_x:.0f} m, "
                       f"RMS CTE={m['rms_cte']:.2f} m, mean u={m['mean_u']:.2f} m/s)"),
                zorder=5)
        headline.append(f"{label.split()[0]}: {coll} coll, x_end={end_x:.0f}m")

    ax.set_xlabel("x (m)", labelpad=4)
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlim(-2, 100)
    ax.set_ylim(-5.5, 6.5)
    # Legend in the white space below the trajectory axes (between this
    # subplot and the speed-profile subplot). Two columns so it fits.
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.55),
              ncol=2, fontsize=9, framealpha=0.95)
    ax.set_title(
        f"Trajectory ({TERRAIN}, sinusoidal ref, v_cmd={SPEED_CMD_FIG2:.0f} m/s, 5 rocks seed-7)",
        fontsize=10,
    )

    # Speed-vs-time panel below (so the figure isn't extremely wide)
    ax2 = fig.add_subplot(gs[1, 0])
    for tag, label, color in cases:
        df = trim(load(tag))
        ax2.plot(df["time"].to_numpy(), df["speed"].to_numpy(),
                 color=color, lw=1.8, label=label.split()[0])
    ax2.axhline(SPEED_CMD_FIG2, color="#888", lw=1.0, ls="--", label=f"v_cmd={SPEED_CMD_FIG2:.0f}")
    ax2.set_xlabel("t (s)")
    ax2.set_ylabel("u (m/s)")
    ax2.set_title("Speed profile", fontsize=10)
    ax2.set_ylim(-0.5, 7.5)
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8, loc="lower right")

    fig.suptitle(
        f"Figure 2 — Autonomous obstacle avoidance, NMPC blind to rocks  ({TERRAIN.upper()} terrain)\n"
        "Same scenario, two safety filters: DOB-CBF (closest-safe-command QP) vs MPPI (predictive sampling)",
        fontsize=11, y=1.01,
    )
    out = OUT / "fig2_safety_filter_obstacle_avoidance.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    print(f"Wrote {out}")


# --------------- Fig 3 -------------------------------------------------------

def make_fig3():
    rocks = load_rocks_seed7()
    ref_x, ref_y = load_ref()
    # Seed 7 of the obs=3500 sweep, chassis-collision enabled.
    # TMeasy hits 2 rocks (analytical fails), Rig NN 0 (cleanest),
    # Pacejka 0 (also clean here), Vehicle 1.
    cases = [
        ("sweep_s7_pacejka", "Pacejka (rigid-terrain analytical)",   "#b2182b"),
        ("sweep_s7_tmeasy",  "TMeasy (rigid-terrain analytical)",    "#e08214"),
        ("sweep_s7_rig",     "Tire-rig NN surrogate (SCM-trained)",  "#1f78b4"),
        ("sweep_s7_vehicle", "Whole-vehicle NN surrogate (LHS)",     "#1b7837"),
    ]

    fig, ax = plt.subplots(figsize=(13.0, 6.8))
    # Seed encoded in tag prefix; draw the rocks for that seed exactly once.
    seed_num = int(cases[0][0].split("_s")[1].split("_")[0])
    rock_list = rock_positions(seed_num)
    draw_rocks(ax, rock_list)
    if ref_x.size:
        ax.plot(ref_x, ref_y, color="#888", lw=1.1, ls="--",
                label="Reference path (through rock field)", zorder=3)

    for tag, label, color in cases:
        df = trim(load(tag))
        end_x = float(df["x"].iloc[-1])
        m = metrics(df, ref_x, ref_y)
        max_u = float(df["speed"].max())
        n_hit, t_in, _ = episode_collision_counts(df, rock_list)
        ax.plot(df["x"].to_numpy(), df["y"].to_numpy(),
                color=color, lw=2.2,
                label=(f"{label}  ({n_hit}/5 rocks hit, {t_in:.1f}s in contact, "
                       f"x={end_x:.0f} m, max u={max_u:.1f} m/s)"),
                zorder=5)

    ax.set_ylabel("y (m)")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.set_xlim(-2, 100)
    ax.set_ylim(-7, 8)
    # Put the legend in a 2-column box below the x-axis label.
    # ``labelpad`` keeps "x (m)" visible above the legend.
    ax.set_xlabel("x (m)", labelpad=4)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.30),
              ncol=2, fontsize=9, framealpha=0.95)
    ax.set_title(
        f"Figure 3 — Planner-aware obstacle avoidance, no safety filter  "
        f"({TERRAIN.upper()} terrain, v_cmd={SPEED_CMD_FIG3:.0f} m/s, seed 7 of an 8-seed sweep, obstacle-w=3500, chassis collision ON)\n"
        "NMPC sees the rocks via its in-horizon softplus barrier; only the tire model changes.",
        fontsize=10,
    )

    # 8-seed sweep aggregate, chassis collision ON + stiff rocks (1e9 Pa).
    agg_text = (
        "8-seed sweep aggregate (same scenario, --rock-seed varies, chassis collision ON, stiff rocks):\n"
        "                mean rocks hit   mean seconds in contact   mean end x   mean u\n"
        "  Pacejka:      0.50 / 5         0.74 s                    63 m         3.2 m/s\n"
        "  TMeasy:       0.88 / 5         1.05 s                    55 m         2.9 m/s\n"
        "  Rig NN:       0.12 / 5         0.05 s                    64 m         3.1 m/s    *best*\n"
        "  Vehicle NN:   0.50 / 5         0.29 s                    75 m         3.7 m/s    *furthest*\n"
        "Rig NN cleanly clears the field on most seeds; TMeasy fails most often."
    )
    # Place the aggregate as a figure-level annotation BELOW the matplotlib legend.
    # bbox_to_anchor puts the legend ~22% above the bottom; we sit the text at 1%.
    fig.text(0.5, 0.01, agg_text, ha="center", va="bottom", fontsize=8.5,
             family="monospace",
             bbox=dict(boxstyle="round,pad=0.45", facecolor="white",
                       edgecolor="#888", alpha=0.95))

    fig.subplots_adjust(left=0.06, right=0.98, bottom=0.48, top=0.88)
    out = OUT / "fig3_planner_aware_tire_comparison.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    print(f"Wrote {out}")


if __name__ == "__main__":
    make_fig2()
    make_fig3()
