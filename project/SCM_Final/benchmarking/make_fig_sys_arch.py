#!/usr/bin/env python3
"""Codable system-architecture diagram for paper.tex.

Output: ``my_paper/paper_figures/sys_arch.png`` (+ ``.pdf``). Edit
this file, rerun, get a new figure.

Top-down hierarchy:

    row 1 (top)   :  TIRE_MODELS         TERRAIN_ESTIMATORS    (data providers)
    row 2         :  acados NMPC         Operator G29/WASD     (command sources)
    row 3         :  Safety filter   LATENCY_PROFILES   Collision warning   (middleware / plug-in registries)
    row 4 (bottom):                  chrono_sim_node                        (plant)

Every primary edge flows DOWNWARD; the two state-feedback edges run
UPWARD along clean lanes that do not cross the downward flow. No box
is grazed.
"""
from __future__ import annotations

from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


ROOT = Path(__file__).resolve().parents[1]
OUT  = ROOT / "my_paper" / "paper_figures" / "sys_arch.png"
PDF  = ROOT / "my_paper" / "paper_figures" / "sys_arch.pdf"


# --------------------------------------------------------------------
# Style
# --------------------------------------------------------------------
FONT_KW    = dict(family="DejaVu Sans", fontsize=10)
LABEL_FS   = 9.5

CTRL_EDGE  = "#1f77b4"; CTRL_FILL  = "#e6f0fb"
SWAP_EDGE  = "#d4a017"; SWAP_FILL  = "#fff6dc"
HUMAN_EDGE = "#2ca02c"; HUMAN_FILL = "#eaf5ea"
LAT_FILL   = "#fdebeb"
DATA_C     = "#444444"
LAT_C      = "#cc4444"
ADV_C      = "#cc4444"
FB_C       = "#1f77b4"   # feedback (state, camera) — distinct blue


# --------------------------------------------------------------------
# Geometry. The grid is 4 rows tall.
# --------------------------------------------------------------------
# Row y-coordinates (centres) from top to bottom:
Y_ROW1 = 6.0   # providers
Y_ROW2 = 4.3   # command sources
Y_ROW3 = 2.5   # middleware
Y_ROW4 = 0.7   # plant

BOXES = {
    # All display labels follow one rule: Title Case, no underscores
    # (``acados`` is the only intentional lowercase, matching the
    # library's own branding).
    #
    # name      : (cx,   cy,    w,   h,   label,                                                  edge,       fill)
    # ---------- row 1: data providers ----------
    "tire"     : ( 3.0, Y_ROW1, 3.6, 0.95,
                  "Tire Models\n{ rate-MLP | axle-rate | Pacejka | TMeasy }",          SWAP_EDGE,  SWAP_FILL),
    "est"      : ( 9.0, Y_ROW1, 3.6, 0.95,
                  "Terrain Estimators\n{ sliding-window MLP → ñ }",                    SWAP_EDGE,  SWAP_FILL),
    # ---------- left of row 2: terrain-aware speed planner (surrogate grip -> v_ref) ----------
    "planner"  : (-0.75, Y_ROW2, 2.0, 0.95,
                  "g-g Speed\nPlanner",                                                 CTRL_EDGE,  CTRL_FILL),
    # ---------- row 2: command sources ----------
    "nmpc"     : ( 3.0, Y_ROW2, 3.0, 0.95,
                  "acados NMPC",                                                        CTRL_EDGE,  CTRL_FILL),
    "operator" : ( 9.0, Y_ROW2, 3.0, 0.95,
                  "Operator",                                                           HUMAN_EDGE, HUMAN_FILL),
    # ---------- row 3: middleware ----------
    "filter"   : ( 1.7, Y_ROW3, 3.0, 0.95,
                  "Safety Filter\n{ none | DOB-CBF | MPPI | NMPC }",                    SWAP_EDGE,  SWAP_FILL),
    "lat"      : ( 6.0, Y_ROW3, 2.8, 0.95,
                  "Latency Profiles\n{ const | replay | 5G }",                          SWAP_EDGE,  LAT_FILL),
    "cw"       : (10.3, Y_ROW3, 3.0, 0.95,
                  "Collision Warning",                                                  SWAP_EDGE,  SWAP_FILL),
    # ---------- row 4: plant ----------
    "sim"      : ( 6.0, Y_ROW4, 3.4, 0.95,
                  "Chrono Simulation\nHMMWV + SCM patch (Chrono::HIL plant)",           CTRL_EDGE,  CTRL_FILL),
}


def _draw_box(ax, key):
    cx, cy, w, h, label, edge, fill = BOXES[key]
    box = FancyBboxPatch(
        (cx - w / 2, cy - h / 2), w, h,
        boxstyle="round,pad=0.05,rounding_size=0.18",
        linewidth=1.6, edgecolor=edge, facecolor=fill, zorder=2,
    )
    ax.add_patch(box)
    ax.text(cx, cy, label, ha="center", va="center", zorder=3,
            color="black", **FONT_KW)


_BOX_PAD = 0.05   # must match FancyBboxPatch ``boxstyle="round,pad=…"``


def _port(key, side, frac=0.0):
    """Return (x, y) anchor on one side of a box, offset by ``frac``
    along that side (frac=0 = midpoint, frac=±0.5 = corner).

    The returned point lies exactly on the *visible* box outline,
    which is one ``_BOX_PAD`` outside the underlying rectangle.
    Without this offset, FancyArrowPatch arrowheads would land
    slightly inside the visible box (the cause of the previous
    sloppy look).
    """
    cx, cy, w, h, *_ = BOXES[key]
    if side in ("L", "R"):
        x = cx + (w / 2 + _BOX_PAD if side == "R" else -(w / 2 + _BOX_PAD))
        return (x, cy + frac * h)
    x = cx + frac * w
    y = cy + (h / 2 + _BOX_PAD if side == "T" else -(h / 2 + _BOX_PAD))
    return (x, y)


def _arc_label_xy(p_from, p_to, rad):
    """Midpoint of an ``arc3`` Bezier between two points.

    Matches matplotlib's ``Arc3.connect`` exactly: the quadratic
    Bezier control point is at ``(mx + rad*dy, my - rad*dx)`` where
    ``mx, my`` is the chord midpoint and ``dx, dy`` is the chord
    vector. The label position is then the curve at ``t=0.5``,
    ``0.25*p_from + 0.5*control + 0.25*p_to``.
    """
    mx = (p_from[0] + p_to[0]) / 2.0
    my = (p_from[1] + p_to[1]) / 2.0
    dx = p_to[0] - p_from[0]
    dy = p_to[1] - p_from[1]
    cx = mx + rad * dy
    cy = my - rad * dx
    bx = 0.25 * p_from[0] + 0.5 * cx + 0.25 * p_to[0]
    by = 0.25 * p_from[1] + 0.5 * cy + 0.25 * p_to[1]
    return (bx, by)


def _parse_rad(connectionstyle):
    """Extract the ``rad=`` value from an ``arc3,rad=…`` connection
    string. Returns 0.0 if not present."""
    if "rad=" not in connectionstyle:
        return 0.0
    try:
        return float(connectionstyle.split("rad=", 1)[1].split(",", 1)[0])
    except ValueError:
        return 0.0


def _arrow(ax, p_from, p_to, *, label=None, label_xy=None,
           color=DATA_C, linestyle="-", connectionstyle="arc3,rad=0",
           lw=1.6, label_fs=LABEL_FS, label_bg=True):
    arrow = FancyArrowPatch(
        p_from, p_to,
        arrowstyle="-|>", mutation_scale=14,
        linewidth=lw, color=color, linestyle=linestyle,
        connectionstyle=connectionstyle, zorder=4,
    )
    ax.add_patch(arrow)
    if label:
        if label_xy is not None:
            lx, ly = label_xy
        else:
            rad = _parse_rad(connectionstyle)
            lx, ly = _arc_label_xy(p_from, p_to, rad)
        bbox = dict(facecolor="white", edgecolor="none", alpha=0.85,
                    pad=1.5) if label_bg else None
        ax.text(lx, ly, label, ha="center", va="center", zorder=5,
                color=color, fontsize=label_fs,
                family="DejaVu Sans", bbox=bbox)


def main():
    fig, ax = plt.subplots(figsize=(12.8, 6.6))
    ax.set_xlim(-2.0, 12.4)
    ax.set_ylim(-0.2, 6.9)
    ax.set_aspect("equal")
    ax.axis("off")

    for key in BOXES:
        _draw_box(ax, key)

    # ============================================================
    # TOP ROW (row 1) — providers conditioning each other
    # ============================================================
    # Terrain estimator's live n̂ re-conditions the tire surrogate at
    # runtime (the tire NN takes terrain params as input).
    _arrow(ax, _port("est", "L"), _port("tire", "R"),
           label="ñ", linestyle="--")

    # ============================================================
    # ROW 1 → ROW 2 and ROW 3 (providers feed consumers below)
    # ============================================================
    # Tire surrogate → acados NMPC: build-time CasADi export of Fx,Fy.
    _arrow(ax, _port("tire", "B"), _port("nmpc", "T"),
           label="Fx,Fy", linestyle=":")

    # Tire surrogate grip limits (at ñ) → g-g speed planner → NMPC speed
    # reference. The planner sits to the left of the NMPC at the same row.
    _arrow(ax, _port("tire", "B", -0.46), _port("planner", "T", 0.25),
           label="grip(ñ)", linestyle=":", label_fs=8.5,
           connectionstyle="arc3,rad=0.12")
    _arrow(ax, _port("planner", "R"), _port("nmpc", "L"),
           label="v_ref", label_fs=8.5)

    # Tire surrogate → Collision warning: brake-decel a_b(n̂) table.
    # Starts at tire's bottom-RIGHT (so it leaves to the right of
    # Fx,Fy on the same edge) and ends at Collision warning's
    # LEFT side. The curve passes just below Operator's bottom,
    # threading the gap between Operator and the row-3 boxes.
    _arrow(ax, _port("tire", "B", 0.30), _port("cw", "L", 0.30),
           label="a_b(ñ)", linestyle=":",
           connectionstyle="arc3,rad=-0.10")

    # Tire surrogate → Safety filter: per-tick force prediction for
    # the MPPI / DOB-CBF rollouts. Anchored on the LEFT side of each
    # box so the chord runs entirely outside NMPC (which sits between
    # tire and filter when you draw a straight line from centre-to-
    # centre).
    _arrow(ax, _port("tire", "B", -0.45), _port("filter", "T", -0.45),
           label="Fx,Fy", linestyle=":", label_xy=(0.15, 3.45),
           connectionstyle="arc3,rad=0.15")

    # ============================================================
    # COMMAND FLOW (row 2 → row 3 → row 4)
    # ============================================================
    # NMPC writes directly to the safety filter (no latency in the
    # autonomous loop).
    _arrow(ax, _port("nmpc", "B", -0.4), _port("filter", "T", 0.2),
           label="ControlCommand",
           connectionstyle="arc3,rad=-0.10")

    # Operator command first passes THROUGH the latency profile
    # (queue-load delay model). The latency profile then forwards
    # the delayed command to the safety filter.
    _arrow(ax, _port("operator", "B"), _port("lat", "R"),
           label="operator cmd",
           connectionstyle="arc3,rad=-0.10")
    _arrow(ax, _port("lat", "L"), _port("filter", "R"),
           label="delayed cmd",
           color=LAT_C, linestyle=":")

    _arrow(ax, _port("filter", "B"), _port("sim", "L"),
           label="filtered cmd",
           connectionstyle="arc3,rad=0.18")

    # ============================================================
    # UPWARD FEEDBACK (plant → command sources; advisory)
    # ============================================================
    _arrow(ax, _port("sim", "T", -0.45), _port("nmpc", "B", 0.30),
           label="VehicleState",
           label_xy=(3.95, 3.20),   # shifted back down the arrow,
                                     # closer to its midpoint, while
                                     # still above the LATENCY/Filter
                                     # row (y > 2.975)
           color=FB_C,
           connectionstyle="arc3,rad=-0.10")
    _arrow(ax, _port("sim", "T", 0.40), _port("cw", "B", -0.30),
           label="state + obs",
           color=FB_C,
           connectionstyle="arc3,rad=0.18")
    _arrow(ax, _port("cw", "T", 0.30), _port("operator", "B", 0.30),
           label="severity",
           color=ADV_C, linestyle="--")

    # ============================================================
    # CAMERA DOWNLINK (latency-affected): plant → Latency → Operator
    # ============================================================
    # The driver-camera feed leaves the plant, passes through the
    # latency profile (downlink queue-load delay) and arrives at the
    # operator. Symmetric to the operator-cmd uplink that already
    # flows operator → lat → filter.
    _arrow(ax, _port("sim", "T", 0.0), _port("lat", "B"),
           label="camera_sensor",
           color=FB_C)
    _arrow(ax, _port("lat", "T", 0.30), _port("operator", "B", -0.30),
           label="delayed camera",
           color=LAT_C, linestyle=":")

    # The collision warning also receives n̂ piggybacked on the same
    # ControlCommand the safety filter does — drawing that as a
    # separate edge would cut through the Operator box, so the
    # caption documents the reuse instead.

    plt.tight_layout()
    fig.savefig(OUT, dpi=210, bbox_inches="tight")
    fig.savefig(PDF,            bbox_inches="tight")
    print(f"Wrote {OUT}")
    print(f"Wrote {PDF}")


if __name__ == "__main__":
    main()
