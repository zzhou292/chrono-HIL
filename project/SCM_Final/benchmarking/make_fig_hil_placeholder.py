#!/usr/bin/env python3
"""Placeholder HIL (human-in-the-loop) result figures.

The HIL safety-filter rounds are run manually with a Logitech G29 (see
``human_delay_compensation_rounds.py``); those sims are pending. This writes
clearly-marked PLACEHOLDER PNGs into ``my_paper/paper_figures/`` under the
exact filenames that ``human_delay_compensation_rounds.py`` emits, so the
paper has figure slots (caption + label + placement) ready now and the real
runs drop straight in:

    human_delay_compensation_summary.png   (metrics vs operator latency, by filter)
    human_delay_collision_heatmap.png      (collisions, delay x filter)

After the HIL rounds finish, copy the generated PNGs from the run's
``figures/`` dir over these placeholders (or wire a PublishSpec) and recompile.
"""
from __future__ import annotations
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FIG = Path(__file__).resolve().parents[1] / "my_paper" / "paper_figures"
CMD = "python benchmarking/human_delay_compensation_rounds.py --auto-start"

# (filename, what-it-will-show, figsize)
SPECS = [
    ("human_delay_compensation_summary.png",
     "HIL safety-filter metrics vs. operator latency (by filter)\n"
     "collisions / clearance / intervention rate / intrusiveness / tracking",
     (11.0, 5.0)),
    ("human_delay_collision_heatmap.png",
     "HIL collisions per run  (operator delay x safety filter)",
     (11.0, 4.2)),
]


def _placeholder(path: Path, what: str, figsize):
    fig, ax = plt.subplots(figsize=figsize)
    ax.axis("off")
    ax.add_patch(plt.Rectangle((0.02, 0.02), 0.96, 0.96, transform=ax.transAxes,
                               fill=True, facecolor="0.93", edgecolor="0.4",
                               lw=2.0, ls="--"))
    ax.text(0.5, 0.74, "PLACEHOLDER", transform=ax.transAxes, ha="center",
            va="center", fontsize=34, fontweight="bold", color="0.45")
    ax.text(0.5, 0.55, "human-in-the-loop results pending", transform=ax.transAxes,
            ha="center", va="center", fontsize=15, color="0.35")
    ax.text(0.5, 0.40, what, transform=ax.transAxes, ha="center", va="center",
            fontsize=12, color="0.3")
    ax.text(0.5, 0.13, f"regenerate via:\n{CMD}", transform=ax.transAxes,
            ha="center", va="center", fontsize=9, color="0.5", family="monospace")
    FIG.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  [placeholder] {path.name}")


def main():
    for name, what, figsize in SPECS:
        _placeholder(FIG / name, what, figsize)


if __name__ == "__main__":
    main()
