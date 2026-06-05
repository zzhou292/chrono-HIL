#!/usr/bin/env python3
"""Paper figure: brake-decel validation.

Two-panel figure:
  * Left: actual mean decel vs predictions (rig-NN analytical and the
    linear-interp fallback) per terrain.
  * Right: predicted stopping distance vs actual stopping distance —
    scatter with y=x reference and ±0.5 m band.

Built from the per-test data in `deliverables/brake_test_vs_predicted.csv`.
"""
from __future__ import annotations
import glob
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "my_paper" / "paper_figures"
OUT_DIR.mkdir(parents=True, exist_ok=True)

TERRAIN_N = {"clay": 0.50, "dirt": 0.70, "sand": 1.10}
TERRAIN_COLOR = {"clay": "#1f77b4", "dirt": "#8c564b", "sand": "#d4a017"}


def _build_df() -> pd.DataFrame:
    """Predicted-vs-actual stopping data from the newest brake_test sweep.

    Measured deceleration / stopping distance come from the latest
    ``brake_test_*`` Chrono runs; the predicted columns query the same
    analytical brake-decel table the collision-warning module builds from
    the rig surrogate (and the hand-tuned linear fallback), so the figure
    is regenerated end-to-end with no hand-authored intermediate CSV.
    """
    sys.path.insert(0, str(ROOT / "simulation"))
    sys.path.insert(0, str(ROOT / "simulation" / "safety"))
    from collision_warning import make_collision_warning_system
    from learned_terrain_estimator import (TERRAIN_PRESETS,
                                            terrain_preset_to_internal)
    nmap = {t: terrain_preset_to_internal(TERRAIN_PRESETS[t])["n"]
            for t in ("clay", "dirt", "sand")}
    ws = make_collision_warning_system(
        flavor="ttc", tire_model_dir=str(ROOT / "nn_models" / "rig_rate_64_32"),
        verbose=False)
    wsf = make_collision_warning_system(flavor="ttc", verbose=False)
    wsf._brake_table = []  # force the hand-tuned linear fallback
    a_an = {t: ws._brake_decel_for_terrain(nmap[t]) for t in nmap}
    a_fb = {t: wsf._brake_decel_for_terrain(nmap[t]) for t in nmap}
    bt_csv = sorted(glob.glob(str(ROOT / "benchmarking" / "results"
                                  / "brake_test_*" / "results.csv")),
                    key=lambda p: Path(p).stat().st_mtime)
    if not bt_csv:
        raise FileNotFoundError("no brake_test_* results -- run "
                                "benchmarking/brake_test.py first")
    bt = pd.read_csv(bt_csv[-1])
    bt = bt[bt["ok"]] if "ok" in bt else bt
    rows = []
    for _, r in bt.iterrows():
        t, u = r["terrain"], r["u_initial"]
        rows.append(dict(terrain=t, u_initial=u,
                         a_actual_mean=r["a_mean"], d_actual=r["d_stop"],
                         a_pred_analytical=a_an[t], a_pred_fallback=a_fb[t],
                         d_pred_analytical=u * u / (2 * a_an[t]),
                         d_pred_fallback=u * u / (2 * a_fb[t])))
    return pd.DataFrame(rows)


def main():
    df = _build_df()

    fig, (ax_decel, ax_dist) = plt.subplots(1, 2, figsize=(11.5, 4.6))

    # ---- Left: decel bars per terrain ----
    terrains = ["clay", "dirt", "sand"]
    x = np.arange(len(terrains))
    width = 0.27

    actual = [df[df.terrain == t].a_actual_mean.mean() for t in terrains]
    rig_nn = [df[df.terrain == t].a_pred_analytical.mean() for t in terrains]
    linear = [df[df.terrain == t].a_pred_fallback.mean() for t in terrains]

    ax_decel.bar(x - width, actual, width, color="#222", alpha=0.85,
                  label="Actual mean (Chrono)")
    ax_decel.bar(x,         rig_nn, width, color="#d95f02", alpha=0.85,
                  label="Rig-NN analytical")
    ax_decel.bar(x + width, linear, width, color="#aaaaaa", alpha=0.85,
                  label="Linear-interp fallback")

    for i, v in enumerate(actual):
        ax_decel.text(i - width, v + 0.05, f"{v:.2f}",
                      ha="center", va="bottom", fontsize=8)
    for i, v in enumerate(rig_nn):
        ax_decel.text(i, v + 0.05, f"{v:.2f}",
                      ha="center", va="bottom", fontsize=8)
    for i, v in enumerate(linear):
        ax_decel.text(i + width, v + 0.05, f"{v:.2f}",
                      ha="center", va="bottom", fontsize=8)
    ax_decel.set_xticks(x)
    ax_decel.set_xticklabels([f"{t}\n(n={TERRAIN_N[t]:.1f})" for t in terrains])
    ax_decel.set_ylabel("Mean deceleration over stop (m/s²)", fontsize=11)
    ax_decel.set_title("Predicted vs actual mean braking decel\n"
                       "(9 stopping trials per terrain, mean across seeds + speeds)",
                       fontsize=11)
    ax_decel.set_ylim(0, max(max(rig_nn), max(linear), max(actual)) * 1.3)
    ax_decel.legend(loc="upper left", fontsize=9, framealpha=0.95)
    ax_decel.grid(axis="y", alpha=0.3)

    # ---- Right: stopping distance scatter ----
    d_max = max(df.d_actual.max(), df.d_pred_analytical.max(),
                df.d_pred_fallback.max()) * 1.1
    grid = np.linspace(0, d_max, 100)
    ax_dist.fill_between(grid, grid - 0.5, grid + 0.5, color="#888", alpha=0.10,
                          label="±0.5 m band")
    ax_dist.plot(grid, grid, "--", color="#444", lw=1.0,
                  label="perfect (y = x)", zorder=3)

    for t in terrains:
        sub = df[df.terrain == t]
        c = TERRAIN_COLOR[t]
        ax_dist.scatter(sub.d_actual, sub.d_pred_analytical, marker="o", s=60,
                         facecolor=c, edgecolor="black", linewidth=0.6,
                         alpha=0.9, label=f"{t} — rig-NN", zorder=5)
        ax_dist.scatter(sub.d_actual, sub.d_pred_fallback, marker="x", s=60,
                         color=c, alpha=0.7, zorder=4)

    # Single "linear fallback" legend entry
    ax_dist.scatter([], [], marker="x", s=60, color="#444",
                     label="× = linear-interp fallback")

    ax_dist.set_xlabel("Actual stopping distance (m)", fontsize=11)
    ax_dist.set_ylabel("Predicted stopping distance (m)", fontsize=11)
    mae_nn = (df.d_pred_analytical - df.d_actual).abs().mean()
    mae_fb = (df.d_pred_fallback   - df.d_actual).abs().mean()
    ax_dist.set_title(f"Predicted vs actual stopping distance\n"
                      f"mean |error|: rig-NN = {mae_nn:.2f} m, "
                      f"linear-interp = {mae_fb:.2f} m  (n=27 trials)",
                      fontsize=11)
    ax_dist.set_xlim(0, d_max); ax_dist.set_ylim(0, d_max)
    ax_dist.set_aspect("equal", adjustable="box")
    ax_dist.grid(alpha=0.3)
    ax_dist.legend(loc="upper left", fontsize=8, framealpha=0.95)

    fig.tight_layout()
    out = OUT_DIR / "cw_brake_validation.png"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
