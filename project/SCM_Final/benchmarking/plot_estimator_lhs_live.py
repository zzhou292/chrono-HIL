#!/usr/bin/env python3
"""Regenerate the LHS-100 terrain-estimator figures (paper Figs 8/9) as LIVE
closed-loop results, replacing the offline UKF-replay versions. Reads
``benchmarking/closed_loop_estimator_lhs_runs.csv`` (from
``closed_loop_estimator_lhs.py``: each of 100 uniform-LHS Bekker-Mohr soils
driven through the full NMPC with each estimator backend live; tail-window n).

Writes:
  * lhs100_fair.png       (Fig 8): est-vs-true scatter + error CDF, 4 backends.
  * estimator_overall.png (Fig 9): median %err + %-within-band summary bars.
No Chrono re-sim.
"""
from __future__ import annotations
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
CSV = ROOT / "benchmarking" / "closed_loop_estimator_lhs_runs.csv"
FIGDIR = ROOT / "my_paper" / "paper_figures"
STYLE = [("MLP", "MLP (window)", "#4c78a8"),
         ("Bekker-UKF", "Bekker-UKF (force)", "#b07aa1"),
         ("NN-UKF", "NN-UKF (force)", "#dd8452"),
         ("Fused-UKF", "Fused-UKF (deployed)", "#59a14f")]


def _ok(df, key):
    m = df[(df.backend == key) & (df.status == "ok")].copy()
    m["pct"] = 100.0 * m["abs_dn"] / m["n_true"]
    return m


def plot_figures(csv_path: Path = CSV, figdir: Path = FIGDIR):
    df = pd.read_csv(csv_path)
    figdir.mkdir(parents=True, exist_ok=True)

    # ---- Fig 8: per-backend convergence small-multiples + pooled error CDF ----
    # One scatter panel per backend (faded per-soil points + a bold binned-median
    # trend) so each estimator's true-vs-estimated relationship is legible instead
    # of a 400-point overplot in which everything reads as a flat band near 0.5.
    lo, hi = 0.50, 1.12
    bins = np.linspace(0.52, 1.08, 7)
    bctr = 0.5 * (bins[:-1] + bins[1:])
    fig = plt.figure(figsize=(13.6, 5.2))
    gs = fig.add_gridspec(2, 3, width_ratios=[1.0, 1.0, 1.35], wspace=0.28, hspace=0.42)
    panel_ax = [fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1]),
                fig.add_subplot(gs[1, 0]), fig.add_subplot(gs[1, 1])]
    for ax, (key, label, col) in zip(panel_ax, STYLE):
        m = _ok(df, key)
        ax.fill_between([lo, hi], [lo * 0.8, hi * 0.8], [lo * 1.2, hi * 1.2],
                        color="0.6", alpha=0.16, lw=0, zorder=0)
        ax.plot([lo, hi], [lo, hi], "k-", lw=0.9, zorder=1)
        ax.scatter(m["n_true"], m["est_n"], s=11, alpha=0.28, color=col,
                   edgecolors="none", zorder=2)
        med_tr = np.array([m[(m.n_true >= b0) & (m.n_true < b1)]["est_n"].median()
                           if ((m.n_true >= b0) & (m.n_true < b1)).sum() >= 3 else np.nan
                           for b0, b1 in zip(bins[:-1], bins[1:])])
        good = np.isfinite(med_tr)
        ax.plot(bctr[good], med_tr[good], "-o", color=col, lw=2.4, ms=5,
                mec="white", mew=0.9, zorder=3)
        ax.set_xlim(lo, hi); ax.set_ylim(0.38, 1.22)
        ax.set_title(f"{label.split(' (')[0]}  (med {np.median(m['pct']):.0f}%)", fontsize=9.5)
        ax.grid(alpha=0.25); ax.tick_params(labelsize=7.5)
    panel_ax[2].set_xlabel("true $n$", fontsize=9); panel_ax[3].set_xlabel("true $n$", fontsize=9)
    panel_ax[0].set_ylabel(r"estimated $n$", fontsize=9); panel_ax[2].set_ylabel(r"estimated $n$", fontsize=9)
    panel_ax[0].set_xticklabels([]); panel_ax[1].set_xticklabels([])
    panel_ax[1].set_yticklabels([]); panel_ax[3].set_yticklabels([])
    axc = fig.add_subplot(gs[:, 2])
    for key, label, col in STYLE:
        m = _ok(df, key); x = np.sort(m["pct"].to_numpy()); y = np.arange(1, len(x) + 1) / len(x)
        axc.plot(x, y, color=col, lw=1.9, label=f"{label.split(' (')[0]}  med={np.median(m['pct']):.1f}%")
    axc.axvline(20, color="0.5", ls="--", lw=0.9)
    axc.set_xlim(0, 60); axc.set_ylim(0, 1)
    axc.set_xlabel(r"$|\Delta n|/n_\mathrm{true}$  (%-error)"); axc.set_ylabel("empirical CDF")
    axc.set_title("Pooled error CDF (closed loop)", fontsize=10.5)
    axc.legend(loc="lower right", fontsize=8.5); axc.grid(alpha=0.3)
    fig.suptitle("Per-backend convergence across 100 uniform-LHS Bekker-Mohr soils (live closed loop): "
                 "faded points are individual soils, bold line is the binned median",
                 fontsize=10.5, y=1.0)
    fig.savefig(figdir / "lhs100_fair.png", dpi=170, bbox_inches="tight"); plt.close(fig)

    # ---- Fig 9: median %err + %-within-band summary ----
    labels = [s[1].split(" (")[0] for s in STYLE]; cols = [s[2] for s in STYLE]
    med = []; w10 = []; w20 = []
    for key, _, _ in STYLE:
        m = _ok(df, key); pe = m["pct"].to_numpy()
        med.append(np.median(pe)); w10.append(100.0 * np.mean(pe <= 10)); w20.append(100.0 * np.mean(pe <= 20))
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11.0, 4.3))
    x = np.arange(len(STYLE))
    b = a1.bar(x, med, color=cols)
    for xi, v in zip(x, med): a1.text(xi, v + 0.3, f"{v:.1f}", ha="center", fontsize=9)
    a1.set_xticks(x); a1.set_xticklabels(labels, fontsize=9)
    a1.set_ylabel(r"median $|\Delta n|/n_\mathrm{true}$  (%)")
    a1.set_title("Median closed-loop estimator error", fontsize=10.5); a1.grid(axis="y", alpha=0.3)
    w = 0.38
    a2.bar(x - w / 2, w10, w, label=r"$\leq$10%", color="#9ecae1")
    a2.bar(x + w / 2, w20, w, label=r"$\leq$20%", color="#3182bd")
    for xi, v in zip(x - w / 2, w10): a2.text(xi, v + 1, f"{v:.0f}", ha="center", fontsize=8)
    for xi, v in zip(x + w / 2, w20): a2.text(xi, v + 1, f"{v:.0f}", ha="center", fontsize=8)
    a2.set_xticks(x); a2.set_xticklabels(labels, fontsize=9)
    a2.set_ylabel("% of soils within band"); a2.set_ylim(0, 100)
    a2.set_title("Fraction within accuracy band", fontsize=10.5)
    a2.legend(fontsize=9); a2.grid(axis="y", alpha=0.3)
    fig.suptitle("Unified terrain-estimator head-to-head, 100 LHS soils, live closed loop",
                 fontsize=11.5, y=1.02)
    fig.tight_layout(); fig.savefig(figdir / "estimator_overall.png", dpi=170, bbox_inches="tight"); plt.close(fig)
    return figdir / "lhs100_fair.png", figdir / "estimator_overall.png"


if __name__ == "__main__":
    for p in plot_figures():
        print(f"Wrote {p}")
