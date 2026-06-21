#!/usr/bin/env python3
"""Regenerate the per-terrain n_hat convergence figure for the abstract.

The auto-published ``closed_loop_estimator_learned.png`` is a 3-bar
ID/OOD summary, but the abstract caption says
``"Convergence of n_hat on canonical clay/dirt/sand under closed-loop
NMPC at 5 m/s."``  This script reads the per-run diag CSVs from the
latest ``terrain_estimator_benchmark_*`` folder, plots n_hat vs time
for clay/sand (the ID cases) plus a representative OOD case, then
overwrites the published figure with the time-series version.
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parent.parent
RESULTS = ROOT / "paper_scripts" / "results"
PAPER_FIG = ROOT / "my_paper" / "paper_figures"


def latest_estimator_dir() -> Path:
    cands = sorted(RESULTS.glob("terrain_estimator_benchmark_*"),
                   key=lambda p: p.stat().st_mtime)
    return cands[-1]


def main() -> None:
    src_dir = latest_estimator_dir()
    print(f"Source: {src_dir}")
    df = pd.read_csv(src_dir / "results.csv")
    ok = df[df["status"] == "ok"].copy()
    if ok.empty:
        sys.exit("no ok rows in results.csv")

    # Pick representative runs: clay (ID), sand (ID), terrain1 (OOD).
    picks = {}
    for case in ("clay", "sand"):
        sub = ok[(ok["distribution"] == "id") & (ok["case_label"] == case)]
        if not sub.empty:
            picks[case] = sub.sort_values("seed").iloc[0]
    sub_ood = ok[(ok["distribution"] == "ood") & (ok["case_label"] == "terrain1")]
    if not sub_ood.empty:
        picks["terrain1 (OOD)"] = sub_ood.sort_values("seed").iloc[0]

    colors = {"clay": "#1f77b4", "sand": "#d62728", "terrain1 (OOD)": "#2ca02c"}
    fig, ax = plt.subplots(figsize=(7.5, 4.5))

    for label, row in picks.items():
        diag_path = Path(row["diag_csv"])
        if not diag_path.exists():
            print(f"  [warn] missing diag for {label}: {diag_path}")
            continue
        diag = pd.read_csv(diag_path)
        if "sim_time" not in diag.columns or "n_terrain_est" not in diag.columns:
            print(f"  [warn] {label}: diag missing required columns")
            continue
        t = pd.to_numeric(diag["sim_time"], errors="coerce").to_numpy(dtype=float)
        n = pd.to_numeric(diag["n_terrain_est"], errors="coerce").to_numpy(dtype=float)
        good = np.isfinite(t) & np.isfinite(n)
        ax.plot(t[good], n[good], lw=1.6, alpha=0.85,
                color=colors.get(label, "#7f7f7f"), label=label)
        true_n = float(row["true_n"])
        ax.axhline(true_n, lw=0.8, ls="--", color=colors.get(label, "#7f7f7f"),
                   alpha=0.55, label=f"true n_{label}={true_n:.2f}")

    ax.set_xlabel("simulation time (s)")
    ax.set_ylabel(r"$\hat n$ (Bekker sinkage exponent)")
    ax.set_title("Online terrain estimator: $\\hat n$ convergence under closed-loop NMPC at 5 m/s")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, ncols=2, loc="best")
    fig.tight_layout()

    out = PAPER_FIG / "closed_loop_estimator_learned.png"
    fig.savefig(out, dpi=240)
    plt.close(fig)
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
