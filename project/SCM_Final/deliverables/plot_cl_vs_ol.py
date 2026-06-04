#!/usr/bin/env python3
"""plot_cl_vs_ol.py — closed-loop vs open-loop estimator comparison.

Reads the two 100-LHS benchmark CSVs
(``my_paper/paper_figures/lhs100_cl.csv`` and ``lhs100_fair.csv``)
and produces a 2 x 3 panel comparison figure showing each estimator
under both excitation modes (CL = PI cruise at 5 m/s, OL = constant
throttle 0.75; both at steer amplitude 0.6 rad). Same LHS terrain
seed (42) so each row directly compares an estimator's CL vs OL
behaviour on the same 100 soils.
"""
from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    cl = pd.read_csv(ROOT / "my_paper/paper_figures/lhs100_cl.csv")
    ol = pd.read_csv(ROOT / "my_paper/paper_figures/lhs100_fair.csv")
    cl["mode"] = "CL (PI cruise, 5 m/s)"
    ol["mode"] = "OL (throttle 0.75)"
    df = pd.concat([cl, ol], ignore_index=True)

    estimators = ["Bekker-UKF", "NN-UKF", "Learned MLP"]
    colors = {"CL (PI cruise, 5 m/s)": "#1f77b4", "OL (throttle 0.75)": "#d62728"}

    fig, axes = plt.subplots(2, 3, figsize=(15.5, 8.0))

    # Top row: scatter true_n vs estimated, one panel per estimator
    for col, est in enumerate(estimators):
        ax = axes[0, col]
        ax.plot([0.4, 1.3], [0.4, 1.3], "k-", lw=1.0, label="$y\\!=\\!x$")
        for band, alpha_b in [(0.10, 0.13), (0.20, 0.07)]:
            x = np.linspace(0.4, 1.3, 100)
            ax.fill_between(x, x * (1 - band), x * (1 + band),
                             color="k", alpha=alpha_b,
                             label=f"$\\pm{int(band*100)}\\,\\%$")
        for mode in ("CL (PI cruise, 5 m/s)", "OL (throttle 0.75)"):
            sub = df[(df.estimator == est) & (df["mode"] == mode)].dropna(subset=["converged_n"])
            ax.scatter(sub.n_true, sub.converged_n, s=24, alpha=0.55,
                        color=colors[mode], edgecolors="white",
                        linewidths=0.5, label=mode)
        ax.set_xlabel("True $n$")
        ax.set_ylabel("Estimated $n$ (tail mean)")
        ax.set_xlim(0.35, 1.35); ax.set_ylim(0.35, 1.40)
        ax.set_title(est)
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left", fontsize=8.5, framealpha=0.92)

    # Bottom row: error CDF per mode, one panel per estimator
    for col, est in enumerate(estimators):
        ax = axes[1, col]
        for mode in ("CL (PI cruise, 5 m/s)", "OL (throttle 0.75)"):
            sub = df[(df.estimator == est) & (df["mode"] == mode)].dropna(subset=["pct_err"])
            errs = np.sort(sub.pct_err.to_numpy())
            if errs.size == 0:
                continue
            y = np.arange(1, errs.size + 1) / errs.size
            med = float(np.median(errs))
            p90 = float(np.percentile(errs, 90))
            within10 = (sub.pct_err <= 10).mean() * 100
            within20 = (sub.pct_err <= 20).mean() * 100
            ax.plot(errs, y, lw=1.7, color=colors[mode],
                    label=f"{mode}\n  med {med:.1f}%, p90 {p90:.1f}%\n"
                           f"  ≤10 % {within10:.0f}%, ≤20 % {within20:.0f}%")
        for thresh in (10.0, 20.0):
            ax.axvline(thresh, color="gray", lw=0.5, ls="--")
        ax.set_xlabel("$|\\Delta n| / n_{\\mathrm{true}}$  (%-error)")
        ax.set_ylabel("Empirical CDF")
        ax.set_xlim(0.0, 80.0); ax.set_ylim(0.0, 1.0)
        ax.set_title(f"{est} — error CDF")
        ax.grid(alpha=0.3)
        ax.legend(loc="lower right", fontsize=7.8, framealpha=0.92)

    fig.suptitle(
        "Closed-loop vs open-loop convergence on 100 uniform-LHS Bekker--Mohr "
        "terrains  (seed 42, $n \\in [0.40, 1.30]$, steer amp 0.6 rad)",
        fontsize=12, y=1.00)
    fig.tight_layout()
    out = ROOT / "my_paper" / "paper_figures" / "lhs100_cl_vs_ol.png"
    fig.savefig(out, dpi=160, bbox_inches="tight")
    print(f"Wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
