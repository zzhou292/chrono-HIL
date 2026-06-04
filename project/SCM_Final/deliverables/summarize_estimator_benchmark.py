#!/usr/bin/env python3
"""summarize_estimator_benchmark.py
=====================================

Unified head-to-head of the three terrain estimators — Bekker-UKF,
NN-UKF (whole-vehicle Fy surrogate), and the deployed sliding-window
MLP — across BOTH excitation modes on the same 100 LHS Bekker--Mohr
terrains:

* **OL** open-loop Buzhardt constant-throttle  (``lhs100_fair.csv``)
* **CL** closed-loop PI-cruise to 5 m/s         (``lhs100_cl.csv``)

The two CSVs share the same terrain seed (42) and the same trained
surrogate, so the only difference between them is the excitation.

Outputs:

* ``my_paper/paper_figures/estimator_overall.csv`` — long-form table
  (estimator x mode x {mean, median, p90, p95, <=10%, <=20%}) plus a
  pooled ``both`` row per estimator (200 runs).
* ``my_paper/paper_figures/estimator_overall.png`` — grouped bar
  panel (median + within-band rates) and a pooled error CDF, so the
  overall winner is readable at a glance.
* prints a ranked summary to stdout.
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
FIG = ROOT / "my_paper" / "paper_figures"
ESTIMATORS = ["Bekker-UKF", "NN-UKF", "Learned MLP"]
COLORS = {"Bekker-UKF": "#1f77b4", "NN-UKF": "#2ca02c", "Learned MLP": "#d62728"}


def _stats(s: pd.Series) -> dict:
    return {
        "n": int(s.size),
        "mean": float(s.mean()),
        "median": float(s.median()),
        "p90": float(s.quantile(0.90)),
        "p95": float(s.quantile(0.95)),
        "within10": float((s <= 10).mean() * 100.0),
        "within20": float((s <= 20).mean() * 100.0),
    }


def main() -> int:
    cl = pd.read_csv(FIG / "lhs100_cl.csv");   cl["mode"] = "CL"
    ol = pd.read_csv(FIG / "lhs100_fair.csv"); ol["mode"] = "OL"
    both = pd.concat([cl, ol], ignore_index=True)

    rows = []
    for est in ESTIMATORS:
        for mode, df in (("OL", ol), ("CL", cl), ("both", both)):
            s = df[df.estimator == est]["pct_err"].dropna()
            rows.append({"estimator": est, "mode": mode, **_stats(s)})
    summary = pd.DataFrame(rows)
    out_csv = FIG / "estimator_overall.csv"
    summary.to_csv(out_csv, index=False)

    # ---- stdout ranked report -----------------------------------------
    print("\n=== Per-mode + pooled error |Δn|/n_true (%) ===")
    hdr = f"{'estimator':<12} {'mode':<5} {'mean':>6} {'median':>7} {'p90':>6} {'<=10%':>6} {'<=20%':>6}"
    print(hdr); print("-" * len(hdr))
    for _, r in summary.iterrows():
        print(f"{r.estimator:<12} {r['mode']:<5} {r['mean']:6.1f} "
              f"{r['median']:7.1f} {r['p90']:6.1f} {r['within10']:6.0f} "
              f"{r['within20']:6.0f}")

    pooled = summary[summary["mode"] == "both"].sort_values("median")
    print("\n=== OVERALL ranking (pooled over both modes, 200 runs each) ===")
    for rank, (_, r) in enumerate(pooled.iterrows(), 1):
        print(f"  {rank}. {r.estimator:<12} median={r['median']:.1f}%  "
              f"mean={r['mean']:.1f}%  within20={r['within20']:.0f}%")
    best = pooled.iloc[0]["estimator"]
    print(f"\nOverall best: {best}")

    # ---- figure: grouped bars (median per mode) + pooled CDF ----------
    fig, (ax_bar, ax_cdf) = plt.subplots(1, 2, figsize=(13.0, 4.8))

    modes = ["OL", "CL"]
    x = np.arange(len(modes))
    w = 0.25
    for i, est in enumerate(ESTIMATORS):
        meds = [summary[(summary.estimator == est) & (summary["mode"] == m)]
                ["median"].iloc[0] for m in modes]
        bars = ax_bar.bar(x + (i - 1) * w, meds, w, label=est,
                          color=COLORS[est], alpha=0.85)
        for b, v in zip(bars, meds):
            ax_bar.text(b.get_x() + b.get_width() / 2, v + 0.4,
                        f"{v:.1f}", ha="center", va="bottom", fontsize=8)
    ax_bar.set_xticks(x)
    ax_bar.set_xticklabels(["Open-loop\n(throttle 0.75)",
                            "Closed-loop\n(PI cruise 5 m/s)"])
    ax_bar.set_ylabel(r"median $|\Delta n|/n_{\mathrm{true}}$  (%)")
    ax_bar.set_title("Median sinkage-exponent error by excitation mode")
    ax_bar.grid(axis="y", alpha=0.3)
    ax_bar.legend(fontsize=9, framealpha=0.92)

    for est in ESTIMATORS:
        s = np.sort(both[both.estimator == est]["pct_err"].to_numpy())
        y = np.arange(1, s.size + 1) / s.size
        st = _stats(both[both.estimator == est]["pct_err"])
        ax_cdf.plot(s, y, lw=1.8, color=COLORS[est],
                    label=f"{est}  med={st['median']:.1f}%  "
                          f"≤20%={st['within20']:.0f}%")
    for thr in (10.0, 20.0):
        ax_cdf.axvline(thr, color="gray", lw=0.5, ls="--")
    ax_cdf.set_xlim(0, 80); ax_cdf.set_ylim(0, 1)
    ax_cdf.set_xlabel(r"$|\Delta n|/n_{\mathrm{true}}$  (%)")
    ax_cdf.set_ylabel("Empirical CDF")
    ax_cdf.set_title("Pooled over both modes (200 runs / estimator)")
    ax_cdf.grid(alpha=0.3)
    ax_cdf.legend(loc="lower right", fontsize=9, framealpha=0.92)

    fig.suptitle("Terrain-estimator head-to-head — 100 LHS Bekker–Mohr "
                 "terrains × open-loop & closed-loop excitation",
                 fontsize=12, y=1.01)
    fig.tight_layout()
    out_png = FIG / "estimator_overall.png"
    fig.savefig(out_png, dpi=170, bbox_inches="tight")
    print(f"\nWrote {out_csv}\nWrote {out_png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
