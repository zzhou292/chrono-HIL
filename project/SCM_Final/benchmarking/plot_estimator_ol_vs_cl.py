#!/usr/bin/env python3
"""Open-loop vs closed-loop terrain-estimator convergence (paper Sec. VI).

Replaces the old MLP-only open-loop diagnostic. For each estimator and soil it
shows tail |dn| under a scripted open-loop excitation maneuver (sustained
0.6-rad steer) vs under closed-loop NMPC path-tracking. The contrast isolates
the excitation/observability axis:

* NN-UKF: firm-sand |dn| 0.44 (CL) -> 0.09 (OL) -- the force channel becomes
  observable under strong excitation. Observability-limited, fixable.
* Bekker-UKF: firm-sand stays ~0.48 in BOTH -- model-limited (analytical Bekker
  law is wrong on sand), excitation cannot save it.
* MLP: ~unchanged (vibration features, excitation-agnostic).
* Fused-UKF (deployed): strong in both; its CL clay wart relaxes open-loop.

Reads benchmarking/estimator_ol_vs_cl_summary.csv.
"""
from __future__ import annotations
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
CSV = ROOT / "benchmarking" / "estimator_ol_vs_cl_summary.csv"
OUT = ROOT / "my_paper" / "paper_figures" / "open_loop_estimator_diagnostic.png"
ORDER = ["MLP", "Bekker-UKF", "NN-UKF", "Fused-UKF"]
SOILS = ["clay", "dirt", "sand"]


def plot_figures(csv_path: Path = CSV, out_path: Path = OUT) -> Path:
    df = pd.read_csv(csv_path)
    fig, axes = plt.subplots(1, 3, figsize=(13.0, 4.2), sharey=True)
    x = np.arange(len(ORDER)); w = 0.38
    for ax, soil in zip(axes, SOILS):
        s = df[df.soil == soil].set_index("backend")
        ol = [float(s.loc[b, "OL_abs_dn"]) for b in ORDER]
        cl = [float(s.loc[b, "CL_abs_dn"]) for b in ORDER]
        b1 = ax.bar(x - w / 2, ol, w, label="open-loop maneuver", color="#2ca02c")
        b2 = ax.bar(x + w / 2, cl, w, label="closed-loop tracking", color="#d62728")
        for bars in (b1, b2):
            for r in bars:
                ax.text(r.get_x() + r.get_width() / 2, r.get_height() + 0.008,
                        f"{r.get_height():.2f}", ha="center", va="bottom", fontsize=7)
        ax.axhline(0.10, color="0.6", lw=0.8, ls="--")
        ax.set_xticks(x); ax.set_xticklabels(ORDER, rotation=20, fontsize=8.5)
        ax.set_title(f"{soil}", fontsize=11)
        ax.grid(axis="y", alpha=0.3)
    axes[0].set_ylabel(r"tail $|\Delta n|$")
    axes[0].legend(loc="upper left", fontsize=8.5)
    axes[-1].set_ylim(0, max(0.5, df[["OL_abs_dn", "CL_abs_dn"]].to_numpy().max() * 1.12))
    fig.suptitle("Open-loop excitation vs. closed-loop tracking: the force-UKF's firm-sand failure is "
                 "an excitation/observability gap (NN-UKF sand 0.44→0.09), not the deployed Fused-UKF's",
                 fontsize=10.8, y=1.02)
    fig.tight_layout()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=180, bbox_inches="tight")
    plt.close(fig)
    return out_path


if __name__ == "__main__":
    print(f"Wrote {plot_figures()}")
