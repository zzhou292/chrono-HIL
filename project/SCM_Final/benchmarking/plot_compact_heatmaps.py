#!/usr/bin/env python3
"""Compact per-scenario heatmaps for the safety / latency / DOB figures.

The original per-scenario heatmaps listed every (terrain, path, speed, bumpiness)
combination on a single tall axis (~27-48 rows), producing figures taller than a
page. This regenerates them in a compact, page-friendly form: averaged over
bumpiness and seeds, faceted by terrain (clay/dirt/sand) into three side-by-side
panels with rows = path x speed and columns = variant. Same qualitative pattern,
~1/3 the height, drop-in to the existing paper filenames.
"""
from __future__ import annotations
import glob, sys
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "benchmarking" / "results"
FIG = ROOT / "my_paper" / "paper_figures"
TERRAINS = ["clay", "dirt", "sand"]

# (result-prefix, paper-filename, metric col, cmap, metric label, lower_is_better)
SPECS = [
    ("safety_filter_sweep", "safety_filter_collision_heatmap.png", "collisions", "RdYlGn_r", "mean obstacles hit", True),
    ("dob_cbf_nn_ablation", "dob_cbf_nn_ablation_heatmap.png", "collisions", "RdYlGn_r", "mean obstacles hit", True),
    ("autonomous_obstacle_tire_model_sweep_dob_cbf_mpc_blind", "autonomous_obstacle_collision_heatmap.png", "collisions", "RdYlGn_r", "mean obstacles hit", True),
    ("latency_compensation_sweep", "latency_compensation_collision_heatmap.png", "collisions", "RdYlGn_r", "mean obstacles hit", True),
    ("throttle_dob_ablation", "throttle_dob_ablation_speed_heatmap.png", "speed_ratio", "RdYlGn", "speed retention", False),
]


def _latest(prefix):
    hits = sorted(glob.glob(str(RES / f"{prefix}_*" / "results.csv")))
    # exclude prefixes that are supersets (e.g. safety_filter_sweep vs _planner_aware)
    hits = [h for h in hits if Path(h).parent.name.split("_20")[0] == prefix]
    return Path(hits[-1]) if hits else None


def _render(spec):
    prefix, outname, metric, cmap, label, lower_better = spec
    rc = _latest(prefix)
    if rc is None:
        print(f"  [skip] {outname}: no results for {prefix}")
        return False
    df = pd.read_csv(rc)
    if "status" in df:
        df = df[df["status"].astype(str) == "ok"]
    for c in ("terrain", "path", "speed_mps", "variant", metric):
        if c not in df.columns:
            print(f"  [skip] {outname}: missing column {c}")
            return False
    df[metric] = pd.to_numeric(df[metric], errors="coerce")
    df = df.dropna(subset=[metric])
    df["ps"] = df["path"].astype(str) + "/v" + df["speed_mps"].astype(float).map(lambda v: f"{v:g}")
    terr = [t for t in TERRAINS if t in set(df["terrain"])]
    variants = sorted(set(df["variant"]))
    vmin, vmax = float(df[metric].min()), float(df[metric].max())
    if vmax <= vmin:
        vmax = vmin + 1e-6
    n = len(terr)
    width = min(10.5, 2.4 * n + 0.55 * len(variants) + 1.2)
    fig, axes = plt.subplots(1, n, figsize=(width, 3.2), squeeze=False)
    axes = axes[0]
    im = None
    for k, (ax, t) in enumerate(zip(axes, terr)):
        sub = df[df["terrain"] == t]
        piv = sub.pivot_table(index="ps", columns="variant", values=metric, aggfunc="mean")
        piv = piv.reindex(columns=[v for v in variants if v in piv.columns])
        im = ax.imshow(piv.values, aspect="auto", cmap=cmap, vmin=vmin, vmax=vmax)
        ax.set_xticks(range(piv.shape[1]))
        ax.set_xticklabels([c.replace("_", " ") for c in piv.columns], rotation=30, ha="right", fontsize=7)
        ax.set_yticks(range(piv.shape[0]))
        ax.set_yticklabels(list(piv.index) if k == 0 else [""] * piv.shape[0], fontsize=6.5)
        ax.set_title(t, fontsize=10)
        for i in range(piv.shape[0]):
            for j in range(piv.shape[1]):
                v = piv.values[i, j]
                if np.isfinite(v):
                    ax.text(j, i, f"{v:.1f}", ha="center", va="center", fontsize=6,
                            color="black")
    fig.colorbar(im, ax=list(axes), fraction=0.03, pad=0.01, label=label)
    fig.suptitle(f"{label} by scenario (averaged over bumpiness and seeds)", fontsize=10)
    FIG.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIG / outname, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"  [ok] {outname}  ({n} terrains x {len(variants)} variants, {df['ps'].nunique()} path/speed rows)")
    return True


def main():
    for spec in SPECS:
        _render(spec)


if __name__ == "__main__":
    main()
