#!/usr/bin/env python3
"""
Generate paper-ready plots from benchmark results CSV.

Usage:
    python plot_paper_benchmarks.py
    python plot_paper_benchmarks.py --csv path/to/results.csv
    python plot_paper_benchmarks.py --out-dir ./my_figures
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import pandas as pd

# Force unbuffered stdout
os.environ["PYTHONUNBUFFERED"] = "1"
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(line_buffering=True)

UTIL_DIR = Path(__file__).parent
PROJECT_ROOT = UTIL_DIR.parent
SIM_DIR = PROJECT_ROOT / "simulation"
DEFAULT_CSV = SIM_DIR / "plots" / "paper_benchmark" / "paper_benchmark_results.csv"

# ── Model metadata ────────────────────────────────────────────────────
MODEL_TYPE_MAP = {}
for name in ["mlp_12_2", "mlp_16_4", "mlp_16_8", "mlp_24_12", "mlp_32_16"]:
    MODEL_TYPE_MAP[name] = "static"
for name in ["resnet_h8_b2", "resnet_h16_b2", "resnet_h16_b4", "resnet_h32_b2"]:
    MODEL_TYPE_MAP[name] = "static"
for name in ["rate_mlp_16_8", "rate_mlp_24_12", "rate_resnet_h16_b2", "rate_resnet_h32_b2"]:
    MODEL_TYPE_MAP[name] = "rate"
for prefix in ["temp_K3", "temp_K5", "temp_K10"]:
    for suffix in ["mlp_16_8", "mlp_24_12", "resnet_h16", "resnet_h32"]:
        MODEL_TYPE_MAP[f"{prefix}_{suffix}"] = "temporal"

# Also add analytical
for name in ["pacejka", "tmeasy", "linear"]:
    MODEL_TYPE_MAP[name] = "analytical"

TYPE_COLORS = {
    "static":     "#2196F3",
    "rate":       "#FF9800",
    "temporal":   "#E53935",
    "analytical": "#4CAF50",
}
TYPE_LABELS = {
    "static":     "Static NN",
    "rate":       "Rate-augmented NN",
    "temporal":   "Temporal NN",
    "analytical": "Analytical",
}

TERRAIN_MARKERS = {"sand": "o", "clay": "s", "dirt": "^"}
TERRAIN_COLORS = {"sand": "#E8A838", "clay": "#8B4513", "dirt": "#607D3B"}

# Rough param counts for sizing
PARAM_COUNTS = {
    "mlp_12_2": 176, "mlp_16_4": 270, "mlp_16_8": 414, "mlp_24_12": 744,
    "mlp_32_16": 1202, "resnet_h8_b2": 382, "resnet_h16_b2": 1086,
    "resnet_h16_b4": 1918, "resnet_h32_b2": 3678,
    "rate_mlp_16_8": 462, "rate_mlp_24_12": 816,
    "rate_resnet_h16_b2": 1182, "rate_resnet_h32_b2": 3870,
    "temp_K3_mlp_16_8": 726, "temp_K3_mlp_24_12": 1080,
    "temp_K3_resnet_h16": 1374, "temp_K3_resnet_h32": 4254,
    "temp_K5_mlp_16_8": 1038, "temp_K5_mlp_24_12": 1392,
    "temp_K5_resnet_h16": 1662, "temp_K5_resnet_h32": 4542,
    "temp_K10_mlp_16_8": 1818, "temp_K10_mlp_24_12": 2172,
    "temp_K10_resnet_h16": 2442, "temp_K10_resnet_h32": 6114,
}


def load_data(csv_path):
    df = pd.read_csv(csv_path)
    df["model_type"] = df["label"].map(MODEL_TYPE_MAP).fillna("unknown")
    df["n_params"] = df["label"].map(PARAM_COUNTS).fillna(0).astype(int)
    # Solver "success" = converged (status 0) + max-iter (status 2).
    # Status 2 still yields a usable solution; only status 3/4 are true failures.
    df["success_pct"] = (df["success_pct"] + df["maxiter_pct"]).clip(upper=100.0)
    # Drop rows with unknown model type (e.g. legacy nn_mlp_16_4 duplicate)
    df = df[df["model_type"] != "unknown"].reset_index(drop=True)
    return df


# ── Plot 1: Solver success by model type (grouped bar) ───────────────
def plot_solver_success_by_type(df, out_dir):
    """Bar chart: mean solver success % grouped by input paradigm."""
    type_order = ["static", "rate", "temporal"]
    grouped = df[df["model_type"].isin(type_order)].groupby("model_type")
    means = grouped["success_pct"].mean().reindex(type_order)
    stds = grouped["success_pct"].std().reindex(type_order)
    # Clip error bars so they don't exceed [0, 100]
    err_lo = np.minimum(stds, means)
    err_hi = np.minimum(stds, 100.0 - means)
    yerr = np.array([err_lo.values, err_hi.values])

    fig, ax = plt.subplots(figsize=(5, 4))
    x = np.arange(len(type_order))
    bars = ax.bar(x, means, yerr=yerr, capsize=5, width=0.5,
                  color=[TYPE_COLORS[t] for t in type_order],
                  edgecolor="black", linewidth=0.5)
    ax.set_xticks(x)
    ax.set_xticklabels([TYPE_LABELS[t] for t in type_order])
    ax.set_ylabel("Solver Success Rate (%)")
    ax.set_ylim(0, 110)
    ax.axhline(100, color="gray", linestyle="--", linewidth=0.5, alpha=0.5)

    # Annotate means
    for bar, m in zip(bars, means):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 3,
                f"{m:.0f}%", ha="center", va="bottom", fontsize=10, fontweight="bold")

    ax.set_title("MPC Solver Success by Input Paradigm")
    fig.tight_layout()
    path = out_dir / "solver_success_by_type.png"
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  {path.name}")
    return path


# ── Plot 2: Solver success heatmap (model × terrain) ─────────────────
def plot_solver_heatmap(df, out_dir):
    """Heatmap of solver success averaged across paths, for each model × terrain."""
    pivot = df.pivot_table(
        values="success_pct", index="label", columns="terrain",
        aggfunc="mean"
    )
    # Sort by model type then name
    type_sort = {"static": 0, "rate": 1, "temporal": 2}
    pivot["_sort"] = pivot.index.map(lambda x: (type_sort.get(MODEL_TYPE_MAP.get(x, ""), 3), x))
    pivot = pivot.sort_values("_sort").drop(columns="_sort")

    fig, ax = plt.subplots(figsize=(6, max(8, len(pivot) * 0.35)))
    terrain_order = ["sand", "clay", "dirt"]
    data = pivot[terrain_order].values
    im = ax.imshow(data, aspect="auto", cmap="RdYlGn", vmin=0, vmax=100)

    ax.set_xticks(range(len(terrain_order)))
    ax.set_xticklabels([t.capitalize() for t in terrain_order])
    ax.set_yticks(range(len(pivot)))
    ax.set_yticklabels(pivot.index, fontsize=7)

    # Annotate cells
    for i in range(len(pivot)):
        for j in range(len(terrain_order)):
            v = data[i, j]
            if np.isfinite(v):
                color = "white" if v < 40 else "black"
                ax.text(j, i, f"{v:.0f}", ha="center", va="center",
                        fontsize=6, color=color, fontweight="bold")

    # Type separators
    types_in_order = [MODEL_TYPE_MAP.get(m, "") for m in pivot.index]
    for i in range(1, len(types_in_order)):
        if types_in_order[i] != types_in_order[i-1]:
            ax.axhline(i - 0.5, color="black", linewidth=1.5)

    cbar = fig.colorbar(im, ax=ax, shrink=0.6, label="Solver Success %")
    ax.set_title("Solver Success Rate by Model & Terrain\n(averaged across paths)")
    fig.tight_layout()
    path = out_dir / "solver_success_heatmap.png"
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  {path.name}")
    return path


# ── Plot 2b: Best NN vs analytical models ─────────────────────────────
def plot_nn_vs_analytical(df, out_dir):
    """Grouped bar: RMS CTE for analytical models + best static NN, by terrain."""
    analytical_labels = ["pacejka", "tmeasy"]
    # Pick best static NN (lowest mean RMS CTE across terrains)
    static = df[(df["model_type"] == "static") & (df["status"] == "ok")]
    best_nn_label = static.groupby("label")["rms_cte_m"].mean().idxmin()

    compare_labels = analytical_labels + [best_nn_label]
    sub = df[df["label"].isin(compare_labels) & (df["status"] == "ok")].copy()

    terrain_order = ["sand", "clay", "dirt"]
    pivot = sub.pivot_table(
        values="rms_cte_m", index="label", columns="terrain", aggfunc="mean"
    ).reindex(compare_labels)[terrain_order]

    fig, ax = plt.subplots(figsize=(7, 5))
    x = np.arange(len(compare_labels))
    width = 0.22

    for i, terrain in enumerate(terrain_order):
        vals = pivot[terrain].values
        ax.bar(x + i * width, vals, width,
               label=terrain.capitalize(),
               color=TERRAIN_COLORS[terrain],
               edgecolor="black", linewidth=0.3)

    display_labels = [
        f"{l} (best NN)" if l == best_nn_label else l
        for l in compare_labels
    ]
    ax.set_xticks(x + width)
    ax.set_xticklabels(display_labels, fontsize=9)
    ax.set_ylabel("RMS Cross-Track Error (m)")
    ax.legend()
    ax.set_title("Analytical Tire Models vs Best Static NN")
    fig.tight_layout()
    path = out_dir / "nn_vs_analytical.png"
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  {path.name}")
    return path


# ── Plot 3: RMS CTE vs solver success (scatter) ──────────────────────
def plot_cte_vs_solver(df, out_dir):
    """Scatter: RMS CTE vs solver success, colored by model type."""
    fig, ax = plt.subplots(figsize=(7, 5))
    ok = df[df["status"] == "ok"].copy()

    for mtype in ["static", "rate", "temporal"]:
        subset = ok[ok["model_type"] == mtype]
        ax.scatter(subset["success_pct"], subset["rms_cte_m"],
                   c=TYPE_COLORS[mtype], label=TYPE_LABELS[mtype],
                   alpha=0.6, s=30, edgecolors="black", linewidth=0.3)

    ax.set_xlabel("Solver Success Rate (%)")
    ax.set_ylabel("RMS Cross-Track Error (m)")
    ax.set_yscale("log")
    ax.set_xlim(-5, 105)
    ax.legend(loc="upper left")
    ax.set_title("Tracking Error vs. Solver Success")

    # Reference region for "good" performance
    ax.axvspan(90, 105, alpha=0.08, color="green")
    ax.axhspan(0, 0.3, alpha=0.08, color="green")

    fig.tight_layout()
    path = out_dir / "cte_vs_solver_success.png"
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  {path.name}")
    return path


# ── Plot 4: Static model comparison across terrains ──────────────────
def plot_static_comparison(df, out_dir):
    """Grouped bar: RMS CTE for each static model, grouped by terrain."""
    static = df[(df["model_type"] == "static") & (df["status"] == "ok")].copy()
    # Only include runs with >80% solver success (meaningful tracking)
    static = static[static["success_pct"] > 80]

    # Average across paths for each (model, terrain)
    pivot = static.pivot_table(
        values="rms_cte_m", index="label", columns="terrain", aggfunc="mean"
    )
    # Sort models by param count
    model_order = sorted(pivot.index, key=lambda x: PARAM_COUNTS.get(x, 9999))
    pivot = pivot.reindex(model_order)
    terrain_order = ["sand", "clay", "dirt"]
    pivot = pivot[terrain_order]

    fig, ax = plt.subplots(figsize=(10, 5))
    x = np.arange(len(model_order))
    width = 0.25

    for i, terrain in enumerate(terrain_order):
        vals = pivot[terrain].values
        bars = ax.bar(x + i * width, vals, width,
                      label=terrain.capitalize(),
                      color=TERRAIN_COLORS[terrain],
                      edgecolor="black", linewidth=0.3)

    ax.set_xticks(x + width)
    ax.set_xticklabels(model_order, rotation=45, ha="right", fontsize=8)
    ax.set_ylabel("RMS Cross-Track Error (m)")
    ax.legend()
    ax.set_title("Static NN Models: Tracking Accuracy by Terrain\n(only runs with >80% solver success)")
    fig.tight_layout()
    path = out_dir / "static_model_comparison.png"
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  {path.name}")
    return path


# ── Plot 5: Solver success by temporal window K ──────────────────────
def plot_temporal_vs_K(df, out_dir):
    """Line plot: solver success vs temporal window K."""
    temporal = df[df["model_type"] == "temporal"].copy()

    def extract_K(label):
        for part in label.split("_"):
            if part.startswith("K") and part[1:].isdigit():
                return int(part[1:])
        return 0
    temporal["K"] = temporal["label"].apply(extract_K)

    # Also add static as K=0
    static = df[df["model_type"] == "static"].copy()
    static["K"] = 0
    combined = pd.concat([static, temporal], ignore_index=True)

    grouped = combined.groupby("K")["success_pct"].agg(["mean", "std"]).reset_index()
    grouped = grouped.sort_values("K")

    fig, ax = plt.subplots(figsize=(6, 4))
    ax.errorbar(grouped["K"], grouped["mean"], yerr=grouped["std"],
                marker="o", capsize=5, color="#333", linewidth=2, markersize=8)
    ax.fill_between(grouped["K"],
                    (grouped["mean"] - grouped["std"]).clip(0),
                    (grouped["mean"] + grouped["std"]).clip(0, 100),
                    alpha=0.15, color="#2196F3")

    ax.set_xlabel("Temporal Window K (0 = static)")
    ax.set_ylabel("Solver Success Rate (%)")
    ax.set_ylim(-5, 110)
    ax.set_xticks(sorted(grouped["K"].unique()))
    ax.axhline(100, color="gray", linestyle="--", linewidth=0.5)
    ax.set_title("MPC Solver Success vs. Temporal Window Size")
    fig.tight_layout()
    path = out_dir / "solver_success_vs_K.png"
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  {path.name}")
    return path


# ── Plot 6: Solve time vs param count ────────────────────────────────
def plot_solve_time_vs_params(df, out_dir):
    """Scatter: mean solve time vs param count, colored by type."""
    ok = df[(df["status"] == "ok") & (df["n_params"] > 0)].copy()
    avg = ok.groupby("label").agg(
        mean_solve_ms=("mean_solve_ms", "mean"),
        n_params=("n_params", "first"),
        model_type=("model_type", "first")
    ).reset_index()

    fig, ax = plt.subplots(figsize=(7, 5))
    for mtype in ["static", "rate", "temporal"]:
        sub = avg[avg["model_type"] == mtype]
        ax.scatter(sub["n_params"], sub["mean_solve_ms"],
                   c=TYPE_COLORS[mtype], label=TYPE_LABELS[mtype],
                   s=60, edgecolors="black", linewidth=0.5, zorder=3)

    ax.set_xlabel("Number of NN Parameters")
    ax.set_ylabel("Mean Solve Time (ms)")
    ax.set_xscale("log")
    ax.legend()
    ax.set_title("MPC Solve Time vs. Model Size")
    ax.axhline(10, color="red", linestyle="--", linewidth=0.5, alpha=0.5,
               label="10ms real-time limit")
    fig.tight_layout()
    path = out_dir / "solve_time_vs_params.png"
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  {path.name}")
    return path


# ── Plot 7: Per-path breakdown for static models ─────────────────────
def plot_path_breakdown(df, out_dir):
    """Faceted bar: RMS CTE for static models, one subplot per path."""
    static = df[(df["model_type"] == "static") & (df["status"] == "ok")].copy()
    static = static[static["success_pct"] > 80]
    paths = ["lane_change", "double_lane_change", "sinusoidal"]

    model_order = sorted(static["label"].unique(),
                         key=lambda x: PARAM_COUNTS.get(x, 9999))

    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5), sharey=True)
    for ax, pname in zip(axes, paths):
        sub = static[static["path"] == pname]
        pivot = sub.pivot_table("rms_cte_m", "label", "terrain", "mean")
        pivot = pivot.reindex(model_order).dropna(how="all")
        terrain_order = ["sand", "clay", "dirt"]

        x = np.arange(len(pivot))
        width = 0.25
        for i, t in enumerate(terrain_order):
            if t in pivot.columns:
                ax.bar(x + i * width, pivot[t].values, width,
                       color=TERRAIN_COLORS[t], label=t.capitalize() if ax == axes[0] else "",
                       edgecolor="black", linewidth=0.3)
        ax.set_xticks(x + width)
        ax.set_xticklabels(pivot.index, rotation=45, ha="right", fontsize=7)
        ax.set_title(pname.replace("_", " ").title())
        if ax == axes[0]:
            ax.set_ylabel("RMS CTE (m)")

    axes[0].legend(loc="upper left", fontsize=8)
    fig.suptitle("Static NN Tracking by Path & Terrain (solver success > 80%)", y=1.02)
    fig.tight_layout()
    path = out_dir / "static_path_breakdown.png"
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"  {path.name}")
    return path


def main():
    p = argparse.ArgumentParser(description="Plot paper benchmark results")
    p.add_argument("--csv", type=Path, default=DEFAULT_CSV,
                   help="Path to benchmark results CSV")
    p.add_argument("--out-dir", type=Path, default=None,
                   help="Output directory for plots (default: same as CSV)")
    args = p.parse_args()

    if not args.csv.exists():
        print(f"ERROR: CSV not found: {args.csv}")
        print("Run 'python run_paper_benchmarks.py' first.")
        sys.exit(1)

    out_dir = args.out_dir or args.csv.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Loading: {args.csv}")
    df = load_data(args.csv)
    print(f"  {len(df)} rows, {df['label'].nunique()} models, "
          f"{df['terrain'].nunique()} terrains, {df['path'].nunique()} paths")
    print(f"Output: {out_dir}\n")

    print("Generating plots:")
    plot_solver_success_by_type(df, out_dir)
    plot_solver_heatmap(df, out_dir)
    plot_nn_vs_analytical(df, out_dir)
    plot_cte_vs_solver(df, out_dir)
    plot_static_comparison(df, out_dir)
    plot_temporal_vs_K(df, out_dir)
    plot_solve_time_vs_params(df, out_dir)
    plot_path_breakdown(df, out_dir)

    print(f"\nDone — 8 figures saved to {out_dir}")


if __name__ == "__main__":
    main()
