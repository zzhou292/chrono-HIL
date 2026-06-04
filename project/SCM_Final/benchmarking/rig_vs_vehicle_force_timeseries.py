#!/usr/bin/env python3
"""Build the predicted-vs-actual tire-force time-series figure (paper §III-C)
AND the quantitative smoothness / over-fit metrics that underpin the
discussion in RIG_VS_VEHICLE_FINDINGS.md §5.

Reads each successful diag CSV in the rig-vs-vehicle sweeps and computes,
per surrogate:

* `rmse_Fy_<axle>`         — RMSE of (pred_Fy - actual_Fy)        (force fit)
* `std_pred_Fy_<axle>`     — std(pred_Fy) over the run            (output bandwidth)
* `std_dpred_Fy_<axle>`    — std(d(pred_Fy)/dt)                   (output rate)
* `std_dsteer`             — std(d(steering)/dt)                  (control twitchiness)
* `std_dthrottle`          — std(d(throttle)/dt)
* `force_resid_bias`       — DOB-integrated residual magnitude
* `force_resid_updates`    — DOB update count
* `pred_actual_corr_<axle>`— corr( pred_Fy, actual_Fy )           (gradient direction)

Together these distinguish:

* "rig surrogate is smoother" -> lower std_dpred_Fy and lower std_dsteer
* "vehicle surrogate fits force better" -> lower rmse_Fy and higher
  pred_actual_corr
* "DOB compensates the bias" -> larger force_resid_bias for the worse-fit
  surrogate, similar tracking outcome.

Writes:

* `rig_vs_vehicle_force_metrics.csv`  -- per-run metrics
* `rig_vs_vehicle_force_metrics_summary.csv` -- aggregated per-surrogate
* `rig_vs_vehicle_pred_vs_actual.png` -- the paper figure: pred vs actual
  Fy time series for all four surrogates on one paired scenario.
* `rig_vs_vehicle_smoothness_bars.png` -- aggregated smoothness metrics.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
GEN_COLOR = {"rig": "#d95f02", "vehicle": "#1f78b4"}
VARIANT_GEN = {
    # legacy lineup
    "rig_static":       ("rig",     "static"),
    "rig_rate":         ("rig",     "rate"),
    "vehicle_static":   ("vehicle", "static"),
    "vehicle_rate":     ("vehicle", "rate"),
    # matched-arch + LHS-vehicle lineup (post-retrain)
    "rig_static_lg":       ("rig",     "static"),
    "rig_rate_lg":         ("rig",     "rate"),
    "rig_rate_xl":         ("rig",     "rate"),
    "vehicle_static_lhs":  ("vehicle", "static"),
    "vehicle_rate_lhs":    ("vehicle", "rate"),
    "vehicle_rate_xl_lhs": ("vehicle", "rate"),
}
ORDER_LEGACY = ["rig_static", "vehicle_static", "rig_rate", "vehicle_rate"]
ORDER_NEW    = ["rig_static_lg", "vehicle_static_lhs",
                "rig_rate_lg",   "vehicle_rate_lhs",
                "rig_rate_xl",   "vehicle_rate_xl_lhs"]
ORDER = ORDER_LEGACY  # default; main() switches based on which variants exist


# ---------------------------------------------------------------- per-run --

def _diag_for(run_dir: Path) -> Path | None:
    """Find the diag_*.csv inside a per-run dir."""
    cands = sorted(run_dir.glob("**/diag_*.csv"))
    return cands[0] if cands else None


def per_run_metrics(diag_csv: Path, metric_start: float = 2.0) -> dict:
    df = pd.read_csv(diag_csv)
    df = df[df["sim_time"] > metric_start].reset_index(drop=True)
    out: dict = {"n_rows": int(len(df))}
    if len(df) < 10:
        return out
    t = df["sim_time"].to_numpy()
    dt = np.diff(t).mean() if len(t) > 1 else 0.01
    for axle in ("front", "rear"):
        pred = df.get(f"pred_Fy_{axle}", pd.Series([], dtype=float)).to_numpy()
        actual = df.get(f"actual_Fy_{axle}", pd.Series([], dtype=float)).to_numpy()
        if pred.size and actual.size:
            res = pred - actual
            out[f"rmse_Fy_{axle}"]      = float(np.sqrt(np.mean(res ** 2)))
            out[f"std_pred_Fy_{axle}"]  = float(np.std(pred))
            out[f"std_actual_Fy_{axle}"]= float(np.std(actual))
            out[f"std_dpred_Fy_{axle}"] = float(np.std(np.diff(pred) / dt))
            out[f"std_dactual_Fy_{axle}"] = float(np.std(np.diff(actual) / dt))
            if np.std(pred) > 1e-3 and np.std(actual) > 1e-3:
                out[f"pred_actual_corr_{axle}"] = float(
                    np.corrcoef(pred, actual)[0, 1])
            else:
                out[f"pred_actual_corr_{axle}"] = float("nan")
    for col in ("steering", "throttle"):
        if col in df.columns:
            v = df[col].to_numpy()
            out[f"std_d{col}"] = float(np.std(np.diff(v) / dt))
    if "force_resid_bias_f" in df.columns:
        out["force_resid_bias_f_mean"] = float(np.abs(df["force_resid_bias_f"]).mean())
    if "force_resid_bias_r" in df.columns:
        out["force_resid_bias_r_mean"] = float(np.abs(df["force_resid_bias_r"]).mean())
    if "force_resid_updates" in df.columns:
        out["force_resid_updates"] = float(df["force_resid_updates"].iloc[-1])
    return out


# -------------------------------------------------------------- collection --

def collect(results_csvs: list[Path]) -> pd.DataFrame:
    rows = []
    for results_csv in results_csvs:
        if not results_csv.exists(): continue
        df = pd.read_csv(results_csv)
        ok = df[df["status"] == "ok"]
        for _, r in ok.iterrows():
            diag_path = r.get("diag_csv", "")
            if not isinstance(diag_path, str) or not diag_path:
                continue
            p = Path(diag_path)
            if not p.exists():
                continue
            try:
                m = per_run_metrics(p)
            except Exception as e:
                m = {"err": str(e)}
            m.update(dict(
                variant=r["variant"], terrain=r["terrain"], path=r["path"],
                speed_mps=r["speed_mps"], bumpiness=r["bumpiness"],
                seed=r["seed"], rms_cte_m=r["rms_cte_m"],
                diag_csv=str(p),
            ))
            rows.append(m)
    return pd.DataFrame(rows)


# ------------------------------------------------------------- per-cell fig -

def pred_vs_actual_panel(ax, diag_csv: Path, variant: str, axle: str = "front",
                         t_start: float = 2.0, t_end: float = 12.0) -> None:
    df = pd.read_csv(diag_csv)
    df = df[(df["sim_time"] >= t_start) & (df["sim_time"] <= t_end)]
    if df.empty: return
    t = df["sim_time"].to_numpy()
    pred = df[f"pred_Fy_{axle}"].to_numpy()
    actual = df[f"actual_Fy_{axle}"].to_numpy()
    res = pred - actual
    rmse = np.sqrt(np.mean(res ** 2))
    gen, sig = VARIANT_GEN[variant]
    ax.plot(t, actual, color="#222", lw=1.2, label="Chrono ground truth")
    ax.plot(t, pred, color=GEN_COLOR[gen], lw=1.2, alpha=0.9,
            label=f"{variant} pred")
    ax.set_title(f"{variant}    RMSE = {rmse:.0f} N    "
                 f"std(pred) = {np.std(pred):.0f} N    "
                 f"std(d/dt pred) = {np.std(np.diff(pred) / 0.012):.0f} N/s",
                 fontsize=10)
    ax.set_xlabel("sim time (s)")
    ax.set_ylabel(f"{axle}-axle  Fy (N)")
    ax.grid(alpha=0.3)
    ax.legend(loc="lower right", fontsize=8)


def build_pred_vs_actual_figure(diag_map: dict[str, Path],
                                out_path: Path,
                                scenario_label: str,
                                axle: str = "front") -> None:
    n = len(ORDER)
    fig, axes = plt.subplots(n, 1, figsize=(11, max(2.6 * n, 8)), sharex=True)
    if n == 1: axes = [axes]
    for ax, variant in zip(axes, ORDER):
        d = diag_map.get(variant)
        if d is None or not d.exists():
            ax.set_title(f"{variant}  -- missing diag CSV"); continue
        pred_vs_actual_panel(ax, d, variant, axle=axle)
    fig.suptitle(f"Predicted vs Chrono ground-truth $F_y$ ({axle} axle) "
                 f"— {scenario_label}", fontsize=12)
    fig.text(0.5, 0.005, "All four surrogates on the same scenario seed; "
             "rig-trained columns produce lower-bandwidth force predictions "
             "while tracking the Chrono trajectory shape.",
             ha="center", fontsize=8, color="#444")
    fig.tight_layout(rect=(0, 0.02, 1, 0.97))
    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out_path}")


# --------------------------------------------------------- aggregate figs --

def aggregate_figure(metrics_df: pd.DataFrame, out_path: Path) -> None:
    """Bar chart of smoothness / fit metrics per surrogate."""
    if metrics_df.empty: return
    metrics_df = metrics_df.copy()
    metrics_df["generation"] = metrics_df["variant"].map(
        lambda v: VARIANT_GEN[v][0])
    metrics_df["signature"]  = metrics_df["variant"].map(
        lambda v: VARIANT_GEN[v][1])

    panels = [
        ("rmse_Fy_front",      "RMSE $F_y$ front (N) — lower better"),
        ("rmse_Fy_rear",       "RMSE $F_y$ rear (N) — lower better"),
        ("std_dpred_Fy_front", "std d/dt(pred $F_y$) front (N/s) — lower = smoother surrogate"),
        ("std_dpred_Fy_rear",  "std d/dt(pred $F_y$) rear (N/s) — lower = smoother surrogate"),
        ("pred_actual_corr_front",
                              "corr(pred, actual) $F_y$ front — higher = right shape"),
        ("pred_actual_corr_rear",
                              "corr(pred, actual) $F_y$ rear — higher = right shape"),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(13.5, 7.5))
    x = np.arange(len(ORDER))
    colors = [GEN_COLOR[VARIANT_GEN[v][0]] for v in ORDER]
    for ax, (col, title) in zip(axes.flat, panels):
        if col not in metrics_df.columns:
            ax.set_visible(False); continue
        means = (metrics_df.groupby("variant")[col].mean()
                            .reindex(ORDER).values)
        stds  = (metrics_df.groupby("variant")[col].std()
                            .reindex(ORDER).values)
        ax.bar(x, means, yerr=stds, color=colors,
               edgecolor="black", linewidth=0.4, capsize=3)
        ax.set_xticks(x); ax.set_xticklabels(ORDER, rotation=20, ha="right")
        ax.set_title(title, fontsize=10)
        ax.grid(axis="y", alpha=0.3)
    fig.suptitle("Per-surrogate force-prediction fidelity and smoothness "
                 "(aggregated over all OK runs)", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out_path}")


# --------------------------------------------------------------------- main -

def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--sweep", default=str(ROOT / "benchmarking" / "results"
                                          / "rig_vs_vehicle_tire_sweep_20260519_233737"),
                   help="Primary sweep dir (contains results.csv + raw/).")
    p.add_argument("--extra-sweep",
                   default=str(ROOT / "benchmarking" / "results"
                               / "rig_vs_vehicle_tire_sweep_20260520_001233"),
                   help="Second sweep dir to also include (e.g. the rerun).")
    p.add_argument("--cell-terrain", default="sand")
    p.add_argument("--cell-path", default="sinusoidal")
    p.add_argument("--cell-speed", type=float, default=5.0)
    p.add_argument("--cell-bump", type=int, default=0)
    p.add_argument("--cell-seed", type=int, default=900)
    p.add_argument("--out-dir", default=None)
    args = p.parse_args()

    sweep = Path(args.sweep)
    out_dir = Path(args.out_dir) if args.out_dir else sweep
    extra = Path(args.extra_sweep) if args.extra_sweep else None
    print(f"primary sweep: {sweep}")
    if extra:
        print(f"extra sweep:   {extra}")

    results_csvs = [sweep / "results.csv"]
    if extra and (extra / "results.csv").exists():
        results_csvs.append(extra / "results.csv")

    # Collect per-run metrics for every OK run across both sweeps.
    metrics_df = collect(results_csvs)
    # Pick the right ORDER based on which variants actually show up.
    global ORDER
    if not metrics_df.empty:
        present = set(metrics_df["variant"].unique())
        new_hit = sum(1 for v in ORDER_NEW if v in present)
        legacy_hit = sum(1 for v in ORDER_LEGACY if v in present)
        ORDER = ORDER_NEW if new_hit >= legacy_hit else ORDER_LEGACY
    metrics_csv = out_dir / "rig_vs_vehicle_force_metrics.csv"
    metrics_df.to_csv(metrics_csv, index=False)
    print(f"wrote {metrics_csv}  ({len(metrics_df)} rows)")

    # Aggregate summary table.
    agg_cols = [c for c in metrics_df.columns
                if c.startswith(("rmse_Fy_", "std_pred_Fy_", "std_actual_Fy_",
                                 "std_dpred_Fy_", "std_dactual_Fy_",
                                 "pred_actual_corr_",
                                 "std_dsteering", "std_dthrottle",
                                 "force_resid_bias", "force_resid_updates"))]
    summary = (metrics_df.groupby("variant")[agg_cols]
                          .mean()
                          .reindex(ORDER))
    summary_csv = out_dir / "rig_vs_vehicle_force_metrics_summary.csv"
    summary.to_csv(summary_csv)
    print(f"wrote {summary_csv}")
    print()
    print(summary.round(2).to_string())

    # The paired predicted-vs-actual figure for one canonical cell.
    diag_map: dict[str, Path] = {}
    cell_token = (f"{args.cell_terrain}_{args.cell_path}_"
                  f"v{args.cell_speed:g}_b{args.cell_bump}_s{args.cell_seed}")
    for variant in ORDER:
        # Try primary sweep first, then extra.
        match = None
        for sw in (sweep, extra) if extra else (sweep,):
            cands = sorted((sw / "raw").glob(f"*{variant}_{cell_token}"))
            if cands:
                d = _diag_for(cands[0])
                if d is not None:
                    match = d; break
        if match is None:
            print(f"WARNING: no diag CSV for {variant} on cell {cell_token}")
            continue
        diag_map[variant] = match
        print(f"  {variant}: {match}")

    if len(diag_map) == len(ORDER):
        fig_path = out_dir / "rig_vs_vehicle_pred_vs_actual.png"
        scenario_label = (
            f"{args.cell_terrain} / {args.cell_path} / "
            f"v_ref = {args.cell_speed:g} m/s / b = {args.cell_bump} / "
            f"seed = {args.cell_seed}")
        build_pred_vs_actual_figure(diag_map, fig_path, scenario_label,
                                    axle="front")
        build_pred_vs_actual_figure(
            diag_map, out_dir / "rig_vs_vehicle_pred_vs_actual_rear.png",
            scenario_label, axle="rear")
    else:
        print(f"Need {len(ORDER)} diag CSVs for the figure, got {len(diag_map)}")

    aggregate_figure(metrics_df,
                     out_dir / "rig_vs_vehicle_smoothness_bars.png")


if __name__ == "__main__":
    main()
