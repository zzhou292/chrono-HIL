#!/usr/bin/env python3
"""Post-process rig-vs-vehicle sweep: emit the markdown tables we want to
splice into RIG_VS_VEHICLE_FINDINGS.md.

Reads ``results.csv`` from the most recent rig_vs_vehicle_tire_sweep_* dir
(or the path given on the command line) and prints three tables:

1. Mean metrics per surrogate over all 288 runs.
2. Static-vs-static and rate-vs-rate paired delta.
3. Per-terrain mean RMS CTE / mean speed for each surrogate.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pandas as pd

GENERATION = {
    "rig_static": "rig", "rig_rate": "rig",
    "vehicle_static": "vehicle", "vehicle_rate": "vehicle",
    # post-retrain labels
    "rig_static_lg":       "rig",     "rig_rate_lg":         "rig",
    "rig_rate_xl":         "rig",
    "vehicle_static_lhs":  "vehicle", "vehicle_rate_lhs":    "vehicle",
    "vehicle_rate_xl_lhs": "vehicle",
}
SIGNATURE = {
    "rig_static": "static", "vehicle_static": "static",
    "rig_rate": "rate",    "vehicle_rate": "rate",
    "rig_static_lg":       "static",
    "rig_rate_lg":         "rate",
    "rig_rate_xl":         "rate",
    "vehicle_static_lhs":  "static",
    "vehicle_rate_lhs":    "rate",
    "vehicle_rate_xl_lhs": "rate",
}
# `ORDER` is used for reindexing and as the universe of cells the
# win-counter sees.  Use the union of all variants that appear in the
# results.csv if it's the new lineup; else fall back to legacy.
ORDER_LEGACY = ["rig_static", "vehicle_static", "rig_rate", "vehicle_rate"]
ORDER_NEW    = ["rig_static_lg", "vehicle_static_lhs",
                "rig_rate_lg",   "vehicle_rate_lhs",
                "rig_rate_xl",   "vehicle_rate_xl_lhs"]


def _select_order(variants_in_csv: set) -> list:
    """Pick the right column order based on which lineup the CSV has."""
    new_hit = sum(1 for v in ORDER_NEW if v in variants_in_csv)
    legacy_hit = sum(1 for v in ORDER_LEGACY if v in variants_in_csv)
    return ORDER_NEW if new_hit >= legacy_hit else ORDER_LEGACY


ORDER = ORDER_LEGACY  # placeholder; replaced in main()


def md_table(df: pd.DataFrame, fmt: dict[str, str]) -> str:
    cols = list(df.columns)
    header = "| " + " | ".join(cols) + " |"
    sep    = "|" + "|".join("---" if c == cols[0] else "---:" for c in cols) + "|"
    rows = []
    for _, r in df.iterrows():
        vals = []
        for c in cols:
            v = r[c]
            if c in fmt and isinstance(v, (int, float, np.integer, np.floating)) and pd.notna(v):
                spec = fmt[c]
                if spec.endswith("d}"):
                    vals.append(spec.format(int(v)))
                else:
                    vals.append(spec.format(float(v)))
            else:
                vals.append(str(v))
        rows.append("| " + " | ".join(vals) + " |")
    return "\n".join([header, sep, *rows])


def main() -> int:
    if len(sys.argv) > 1:
        path = Path(sys.argv[1])
    else:
        root = Path(__file__).resolve().parents[1]
        cands = sorted((root / "benchmarking" / "results").glob(
            "rig_vs_vehicle_tire_sweep_*"))
        if not cands:
            print("No sweep dir found"); return 1
        path = cands[-1] / "results.csv"
    print(f"# Reading {path}\n")
    df = pd.read_csv(path)
    ok = df[df["status"] == "ok"].copy()
    if ok.empty:
        print("(no successful runs)"); return 1
    global ORDER
    ORDER = _select_order(set(ok["variant"].unique()))
    ok["generation"] = ok["variant"].map(GENERATION)
    ok["signature"]  = ok["variant"].map(SIGNATURE)

    print(f"### Run counts\n\nTotal: {len(df)}, ok: {len(ok)}, "
          f"failed: {len(df) - len(ok)}\n")

    # 1) Per-variant means over all scenarios
    g = (ok.groupby("variant")
           .agg(n=("variant", "size"),
                mean_speed=("mean_speed_mps", "mean"),
                rms_cte=("rms_cte_m", "mean"),
                max_cte=("max_abs_cte_m", "mean"),
                speed_ratio=("speed_ratio", "mean"),
                solve_ms=("mean_solve_ms", "mean"))
           .reindex(ORDER).reset_index())
    g["n"] = g["n"].fillna(0).astype(int)
    g.columns = ["surrogate", "n", "mean speed (m/s)",
                 "RMS CTE (m)", "max |CTE| (m)", "speed/v_ref",
                 "solve (ms)"]
    print("### 4.1 Mean metrics by surrogate (all scenarios)\n")
    print(md_table(g, {
        "mean speed (m/s)": "{:.2f}", "RMS CTE (m)": "{:.3f}",
        "max |CTE| (m)": "{:.2f}", "speed/v_ref": "{:.2f}",
        "solve (ms)": "{:.2f}", "n": "{:d}"}))
    print()

    # 2) Paired static / rate
    paired_rows = []
    for sig in ("static", "rate"):
        rig = ok[ok["variant"] == f"rig_{sig}"]
        veh = ok[ok["variant"] == f"vehicle_{sig}"]
        if rig.empty or veh.empty:
            continue
        paired_rows.append({
            "signature": sig,
            "rig mean speed":  rig["mean_speed_mps"].mean(),
            "veh mean speed":  veh["mean_speed_mps"].mean(),
            "Δ speed":         veh["mean_speed_mps"].mean() - rig["mean_speed_mps"].mean(),
            "rig RMS CTE":     rig["rms_cte_m"].mean(),
            "veh RMS CTE":     veh["rms_cte_m"].mean(),
            "Δ RMS CTE":       veh["rms_cte_m"].mean() - rig["rms_cte_m"].mean(),
            "rig max|CTE|":    rig["max_abs_cte_m"].mean(),
            "veh max|CTE|":    veh["max_abs_cte_m"].mean(),
        })
    paired_df = pd.DataFrame(paired_rows)
    print("### 4.2 Paired comparison (vehicle − rig)\n")
    print(md_table(paired_df, {c: "{:.3f}" for c in paired_df.columns
                               if c != "signature"}))
    print()

    # 3) Per-terrain breakdown
    pt = (ok.groupby(["terrain", "variant"])
            .agg(mean_speed=("mean_speed_mps", "mean"),
                 rms_cte=("rms_cte_m", "mean"))
            .reset_index())
    pivot_cte = pt.pivot(index="terrain", columns="variant",
                          values="rms_cte").reindex(columns=ORDER).reset_index()
    pivot_v   = pt.pivot(index="terrain", columns="variant",
                          values="mean_speed").reindex(columns=ORDER).reset_index()
    print("### 4.3a Per-terrain RMS CTE (m)\n")
    print(md_table(pivot_cte, {c: "{:.3f}" for c in pivot_cte.columns
                                if c != "terrain"}))
    print()
    print("### 4.3b Per-terrain mean speed (m/s)\n")
    print(md_table(pivot_v, {c: "{:.2f}" for c in pivot_v.columns
                              if c != "terrain"}))
    print()

    # 4) Per-path breakdown
    pp = (ok.groupby(["path", "variant"])
            .agg(rms_cte=("rms_cte_m", "mean"))
            .reset_index())
    pivot_p = pp.pivot(index="path", columns="variant",
                        values="rms_cte").reindex(columns=ORDER).reset_index()
    print("### 4.3c Per-path RMS CTE (m)\n")
    print(md_table(pivot_p, {c: "{:.3f}" for c in pivot_p.columns
                              if c != "path"}))
    print()

    # 5) Win/loss count: per cell, which surrogate has the lowest RMS CTE?
    cells = (ok.groupby(["terrain", "path", "speed_mps", "bumpiness", "variant"])
               ["rms_cte_m"].mean().reset_index())
    wins = {v: 0 for v in ORDER}
    n_cells = 0
    for (t, p, s, b), sub in cells.groupby(["terrain", "path", "speed_mps", "bumpiness"]):
        if len(sub) < len(ORDER): continue
        n_cells += 1
        winner = sub.loc[sub["rms_cte_m"].idxmin(), "variant"]
        if winner in wins:
            wins[winner] += 1
    print(f"### Cell-winner count over {n_cells} scenarios (lowest RMS CTE)\n")
    w = pd.DataFrame([{"surrogate": v, "wins": wins[v]} for v in ORDER])
    print(md_table(w, {"wins": "{:d}"}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
