#!/usr/bin/env python3
"""Aggregate a batch of per-(link, scenario) convoy_counterfactual_eval runs
from a real human-teleoperation session into one table + figure.

Each input dir is ONE recorded human drive replayed filter-off vs DOB-CBF-on on
the same convoy preset and link. This rolls them up into the paper's
human-intent corroboration of the safety filter: on genuine (competent) human
intent the operator may be collision-free yet skim hazards with near-zero
margin; DOB-CBF restores clearance at low intrusiveness without ever inducing a
collision. (The collision-PREVENTION headline comes from the generated
adversarial-intent counterfactual + the latency-awareness dose-response; this
script measures the filter's effect on real human traces.)

Usage:
  python benchmarking/aggregate_teleop_counterfactual.py \
      --dirs benchmarking/results/convoy_counterfactual_eval_2026..._* \
      --out-name teleop_counterfactual
"""
from __future__ import annotations
import argparse, glob, re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "benchmarking" / "results"
FIGDIR = ROOT / "my_paper" / "paper_figures"

LINK_LABEL = {"5g_nhits_geforce": "good link", "5g_nhits_congested": "congested link"}


def _parse_cell(cell: str) -> tuple[str, str]:
    """cell like '0000_none_lead_brake_5g_nhits_geforce_clay_straight_v4_b0_r0'
    -> (link, scenario)."""
    m = re.search(r"(5g_nhits_\w+?)_(?:clay|sand|dirt)", cell)
    link = m.group(1) if m else "unknown"
    # scenario = the convoy preset, between the filter tag and the link tag
    s = cell
    s = re.sub(r"^\d+_(none|dob_cbf|dob_blind|dob_aware)_", "", s)
    s = re.sub(rf"_{re.escape(link)}_.*$", "", s)
    return link, s


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dirs", nargs="+", required=True,
                   help="convoy_counterfactual_eval_* dirs (globs ok)")
    p.add_argument("--out-name", default="teleop_counterfactual")
    args = p.parse_args()

    dirs = []
    for d in args.dirs:
        dirs.extend(sorted(glob.glob(d)))
    rows = []
    for d in dirs:
        rc = Path(d) / "results.csv"
        if not rc.exists():
            continue
        df = pd.read_csv(rc)
        if df.empty:
            continue
        none = df[df["filter"] == "none"]
        dob = df[df["filter"] == "dob_cbf"]
        if none.empty or dob.empty:
            continue
        link, scen = _parse_cell(str(none.iloc[0]["cell"]))
        rows.append({
            "link": link, "scenario": scen,
            "none_collided": int(none["collided"].iloc[0]),
            "dob_collided": int(dob["collided"].iloc[0]),
            "none_clearance_m": round(float(none["min_clearance_m"].iloc[0]), 3),
            "dob_clearance_m": round(float(dob["min_clearance_m"].iloc[0]), 3),
            "clearance_gain_m": round(float(dob["min_clearance_m"].iloc[0])
                                      - float(none["min_clearance_m"].iloc[0]), 3),
            "none_progress_m": round(float(none["progress_x_m"].iloc[0]), 1),
            "dob_progress_m": round(float(dob["progress_x_m"].iloc[0]), 1),
            "mean_abs_dsteer": round(float(dob["mean_abs_dsteer"].iloc[0]), 4),
            "mean_abs_dthrottle": round(float(dob["mean_abs_dthrottle"].iloc[0]), 4),
        })
    if not rows:
        raise SystemExit("no usable results.csv found in the given dirs")
    out = pd.DataFrame(rows).sort_values(["link", "scenario"]).reset_index(drop=True)
    csv_path = FIGDIR / f"{args.out_name}_results.csv"
    out.to_csv(csv_path, index=False)

    # ---- headline aggregate numbers ----
    n = len(out)
    print(f"\n=== teleop counterfactual: {n} recorded human drives "
          f"({out['link'].nunique()} links x {out['scenario'].nunique()} scenarios) ===")
    print(out.to_string(index=False))
    agg = {
        "drives": n,
        "human_collisions": int(out["none_collided"].sum()),
        "filtered_collisions": int(out["dob_collided"].sum()),
        "median_human_clearance_m": round(out["none_clearance_m"].median(), 3),
        "median_filtered_clearance_m": round(out["dob_clearance_m"].median(), 3),
        "min_human_clearance_m": round(out["none_clearance_m"].min(), 3),
        "median_clearance_gain_m": round(out["clearance_gain_m"].median(), 3),
        "mean_abs_dsteer": round(out["mean_abs_dsteer"].mean(), 4),
        "mean_abs_dthrottle": round(out["mean_abs_dthrottle"].mean(), 4),
    }
    print("\nAggregate:")
    for k, v in agg.items():
        print(f"  {k:28s}: {v}")
    pd.DataFrame([agg]).to_csv(FIGDIR / f"{args.out_name}_summary.csv", index=False)

    # ---- figure: per-(link,scenario) min clearance, human vs filtered ----
    out["label"] = out["scenario"] + "\n(" + out["link"].map(
        lambda l: LINK_LABEL.get(l, l)) + ")"
    x = range(n)
    fig, ax = plt.subplots(figsize=(max(7, 1.15 * n), 4.2))
    w = 0.4
    ax.bar([i - w / 2 for i in x], out["none_clearance_m"], w,
           label="operator (no filter)", color="#b0392b")
    ax.bar([i + w / 2 for i in x], out["dob_clearance_m"], w,
           label="+ DOB-CBF", color="#28c76f")
    ax.axhline(0, color="k", lw=0.8)
    # A clearance < 0 is a collision (hazard overlap); annotate the operator's.
    for i, r in out.iterrows():
        if r["none_collided"]:
            ax.annotate("operator\ncollision", (i - w / 2, min(r["none_clearance_m"], 0)),
                        textcoords="offset points", xytext=(0, -2), ha="center",
                        va="top", fontsize=6.5, color="#b0392b")
    net = agg["human_collisions"] - agg["filtered_collisions"]
    ax.set_xticks(list(x))
    ax.set_xticklabels(out["label"], fontsize=7.5)
    ax.set_ylabel("minimum clearance to hazard (m)")
    ax.set_title(f"DOB-CBF on recorded human intent: median clearance "
                 f"{agg['median_human_clearance_m']:.2f}$\\to${agg['median_filtered_clearance_m']:.2f} m "
                 f"at $|\\Delta\\mathrm{{steer}}|\\!\\approx\\!{agg['mean_abs_dsteer']:.3f}$; "
                 f"{net} of {agg['human_collisions']} operator collisions averted", fontsize=9.5)
    ax.legend(fontsize=8.5); ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    fig_path = FIGDIR / f"{args.out_name}_clearance.png"
    fig.savefig(fig_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"\nWrote:\n  {csv_path}\n  {FIGDIR / (args.out_name + '_summary.csv')}\n  {fig_path}")


if __name__ == "__main__":
    main()
