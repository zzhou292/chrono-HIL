#!/usr/bin/env python3
"""Counterfactual safety-filter evaluation on the convoy scenario.

Replays ONE operator command trace (a recorded human run's sim_diag.csv, or a
generated "reckless" straight-at-the-convoy intent) through the identical
convoy scenario with the filter OFF and with each filter. Because the sim is
bit-for-bit deterministic, every outcome difference is purely the filter's
effect -- a causal "harm prevented" measurement that needs no path-tracking
CTE and no human-variability averaging.

Reported per (filter, delay): did it COLLIDE, min clearance, near-misses,
intervention rate, intrusiveness (|Δsteer|,|Δthrottle|), and progress. The
headline is the counterfactual delta vs the filter-OFF baseline on the same
intent: collisions prevented, clearance gained.

Examples:
  # generated reckless intent into a braking lead, off vs DOB-CBF vs MPPI:
  python convoy_counterfactual_eval.py --convoy lead_brake --filters none dob_cbf mppi

  # replay a recorded human trace under teleop latency:
  python convoy_counterfactual_eval.py --trace runs/op1/sim_diag.csv \
      --convoy gauntlet --delays 0.0 0.30 --filters none dob_cbf mppi
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (  # noqa: E402
    PROJECT_ROOT,
    parse_shield_csv,
    run_process,
    save_summary_markdown,
    timestamped_result_dir,
    write_manifest,
)

SIM_NODE = PROJECT_ROOT / "simulation" / "chrono_sim_node.py"


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--trace", default="",
                   help="Operator command trace CSV (a run's sim_diag.csv). If "
                        "omitted, a reckless straight-ahead intent is generated.")
    p.add_argument("--trace-dir", default="",
                   help="Batch mode: a HIL collection session dir (human_delay_"
                        "compensation_rounds_*). Every recorded round's sim_diag.csv "
                        "is replayed filter-off vs each filter at its recorded delay; "
                        "harm-prevented is aggregated per filter across rounds. Use "
                        "--convoy to set the scenario the rounds were collected in.")
    p.add_argument("--reckless-throttle", type=float, default=0.6,
                   help="Throttle of the generated reckless intent (when no --trace).")
    p.add_argument("--convoy", nargs="+", default=["lead_brake", "cut_in", "stalled"],
                   help="Convoy preset(s) to sweep (lead_brake/cut_in/stalled/convoy/"
                        "jam/gauntlet/...). Each is replayed off vs each filter.")
    p.add_argument("--filters", nargs="+", default=["none", "dob_cbf"],
                   choices=["none", "dob_cbf"])
    p.add_argument("--delays", nargs="+", type=float, default=[0.0],
                   help="Command-path (uplink) delays applied to the replayed intent.")
    p.add_argument("--terrain", default="clay")
    p.add_argument("--time", type=float, default=20.0)
    p.add_argument("--mesh-resolution", type=float, default=0.12)
    p.add_argument("--safety-buffer", type=float, default=0.25)
    p.add_argument("--shield-horizon", type=int, default=12)
    p.add_argument("--mppi-samples", type=int, default=384)
    p.add_argument("--workers", type=int, default=4)
    p.add_argument("--timeout", type=float, default=400.0)
    p.add_argument("--base-port", type=int, default=11200)
    return p.parse_args()


def generate_reckless_trace(path: Path, duration: float, throttle: float) -> None:
    """Straight-ahead, constant-throttle 'intent' that drives into the convoy."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time", "steering_op", "throttle_op", "braking_op"])
        t = 0.0
        while t <= duration + 2.0:
            w.writerow([f"{t:.3f}", "0.0", f"{throttle:.3f}", "0.0"])
            t += 0.1


def discover_round_traces(trace_dir: Path) -> list[tuple[str, float, str]]:
    """Find each recorded round's (sim_diag.csv, delay, label) in a session dir."""
    import re
    out = []
    for f in sorted(trace_dir.glob("raw/*/sim_diag.csv")):
        name = f.parent.name
        m = re.search(r"delay([0-9.]+)", name)
        out.append((str(f), float(m.group(1)) if m else 0.0, name))
    return out


@dataclass(frozen=True)
class Task:
    idx: int
    filter_name: str
    delay: float
    sim_port: int
    run_dir: str
    trace: str
    convoy: str
    terrain: str
    time_s: float
    mesh: float
    buffer: float
    horizon: int
    mppi_samples: int
    timeout: float
    cell: str = ""          # baseline-matching key (scenario or recorded round)


def _build_cmd(t: Task) -> list[str]:
    cmd = [
        sys.executable, "-u", str(SIM_NODE),
        "--terrain", t.terrain, "--time", str(t.time_s), "--vis-mode", "none",
        "--sim-port", str(t.sim_port), "--mesh-resolution", str(t.mesh), "--no-noise",
        "--rocks", "0", "--convoy", t.convoy,
        "--replay-cmds", t.trace,
        "--sim-diag-csv", str(Path(t.run_dir) / "sim_diag.csv"),
    ]
    if t.delay > 0:
        cmd += ["--manual-input-delay", str(t.delay), "--teleop-delay", str(t.delay)]
    if t.filter_name != "none":
        cmd += ["--safety-filter", "--safety-flavor", t.filter_name,
                "--safety-buffer", str(t.buffer), "--shield-horizon", str(t.horizon)]
        if t.filter_name == "mppi":
            cmd += ["--mppi-samples", str(t.mppi_samples)]
    return cmd


def _metrics_from_run(run_dir: Path) -> dict:
    """Outcome of one replay run: collision (binary), clearance, progress."""
    out = {"status": "ok", "collided": 0, "min_clearance_m": math.nan,
           "near_misses": 0, "progress_x_m": math.nan,
           "mean_abs_dsteer": math.nan, "mean_abs_dthrottle": math.nan}
    diag = run_dir / "sim_diag.csv"
    try:
        d = pd.read_csv(diag)
    except Exception:
        out["status"] = "no_diag"
        return out
    if d.empty:
        out["status"] = "empty"
        return out
    out["collided"] = int(int(pd.to_numeric(d["collisions"], errors="coerce").max() or 0) > 0)
    out["near_misses"] = int(pd.to_numeric(d["near_misses"], errors="coerce").max() or 0)
    clr = pd.to_numeric(d.get("nearest_clearance_m", pd.Series(dtype=float)), errors="coerce")
    out["min_clearance_m"] = float(clr.min()) if clr.notna().any() else math.nan
    out["progress_x_m"] = float(pd.to_numeric(d["x"], errors="coerce").iloc[-1])
    # intrusiveness from the shield log (intervention magnitude)
    for name in ("cbf_filter_log.csv", "mppi_shield_log.csv", "nmpc_shield_log.csv"):
        sh = run_dir / name
        if sh.exists():
            m = parse_shield_csv(sh)
            out["mean_abs_dsteer"] = m.get("mean_abs_dsteer", math.nan)
            out["mean_abs_dthrottle"] = m.get("mean_abs_dthrottle", math.nan)
            out["intervention_rate_pct"] = m.get("intervention_rate_pct", math.nan)
            break
    return out


def _run_one(task: Task) -> dict:
    run_dir = Path(task.run_dir)
    rc, wall, _ = run_process(_build_cmd(task), run_dir, task.timeout)
    row = {"idx": task.idx, "cell": task.cell, "convoy": task.convoy,
           "filter": task.filter_name, "delay_s": task.delay,
           "rc": rc, "wall_s": round(wall, 1)}
    row.update(_metrics_from_run(run_dir))
    if rc != 0 and row["status"] == "ok":
        row["status"] = f"exit_{rc}"
    return row


def plot_figures(summary: pd.DataFrame, out_dir: Path) -> None:
    """Aggregate bars: collision rate, clearance, intrusiveness per filter."""
    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    if summary.empty:
        return
    s = summary.set_index("filter")
    order = [f for f in ("none", "dob_cbf", "mppi", "nmpc") if f in s.index]
    colors = {"none": "#b0392b", "dob_cbf": "#28c76f", "mppi": "#2d6cdf", "nmpc": "#9b59b6"}
    x = range(len(order)); cols = [colors.get(f, "#888") for f in order]
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.8))
    axes[0].bar(x, [s.loc[f, "collision_rate"] for f in order], color=cols)
    axes[0].set_ylabel("collision rate"); axes[0].set_title("Collisions (same operator intent)")
    axes[1].bar(x, [s.loc[f, "mean_clearance_m"] for f in order], color=cols)
    axes[1].axhline(0, color="r", ls=":", lw=1); axes[1].set_ylabel("mean min clearance (m)")
    axes[1].set_title("Clearance (>0 = safe)")
    dsteer = [s.loc[f, "mean_abs_dsteer"] if "mean_abs_dsteer" in s.columns and f != "none"
              else 0.0 for f in order]
    axes[2].bar(x, dsteer, color=cols)
    axes[2].set_ylabel("mean |Δsteer|"); axes[2].set_title("Filter intrusiveness")
    for ax in axes:
        ax.set_xticks(list(x)); ax.set_xticklabels(order, rotation=15); ax.grid(alpha=0.3, axis="y")
    fig.suptitle("Convoy counterfactual: replay one operator intent, filter off vs on")
    fig.tight_layout()
    fig.savefig(fig_dir / "convoy_counterfactual.png", dpi=200)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    out_dir = timestamped_result_dir("convoy_counterfactual_eval")
    write_manifest(out_dir, args, "Counterfactual safety-filter eval on the convoy scenario.")
    print(f"Output: {out_dir}")

    tasks, idx = [], 0
    if args.trace_dir:
        # Batch: each recorded round (sim_diag.csv) is one cell, replayed off vs
        # each filter at its recorded delay, all in the session's convoy scenario.
        preset0 = args.convoy[0]
        rounds = discover_round_traces(Path(args.trace_dir).expanduser().resolve())
        if not rounds:
            raise SystemExit(f"no */sim_diag.csv under {args.trace_dir}/raw/")
        print(f"Batch: {len(rounds)} recorded rounds x {len(args.filters)} filters, "
              f"convoy={preset0}")
        for trace_f, delay, name in rounds:
            for filt in args.filters:
                run_dir = out_dir / "raw" / f"{idx:03d}_{name}_{filt}"
                tasks.append(Task(idx, filt, delay, args.base_port + 2 * idx, str(run_dir),
                                  trace_f, preset0, args.terrain, args.time, args.mesh_resolution,
                                  args.safety_buffer, args.shield_horizon, args.mppi_samples,
                                  args.timeout, cell=name))
                idx += 1
    else:
        # Single trace (recorded or generated) replayed across preset x delay cells.
        if args.trace:
            trace = str(Path(args.trace).expanduser().resolve())
            print(f"Replaying recorded trace: {trace}")
        else:
            trace = str(out_dir / "reckless_trace.csv")
            generate_reckless_trace(Path(trace), args.time, args.reckless_throttle)
            print(f"Generated reckless intent (throttle={args.reckless_throttle}): {trace}")
        for preset in args.convoy:
            for filt in args.filters:
                for delay in args.delays:
                    run_dir = out_dir / "raw" / f"{idx:03d}_{preset}_{filt}_d{delay:.2f}"
                    tasks.append(Task(idx, filt, delay, args.base_port + 2 * idx, str(run_dir),
                                      trace, preset, args.terrain, args.time, args.mesh_resolution,
                                      args.safety_buffer, args.shield_horizon, args.mppi_samples,
                                      args.timeout, cell=f"{preset}@d{delay:.2f}"))
                    idx += 1

    rows = []
    # Cache prewarm: run task 0 solo (acados/CasADi codegen) then pool the rest.
    print(f"[1/{len(tasks)}] {tasks[0].filter_name} delay={tasks[0].delay:.2f} (prewarm)")
    rows.append(_run_one(tasks[0]))
    if len(tasks) > 1:
        with ProcessPoolExecutor(max_workers=max(1, args.workers)) as ex:
            futs = {ex.submit(_run_one, t): t for t in tasks[1:]}
            for fut in as_completed(futs):
                rows.append(fut.result())
                pd.DataFrame(rows).sort_values("idx").to_csv(out_dir / "results.csv", index=False)
    df = pd.DataFrame(rows).sort_values("idx").reset_index(drop=True)
    df.to_csv(out_dir / "results.csv", index=False)

    # Aggregate per filter across all (scenario, delay) cells. Each cell is one
    # replay of the same operator intent; the baseline is the filter-off run of
    # the SAME (scenario, delay), matched, so harm-prevented is causal.
    ok = df[df["status"] == "ok"].copy()
    base = ok[ok["filter"] == "none"].drop_duplicates("cell").set_index("cell")
    summary_rows = []
    for filt in args.filters:
        sub = ok[ok["filter"] == filt]
        if sub.empty:
            continue
        n = len(sub); coll = int(sub["collided"].sum())
        rec = {"filter": filt, "n_cells": n, "collisions": coll,
               "collision_rate": round(coll / n, 3),
               "mean_clearance_m": round(sub["min_clearance_m"].mean(), 3)}
        if filt != "none":
            prevented = base_coll = 0
            for _, r in sub.iterrows():
                key = r["cell"]
                if key in base.index:
                    bc = int(base.loc[key, "collided"])
                    base_coll += bc
                    if bc == 1 and r["collided"] == 0:
                        prevented += 1
            rec["baseline_collisions"] = base_coll
            rec["collisions_prevented"] = prevented
            rec["mean_abs_dsteer"] = round(sub["mean_abs_dsteer"].mean(), 3)
            rec["mean_abs_dthrottle"] = round(sub["mean_abs_dthrottle"].mean(), 3)
        summary_rows.append(rec)
    summary = pd.DataFrame(summary_rows)
    summary.to_csv(out_dir / "summary.csv", index=False)
    ok.sort_values("idx").to_csv(out_dir / "summary_per_cell.csv", index=False)
    plot_figures(summary, out_dir)
    save_summary_markdown(out_dir, "Convoy Counterfactual Safety-Filter Eval", summary, [
        f"Scenarios: {', '.join(args.convoy)} x delays {args.delays} "
        f"({len(args.convoy) * len(args.delays)} cells), terrain {args.terrain}.",
        "Identical operator intent replayed filter-off vs each filter on each "
        "(scenario, delay) cell; the sim is deterministic so differences are causal. "
        "collisions_prevented = cells where the filter-off baseline collided but the "
        "filter did not. CTE is deliberately not reported -- the operator is avoiding, "
        "not path-tracking; intrusiveness is the mean command correction.",
    ])
    print(f"\nDone: {out_dir}")
    print(summary.to_string(index=False))


if __name__ == "__main__":
    main()
