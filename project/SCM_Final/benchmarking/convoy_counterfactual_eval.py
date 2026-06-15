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
    p.add_argument("--reckless-throttle", type=float, default=0.6,
                   help="Throttle of the generated reckless intent (when no --trace).")
    p.add_argument("--convoy", default="lead_brake",
                   help="Convoy preset (lead_brake/cut_in/stalled/convoy/jam/gauntlet/...).")
    p.add_argument("--filters", nargs="+", default=["none", "dob_cbf", "mppi"],
                   choices=["none", "dob_cbf", "mppi", "nmpc"])
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
    row = {"idx": task.idx, "filter": task.filter_name, "delay_s": task.delay,
           "rc": rc, "wall_s": round(wall, 1)}
    row.update(_metrics_from_run(run_dir))
    if rc != 0 and row["status"] == "ok":
        row["status"] = f"exit_{rc}"
    return row


def plot_figures(df: pd.DataFrame, out_dir: Path) -> None:
    fig_dir = out_dir / "figures"
    fig_dir.mkdir(parents=True, exist_ok=True)
    ok = df[df["status"] == "ok"].copy()
    if ok.empty:
        return
    fig, axes = plt.subplots(1, 3, figsize=(13, 4))
    for f in ok["filter"].unique():
        s = ok[ok["filter"] == f].sort_values("delay_s")
        axes[0].plot(s["delay_s"], s["collided"], marker="o", label=f)
        axes[1].plot(s["delay_s"], s["min_clearance_m"], marker="o", label=f)
        axes[2].plot(s["delay_s"], s["mean_abs_dsteer"], marker="o", label=f)
    axes[0].set_ylabel("collided (1=yes)"); axes[0].set_title("Collision (same intent)")
    axes[1].axhline(0, color="r", ls=":", lw=1); axes[1].set_ylabel("min clearance (m)")
    axes[1].set_title("Clearance (>0 = safe)")
    axes[2].set_ylabel("mean |Δsteer| (intervention)"); axes[2].set_title("Filter intrusiveness")
    for ax in axes:
        ax.set_xlabel("command delay (s)"); ax.legend(); ax.grid(alpha=0.3)
    fig.suptitle("Convoy counterfactual: identical operator intent, filter off vs on")
    fig.tight_layout()
    fig.savefig(fig_dir / "convoy_counterfactual.png", dpi=200)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    out_dir = timestamped_result_dir("convoy_counterfactual_eval")
    write_manifest(out_dir, args, "Counterfactual safety-filter eval on the convoy scenario.")
    print(f"Output: {out_dir}")

    # Resolve the operator intent trace (recorded or generated).
    if args.trace:
        trace = str(Path(args.trace).expanduser().resolve())
        print(f"Replaying recorded trace: {trace}")
    else:
        trace = str(out_dir / "reckless_trace.csv")
        generate_reckless_trace(Path(trace), args.time, args.reckless_throttle)
        print(f"Generated reckless intent (throttle={args.reckless_throttle}): {trace}")

    tasks, idx = [], 0
    for filt in args.filters:
        for delay in args.delays:
            run_dir = out_dir / "raw" / f"{idx:03d}_{filt}_delay{delay:.2f}"
            tasks.append(Task(idx, filt, delay, args.base_port + 2 * idx, str(run_dir),
                              trace, args.convoy, args.terrain, args.time, args.mesh_resolution,
                              args.safety_buffer, args.shield_horizon, args.mppi_samples, args.timeout))
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

    # Counterfactual harm-prevented vs the filter-OFF baseline at each delay.
    summary_rows = []
    for delay in sorted(df["delay_s"].unique()):
        sub = df[(df["delay_s"] == delay) & (df["status"] == "ok")]
        base = sub[sub["filter"] == "none"]
        base_coll = int(base["collided"].iloc[0]) if not base.empty else None
        base_clr = float(base["min_clearance_m"].iloc[0]) if not base.empty else math.nan
        for _, r in sub.iterrows():
            summary_rows.append({
                "filter": r["filter"], "delay_s": delay,
                "collided": int(r["collided"]),
                "collision_prevented": (1 if (base_coll == 1 and r["collided"] == 0) else 0)
                                       if (base_coll is not None and r["filter"] != "none") else "",
                "min_clearance_m": round(r["min_clearance_m"], 3),
                "clearance_gain_m": round(r["min_clearance_m"] - base_clr, 3)
                                    if (r["filter"] != "none" and math.isfinite(base_clr)) else "",
                "mean_abs_dsteer": round(r.get("mean_abs_dsteer", math.nan), 3),
                "mean_abs_dthrottle": round(r.get("mean_abs_dthrottle", math.nan), 3),
                "progress_x_m": round(r["progress_x_m"], 1),
            })
    summary = pd.DataFrame(summary_rows)
    summary.to_csv(out_dir / "summary.csv", index=False)
    plot_figures(df, out_dir)
    save_summary_markdown(out_dir, "Convoy Counterfactual Safety-Filter Eval", summary, [
        f"Scenario: --convoy {args.convoy}, terrain {args.terrain}, {args.time:.0f}s.",
        "Identical operator intent replayed filter-off vs each filter (deterministic "
        "sim), so differences are causal. 'collision_prevented' = baseline (none) "
        "collided but this filter did not. CTE is deliberately not reported -- the "
        "operator is avoiding, not path-tracking.",
    ])
    print(f"\nDone: {out_dir}")
    print(summary.to_string(index=False))


if __name__ == "__main__":
    main()
