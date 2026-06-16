#!/usr/bin/env python3
"""Smallest real-time-safe MPPI sample count K.

The scaling study (safety_filter_scaling.py) showed MPPI's solve time is set by
its sample count K -- linear, with the deployed K=384 sitting right at the
10 Hz / 100 ms budget. This asks the companion question a reviewer would: how
low can K go before safety degrades? It replays a fixed aggressive operator
intent through the convoy scenarios at each K and reports the collision rate +
clearance, alongside the per-K solve time, to identify the K that is both
collision-free AND real-time (the justified operating point).

Standalone safety runs via the counterfactual replay machinery; solve time via
the scaling-harness timing. No human in the loop.

  python mppi_k_safety.py
  python mppi_k_safety.py --k-values 64 128 256 384 512 --convoy lead_brake cut_in stalled
"""
from __future__ import annotations

import os
for _v in ("OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS", "OMP_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import argparse
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "simulation"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from convoy_counterfactual_eval import Task, _run_one, generate_reckless_trace  # noqa: E402
from safety_filter_scaling import build_filter, time_calls, make_obstacles, BUDGET_MS  # noqa: E402
from param_consistency import (  # noqa: E402
    get_vehicle_params_for_demo, get_terrain_preset, terrain_preset_to_internal)
from nn_tire_model import load_nn_tire_model  # noqa: E402
from common import timestamped_result_dir, write_manifest, save_summary_markdown  # noqa: E402


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--k-values", nargs="+", type=int, default=[64, 128, 256, 384, 512])
    p.add_argument("--convoy", nargs="+", default=["lead_brake", "cut_in", "stalled"])
    p.add_argument("--delays", nargs="+", type=float, default=[0.0, 0.30])
    p.add_argument("--reckless-throttle", type=float, default=0.6)
    p.add_argument("--terrain", default="clay")
    p.add_argument("--time", type=float, default=18.0)
    p.add_argument("--horizon", type=int, default=12)
    p.add_argument("--mesh-resolution", type=float, default=0.12)
    p.add_argument("--workers", type=int, default=6)
    p.add_argument("--timeout", type=float, default=400.0)
    p.add_argument("--base-port", type=int, default=11600)
    p.add_argument("--reps", type=int, default=40, help="solve-time timing reps per K")
    return p.parse_args()


def measure_solve_times(k_values, horizon, terrain, reps):
    """Standalone p95 solve time of MPPI at each K (N=8 obstacles)."""
    vp = get_vehicle_params_for_demo()
    tp = terrain_preset_to_internal(get_terrain_preset(terrain))
    nn = load_nn_tire_model(str(ROOT / "nn_models" / "vehicle_rate_64_32_lhs"), tp)
    rng = np.random.default_rng(0)
    obs = make_obstacles(8, rng)
    out = {}
    for k in k_values:
        f = build_filter("mppi", nn, vp, tp, horizon, k)
        _, p95, _ = time_calls(f, obs, reps)
        out[k] = round(p95, 1)
    return out


def main() -> None:
    args = parse_args()
    out_dir = timestamped_result_dir("mppi_k_safety")
    write_manifest(out_dir, args, "Smallest real-time-safe MPPI sample count.")
    print(f"Output: {out_dir}")

    trace = str(out_dir / "reckless_trace.csv")
    generate_reckless_trace(Path(trace), args.time, args.reckless_throttle)

    # Build one MPPI run per (K, scenario, delay).
    tasks, meta, idx = [], {}, 0
    for k in args.k_values:
        for scen in args.convoy:
            for delay in args.delays:
                run_dir = out_dir / "raw" / f"{idx:03d}_K{k}_{scen}_d{delay:.2f}"
                tasks.append(Task(idx, "mppi", delay, args.base_port + 2 * idx, str(run_dir),
                                  trace, scen, args.terrain, args.time, args.mesh_resolution,
                                  0.25, args.horizon, k, args.timeout, cell=f"{scen}@{delay}"))
                meta[idx] = {"K": k, "convoy": scen, "delay_s": delay}
                idx += 1

    rows = []
    print(f"[1/{len(tasks)}] K={tasks[0].mppi_samples} (prewarm)")
    rows.append(_run_one(tasks[0]))
    if len(tasks) > 1:
        with ProcessPoolExecutor(max_workers=max(1, args.workers)) as ex:
            futs = {ex.submit(_run_one, t): t for t in tasks[1:]}
            for fut in as_completed(futs):
                rows.append(fut.result())
    df = pd.DataFrame(rows)
    df["K"] = df["idx"].map(lambda i: meta[i]["K"])
    df = df.sort_values(["K", "idx"]).reset_index(drop=True)
    df.to_csv(out_dir / "results.csv", index=False)

    solve = measure_solve_times(args.k_values, args.horizon, args.terrain, args.reps)

    # Aggregate safety per K.
    srows = []
    ok = df[df["status"] == "ok"]
    for k in args.k_values:
        sub = ok[ok["K"] == k]
        n = len(sub); coll = int(sub["collided"].sum())
        srows.append({"K": k, "n_cells": n, "collisions": coll,
                      "collision_rate": round(coll / n, 3) if n else float("nan"),
                      "mean_clearance_m": round(sub["min_clearance_m"].mean(), 3) if n else float("nan"),
                      "p95_solve_ms": solve.get(k, float("nan")),
                      "real_time": int(solve.get(k, 1e9) <= BUDGET_MS)})
    summary = pd.DataFrame(srows)
    summary.to_csv(out_dir / "summary.csv", index=False)
    plot_figures(summary, out_dir)

    safe = summary[summary["collision_rate"] == 0.0]
    safe_rt = safe[safe["real_time"] == 1]
    pick = int(safe_rt["K"].min()) if not safe_rt.empty else None
    save_summary_markdown(out_dir, "Smallest Real-Time-Safe MPPI K", summary, [
        f"Scenarios: {', '.join(args.convoy)} x delays {args.delays}; aggressive "
        "intent replayed at each K. Budget {:.0f} ms (10 Hz).".format(BUDGET_MS),
        f"Smallest collision-free AND real-time K: {pick}." if pick is not None
        else "No tested K is both collision-free and real-time.",
    ])
    print(f"\nDone: {out_dir}")
    print(summary.to_string(index=False))
    if pick is not None:
        print(f"\n>> Smallest collision-free + real-time K = {pick} "
              f"(p95 {solve.get(pick)} ms); deployed K=384.")


def plot_figures(summary: pd.DataFrame, out_dir: Path) -> None:
    fig_dir = out_dir / "figures"; fig_dir.mkdir(parents=True, exist_ok=True)
    s = summary.sort_values("K")
    fig, ax1 = plt.subplots(figsize=(7, 4.3))
    ax1.plot(s["K"], s["collision_rate"], marker="o", color="#b0392b", label="collision rate")
    ax1.set_xlabel("MPPI samples K"); ax1.set_ylabel("collision rate", color="#b0392b")
    ax1.set_ylim(-0.05, 1.05); ax1.tick_params(axis="y", labelcolor="#b0392b")
    ax2 = ax1.twinx()
    ax2.plot(s["K"], s["p95_solve_ms"], marker="s", color="#2d6cdf", label="p95 solve (ms)")
    ax2.axhline(BUDGET_MS, color="#2d6cdf", ls="--", lw=1, label=f"{BUDGET_MS:.0f} ms budget")
    ax2.set_ylabel("p95 solve time (ms)", color="#2d6cdf"); ax2.tick_params(axis="y", labelcolor="#2d6cdf")
    ax1.axvline(384, color="gray", ls=":", lw=1)
    ax1.text(384, 0.5, " deployed K=384", color="gray", fontsize=8, rotation=90, va="center")
    ax1.set_title("MPPI: safety vs. real-time cost across sample count K")
    fig.tight_layout()
    fig.savefig(fig_dir / "mppi_k_safety.png", dpi=200)
    plt.close(fig)


if __name__ == "__main__":
    main()
