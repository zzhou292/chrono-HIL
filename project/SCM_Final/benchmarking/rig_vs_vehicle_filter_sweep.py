#!/usr/bin/env python3
"""Rig-trained vs whole-vehicle-trained tire NN x safety-filter sweep.

Pairs the closed-loop tire-model story (rig_rate vs vehicle_rate) with the
four shipped safety-filter flavors. Each cell answers: does swapping the
underlying tire surrogate change the safety filter's collision-avoidance
behavior?

    surrogate axis :  rig_rate (rig_rate_64_32)
                      vehicle_rate (vehicle_rate_64_32_lhs)
    filter axis    :  none, dob_cbf

Configurations follow `safety_filter_sweep.py` so the metric set
(collisions, near-misses, min clearance, intervention rate, dsteer, RMS
CTE) is directly comparable to the main paper safety table. Sensor noise
is on; rocks are placed along the path so the filter has something to do.

Parallelism: per-model serial prewarm + ProcessPool (workers=6 default),
following AGENTS.md.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (  # noqa: E402
    RunResult,
    launch_and_collect,
    save_summary_markdown,
    summarize_by_variant,
    timestamped_result_dir,
    write_manifest,
    write_results_csv,
)


SURROGATES = {
    "rig":             "rig_rate_64_32",
    "vehicle":         "vehicle_rate_64_32_lhs",
}
FLAVORS = ("none", "dob_cbf")
GEN_COLOR = {"rig": "#d95f02", "vehicle": "#1f78b4"}


def extra_args_for(flavor: str, args: argparse.Namespace) -> list[str]:
    """Mirror the dispatch from safety_filter_sweep.py for parity."""
    extra: list[str] = []
    if flavor != "none":
        extra += [
            "--safety-filter", "--safety-flavor", flavor,
            "--shield-horizon", str(args.shield_horizon),
            "--safety-buffer", str(args.safety_buffer),
        ]
    return extra


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--surrogates", nargs="+", default=["rig", "vehicle"],
                   choices=list(SURROGATES))
    p.add_argument("--flavors", nargs="+", default=list(FLAVORS),
                   choices=list(FLAVORS))
    p.add_argument("--terrains", nargs="+", default=["clay", "sand"])
    p.add_argument("--paths", nargs="+", default=["sinusoidal", "right_left"])
    p.add_argument("--speeds", nargs="+", type=float, default=[5.0, 7.0])
    p.add_argument("--bumpiness", nargs="+", type=int, default=[0])
    p.add_argument("--seeds", type=int, default=2)
    p.add_argument("--base-seed", type=int, default=800)
    p.add_argument("--rocks", type=int, default=5)
    p.add_argument("--shield-horizon", type=int, default=18)
    p.add_argument("--safety-buffer", type=float, default=0.5)
    p.add_argument("--time", type=float, default=12.0)
    p.add_argument("--lead-in", type=float, default=5.0)
    p.add_argument("--timeout", type=float, default=180.0)
    p.add_argument("--workers", type=int, default=6)
    p.add_argument("--base-port", type=int, default=9000)
    p.add_argument("--quick", action="store_true",
                   help="Tiny smoke: 1 terrain x 1 path x 1 speed x 1 seed.")
    return p.parse_args()


@dataclass(frozen=True)
class _Task:
    idx: int
    surrogate: str       # "rig" / "vehicle"
    flavor: str
    nn_model: str
    terrain: str
    path: str
    speed: float
    bumpiness: int
    seed: int
    run_dir_str: str
    sim_port: int
    ctrl_port: int
    sim_time: float
    timeout: float
    lead_in: float
    rocks: int
    extra_args: tuple  # tuple-of-str (pickle-friendly, immutable)


def _run_one(task: _Task) -> RunResult:
    # ACADOS_UNIQUE_BUILD_DIR=1 makes acados_mpc_solver.py write into
    # /tmp/acados_mpc_<model>_<fp_hash>_<PID>/ -- per-worker build dirs so
    # parallel workers never share a codegen cache. Without this, even
    # per-model prewarm has windows where one worker invalidates the cache
    # while another is reading it; we hit 69/128 failures the first time.
    # The pattern is borrowed from data_collection/collect_closed_loop_data.py.
    os.environ.setdefault("ACADOS_UNIQUE_BUILD_DIR", "1")
    return launch_and_collect(
        experiment="rig_vs_vehicle_filter_sweep",
        variant=f"{task.surrogate}_{task.flavor}",
        controller_mode="standard",
        mpc_model="nn",
        nn_model=task.nn_model,
        terrain=task.terrain, path=task.path,
        speed=task.speed, bumpiness=task.bumpiness, seed=task.seed,
        run_dir=Path(task.run_dir_str),
        sim_port=task.sim_port, ctrl_port=task.ctrl_port,
        sim_time=task.sim_time, timeout=task.timeout,
        rocks=task.rocks, lead_in=task.lead_in,
        extra_args=list(task.extra_args),
    )


# ---------------------------------------------------------------- figures ---

def _caption(args: argparse.Namespace) -> str:
    return (f"surrogates={','.join(args.surrogates)}  "
            f"flavors={','.join(args.flavors)}  "
            f"terrains={','.join(args.terrains)}  "
            f"paths={','.join(args.paths)}  "
            f"speeds={','.join(f'{s:g}' for s in args.speeds)}m/s  "
            f"rocks={args.rocks}  seeds={args.seeds}  "
            f"sim={args.time:g}s (lead-in {args.lead_in:g}s)")


def plot_safety_bars(ok: pd.DataFrame, out_dir: Path, caption: str) -> None:
    """One panel per metric; x = filter flavor; color = generation."""
    g = (ok.groupby(["flavor", "surrogate"])
           .agg(collisions=("collisions", "mean"),
                near=("near_misses", "mean"),
                clearance=("min_clearance_m", "mean"),
                intervention=("intervention_rate_pct", "mean"),
                rms_cte=("rms_cte_m", "mean"),
                dsteer=("mean_abs_dsteer", "mean"))
           .reset_index())
    g.to_csv(out_dir / "summary_by_flavor.csv", index=False)

    flavors = list(FLAVORS)
    specs = [
        ("collisions",   "collisions (mean count, lower better)",      "Collisions"),
        ("near",         "near-misses (mean count)",                   "Near misses"),
        ("clearance",    "min clearance (m, higher better)",           "Min clearance"),
        ("intervention", "intervention rate (%, lower better)",        "Intervention rate"),
        ("rms_cte",      "RMS CTE (m, lower better)",                  "Tracking error"),
        ("dsteer",       "mean |dsteer/dt|  (lower = smoother)",        "Steering smoothness"),
    ]
    fig, axes = plt.subplots(2, 3, figsize=(13.5, 7.5))
    x = np.arange(len(flavors))
    width = 0.36
    for ax, (key, ylabel, title) in zip(axes.flat, specs):
        for i, surr in enumerate(["rig", "vehicle"]):
            sub = g[g["surrogate"] == surr].set_index("flavor").reindex(flavors)
            ax.bar(x + (i - 0.5) * width, sub[key].fillna(0).values,
                   width=width, color=GEN_COLOR[surr],
                   edgecolor="black", linewidth=0.4,
                   label=f"{surr}-trained")
        ax.set_xticks(x); ax.set_xticklabels(flavors)
        ax.set_ylabel(ylabel); ax.set_title(title)
        ax.grid(axis="y", alpha=0.3)
        if key == "intervention" or key == "collisions" or key == "near":
            ax.set_ylim(bottom=0)
        ax.legend(fontsize=8)
    fig.suptitle("Rig-trained vs whole-vehicle-trained tire NN x safety filter")
    fig.text(0.5, 0.005, caption, ha="center", fontsize=8, color="#444")
    fig.tight_layout(rect=(0, 0.02, 1, 0.97))
    fig.savefig(out_dir / "rig_vs_vehicle_filter_bars.png",
                dpi=200, bbox_inches="tight")
    plt.close(fig)


def plot_collision_heatmap(ok: pd.DataFrame, out_dir: Path, caption: str) -> None:
    """Heatmap of collisions per (surrogate, flavor) x (terrain, path, speed)."""
    ok = ok.copy()
    ok["scenario"] = (ok["terrain"] + " / " + ok["path"]
                      + " / v=" + ok["speed_mps"].astype(str))
    ok["variant"]  = ok["surrogate"] + " / " + ok["flavor"]
    pivot = (ok.pivot_table(index="scenario", columns="variant",
                            values="collisions", aggfunc="sum")
               .reindex(columns=sorted(ok["variant"].unique()))
               .fillna(0))
    if pivot.empty: return
    fig, ax = plt.subplots(figsize=(1.0 * len(pivot.columns) + 3,
                                    0.45 * len(pivot.index) + 2.5))
    im = ax.imshow(pivot.values, aspect="auto", cmap="Reds", vmin=0)
    ax.set_xticks(range(len(pivot.columns)))
    ax.set_xticklabels(pivot.columns, rotation=30, ha="right")
    ax.set_yticks(range(len(pivot.index)))
    ax.set_yticklabels(pivot.index, fontsize=8)
    for i in range(pivot.shape[0]):
        for j in range(pivot.shape[1]):
            v = pivot.values[i, j]
            color = "white" if v > pivot.values.max() * 0.55 else "black"
            ax.text(j, i, f"{int(v)}", ha="center", va="center",
                    fontsize=7, color=color)
    ax.set_title("Total collisions per scenario x (surrogate / filter)")
    fig.colorbar(im, ax=ax, fraction=0.035, label="collisions")
    fig.text(0.5, -0.01, caption, ha="center", fontsize=8, color="#444")
    fig.tight_layout()
    fig.savefig(out_dir / "rig_vs_vehicle_filter_collisions_heatmap.png",
                dpi=200, bbox_inches="tight")
    plt.close(fig)


def plot_figures(results_csv: Path, out_dir: Path,
                 args: argparse.Namespace) -> None:
    df = pd.read_csv(results_csv)
    ok = df[df["status"] == "ok"].copy()
    if ok.empty:
        print("[plot] no ok runs"); return
    # Recover surrogate / flavor from variant.
    ok["surrogate"] = ok["variant"].str.split("_", n=1).str[0]
    ok["flavor"]    = ok["variant"].str.split("_", n=1).str[1]
    caption = _caption(args)
    plot_safety_bars(ok, out_dir, caption)
    plot_collision_heatmap(ok, out_dir, caption)


# --------------------------------------------------------------------- main -

def main() -> None:
    args = parse_args()
    if args.quick:
        args.surrogates = list(SURROGATES)
        args.flavors    = list(FLAVORS)
        args.terrains   = ["clay"]
        args.paths      = ["sinusoidal"]
        args.speeds     = [5.0]
        args.bumpiness  = [0]
        args.seeds      = 1
        args.time       = min(args.time, 6.0)

    out_dir = timestamped_result_dir("rig_vs_vehicle_filter_sweep")
    write_manifest(out_dir, args,
                   "Rig vs whole-vehicle tire NN paired with safety-filter "
                   "flavors {none, dob_cbf}. Sensor noise on. "
                   "Per-model acados codegen prewarm before parallel pool.")
    print(f"Output: {out_dir}")

    tasks: list[_Task] = []
    for surr in args.surrogates:
        nn_model = SURROGATES[surr]
        for flavor in args.flavors:
            extra = tuple(extra_args_for(flavor, args))
            for terrain in args.terrains:
                for path in args.paths:
                    for speed in args.speeds:
                        for bump in args.bumpiness:
                            for seed_i in range(args.seeds):
                                seed = args.base_seed + seed_i
                                idx = len(tasks)
                                sim_port = args.base_port + 2 * idx
                                run_dir = out_dir / "raw" / (
                                    f"{idx:04d}_{surr}_{flavor}_{terrain}_"
                                    f"{path}_v{speed:g}_b{bump}_s{seed}"
                                )
                                tasks.append(_Task(
                                    idx=idx, surrogate=surr, flavor=flavor,
                                    nn_model=nn_model,
                                    terrain=terrain, path=path,
                                    speed=speed, bumpiness=bump, seed=seed,
                                    run_dir_str=str(run_dir),
                                    sim_port=sim_port, ctrl_port=sim_port + 1,
                                    sim_time=args.time, timeout=args.timeout,
                                    lead_in=args.lead_in, rocks=args.rocks,
                                    extra_args=extra,
                                ))
    total = len(tasks)
    workers = max(1, min(args.workers, total))
    print(f"[rig_vs_vehicle_filter] {total} runs across {workers} workers "
          f"({len(args.surrogates)} surrogates x {len(args.flavors)} filters)",
          flush=True)

    def _log(res: RunResult, done: int) -> None:
        print(f"  [{done:4d}/{total}] {res.status:>8} {res.variant:>20} "
              f"{res.terrain}/{res.path} v={res.speed_mps:g} s={res.seed}  "
              f"col={res.collisions} near={res.near_misses} "
              f"clear={res.min_clearance_m:.2f} "
              f"interv={res.intervention_rate_pct:.1f}%  "
              f"rms_cte={res.rms_cte_m:.3f}", flush=True)

    results: list[RunResult] = []
    t0 = time.time()

    # Per-model serial prewarm AND per-flavor prewarm (some flavors trigger
    # extra acados/MPPI codegen on first build). Warm one combination per
    # unique (nn_model, flavor) before parallel dispatch.
    remaining = list(tasks)
    if workers > 1 and tasks:
        seen: set[tuple[str, str]] = set()
        warm: list[_Task] = []
        for t in tasks:
            key = (t.nn_model, t.flavor)
            if key not in seen:
                seen.add(key)
                warm.append(t)
        print(f"[rig_vs_vehicle_filter] prewarming {len(warm)} unique "
              f"(nn_model x flavor) combos serially ...", flush=True)
        for wt in warm:
            res = _run_one(wt)
            results.append(res)
            write_results_csv(out_dir / "results.csv", results)
            _log(res, len(results))
        warm_idx = {t.idx for t in warm}
        remaining = [t for t in tasks if t.idx not in warm_idx]

    with ProcessPoolExecutor(max_workers=workers) as ex:
        futs = {ex.submit(_run_one, t): t for t in remaining}
        for fut in as_completed(futs):
            t = futs[fut]
            try:
                res = fut.result()
            except Exception as exc:  # pragma: no cover
                res = RunResult(
                    experiment="rig_vs_vehicle_filter_sweep",
                    variant=f"{t.surrogate}_{t.flavor}",
                    controller_mode="standard", mpc_model="nn",
                    nn_model=t.nn_model,
                    terrain=t.terrain, path=t.path, speed_mps=t.speed,
                    bumpiness=t.bumpiness, seed=t.seed,
                    run_dir=t.run_dir_str, rc=-1, wall_s=0.0,
                    status=f"worker_exc:{exc!r}",
                )
            results.append(res)
            write_results_csv(out_dir / "results.csv", results)
            _log(res, len(results))

    wall = time.time() - t0
    print(f"[rig_vs_vehicle_filter] {total} runs done in {wall/60:.1f} min "
          f"(avg {wall/max(total,1):.1f} s/run wall)")

    summary = summarize_by_variant(
        results,
        ["collisions", "near_misses", "min_clearance_m",
         "intervention_rate_pct", "mean_abs_dsteer", "rms_cte_m",
         "mean_speed_mps", "mean_solve_ms"],
    )
    summary.to_csv(out_dir / "summary_by_variant.csv", index=False)
    save_summary_markdown(
        out_dir,
        "Rig vs whole-vehicle tire NN x safety filter sweep",
        summary,
        ["surrogate axis: rig_rate_64_32 vs vehicle_rate_64_32_lhs",
         "filter axis: none / dob_cbf",
         "Standard NMPC planner; safety filter intervenes per its own logic",
         "Sensor noise on; rocks placed along the path"],
    )
    plot_figures(out_dir / "results.csv", out_dir, args)
    print(f"Done: {out_dir}")


if __name__ == "__main__":
    main()
