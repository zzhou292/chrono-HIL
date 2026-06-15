#!/usr/bin/env python3
"""Closed-loop sweep: rig-trained vs whole-vehicle-trained tire NN surrogate.

The active cleanup keeps one checkpoint from each training domain:

    | label        | checkpoint              | inputs | rate-aug |
    | rig_rate     | rig_rate_64_32          | 14     | True     |
    | vehicle_rate | vehicle_rate_64_32_lhs  | 14     | True     |

Older static and canonical-preset checkpoints are archived and can be restored
for historical replay, but they are not part of the default active sweep.

For every (model x terrain x path x speed x bumpiness x seed) run we log
mean speed, RMS / max crosstrack error, mean MPC solve time, and the
diag-CSV path so downstream plotting can dig into time series.
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


MODEL_SPECS = {
    # Six surrogates: three architecture/signature-matched rig-vs-vehicle pairs.
    # The static pair exposes the rig-training advantage; both rate pairs show it
    # vanishing (the "static-only effect" of Sec. III-D / tab:tires_rig_vs_vehicle).
    "rig_static":     dict(nn_model="rig_static_32_16",         generation="rig",     signature="static (32-16)"),
    "vehicle_static": dict(nn_model="vehicle_static_32_16_lhs", generation="vehicle", signature="static (32-16)"),
    "rig_rate32":     dict(nn_model="rig_rate_32_16",           generation="rig",     signature="rate (32-16)"),
    "vehicle_rate32": dict(nn_model="vehicle_rate_32_16_lhs",   generation="vehicle", signature="rate (32-16)"),
    "rig_rate":       dict(nn_model="rig_rate_64_32",           generation="rig",     signature="rate (64-32)"),
    "vehicle_rate":   dict(nn_model="vehicle_rate_64_32_lhs",   generation="vehicle", signature="rate (64-32)"),
}

DEFAULT_MODELS = ["rig_static", "vehicle_static", "rig_rate32", "vehicle_rate32",
                  "rig_rate", "vehicle_rate"]
DEFAULT_TERRAIN = ["clay", "dirt", "sand"]
DEFAULT_PATHS   = ["sinusoidal", "lane_change", "right_left"]
DEFAULT_SPEEDS  = [5.0, 7.0]
DEFAULT_BUMPS   = [0, 4]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--models",    nargs="+", default=DEFAULT_MODELS,
                   choices=list(MODEL_SPECS))
    p.add_argument("--terrains",  nargs="+", default=DEFAULT_TERRAIN)
    p.add_argument("--paths",     nargs="+", default=DEFAULT_PATHS)
    p.add_argument("--speeds",    nargs="+", type=float, default=DEFAULT_SPEEDS)
    p.add_argument("--bumpiness", nargs="+", type=int,   default=DEFAULT_BUMPS)
    p.add_argument("--seeds",     type=int, default=2)
    p.add_argument("--base-seed", type=int, default=900)
    p.add_argument("--time",      type=float, default=12.0)
    p.add_argument("--lead-in",   type=float, default=5.0)
    p.add_argument("--timeout",   type=float, default=180.0)
    p.add_argument("--base-port", type=int, default=7400)
    p.add_argument("--workers", type=int, default=6,
                   help="Parallel worker processes. Each Chrono SCM sim is "
                        "single-threaded enough that 6 fits comfortably on a "
                        "24-core box. Worker 0 always runs first solo to warm "
                        "the acados/CasADi codegen cache (~30 s) and avoid "
                        "races on shared generated files.")
    p.add_argument("--quick", action="store_true",
                   help="Tiny smoke: 1 terrain x 1 path x 1 speed x 1 bump x 1 seed per model.")
    return p.parse_args()


@dataclass(frozen=True)
class _Task:
    """Pickle-friendly description of one closed-loop run."""
    idx: int
    model_key: str
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


def _run_one(task: _Task) -> RunResult:
    """ProcessPool worker: runs one closed-loop sim and returns the RunResult.

    Imported at module top so it pickles cleanly across the pool.
    """
    # Per-worker acados build dir so workers never share a codegen cache.
    # See AGENTS.md rule 7 / parallelism docs and
    # data_collection/collect_closed_loop_data.py for the same pattern.
    os.environ.setdefault("ACADOS_UNIQUE_BUILD_DIR", "1")
    return launch_and_collect(
        experiment="rig_vs_vehicle_tire_sweep",
        variant=task.model_key,
        controller_mode="standard",
        mpc_model="nn",
        nn_model=task.nn_model,
        terrain=task.terrain, path=task.path,
        speed=task.speed, bumpiness=task.bumpiness, seed=task.seed,
        run_dir=Path(task.run_dir_str),
        sim_port=task.sim_port, ctrl_port=task.ctrl_port,
        sim_time=task.sim_time, timeout=task.timeout,
        # Tire-model comparison: hold the speed reference fixed (geometric
        # curvature) so the g-g planner's per-surrogate grip limits do not
        # confound the tracking comparison -- isolates the tracking tire model
        # (same rationale as the Sec. III tire-model sweeps).
        extra_args=["--legacy-speed-ref"],
        lead_in=task.lead_in,
    )


# ---------------------------------------------------------------- figures ---

GEN_COLOR = {"rig": "#d95f02", "vehicle": "#1f78b4"}


def _matrix_caption(args: argparse.Namespace) -> str:
    return (
        f"terrains={','.join(args.terrains)}  "
        f"paths={','.join(args.paths)}  "
        f"speeds={','.join(f'{s:g}' for s in args.speeds)} m/s  "
        f"bumpiness={','.join(str(b) for b in args.bumpiness)}  "
        f"seeds={args.seeds}  sim={args.time:g}s "
        f"(lead-in {args.lead_in:g}s)"
    )


def plot_paired_bars(ok: pd.DataFrame, out_dir: Path, caption: str) -> None:
    """One bar chart per metric. X = signature (static/rate), color = generation."""
    summary = (ok.groupby(["signature", "generation"])
                 .agg(rms_cte=("rms_cte_m", "mean"),
                      rms_cte_std=("rms_cte_m", "std"),
                      mean_speed=("mean_speed_mps", "mean"),
                      mean_speed_std=("mean_speed_mps", "std"),
                      speed_ratio=("speed_ratio", "mean"),
                      max_cte=("max_abs_cte_m", "mean"),
                      max_cte_std=("max_abs_cte_m", "std"),
                      solve_ms=("mean_solve_ms", "mean"),
                      solve_ms_std=("mean_solve_ms", "std"))
                 .reset_index())
    summary.to_csv(out_dir / "summary_paired.csv", index=False)

    specs = [
        ("rms_cte",    "rms_cte_std",    "RMS CTE (m)",          "Tracking error (lower better)"),
        ("max_cte",    "max_cte_std",    "max |CTE| (m)",        "Worst-case tracking"),
        ("mean_speed", "mean_speed_std", "mean speed (m/s)",     "Achieved speed (higher better)"),
        ("solve_ms",   "solve_ms_std",   "MPC solve time (ms)",  "Runtime"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(11, 8.0))
    _sig_order = ["static (32-16)", "rate (32-16)", "rate (64-32)"]
    _present = set(summary["signature"].unique())
    signatures = [s for s in _sig_order if s in _present] or sorted(_present)
    x = np.arange(len(signatures))
    width = 0.36
    for ax, (mean_k, std_k, ylabel, title) in zip(axes.flat, specs):
        for i, gen in enumerate(["rig", "vehicle"]):
            sub = (summary[summary["generation"] == gen]
                   .set_index("signature").reindex(signatures))
            ax.bar(x + (i - 0.5) * width, sub[mean_k].values,
                   width=width, yerr=sub[std_k].fillna(0).values, capsize=3,
                   color=GEN_COLOR[gen], label=f"{gen}-trained",
                   edgecolor="black", linewidth=0.5)
        ax.set_xticks(x)
        ax.set_xticklabels(signatures)
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.3)
        ax.legend(fontsize=9)
    fig.suptitle("Rig-trained vs whole-vehicle-trained tire NN — closed-loop sweep",
                 fontsize=12)
    fig.text(0.5, 0.005, caption, ha="center", fontsize=8, color="#444")
    fig.tight_layout(rect=(0, 0.02, 1, 0.98))
    fig.savefig(out_dir / "rig_vs_vehicle_paired_bars.png",
                dpi=200, bbox_inches="tight")
    plt.close(fig)


def plot_speed_vs_cte_scatter(ok: pd.DataFrame, out_dir: Path, caption: str) -> None:
    fig, ax = plt.subplots(figsize=(8.0, 5.4))
    markers = {"static": "o", "rate": "s"}
    for variant, sub in ok.groupby("variant"):
        gen = sub["generation"].iloc[0]
        sig = sub["signature"].iloc[0]
        ax.scatter(sub["mean_speed_mps"], sub["rms_cte_m"],
                   c=GEN_COLOR[gen], marker=markers[sig], s=55,
                   alpha=0.7, edgecolor="black", linewidth=0.4,
                   label=variant)
    ax.set_xlabel("Achieved mean speed (m/s) — higher is better")
    ax.set_ylabel("RMS crosstrack error (m) — lower is better")
    ax.set_title("Speed vs tracking trade-off — rig vs whole-vehicle surrogate")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=9, title="surrogate")
    fig.text(0.5, -0.02, caption, ha="center", fontsize=8, color="#444")
    fig.tight_layout()
    fig.savefig(out_dir / "rig_vs_vehicle_speed_cte_scatter.png",
                dpi=200, bbox_inches="tight")
    plt.close(fig)


def plot_scenario_heatmap(ok: pd.DataFrame, out_dir: Path, caption: str) -> None:
    """RMS CTE for every scenario x variant cell. Lower is better."""
    ok = ok.copy()
    ok["scenario"] = (
        ok["terrain"] + " / " + ok["path"]
        + " / v=" + ok["speed_mps"].astype(str)
        + " / b=" + ok["bumpiness"].astype(str)
    )
    pivot = ok.pivot_table(index="scenario", columns="variant",
                           values="rms_cte_m", aggfunc="mean")
    pivot = pivot.reindex(columns=DEFAULT_MODELS)
    pivot = pivot.dropna(how="any")
    if pivot.empty:
        return
    fig, ax = plt.subplots(figsize=(1.6 * len(pivot.columns) + 4,
                                    0.40 * len(pivot.index) + 2.4))
    im = ax.imshow(pivot.values, aspect="auto", cmap="viridis_r")
    ax.set_xticks(range(len(pivot.columns)))
    ax.set_xticklabels(pivot.columns, rotation=20, ha="right")
    ax.set_yticks(range(len(pivot.index)))
    ax.set_yticklabels(pivot.index, fontsize=8)
    vmin, vmax = float(np.nanmin(pivot.values)), float(np.nanmax(pivot.values))
    for i in range(pivot.shape[0]):
        for j in range(pivot.shape[1]):
            v = pivot.values[i, j]
            if math.isfinite(v):
                norm = (v - vmin) / (vmax - vmin + 1e-9)
                color = "white" if norm > 0.55 else "black"
                ax.text(j, i, f"{v:.2f}", ha="center", va="center",
                        fontsize=7, color=color)
    ax.set_title("RMS CTE (m) by scenario and tire surrogate — lower is better")
    fig.colorbar(im, ax=ax, fraction=0.035, label="RMS CTE (m)")
    fig.text(0.5, -0.01, caption, ha="center", fontsize=8, color="#444")
    fig.tight_layout()
    fig.savefig(out_dir / "rig_vs_vehicle_scenario_heatmap.png",
                dpi=200, bbox_inches="tight")
    plt.close(fig)


def plot_per_terrain_box(ok: pd.DataFrame, out_dir: Path, caption: str) -> None:
    """Box plot of RMS CTE per terrain, grouped by variant."""
    terrains = sorted(ok["terrain"].unique())
    variants = [v for v in DEFAULT_MODELS if v in ok["variant"].unique()]
    fig, axes = plt.subplots(1, len(terrains),
                             figsize=(4.0 * len(terrains), 4.6), sharey=True)
    if len(terrains) == 1:
        axes = [axes]
    for ax, terr in zip(axes, terrains):
        data, colors, labels = [], [], []
        for v in variants:
            sub = ok[(ok["terrain"] == terr) & (ok["variant"] == v)]
            data.append(sub["rms_cte_m"].dropna().values)
            colors.append(GEN_COLOR[MODEL_SPECS[v]["generation"]])
            labels.append(v)
        bp = ax.boxplot(data, patch_artist=True, widths=0.6, showfliers=True,
                        tick_labels=labels)
        for patch, c in zip(bp["boxes"], colors):
            patch.set_facecolor(c); patch.set_alpha(0.6)
        ax.set_title(f"terrain = {terr}")
        ax.tick_params(axis="x", labelrotation=20)
        ax.grid(axis="y", alpha=0.3)
    axes[0].set_ylabel("RMS CTE (m)")
    fig.suptitle("RMS CTE distribution per terrain — rig vs whole-vehicle surrogate")
    fig.text(0.5, 0.005, caption, ha="center", fontsize=8, color="#444")
    fig.tight_layout(rect=(0, 0.02, 1, 0.96))
    fig.savefig(out_dir / "rig_vs_vehicle_terrain_box.png",
                dpi=200, bbox_inches="tight")
    plt.close(fig)


def plot_figures(results_csv: Path, out_dir: Path, args: argparse.Namespace) -> None:
    df = pd.read_csv(results_csv)
    ok = df[df["status"] == "ok"].copy()
    if ok.empty:
        print("[plot] no successful runs"); return
    ok["generation"] = ok["variant"].map(lambda v: MODEL_SPECS[v]["generation"])
    ok["signature"]  = ok["variant"].map(lambda v: MODEL_SPECS[v]["signature"])
    caption = _matrix_caption(args)
    plot_paired_bars(ok, out_dir, caption)
    plot_speed_vs_cte_scatter(ok, out_dir, caption)
    plot_scenario_heatmap(ok, out_dir, caption)
    plot_per_terrain_box(ok, out_dir, caption)


# --------------------------------------------------------------------- main -

def main() -> None:
    args = parse_args()
    if args.quick:
        args.models    = list(MODEL_SPECS)
        args.terrains  = ["clay"]
        args.paths     = ["lane_change"]
        args.speeds    = [5.0]
        args.bumpiness = [0]
        args.seeds     = 1
        args.time      = min(args.time, 6.0)

    out_dir = timestamped_result_dir("rig_vs_vehicle_tire_sweep")
    write_manifest(out_dir, args,
                   "Rig-trained vs whole-vehicle-trained tire NN surrogate. "
                   "Standard NMPC, sensor noise on by default.")
    print(f"Output: {out_dir}")

    # Build the full task list deterministically; ports stride by 2*idx so
    # parallel workers cannot collide on the ZMQ pub/sub pair.
    tasks: list[_Task] = []
    for model_key in args.models:
        spec = MODEL_SPECS[model_key]
        for terrain in args.terrains:
            for path in args.paths:
                for speed in args.speeds:
                    for bump in args.bumpiness:
                        for seed_i in range(args.seeds):
                            seed = args.base_seed + seed_i
                            idx = len(tasks)
                            sim_port = args.base_port + 2 * idx
                            run_dir = out_dir / "raw" / (
                                f"{idx:04d}_{model_key}_{terrain}_{path}_"
                                f"v{speed:g}_b{bump}_s{seed}"
                            )
                            tasks.append(_Task(
                                idx=idx, model_key=model_key,
                                nn_model=spec["nn_model"],
                                terrain=terrain, path=path,
                                speed=speed, bumpiness=bump, seed=seed,
                                run_dir_str=str(run_dir),
                                sim_port=sim_port, ctrl_port=sim_port + 1,
                                sim_time=args.time, timeout=args.timeout,
                                lead_in=args.lead_in,
                            ))
    total = len(tasks)
    workers = max(1, min(args.workers, total))
    print(f"[rig_vs_vehicle] {total} runs across {workers} workers "
          f"({len(args.models)} models)")

    def _log(res: RunResult, done: int) -> None:
        print(f"  [{done:4d}/{total}] {res.status:>8} {res.variant:>14} "
              f"{res.terrain}/{res.path} v={res.speed_mps:g} b={res.bumpiness} "
              f"s={res.seed}  rms_cte={res.rms_cte_m:.3f} "
              f"u_bar={res.mean_speed_mps:.2f} ratio={res.speed_ratio:.2f}",
              flush=True)

    results: list[RunResult] = []
    t0 = time.time()

    # Cache prewarm — per NN model, not just task 0. acados / CasADi write
    # into a shared codegen cache keyed on the NN model. If several workers
    # hit cold codegen for the SAME model at once, `make` clobbers and the
    # solver `.so` never gets built (this was the actual cause of the 15
    # dirt/sinusoidal failures in the first parallel run: 6 workers all
    # started building closed_loop_v1_mlp_32_16 simultaneously at the model
    # boundary). The fix: warm one task per unique nn_model serially before
    # the pool starts so each model's cache is populated; subsequent workers
    # then reuse the warm cache instead of racing on a cold build.
    remaining = list(tasks)
    if workers > 1 and tasks:
        seen_models: set[str] = set()
        warm_tasks: list[_Task] = []
        for t in tasks:
            if t.nn_model not in seen_models:
                seen_models.add(t.nn_model)
                warm_tasks.append(t)
        print(f"[rig_vs_vehicle] warming acados codegen cache for "
              f"{len(warm_tasks)} unique NN models (serial prewarm) ...",
              flush=True)
        for wt in warm_tasks:
            res = _run_one(wt)
            results.append(res)
            write_results_csv(out_dir / "results.csv", results)
            _log(res, len(results))
        warm_idx = {t.idx for t in warm_tasks}
        remaining = [t for t in tasks if t.idx not in warm_idx]

    with ProcessPoolExecutor(max_workers=workers) as ex:
        futs = {ex.submit(_run_one, t): t for t in remaining}
        for fut in as_completed(futs):
            t = futs[fut]
            try:
                res = fut.result()
            except Exception as exc:  # pragma: no cover  -- defensive
                res = RunResult(
                    experiment="rig_vs_vehicle_tire_sweep",
                    variant=t.model_key, controller_mode="standard",
                    mpc_model="nn", nn_model=t.nn_model,
                    terrain=t.terrain, path=t.path, speed_mps=t.speed,
                    bumpiness=t.bumpiness, seed=t.seed,
                    run_dir=t.run_dir_str, rc=-1, wall_s=0.0,
                    status=f"worker_exc:{exc!r}",
                )
            results.append(res)
            write_results_csv(out_dir / "results.csv", results)
            _log(res, len(results))

    wall = time.time() - t0
    print(f"[rig_vs_vehicle] {total} runs done in {wall/60:.1f} min "
          f"(avg {wall/max(total,1):.1f} s/run wall)")

    write_results_csv(out_dir / "results.csv", results)
    summary = summarize_by_variant(
        results,
        ["rms_cte_m", "max_abs_cte_m", "mean_abs_cte_m",
         "mean_speed_mps", "speed_ratio", "mean_solve_ms", "progress_m"],
    )
    summary.to_csv(out_dir / "summary_by_model.csv", index=False)
    save_summary_markdown(
        out_dir,
        "Rig-trained vs whole-vehicle-trained tire NN — closed-loop sweep",
        summary,
        ["Active comparison: rig_rate vs vehicle_rate (14 inputs, rate-augmented).",
         "Standard NMPC, sensor noise enabled by launch_decoupled default.",
         "Each row averages over all scenarios in the matrix."],
    )
    plot_figures(out_dir / "results.csv", out_dir, args)
    print(f"Done: {out_dir}")


if __name__ == "__main__":
    main()
