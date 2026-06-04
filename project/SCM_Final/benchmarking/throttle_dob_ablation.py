#!/usr/bin/env python3
"""Paper experiment: asymmetric throttle DOB on vs off.

One thing tested: whether the asymmetric throttle DOB on the longitudinal
actuation map closes the residual soft-soil speed gap left by the open-loop
``udot = ax`` integrator.  Sensor noise is enabled in every run.

Disabling is done by clamping the DOB gains to zero
(``--dob-ki 0 --dob-max 0``) so the controller still constructs the DOB hook
but adds no integral action.  This keeps the comparison apples-to-apples
without code paths only enabled by an ``--no-dob`` flag.
"""

from __future__ import annotations

import argparse
import os
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, field
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (  # noqa: E402
    BUMPS,
    DEFAULT_NN_MODEL,
    PATHS,
    SPEEDS,
    TERRAINS,
    RunResult,
    launch_and_collect,
    plot_metric_distribution_grid,
    plot_trajectory_overlays,
    save_summary_markdown,
    summarize_by_variant,
    timestamped_result_dir,
    write_manifest,
    write_results_csv,
)


VARIANTS = {
    "dob_on": dict(
        extra=[],
        note="Standard MPC with default asymmetric throttle DOB (ki=0.15, max=0.35).",
    ),
    "dob_off": dict(
        extra=["--dob-ki", "0.0", "--dob-max", "0.0"],
        note="Standard MPC with DOB gains zeroed; open-loop u_dot = a_x only.",
    ),
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--variants", nargs="+", default=list(VARIANTS),
                   choices=list(VARIANTS))
    p.add_argument("--terrains", nargs="+", default=list(TERRAINS), choices=list(TERRAINS))
    p.add_argument("--paths", nargs="+", default=list(PATHS))
    p.add_argument("--speeds", nargs="+", type=float, default=list(SPEEDS))
    p.add_argument("--bumpiness", nargs="+", type=int, default=list(BUMPS))
    p.add_argument("--seeds", type=int, default=5)
    p.add_argument("--base-seed", type=int, default=600)
    p.add_argument("--time", type=float, default=15.0)
    p.add_argument("--lead-in", type=float, default=5.0)
    p.add_argument("--timeout", type=float, default=180.0)
    p.add_argument("--base-port", type=int, default=8000)
    p.add_argument("--workers", type=int, default=6,
                   help="Parallel worker processes. Worker 0 runs solo first "
                        "to warm the acados/CasADi codegen cache.")
    p.add_argument("--quick", action="store_true",
                   help="Tiny smoke matrix: clay/sinusoidal/v5/b0, one seed.")
    return p.parse_args()


@dataclass(frozen=True)
class _Task:
    """Pickle-friendly description of one closed-loop run."""
    idx: int
    variant: str
    extra: tuple[str, ...]
    note: str
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
    """ProcessPool worker. Each call runs one launch_decoupled closed-loop."""
    os.environ.setdefault("ACADOS_UNIQUE_BUILD_DIR", "1")
    res = launch_and_collect(
        experiment="throttle_dob_ablation",
        variant=task.variant,
        controller_mode="standard",
        mpc_model="nn",
        nn_model=DEFAULT_NN_MODEL,
        terrain=task.terrain, path=task.path,
        speed=task.speed, bumpiness=task.bumpiness, seed=task.seed,
        run_dir=Path(task.run_dir_str),
        sim_port=task.sim_port, ctrl_port=task.ctrl_port,
        sim_time=task.sim_time, timeout=task.timeout,
        rocks=0, lead_in=task.lead_in,
        extra_args=list(task.extra),
    )
    res.notes = task.note
    return res


def plot_figures(results_csv: Path, out_dir: Path) -> None:
    df = pd.read_csv(results_csv)
    ok = df[df["status"] == "ok"].copy()
    if ok.empty:
        return
    fig_dir = out_dir / "figures"

    summary = ok.groupby("variant", sort=False).agg(
        rms_cte=("rms_cte_m", "mean"),
        rms_cte_std=("rms_cte_m", "std"),
        speed_ratio=("speed_ratio", "mean"),
        speed_ratio_std=("speed_ratio", "std"),
        mean_speed=("mean_speed_mps", "mean"),
        mean_speed_std=("mean_speed_mps", "std"),
    ).reset_index()
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.2))
    x = np.arange(len(summary))
    for ax, mean_key, std_key, ylabel in [
        (axes[0], "rms_cte", "rms_cte_std", "RMS CTE (m)"),
        (axes[1], "speed_ratio", "speed_ratio_std", "mean speed / target"),
        (axes[2], "mean_speed", "mean_speed_std", "Mean speed (m/s)"),
    ]:
        ax.bar(x, summary[mean_key], yerr=summary[std_key].fillna(0.0), capsize=3)
        ax.set_xticks(x)
        ax.set_xticklabels(summary["variant"], rotation=20, ha="right")
        ax.set_ylabel(ylabel)
        ax.grid(axis="y", alpha=0.3)
    fig.suptitle("Asymmetric throttle DOB ablation")
    fig.tight_layout()
    fig.savefig(fig_dir / "throttle_dob_summary.png", dpi=220)
    plt.close(fig)

    # Per-terrain speed-ratio bar so the reader sees where DOB earns its keep.
    ok["scenario"] = (
        ok["terrain"] + "/" + ok["path"]
        + "/v" + ok["speed_mps"].astype(str) + "/b" + ok["bumpiness"].astype(str)
    )
    pivot = ok.pivot_table(index="scenario", columns="variant",
                           values="speed_ratio", aggfunc="mean")
    fig, ax = plt.subplots(figsize=(1.3 * len(pivot.columns) + 4,
                                    0.42 * len(pivot.index) + 2.5))
    im = ax.imshow(pivot.values, aspect="auto", cmap="viridis", vmin=0.0,
                   vmax=max(1.2, float(np.nanmax(pivot.values))))
    ax.set_xticks(range(len(pivot.columns)))
    ax.set_xticklabels(pivot.columns, rotation=25, ha="right")
    ax.set_yticks(range(len(pivot.index)))
    ax.set_yticklabels(pivot.index, fontsize=8)
    for i in range(pivot.shape[0]):
        for j in range(pivot.shape[1]):
            v = pivot.values[i, j]
            if np.isfinite(v):
                ax.text(j, i, f"{v:.2f}", ha="center", va="center", fontsize=8)
    ax.set_title("Speed retention by scenario")
    fig.colorbar(im, ax=ax, fraction=0.035)
    fig.tight_layout()
    fig.savefig(fig_dir / "throttle_dob_speed_ratio_heatmap.png", dpi=220)
    plt.close(fig)

    plot_metric_distribution_grid(
        results_csv,
        out_dir,
        [
            ("rms_cte_m", "RMS CTE (m)", "Tracking error"),
            ("speed_ratio", "mean speed / target", "Speed retention"),
            ("mean_speed_mps", "Mean speed (m/s)", "Achieved speed"),
        ],
        "throttle_dob_metric_distributions.png",
        "Asymmetric throttle DOB ablation",
    )
    plot_trajectory_overlays(
        results_csv,
        out_dir,
        filename_prefix="throttle_dob_trajectory_overlay",
        max_scenarios=4,
    )


def main() -> None:
    args = parse_args()
    if args.quick:
        args.terrains = ["clay"]
        args.paths = ["sinusoidal"]
        args.speeds = [5.0]
        args.bumpiness = [0]
        args.seeds = 1
        args.time = min(args.time, 8.0)

    out_dir = timestamped_result_dir("throttle_dob_ablation")
    write_manifest(out_dir, args,
                   "Asymmetric throttle DOB ablation with sensor noise enabled.")
    print(f"Output: {out_dir}")

    tasks: list[_Task] = []
    idx = 0
    for variant in args.variants:
        spec = VARIANTS[variant]
        for terrain in args.terrains:
            for path in args.paths:
                for speed in args.speeds:
                    for bump in args.bumpiness:
                        for seed_i in range(args.seeds):
                            seed = args.base_seed + seed_i
                            sim_port = args.base_port + 2 * idx
                            ctrl_port = sim_port + 1
                            run_dir = out_dir / "raw" / (
                                f"{idx:04d}_{variant}_{terrain}_{path}_v{speed:g}_b{bump}_s{seed}"
                            )
                            tasks.append(_Task(
                                idx=idx, variant=variant,
                                extra=tuple(spec["extra"]),
                                note=spec["note"],
                                terrain=terrain, path=path, speed=speed,
                                bumpiness=bump, seed=seed,
                                run_dir_str=str(run_dir),
                                sim_port=sim_port, ctrl_port=ctrl_port,
                                sim_time=args.time, timeout=args.timeout,
                                lead_in=args.lead_in,
                            ))
                            idx += 1

    total = len(tasks)
    results: list[RunResult] = []

    print(f"[1/{total}] (warmup) {tasks[0].variant} {tasks[0].terrain}/{tasks[0].path} "
          f"v={tasks[0].speed:g} b={tasks[0].bumpiness} seed={tasks[0].seed}")
    first = _run_one(tasks[0])
    results.append(first)
    write_results_csv(out_dir / "results.csv", results)
    print(f"    {first.status}: rms_cte={first.rms_cte_m:.3f} "
          f"speed_ratio={first.speed_ratio:.2f}")

    if len(tasks) > 1:
        completed = 1
        with ProcessPoolExecutor(max_workers=max(1, args.workers)) as ex:
            futs = {ex.submit(_run_one, t): t for t in tasks[1:]}
            for fut in as_completed(futs):
                t = futs[fut]
                res = fut.result()
                results.append(res)
                completed += 1
                write_results_csv(out_dir / "results.csv", results)
                print(f"[{completed}/{total}] {t.variant} {t.terrain}/{t.path} "
                      f"v={t.speed:g} b={t.bumpiness} seed={t.seed}")
                print(f"    {res.status}: rms_cte={res.rms_cte_m:.3f} "
                      f"speed_ratio={res.speed_ratio:.2f}")

    write_results_csv(out_dir / "results.csv", results)
    summary = summarize_by_variant(
        results,
        ["rms_cte_m", "speed_ratio", "mean_speed_mps", "p95_speed_mps", "mean_solve_ms"],
    )
    summary.to_csv(out_dir / "summary_by_variant.csv", index=False)
    save_summary_markdown(
        out_dir,
        "Asymmetric Throttle DOB Ablation",
        summary,
        [
            "Noise policy: sensor noise enabled in every run.",
            "DOB-off uses --dob-ki 0 --dob-max 0 so the controller still constructs the DOB hook but adds no integral action.",
            "Variant notes: " + "; ".join(f"{k}: {VARIANTS[k]['note']}" for k in args.variants),
        ],
    )
    plot_figures(out_dir / "results.csv", out_dir)
    print(f"Done: {out_dir}")


if __name__ == "__main__":
    main()
