#!/usr/bin/env python3
"""Paper experiment: live terrain estimator on/off for tire-model comparison.

The default ``mpc_tire_model_sweep`` runs every model with static terrain
parameters, so it cannot isolate the live-estimator contribution. This sweep
re-runs the same tire models with ``--terrain-estimator`` enabled so the
live-conditioned advantage is measurable. The retained online estimator is
n-only.

Sensor noise is enabled in every run.
"""

from __future__ import annotations

import argparse
import os
import sys
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
    BUMPS,
    DEFAULT_NN_MODEL,
    PATHS,
    SPEEDS,
    TERRAINS,
    RunResult,
    launch_and_collect,
    plot_metric_distribution_grid,
    save_summary_markdown,
    summarize_by_variant,
    timestamped_result_dir,
    write_manifest,
    write_results_csv,
)


# Subset of tire models actually wired into the orchestrator's pilot tier.
# ``estimator`` is True when this row should carry --terrain-estimator.
VARIANTS = {
    "pacejka_static":           dict(mpc="pacejka", nn=DEFAULT_NN_MODEL, estimator=False),
    "tmeasy_static":            dict(mpc="tmeasy",  nn=DEFAULT_NN_MODEL, estimator=False),
    "nn_static":                dict(mpc="nn", nn=DEFAULT_NN_MODEL, estimator=False),
    "nn_estimator":             dict(mpc="nn", nn=DEFAULT_NN_MODEL, estimator=True, mode="n"),
    # Wrong-prior baseline: NN with the controller's static prior locked to
    # dirt regardless of the plant terrain. On clay/sand this exercises the
    # exact mismatch the estimator is meant to fix.
    "nn_wrong_prior":           dict(mpc="nn", nn=DEFAULT_NN_MODEL,
                                     estimator=False, controller_prior="dirt"),
}
DEFAULT_VARIANTS = ["pacejka_static", "tmeasy_static", "nn_static", "nn_estimator"]

VARIANT_LABELS = {
    "pacejka_static": "Pacejka static",
    "tmeasy_static": "TMeasy static",
    "nn_static": "NN static prior",
    "nn_estimator": "NN live n estimator",
    "nn_wrong_prior": "NN wrong prior",
}


def display_variant(value: str) -> str:
    return VARIANT_LABELS.get(str(value), str(value).replace("_", " "))


def add_scenario_labels(ok: pd.DataFrame) -> pd.DataFrame:
    """Scenario labels that show achieved speed beside the requested speed."""
    out = ok.copy()
    stats = (
        out.groupby(["terrain", "speed_mps"], sort=False)
        .agg(mean_speed=("mean_speed_mps", "mean"))
        .reset_index()
    )
    stats["scenario_label"] = stats.apply(
        lambda r: (
            f"{r['terrain']} - cmd {float(r['speed_mps']):.0f}, "
            f"ubar {float(r['mean_speed']):.2f} m/s"
        ),
        axis=1,
    )
    return out.merge(stats, on=["terrain", "speed_mps"], how="left")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--variants", nargs="+", default=DEFAULT_VARIANTS,
                   choices=list(VARIANTS))
    p.add_argument("--terrains", nargs="+", default=list(TERRAINS), choices=list(TERRAINS))
    p.add_argument("--paths", nargs="+", default=list(PATHS))
    p.add_argument("--speeds", nargs="+", type=float, default=list(SPEEDS))
    p.add_argument("--bumpiness", nargs="+", type=int, default=list(BUMPS))
    p.add_argument("--seeds", type=int, default=5)
    p.add_argument("--base-seed", type=int, default=400)
    p.add_argument("--time", type=float, default=20.0,
                   help="Longer than the static sweep so the estimator has time to converge.")
    p.add_argument("--lead-in", type=float, default=5.0)
    p.add_argument("--metric-start", type=float, default=8.0,
                   help="Start KPI window after the estimator has had time to settle.")
    p.add_argument("--timeout", type=float, default=240.0)
    p.add_argument("--base-port", type=int, default=9000)
    p.add_argument("--workers", type=int, default=6,
                   help="Parallel worker processes. Worker 0 runs solo first "
                        "to warm the acados/CasADi codegen cache.")
    p.add_argument("--quick", action="store_true")
    return p.parse_args()


@dataclass(frozen=True)
class _Task:
    """Pickle-friendly description of one closed-loop run."""
    idx: int
    variant: str
    mpc_model: str
    nn_model: str
    extra: tuple[str, ...]
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
    metric_start: float


def _run_one(task: _Task) -> RunResult:
    """ProcessPool worker. Each call runs one launch_decoupled closed-loop."""
    os.environ.setdefault("ACADOS_UNIQUE_BUILD_DIR", "1")
    return launch_and_collect(
        experiment="tire_model_with_estimator_ablation",
        variant=task.variant,
        controller_mode="standard",
        mpc_model=task.mpc_model,
        nn_model=task.nn_model,
        terrain=task.terrain, path=task.path,
        speed=task.speed, bumpiness=task.bumpiness, seed=task.seed,
        run_dir=Path(task.run_dir_str),
        sim_port=task.sim_port, ctrl_port=task.ctrl_port,
        sim_time=task.sim_time, timeout=task.timeout,
        rocks=0, lead_in=task.lead_in,
        extra_args=list(task.extra),
        metric_start=task.metric_start,
    )


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
        solve_ms=("mean_solve_ms", "mean"),
    ).reset_index()
    summary["display_variant"] = summary["variant"].map(display_variant)
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.2))
    x = np.arange(len(summary))
    for ax, mean_key, std_key, ylabel in [
        (axes[0], "rms_cte", "rms_cte_std", "RMS CTE (m)"),
        (axes[1], "mean_speed", "mean_speed_std", "Achieved mean speed (m/s)"),
        (axes[2], "solve_ms", None, "Mean solve time (ms)"),
    ]:
        err = summary[std_key].fillna(0.0) if std_key else None
        ax.bar(x, summary[mean_key], yerr=err, capsize=3)
        ax.set_xticks(x)
        ax.set_xticklabels(summary["display_variant"], rotation=20, ha="right")
        ax.set_ylabel(ylabel)
        ax.grid(axis="y", alpha=0.3)
    fig.suptitle("Tire model x live terrain estimator")
    fig.tight_layout()
    fig.savefig(fig_dir / "tire_estimator_summary.png", dpi=220)
    plt.close(fig)

    ok = add_scenario_labels(ok)
    pivot = ok.pivot_table(index="scenario_label", columns="variant",
                           values="rms_cte_m", aggfunc="mean")
    pivot = pivot.rename(columns=display_variant)
    fig, ax = plt.subplots(figsize=(1.5 * len(pivot.columns) + 4,
                                    0.5 * len(pivot.index) + 3))
    im = ax.imshow(pivot.values, aspect="auto", cmap="viridis_r")
    ax.set_xticks(range(len(pivot.columns)))
    ax.set_xticklabels(pivot.columns, rotation=25, ha="right")
    ax.set_yticks(range(len(pivot.index)))
    ax.set_yticklabels(pivot.index, fontsize=8)
    for i in range(pivot.shape[0]):
        for j in range(pivot.shape[1]):
            v = pivot.values[i, j]
            if np.isfinite(v):
                ax.text(j, i, f"{v:.2f}", ha="center", va="center", fontsize=8,
                        color="white" if v > pivot.values.mean() else "black")
    ax.set_title("RMS CTE (m) by (terrain, speed)")
    fig.colorbar(im, ax=ax, fraction=0.035)
    fig.tight_layout()
    fig.savefig(fig_dir / "tire_estimator_rms_cte_heatmap.png", dpi=220)
    plt.close(fig)

    plot_metric_distribution_grid(
        results_csv,
        out_dir,
        [
            ("rms_cte_m", "RMS CTE (m)", "Tracking error"),
            ("mean_speed_mps", "Achieved mean speed (m/s)", "Actual speed"),
            ("mean_solve_ms", "Mean solve (ms)", "Runtime"),
        ],
        "tire_estimator_metric_distributions.png",
        "Tire model x live terrain estimator",
    )


def main() -> None:
    args = parse_args()
    if args.quick:
        args.terrains = ["clay"]
        args.paths = ["sinusoidal"]
        args.speeds = [5.0]
        args.bumpiness = [0]
        args.seeds = 1
        args.time = min(args.time, 12.0)

    result_prefix = "tire_model_with_estimator_ablation"
    out_dir = timestamped_result_dir(result_prefix)
    write_manifest(out_dir, args,
                   "Tire model sweep with the live terrain estimator on, "
                   "to measure closed-loop tracking and speed retention.")
    print(f"Output: {out_dir}")

    tasks: list[_Task] = []
    idx = 0
    for variant in args.variants:
        spec = VARIANTS[variant]
        # Controlled comparison: fix the speed reference across tire models so the
        # tracking comparison is not confounded by the g-g planner (its grip limits
        # differ per tire model: NN surrogate vs analytical Coulomb fallback). Sec. III.
        extra = ["--legacy-speed-ref"]
        if spec["estimator"]:
            extra += ["--terrain-estimator", "--terrain-estimator-mode", spec.get("mode", "n")]
        if "controller_prior" in spec:
            extra += ["--controller-prior-terrain", spec["controller_prior"]]
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
                                mpc_model=spec["mpc"], nn_model=spec["nn"],
                                extra=tuple(extra),
                                terrain=terrain, path=path, speed=speed,
                                bumpiness=bump, seed=seed,
                                run_dir_str=str(run_dir),
                                sim_port=sim_port, ctrl_port=ctrl_port,
                                sim_time=args.time, timeout=args.timeout,
                                lead_in=args.lead_in,
                                metric_start=args.metric_start,
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
        ["rms_cte_m","speed_ratio","mean_speed_mps","mean_solve_ms"],
    )
    summary.to_csv(out_dir / "summary_by_variant.csv", index=False)
    save_summary_markdown(
        out_dir,
        "Tire model x live terrain estimator",
        summary,
        [
            "Noise policy: sensor noise enabled in every run.",
            "Estimator-off variants use static terrain params; estimator-on adds "
            "--terrain-estimator so terrain parameters are re-conditioned online "
            "from IMU + wheel-speed signals. The joint variant additionally "
            "forwards phi when selected.",
            "Metric window starts after the estimator has had time to settle.",
        ],
    )
    plot_figures(out_dir / "results.csv", out_dir)
    print(f"Done: {out_dir}")


if __name__ == "__main__":
    main()
