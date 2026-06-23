#!/usr/bin/env python3
"""Paper experiment: autonomous proxy for command-latency compensation.

One thing tested: how fixed command-path latency changes obstacle avoidance,
tracking, and speed, and whether delay-aware downstream safety filters preserve
clearance.  This is scripted and repeatable; it complements, but does not
replace, `human_delay_compensation_rounds.py`.
"""

from __future__ import annotations

import argparse
import math
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
    plot_trajectory_overlays,
    save_summary_markdown,
    summarize_by_variant,
    timestamped_result_dir,
    write_manifest,
    write_results_csv,
)


FILTERS = ("none", "dob_cbf")


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--filters", nargs="+", default=list(FILTERS), choices=list(FILTERS))
    p.add_argument("--delays", nargs="+", type=float, default=[0.0, 0.15, 0.30])
    p.add_argument("--terrains", nargs="+", default=list(TERRAINS), choices=list(TERRAINS))
    p.add_argument("--paths", nargs="+", default=list(PATHS))
    p.add_argument("--speeds", nargs="+", type=float, default=list(SPEEDS))
    p.add_argument("--bumpiness", nargs="+", type=int, default=list(BUMPS))
    p.add_argument("--seeds", type=int, default=5)
    p.add_argument("--base-seed", type=int, default=910)
    p.add_argument("--time", type=float, default=10.0)
    p.add_argument("--lead-in", type=float, default=5.0)
    p.add_argument("--rocks", type=int, default=5)
    p.add_argument("--safety-buffer", type=float, default=0.5)
    p.add_argument("--shield-horizon", type=int, default=18)
    p.add_argument("--mpc-delay-comp", choices=["on", "off", "both"], default="on",
                   help="Ablate the standard MPC state delay predictor.")
    p.add_argument("--latency-profile-json", default="",
                   help="Use a time-varying 5G-like latency JSON profile instead of fixed delays.")
    p.add_argument("--timeout", type=float, default=220.0)
    p.add_argument("--base-port", type=int, default=56000)
    p.add_argument("--workers", type=int, default=6,
                   help="Parallel worker processes. Worker 0 runs solo first "
                        "to warm the acados/CasADi codegen cache.")
    p.add_argument("--quick", action="store_true",
                   help="Tiny smoke matrix: clay/sinusoidal, delay 0 and 0.15, one seed.")
    return p.parse_args()


def extra_args(filter_name: str, delay: float, mpc_delay_comp: bool,
               args: argparse.Namespace, run_dir: Path) -> list[str]:
    out: list[str] = ["--mpc-blind-obstacles"]
    if args.latency_profile_json:
        out += ["--latency-profile-json", args.latency_profile_json]
        out += ["--latency-profile-log", str(run_dir / "latency_profile.csv")]
    elif delay > 0.0:
        # In chrono_sim_node this buffers controller commands before applying
        # them, and passes the same delay estimate to safety filters.
        out += ["--teleop-delay", str(delay)]
    if not mpc_delay_comp:
        out.append("--no-delay-comp")
    if filter_name != "none":
        out += [
            "--safety-filter",
            "--safety-flavor", filter_name,
            "--safety-buffer", str(args.safety_buffer),
            "--shield-horizon", str(args.shield_horizon),
        ]
    return out


@dataclass(frozen=True)
class _Task:
    """Pickle-friendly description of one closed-loop run."""
    idx: int
    variant: str
    filter_name: str
    delay: float
    mpc_comp: bool
    latency_profile_name: str
    extra: tuple[str, ...]
    terrain: str
    path: str
    speed: float
    bumpiness: int
    seed: int
    rocks: int
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
        experiment="latency_compensation_sweep",
        variant=task.variant,
        controller_mode="standard",
        mpc_model="nn",
        nn_model=DEFAULT_NN_MODEL,
        terrain=task.terrain, path=task.path,
        speed=task.speed, bumpiness=task.bumpiness, seed=task.seed,
        run_dir=Path(task.run_dir_str),
        sim_port=task.sim_port, ctrl_port=task.ctrl_port,
        sim_time=task.sim_time, timeout=task.timeout,
        rocks=task.rocks, lead_in=task.lead_in,
        extra_args=list(task.extra),
    )
    res.extra.update({
        "filter": task.filter_name,
        "delay_s": task.delay,
        "mpc_delay_comp": "on" if task.mpc_comp else "off",
        "latency_profile": task.latency_profile_name,
    })
    return res


def plot_figures(results_csv: Path, out_dir: Path) -> None:
    df = pd.read_csv(results_csv)
    ok = df[df["status"] == "ok"].copy()
    if ok.empty:
        return
    fig_dir = out_dir / "figures"

    summary = ok.groupby(["filter", "delay_s", "mpc_delay_comp"], sort=False).agg(
        collisions=("collisions", "mean"),
        clearance=("min_clearance_m", "mean"),
        rms_cte=("rms_cte_m", "mean"),
        speed_ratio=("speed_ratio", "mean"),
        intervention=("intervention_rate_pct", "mean"),
    ).reset_index()

    comp_modes = list(summary["mpc_delay_comp"].drop_duplicates())
    for comp in comp_modes:
        sub_all = summary[summary["mpc_delay_comp"] == comp]
        fig, axes = plt.subplots(2, 2, figsize=(12, 7.5), constrained_layout=True)
        specs = [
            ("collisions", "Unique obstacles hit"),
            ("clearance", "Minimum clearance (m)"),
            ("rms_cte", "RMS CTE (m)"),
            ("speed_ratio", "Mean speed / target"),
        ]
        for filter_name, sub in sub_all.groupby("filter", sort=False):
            for ax, (key, ylabel) in zip(axes.flat, specs):
                ax.plot(sub["delay_s"], sub[key], marker="o", label=filter_name)
                ax.set_xlabel("Command delay (s)")
                ax.set_ylabel(ylabel)
                ax.grid(alpha=0.3)
        for ax in axes.flat:
            ax.legend(fontsize=8)
        fig.suptitle(f"Latency compensation sweep (MPC delay comp {comp})")
        fig.savefig(fig_dir / f"latency_compensation_summary_mpc_{comp}.png", dpi=220)
        plt.close(fig)

    ok["scenario"] = ok["terrain"] + "/" + ok["path"] + "/d" + ok["delay_s"].astype(str)
    pivot = ok.pivot_table(index="scenario", columns="variant", values="collisions", aggfunc="mean")
    fig, ax = plt.subplots(figsize=(1.55 * len(pivot.columns) + 4, 0.45 * len(pivot.index) + 2.5))
    im = ax.imshow(pivot.values, aspect="auto", cmap="RdYlGn_r")
    ax.set_xticks(range(len(pivot.columns)))
    ax.set_xticklabels(pivot.columns, rotation=30, ha="right")
    ax.set_yticks(range(len(pivot.index)))
    ax.set_yticklabels(pivot.index, fontsize=8)
    for i in range(pivot.shape[0]):
        for j in range(pivot.shape[1]):
            v = pivot.values[i, j]
            if math.isfinite(v):
                ax.text(j, i, f"{v:.1f}", ha="center", va="center", fontsize=7)
    ax.set_title("Mean unique obstacles hit under command delay")
    fig.colorbar(im, ax=ax, fraction=0.035)
    fig.tight_layout()
    fig.savefig(fig_dir / "latency_collision_heatmap.png", dpi=220)
    plt.close(fig)

    plot_metric_distribution_grid(
        results_csv,
        out_dir,
        [
            ("collisions", "Unique obstacles hit", "Collisions"),
            ("min_clearance_m", "Minimum clearance (m)", "Clearance"),
            ("rms_cte_m", "RMS CTE (m)", "Tracking error"),
            ("speed_ratio", "mean speed / target", "Speed retention"),
            ("intervention_rate_pct", "Intervention rate (%)", "Shield activity"),
        ],
        "latency_metric_distributions.png",
        "Command-latency compensation sweep",
    )
    plot_trajectory_overlays(
        results_csv,
        out_dir,
        filename_prefix="latency_trajectory_overlay",
        max_scenarios=4,
    )


def main() -> None:
    args = parse_args()
    if args.quick:
        args.filters = ["none", "dob_cbf"]
        args.delays = [0.0, 0.15]
        args.terrains = ["clay"]
        args.paths = ["sinusoidal"]
        args.speeds = [5.0]
        args.bumpiness = [0]
        args.seeds = 1
        args.time = min(args.time, 8.0)
    if args.latency_profile_json:
        args.latency_profile_json = str(Path(args.latency_profile_json).expanduser().resolve())
        args.delays = [0.0]

    comp_modes = [True, False] if args.mpc_delay_comp == "both" else [args.mpc_delay_comp == "on"]
    out_dir = timestamped_result_dir("latency_compensation_sweep")
    write_manifest(out_dir, args, "Scripted command-latency compensation sweep with sensor noise enabled.")
    print(f"Output: {out_dir}")

    latency_profile_name = Path(args.latency_profile_json).name if args.latency_profile_json else ""
    tasks: list[_Task] = []
    idx = 0
    for filter_name in args.filters:
        for delay in args.delays:
            for mpc_comp in comp_modes:
                comp_label = "mpc_delay_on" if mpc_comp else "mpc_delay_off"
                variant = f"{filter_name}_d{delay:.2f}_{comp_label}"
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
                                        filter_name=filter_name, delay=delay,
                                        mpc_comp=mpc_comp,
                                        latency_profile_name=latency_profile_name,
                                        extra=tuple(extra_args(filter_name, delay, mpc_comp, args, run_dir)),
                                        terrain=terrain, path=path, speed=speed,
                                        bumpiness=bump, seed=seed,
                                        rocks=args.rocks,
                                        run_dir_str=str(run_dir),
                                        sim_port=sim_port, ctrl_port=ctrl_port,
                                        sim_time=args.time, timeout=args.timeout,
                                        lead_in=args.lead_in,
                                    ))
                                    idx += 1

    total = len(tasks)
    results: list[RunResult] = []

    print(f"[1/{total}] (warmup) {tasks[0].variant} {tasks[0].terrain}/{tasks[0].path} seed={tasks[0].seed}")
    first = _run_one(tasks[0])
    results.append(first)
    write_results_csv(out_dir / "results.csv", results)
    print(f"    {first.status}: collisions={first.collisions} "
          f"clearance={first.min_clearance_m:.2f} rms_cte={first.rms_cte_m:.3f}")

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
                print(f"[{completed}/{total}] {t.variant} {t.terrain}/{t.path} seed={t.seed}")
                print(f"    {res.status}: collisions={res.collisions} "
                      f"clearance={res.min_clearance_m:.2f} rms_cte={res.rms_cte_m:.3f}")

    write_results_csv(out_dir / "results.csv", results)
    df = pd.read_csv(out_dir / "results.csv")
    summary = df.groupby(["extra_filter", "extra_delay_s", "extra_mpc_delay_comp"], sort=False).agg(
        n_runs=("status", "count"),
        n_ok=("status", lambda s: int((s == "ok").sum())),
        collisions_mean=("collisions", "mean"),
        min_clearance_m_mean=("min_clearance_m", "mean"),
        rms_cte_m_mean=("rms_cte_m", "mean"),
        speed_ratio_mean=("speed_ratio", "mean"),
        intervention_rate_pct_mean=("intervention_rate_pct", "mean"),
    ).reset_index()
    summary = summary.rename(columns={
        "extra_filter": "filter",
        "extra_delay_s": "delay_s",
        "extra_mpc_delay_comp": "mpc_delay_comp",
    })
    summary.to_csv(out_dir / "summary_by_filter_delay.csv", index=False)
    save_summary_markdown(
        out_dir,
        "Latency Compensation Sweep",
        summary,
        [
            "Noise policy: sensor noise enabled in every run.",
            "`--teleop-delay` applies fixed command-path delay in the simulator and passes the same delay estimate to safety filters for horizon/buffer inflation.",
            "`--latency-profile-json` applies channel-specific time-varying control/manual/camera latency when provided.",
            "This is an autonomous proxy for latency robustness; true driver behavior still belongs in `human_delay_compensation_rounds.py`.",
        ],
    )
    # Expose friendlier columns for plotting.
    df = df.rename(columns={
        "extra_filter": "filter",
        "extra_delay_s": "delay_s",
        "extra_mpc_delay_comp": "mpc_delay_comp",
    })
    df.to_csv(out_dir / "results.csv", index=False)
    plot_figures(out_dir / "results.csv", out_dir)
    print(f"Done: {out_dir}")


if __name__ == "__main__":
    main()
