#!/usr/bin/env python3
"""Paper experiment: learned terrain estimator in-distribution vs OOD.

One thing tested: how accurately the online learned estimator recovers Bekker n
and, when selected, friction angle phi from noisy closed-loop driving on
canonical terrains and randomly sampled out-of-distribution SCM soils. Each
run starts the estimator from the same neutral dirt initialization used by the
controller.
"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import asdict, dataclass
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
    PROJECT_ROOT,
    SPEEDS,
    TERRAINS,
    RunResult,
    launch_and_collect,
    save_summary_markdown,
    timestamped_result_dir,
    write_manifest,
)


TRUE_N = {"clay": 0.5, "dirt": 0.7, "sand": 1.1}
TRUE_PHI = {"clay": 13.0, "dirt": 29.0, "sand": 30.0}


DIST_LABELS = {
    "id": "Canonical",
    "ood": "Random soils",
}


def _dist_label(value: str) -> str:
    return DIST_LABELS.get(str(value), str(value))


def _case_label(value: str, distribution: str) -> str:
    value = str(value)
    if distribution == "ood" and value.startswith("terrain"):
        return "Random " + value.replace("terrain", "")
    return value.capitalize()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--distributions", nargs="+", default=["id", "ood"], choices=["id", "ood"])
    p.add_argument("--terrains", nargs="+", default=list(TERRAINS), choices=list(TERRAINS),
                   help="Canonical in-distribution terrains.")
    p.add_argument("--paths", nargs="+", default=["sinusoidal"], choices=list(PATHS),
                   help="Sinusoidal is the default paper setting because the estimator needs excitation.")
    p.add_argument("--speeds", nargs="+", type=float, default=[5.0, 7.0])
    # The learned window-MLP estimator is trained on bumpiness {0,4}; bumpiness 8
    # is out-of-distribution for its vertical-dynamics features (see paper §IV),
    # so the benchmark stays within that training envelope by default.
    p.add_argument("--bumpiness", nargs="+", type=int, default=[0, 4])
    p.add_argument("--seeds", type=int, default=5)
    p.add_argument("--base-seed", type=int, default=710)
    p.add_argument("--ood-terrains", type=int, default=6)
    p.add_argument("--time", type=float, default=20.0)
    p.add_argument("--lead-in", type=float, default=5.0)
    p.add_argument("--metric-start", type=float, default=8.0)
    p.add_argument("--speed-weight", type=float, default=15.0)
    p.add_argument("--ay-safety", type=float, default=0.65,
                   help="Forwarded to the reference speed profile. Higher values "
                        "allow faster sinusoidal cornering.")
    p.add_argument("--sine-amplitude", type=float, default=2.0)
    p.add_argument("--sine-wavelength", type=float, default=30.0)
    p.add_argument("--estimator-mode", choices=["n"], default="n",
                   help="Live estimator mode. The retained paper path is n-only.")
    p.add_argument("--estimator-backend",
                   choices=["learned", "nn_ukf", "bekker_ukf", "fused"], default="fused",
                   help="Runtime terrain-estimator backend (default 'fused' = the "
                        "deployed regime-aware blend of the window-MLP and the online "
                        "NN-UKF). 'learned' = window-MLP only; 'nn_ukf'/'bekker_ukf' = "
                        "the online state-augmented UKF (Fy surrogate / analytical Bekker).")
    p.add_argument("--learned-terrain-model-dir", default=None,
                   help="Optional terrain estimator checkpoint directory to "
                        "forward to the controller. Useful for evaluating "
                        "candidate n-only or joint checkpoints.")
    p.add_argument("--timeout", type=float, default=240.0)
    p.add_argument("--base-port", type=int, default=9400)
    p.add_argument("--workers", type=int, default=6,
                   help="Chrono workers after one cache-prewarm run.")
    p.add_argument("--quick", action="store_true",
                   help="Tiny smoke matrix: one ID run and one OOD run.")
    return p.parse_args()


@dataclass(frozen=True)
class BenchmarkTask:
    idx: int
    total: int
    distribution: str
    terrain: str
    case_label: str
    true_n: float
    true_phi: float
    terrain_config: str
    path: str
    speed: float
    bumpiness: int
    seed: int
    sim_port: int
    ctrl_port: int
    run_dir: str
    sim_time: float
    lead_in: float
    timeout: float
    metric_start: float
    speed_weight: float
    ay_safety: float
    sine_amplitude: float
    sine_wavelength: float
    estimator_mode: str
    estimator_backend: str
    learned_terrain_model_dir: str | None
    unique_build_dir: bool


def _run_one(task: BenchmarkTask) -> dict:
    if task.unique_build_dir:
        os.environ.setdefault("ACADOS_UNIQUE_BUILD_DIR", "1")
    extra = [
        "--terrain-estimator",
        "--terrain-estimator-mode", task.estimator_mode,
        "--terrain-estimator-backend", task.estimator_backend,
        "--te-verbose",
        "--speed-weight", str(task.speed_weight),
        "--ay-safety", str(task.ay_safety),
        "--sine-amplitude", str(task.sine_amplitude),
        "--sine-wavelength", str(task.sine_wavelength),
    ]
    if task.learned_terrain_model_dir:
        extra += ["--learned-terrain-model-dir", task.learned_terrain_model_dir]
    if task.terrain_config:
        extra += ["--terrain-config", task.terrain_config]
    res = launch_and_collect(
        experiment="terrain_estimator_benchmark",
        variant=task.distribution,
        controller_mode="standard",
        mpc_model="nn",
        nn_model=DEFAULT_NN_MODEL,
        terrain=task.terrain,
        path=task.path,
        speed=task.speed,
        bumpiness=task.bumpiness,
        seed=task.seed,
        run_dir=Path(task.run_dir),
        sim_port=task.sim_port,
        ctrl_port=task.ctrl_port,
        sim_time=task.sim_time,
        timeout=task.timeout,
        rocks=0,
        lead_in=task.lead_in,
        extra_args=extra,
        metric_start=task.metric_start,
    )
    row = result_row(
        res,
        task.distribution,
        task.case_label,
        task.true_n,
        task.true_phi,
        task.terrain_config,
        task.metric_start,
    )
    row["estimator_backend"] = task.estimator_backend
    row["estimator_mode"] = task.estimator_mode
    row["speed_weight"] = task.speed_weight
    row["ay_safety"] = task.ay_safety
    row["sine_amplitude"] = task.sine_amplitude
    row["sine_wavelength"] = task.sine_wavelength
    row["excitation_source"] = "closed_loop_mpc"
    if task.estimator_mode != "joint":
        # The controller still logs its nominal phi parameter in n-only mode.
        # Treat that as controller context, not a phi estimate.
        for key in (
            "phi_est_final_deg",
            "phi_est_mean_tail_deg",
            "phi_abs_err_final_deg",
            "phi_abs_err_tail_deg",
        ):
            row[key] = math.nan
    return row


def generate_ood_terrains(out_dir: Path, n_terrains: int, seed: int) -> pd.DataFrame:
    terrain_dir = out_dir / "raw" / "ood_terrains"
    cmd = [
        sys.executable, "-u", str(PROJECT_ROOT / "utilities" / "generate_random_terrains.py"),
        "--out-dir", str(terrain_dir),
        "--n-terrains", str(n_terrains),
        "--seed", str(seed),
    ]
    log_path = terrain_dir / "generate.log"
    terrain_dir.mkdir(parents=True, exist_ok=True)
    with log_path.open("w") as f:
        subprocess.run(cmd, cwd=str(PROJECT_ROOT), stdout=f, stderr=subprocess.STDOUT, check=True)
    return pd.read_csv(terrain_dir / "manifest.csv")


def estimator_metrics(diag_csv: str, true_n: float, true_phi: float,
                      metric_start: float) -> dict[str, float]:
    empty = {
        "n_est_final": math.nan,
        "n_est_mean_tail": math.nan,
        "n_abs_err_final": math.nan,
        "n_abs_err_tail": math.nan,
        "phi_est_final_deg": math.nan,
        "phi_est_mean_tail_deg": math.nan,
        "phi_abs_err_final_deg": math.nan,
        "phi_abs_err_tail_deg": math.nan,
        "terrain_update_count": 0,
        "first_update_time_s": math.nan,
    }
    if not diag_csv:
        return empty
    # Guard against header-only / zero-byte diag CSVs (a degenerate run writes
    # one) — an unguarded read raises EmptyDataError and, in a pool worker,
    # takes the whole collection down (cf. CLAUDE.md Parallelism rule 3).
    try:
        df = pd.read_csv(diag_csv)
    except (pd.errors.EmptyDataError, pd.errors.ParserError, FileNotFoundError):
        return empty
    if df.empty or "n_terrain_est" not in df.columns:
        return empty
    t = pd.to_numeric(df.get("sim_time", pd.Series(np.arange(len(df)))), errors="coerce")
    n_est = pd.to_numeric(df["n_terrain_est"], errors="coerce")
    finite = np.isfinite(n_est)
    tail = finite & np.isfinite(t) & (t >= metric_start)
    if not tail.any():
        tail = finite
    updates = pd.to_numeric(df.get("terrain_update_applied", pd.Series(np.zeros(len(df)))), errors="coerce").fillna(0)
    update_mask = updates > 0
    first_update = float(t[update_mask].iloc[0]) if update_mask.any() else math.nan
    final_est = float(n_est[finite].iloc[-1]) if finite.any() else math.nan
    mean_tail = float(n_est[tail].mean()) if tail.any() else math.nan

    phi_final = math.nan
    phi_tail = math.nan
    phi_col = (
        "phi_terrain_estimator_deg"
        if "phi_terrain_estimator_deg" in df.columns
        else "phi_terrain_est_deg"
    )
    if phi_col in df.columns:
        phi_est = pd.to_numeric(df[phi_col], errors="coerce")
        phi_finite = np.isfinite(phi_est)
        phi_tail_mask = phi_finite & np.isfinite(t) & (t >= metric_start)
        if not phi_tail_mask.any():
            phi_tail_mask = phi_finite
        phi_final = float(phi_est[phi_finite].iloc[-1]) if phi_finite.any() else math.nan
        phi_tail = float(phi_est[phi_tail_mask].mean()) if phi_tail_mask.any() else math.nan
    return {
        "n_est_final": final_est,
        "n_est_mean_tail": mean_tail,
        "n_abs_err_final": abs(final_est - true_n) if math.isfinite(final_est) else math.nan,
        "n_abs_err_tail": abs(mean_tail - true_n) if math.isfinite(mean_tail) else math.nan,
        "phi_est_final_deg": phi_final,
        "phi_est_mean_tail_deg": phi_tail,
        "phi_abs_err_final_deg": abs(phi_final - true_phi) if math.isfinite(phi_final) and math.isfinite(true_phi) else math.nan,
        "phi_abs_err_tail_deg": abs(phi_tail - true_phi) if math.isfinite(phi_tail) and math.isfinite(true_phi) else math.nan,
        "terrain_update_count": int(update_mask.sum()),
        "first_update_time_s": first_update,
    }


def result_row(res: RunResult, distribution: str, case_label: str, true_n: float,
               true_phi: float,
               terrain_config: str, metric_start: float) -> dict:
    row = asdict(res)
    row.pop("extra", None)
    row.update({
        "distribution": distribution,
        "case_label": case_label,
        "true_n": true_n,
        "true_phi_deg": true_phi,
        "terrain_config": terrain_config,
    })
    row.update(estimator_metrics(res.diag_csv, true_n, true_phi, metric_start))
    return row


def plot_figures(results_csv: Path, out_dir: Path) -> None:
    df = pd.read_csv(results_csv)
    ok = df[df["status"] == "ok"].copy()
    if ok.empty:
        return
    fig_dir = out_dir / "figures"
    path_label = "sinusoidal only" if ok.get("path", pd.Series()).astype(str).nunique() == 1 and ok["path"].astype(str).iloc[0] == "sinusoidal" else "all paths"
    source_map = {
        "closed_loop_mpc": "closed-loop MPC excitation",
        "open_loop_command": "open-loop commanded excitation",
    }
    if "excitation_source" in ok.columns:
        sources = ok["excitation_source"].fillna("closed_loop_mpc").astype(str).unique()
    else:
        sources = np.asarray(["closed_loop_mpc"])
    if len(sources) == 1:
        source_label = source_map.get(str(sources[0]), str(sources[0]).replace("_", " "))
    else:
        source_label = "mixed excitation sources"
    geom_label = ""
    if {"sine_amplitude", "sine_wavelength"}.issubset(ok.columns):
        amps = pd.to_numeric(ok["sine_amplitude"], errors="coerce").dropna().unique()
        wls = pd.to_numeric(ok["sine_wavelength"], errors="coerce").dropna().unique()
        if len(amps) == 1 and len(wls) == 1:
            geom_label = f", A={float(amps[0]):g} m, lambda={float(wls[0]):g} m"

    summary = ok.groupby("distribution", sort=False).agg(
        n_err=("n_abs_err_tail", "mean"),
        n_err_std=("n_abs_err_tail", "std"),
        cte=("rms_cte_m", "mean"),
        mean_speed=("mean_speed_mps", "mean"),
        target_speed=("speed_mps", "mean"),
        true_n_mean=("true_n", "mean"),
        n_runs=("status", "count"),
    ).reset_index()
    summary["display"] = summary["distribution"].map(_dist_label)

    case = ok.groupby(["distribution", "case_label", "true_n"], sort=False).agg(
        n_err=("n_abs_err_tail", "mean"),
        n_err_std=("n_abs_err_tail", "std"),
        n_est=("n_est_mean_tail", "mean"),
        mean_speed=("mean_speed_mps", "mean"),
        target_speed=("speed_mps", "mean"),
        rms_cte=("rms_cte_m", "mean"),
        runs=("status", "count"),
    ).reset_index()
    case["display"] = [
        _case_label(c, d) for c, d in zip(case["case_label"], case["distribution"])
    ]
    case = case.sort_values(["true_n", "distribution", "display"]).reset_index(drop=True)

    x = np.arange(len(summary))
    fig, axes = plt.subplots(1, 3, figsize=(13.2, 4.2))

    axes[0].bar(x, summary["n_err"], color=["#5b8fb9", "#d08c60"][: len(summary)])
    for i, row in summary.iterrows():
        axes[0].text(i, row["n_err"] + 0.005,
                     f"{row['n_err']:.3f}\nN={int(row['n_runs'])}",
                     ha="center", va="bottom", fontsize=8)
    axes[0].set_xticks(x)
    axes[0].set_xticklabels(summary["display"])
    axes[0].set_ylabel("Mean tail |n error|")
    axes[0].set_title("Aggregate error")
    axes[0].set_ylim(0.0, float(summary["n_err"].max()) * 1.35)
    axes[0].grid(axis="y", alpha=0.3)

    colors = {"id": "#5b8fb9", "ood": "#d08c60"}
    for distribution, sub in case.groupby("distribution", sort=False):
        axes[1].scatter(
            sub["true_n"], np.full(len(sub), _dist_label(distribution)),
            s=80, color=colors.get(distribution, "#777777"), alpha=0.85,
            edgecolor="black", linewidth=0.4,
        )
    axes[1].set_xlabel("True Bekker n in evaluation set")
    axes[1].set_title("Run mix explains ID/OOD")
    axes[1].grid(axis="x", alpha=0.3)

    axes[2].bar(x, summary["mean_speed"], color=["#5b8fb9", "#d08c60"][: len(summary)])
    axes[2].scatter(x, summary["target_speed"], marker="_", s=260,
                    color="black", label="requested")
    for i, row in summary.iterrows():
        axes[2].text(i, row["mean_speed"] + 0.06,
                     f"ubar={row['mean_speed']:.2f}", ha="center", va="bottom", fontsize=8)
    axes[2].set_xticks(x)
    axes[2].set_xticklabels(summary["display"])
    axes[2].set_ylabel("Achieved mean speed (m/s)")
    axes[2].set_title("Actual speed, not command")
    axes[2].grid(axis="y", alpha=0.3)
    axes[2].legend(frameon=False, fontsize=8)

    fig.suptitle(f"Online n-only terrain estimator ({source_label}; {path_label}{geom_label})")
    fig.tight_layout()
    fig.savefig(fig_dir / "terrain_estimator_summary.png", dpi=220)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9.4, 5.4))
    y = np.arange(len(case))
    bar_colors = [colors.get(d, "#777777") for d in case["distribution"]]
    ax.barh(y, case["n_err"], color=bar_colors, alpha=0.88,
            edgecolor="white", linewidth=0.8)
    labels = [
        f"{row.display}  n={row.true_n:.2f}, ubar={row.mean_speed:.2f} m/s"
        for row in case.itertuples()
    ]
    ax.set_yticks(y)
    ax.set_yticklabels(labels, fontsize=8.5)
    ax.invert_yaxis()
    ax.set_xlabel("Mean tail |n error|")
    ax.set_title(f"Per-terrain n-estimation error, {source_label}{geom_label}")
    ax.set_xlim(0.0, float(case["n_err"].max()) * 1.08)
    ax.grid(axis="x", alpha=0.3)
    handles = [
        plt.Line2D([0], [0], color=colors["id"], lw=6, label="Canonical"),
        plt.Line2D([0], [0], color=colors["ood"], lw=6, label="Random soils"),
    ]
    ax.legend(handles=handles, frameon=False, loc="lower right")
    for yi, row in zip(y, case.itertuples()):
        ax.text(row.n_err + 0.003, yi, f"{row.n_err:.3f}",
                va="center", ha="left", fontsize=8)
    fig.text(
        0.01, 0.01,
        "Closed-loop means the NMPC tracks the sine path; labels show true n and achieved mean speed.",
        fontsize=8,
    )
    fig.tight_layout()
    fig.savefig(fig_dir / "terrain_estimator_error_heatmap.png", dpi=220)
    fig.savefig(fig_dir / "terrain_estimator_case_errors.png", dpi=220)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.0, 4.5))
    for distribution, sub in ok.groupby("distribution", sort=False):
        ax.scatter(sub["true_n"], sub["n_est_mean_tail"], label=distribution, alpha=0.75, s=46)
    lims = [
        min(ok["true_n"].min(), ok["n_est_mean_tail"].min()) - 0.05,
        max(ok["true_n"].max(), ok["n_est_mean_tail"].max()) + 0.05,
    ]
    ax.plot(lims, lims, "k--", linewidth=1)
    ax.set_xlim(lims)
    ax.set_ylim(lims)
    ax.set_xlabel("True Bekker n")
    ax.set_ylabel("Estimated tail-mean n")
    ax.set_title(f"True vs estimated n ({path_label})")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(fig_dir / "terrain_estimator_true_vs_estimated.png", dpi=220)
    plt.close(fig)

    if ok["phi_est_mean_tail_deg"].notna().any():
        fig, ax = plt.subplots(figsize=(7.0, 4.5))
        for distribution, sub in ok.groupby("distribution", sort=False):
            ax.scatter(sub["true_phi_deg"], sub["phi_est_mean_tail_deg"],
                       label=distribution, alpha=0.75, s=46)
        finite_phi = ok[["true_phi_deg", "phi_est_mean_tail_deg"]].to_numpy(dtype=float)
        finite_phi = finite_phi[np.isfinite(finite_phi).all(axis=1)]
        if finite_phi.size:
            lo = float(finite_phi.min() - 1.0)
            hi = float(finite_phi.max() + 1.0)
            ax.plot([lo, hi], [lo, hi], "k--", linewidth=1)
            ax.set_xlim(lo, hi)
            ax.set_ylim(lo, hi)
        ax.set_xlabel("True friction angle phi (deg)")
        ax.set_ylabel("Estimated tail-mean phi (deg)")
        ax.legend()
        ax.grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(fig_dir / "terrain_estimator_phi_true_vs_estimated.png", dpi=220)
        plt.close(fig)


def main() -> None:
    args = parse_args()
    if args.quick:
        args.distributions = ["id", "ood"]
        args.terrains = ["clay"]
        args.paths = ["sinusoidal"]
        args.speeds = [5.0]
        args.bumpiness = [0]
        args.seeds = 1
        args.ood_terrains = 1
        args.time = min(args.time, 8.0)

    result_prefix = "terrain_estimator_benchmark"
    description = "Learned n-only terrain estimator ID/OOD benchmark with sensor noise enabled."
    out_dir = timestamped_result_dir(result_prefix)
    write_manifest(out_dir, args, description)
    print(f"Output: {out_dir}")

    ood_manifest = pd.DataFrame()
    if "ood" in args.distributions:
        ood_manifest = generate_ood_terrains(out_dir, args.ood_terrains, args.base_seed)

    rows: list[dict] = []
    idx = 0
    id_cases = [
        dict(distribution="id", terrain=t, case_label=t, true_n=TRUE_N[t],
             true_phi=TRUE_PHI[t], terrain_config="")
        for t in args.terrains
    ]
    ood_cases = []
    for _, r in ood_manifest.iterrows():
        ood_cases.append(dict(
            distribution="ood",
            terrain=str(r["preset_proxy"]),
            case_label=str(r["label"]),
            true_n=float(r["true_n"]),
            true_phi=float(r.get("friction_angle", math.nan)),
            terrain_config=str(r["yaml"]),
        ))
    cases = []
    if "id" in args.distributions:
        cases.extend(id_cases)
    if "ood" in args.distributions:
        cases.extend(ood_cases)

    total = len(cases) * len(args.paths) * len(args.speeds) * len(args.bumpiness) * args.seeds
    learned_model_dir = (
        str(Path(args.learned_terrain_model_dir).resolve())
        if args.learned_terrain_model_dir
        else None
    )
    unique_build_dir = args.workers > 1
    tasks: list[BenchmarkTask] = []
    for case in cases:
        for path in args.paths:
            for speed in args.speeds:
                for bump in args.bumpiness:
                    for seed_i in range(args.seeds):
                        seed = args.base_seed + seed_i
                        sim_port = args.base_port + 2 * idx
                        ctrl_port = sim_port + 1
                        run_dir = out_dir / "raw" / (
                            f"{idx:04d}_{case['distribution']}_{case['case_label']}_{path}_v{speed:g}_b{bump}_s{seed}"
                        )
                        tasks.append(BenchmarkTask(
                            idx=idx,
                            total=total,
                            distribution=case["distribution"],
                            terrain=case["terrain"],
                            case_label=case["case_label"],
                            true_n=case["true_n"],
                            true_phi=case["true_phi"],
                            terrain_config=case["terrain_config"],
                            path=path,
                            speed=speed,
                            bumpiness=bump,
                            seed=seed,
                            sim_port=sim_port,
                            ctrl_port=ctrl_port,
                            run_dir=str(run_dir),
                            sim_time=args.time,
                            lead_in=args.lead_in,
                            timeout=args.timeout,
                            metric_start=args.metric_start,
                            speed_weight=args.speed_weight,
                            ay_safety=args.ay_safety,
                            sine_amplitude=args.sine_amplitude,
                            sine_wavelength=args.sine_wavelength,
                            estimator_mode=args.estimator_mode,
                            estimator_backend=args.estimator_backend,
                            learned_terrain_model_dir=learned_model_dir,
                            unique_build_dir=unique_build_dir,
                        ))
                        idx += 1

    def record(row: dict) -> None:
        rows.append(row)
        pd.DataFrame(rows).to_csv(out_dir / "results.csv", index=False)
        print(
            f"[{len(rows)}/{total}] {row['distribution']}:{row['case_label']} "
            f"{row['path']} v={row['speed_mps']:g} b={row['bumpiness']} "
            f"seed={row['seed']} {row['status']}: "
            f"n_tail={row['n_est_mean_tail']:.3f} "
            f"|err|={row['n_abs_err_tail']:.3f} rms_cte={row['rms_cte_m']:.3f}"
        )

    if tasks:
        print(f"Prewarm: {tasks[0].distribution}:{tasks[0].case_label} on port {tasks[0].sim_port}")
        record(_run_one(tasks[0]))
        remaining = tasks[1:]
        if remaining and args.workers > 1:
            with ProcessPoolExecutor(max_workers=args.workers) as ex:
                futures = {ex.submit(_run_one, task): task for task in remaining}
                for fut in as_completed(futures):
                    record(fut.result())
        else:
            for task in remaining:
                record(_run_one(task))

    results_csv = out_dir / "results.csv"
    pd.DataFrame(rows).to_csv(results_csv, index=False)
    summary = pd.DataFrame(rows).groupby("distribution", sort=False).agg(
        n_runs=("variant", "count"),
        n_ok=("status", lambda s: int((s == "ok").sum())),
        n_abs_err_tail_mean=("n_abs_err_tail", "mean"),
        n_abs_err_tail_std=("n_abs_err_tail", "std"),
        phi_abs_err_tail_deg_mean=("phi_abs_err_tail_deg", "mean"),
        phi_abs_err_tail_deg_std=("phi_abs_err_tail_deg", "std"),
        rms_cte_m_mean=("rms_cte_m", "mean"),
        mean_speed_mps_mean=("mean_speed_mps", "mean"),
        first_update_time_s_mean=("first_update_time_s", "mean"),
    ).reset_index()
    summary.to_csv(out_dir / "summary_by_distribution.csv", index=False)
    save_summary_markdown(
        out_dir,
        "Terrain Estimator Benchmark",
        summary,
        [
            "Noise policy: sensor noise enabled in every run.",
            "The estimator starts from neutral dirt/n=0.7 in every run; OOD terrains are generated into this result directory.",
            f"Estimator tail metrics are averaged after t={args.metric_start:g}s, or over all finite estimates if a quick run is shorter.",
        ],
    )
    plot_figures(results_csv, out_dir)
    print(f"Done: {out_dir}")


if __name__ == "__main__":
    main()
