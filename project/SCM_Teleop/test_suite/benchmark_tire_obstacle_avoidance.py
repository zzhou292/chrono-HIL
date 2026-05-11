#!/usr/bin/env python3
"""
Clay/right_left obstacle-avoidance benchmark across MPC tire models.

Runs each tire model over the same obstacle fields, then averages KPIs across
rounds. Outputs:
  - per-run CSV/JSON
  - aggregate KPI CSV
  - obstacle-hit bar plot
  - tracking KPI bar plot
  - representative trajectory plot with reference path and rocks

Example:
    python test_suite/benchmark_tire_obstacle_avoidance.py --rounds 3

The simulator currently writes collision_log.csv to the project logs/ folder,
so this benchmark runs jobs sequentially and copies that file into each run
folder immediately after the run completes.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import subprocess
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle


ROOT = Path(__file__).resolve().parent.parent
SIM_DIR = ROOT / "simulation"
DEFAULT_OUT_ROOT = SIM_DIR / "plots" / "tire_obstacle_benchmark"
GLOBAL_COLLISION_LOG = ROOT / "logs" / "collision_log.csv"
CHRONO_PYTHON = Path("/home/kyle/miniconda3/envs/chrono/bin/python")

TERRAIN = "clay"
PATH_NAME = "right_left"
DEFAULT_MODELS = ("nn", "pacejka", "pacejka-oracle", "tmeasy")
MODEL_LABELS = {
    "nn": "NN",
    "pacejka": "Pacejka",
    "pacejka-oracle": "Pacejka Oracle",
    "tmeasy": "TMeasy",
}
MODEL_COLORS = {
    "nn": "#2CA02C",
    "pacejka": "#1F77B4",
    "pacejka-oracle": "#9467BD",
    "tmeasy": "#D62728",
}


@dataclass
class RunResult:
    model: str
    round_idx: int
    rock_seed: int
    status: str
    terrain: str
    path: str
    speed_mps: float
    sim_time_s: float
    n_rocks: int
    wall_s: float
    run_dir: str = ""
    diag_csv: str = ""
    collision_csv: str = ""
    avg_abs_cte_m: float = math.nan
    rms_cte_m: float = math.nan
    max_abs_cte_m: float = math.nan
    mean_ref_pos_err_m: float = math.nan
    rms_ref_pos_err_m: float = math.nan
    rms_heading_deg: float = math.nan
    rms_pose_err: float = math.nan
    mean_speed_mps: float = math.nan
    speed_ratio: float = math.nan
    solver_success_pct: float = math.nan
    mean_solve_ms: float = math.nan
    final_x_m: float = math.nan
    progress_frac: float = math.nan
    obstacles_hit: int = 0
    collision_samples: int = 0
    near_miss_obstacles: int = 0
    near_miss_samples: int = 0
    min_clearance_m: float = math.nan
    first_collision_time_s: float = math.nan
    notes: str = ""


def _float(value: str | None, default: float = math.nan) -> float:
    if value is None or value == "":
        return default
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _read_csv_dicts(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def _arr(rows: list[dict[str, str]], key: str) -> np.ndarray:
    return np.asarray([_float(r.get(key)) for r in rows], dtype=float)


def _finite(values: np.ndarray) -> np.ndarray:
    return values[np.isfinite(values)]


def _path_x_end() -> float:
    path_csv = ROOT / "paths" / f"{PATH_NAME}.csv"
    rows = _read_csv_dicts(path_csv)
    xs = _finite(_arr(rows, "x"))
    return float(xs.max()) if len(xs) else 1.0


def parse_diag_csv(path: Path, speed: float, t_start: float) -> dict[str, float]:
    rows = _read_csv_dicts(path)
    if not rows:
        return {}

    t = _arr(rows, "sim_time")
    mask = np.isfinite(t) & (t >= t_start)
    if not mask.any():
        mask = np.isfinite(t)

    cte = _arr(rows, "crosstrack_err")
    heading_deg = _arr(rows, "heading_err_deg")
    x = _arr(rows, "x_fa_true")
    y = _arr(rows, "y_fa_true")
    psi = _arr(rows, "psi_true")
    x_ref = _arr(rows, "x_ref_0")
    y_ref = _arr(rows, "y_ref_0")
    psi_ref = _arr(rows, "psi_ref_0")
    u = _arr(rows, "u_true")
    v_ref = _arr(rows, "v_ref_0")
    solve_ms = _arr(rows, "solve_time_ms")

    cte_m = _finite(cte[mask])
    heading_m = _finite(heading_deg[mask])

    pos_err = np.sqrt((x - x_ref) ** 2 + (y - y_ref) ** 2)
    pos_m = _finite(pos_err[mask])

    psi_err = psi - psi_ref
    psi_err = (psi_err + np.pi) % (2.0 * np.pi) - np.pi
    psi_m = _finite(psi_err[mask])

    u_m = _finite(u[mask])
    v_ref_m = _finite(v_ref[mask])
    solve_m = _finite(solve_ms)

    statuses = [str(r.get("solver_status", "")).strip() for r in rows]
    success = sum(1 for s in statuses if s in ("0", "2"))

    x_finite = _finite(x)
    final_x = float(x_finite[-1]) if len(x_finite) else math.nan
    x_end = _path_x_end()

    out = {
        "avg_abs_cte_m": float(np.mean(np.abs(cte_m))) if len(cte_m) else math.nan,
        "rms_cte_m": float(np.sqrt(np.mean(cte_m ** 2))) if len(cte_m) else math.nan,
        "max_abs_cte_m": float(np.max(np.abs(cte_m))) if len(cte_m) else math.nan,
        "mean_ref_pos_err_m": float(np.mean(pos_m)) if len(pos_m) else math.nan,
        "rms_ref_pos_err_m": float(np.sqrt(np.mean(pos_m ** 2))) if len(pos_m) else math.nan,
        "rms_heading_deg": float(np.sqrt(np.mean(heading_m ** 2))) if len(heading_m) else math.nan,
        "mean_speed_mps": float(np.mean(u_m)) if len(u_m) else math.nan,
        "mean_solve_ms": float(np.mean(solve_m)) if len(solve_m) else math.nan,
        "solver_success_pct": 100.0 * success / max(len(statuses), 1),
        "final_x_m": final_x,
        "progress_frac": final_x / x_end if np.isfinite(final_x) and x_end > 0 else math.nan,
    }
    mean_v_ref = float(np.mean(v_ref_m)) if len(v_ref_m) else speed
    out["speed_ratio"] = out["mean_speed_mps"] / mean_v_ref if mean_v_ref > 1e-6 else math.nan
    if len(pos_m) and len(psi_m) == len(pos_m):
        out["rms_pose_err"] = float(np.sqrt(np.mean(pos_m ** 2 + psi_m ** 2)))
    return out


def parse_collision_csv(path: Path) -> dict[str, float | int]:
    if not path.exists():
        return {}
    rows = _read_csv_dicts(path)
    if not rows:
        return {
            "obstacles_hit": 0,
            "collision_samples": 0,
            "near_miss_obstacles": 0,
            "near_miss_samples": 0,
        }

    hit_ids: set[int] = set()
    near_ids: set[int] = set()
    collision_samples = 0
    near_miss_samples = 0
    clearances = []
    first_collision_time = math.nan

    for r in rows:
        rid = int(_float(r.get("rock_id"), -1))
        is_collision = int(_float(r.get("is_collision"), 0)) == 1
        is_near = int(_float(r.get("is_near_miss"), 0)) == 1
        d = _float(r.get("dist_2d"))
        hard = _float(r.get("hard_margin"))
        if np.isfinite(d) and np.isfinite(hard):
            clearances.append(d - hard)
        if is_collision:
            collision_samples += 1
            if rid >= 0:
                hit_ids.add(rid)
            if not np.isfinite(first_collision_time):
                first_collision_time = _float(r.get("time"))
        if is_near:
            near_miss_samples += 1
            if rid >= 0:
                near_ids.add(rid)

    return {
        "obstacles_hit": len(hit_ids),
        "collision_samples": collision_samples,
        "near_miss_obstacles": len(near_ids),
        "near_miss_samples": near_miss_samples,
        "min_clearance_m": float(np.min(clearances)) if clearances else math.nan,
        "first_collision_time_s": first_collision_time,
    }


def find_latest_diag(plot_dir: Path, created_after: float) -> Path | None:
    if not plot_dir.exists():
        return None
    candidates = [
        p for p in plot_dir.glob("**/diag_*.csv")
        if p.stat().st_mtime >= created_after - 1.0
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda p: p.stat().st_mtime)


def generate_rock_metadata(
    num_rocks: int,
    zone_x: tuple[float, float],
    zone_y: tuple[float, float],
    size_range: tuple[float, float],
    seed: int,
) -> list[dict[str, float]]:
    """Mirror sensors.obstacles.add_rock_obstacles without PyChrono objects."""
    rng = np.random.RandomState(seed)
    rocks: list[dict[str, float]] = []
    attempts = 0
    max_attempts = num_rocks * 10
    exclusions = [(0.0, 0.0, 12.0)]
    while len(rocks) < num_rocks and attempts < max_attempts:
        attempts += 1
        x = float(rng.uniform(zone_x[0], zone_x[1]))
        y = float(rng.uniform(zone_y[0], zone_y[1]))
        excluded = any((x - ex) ** 2 + (y - ey) ** 2 < er ** 2 for ex, ey, er in exclusions)
        if excluded:
            continue
        size = float(rng.uniform(size_range[0], size_range[1]))
        yaw = float(rng.uniform(0.0, 2.0 * np.pi))
        rocks.append({"x": x, "y": y, "r": 0.5 * size, "size": size, "yaw": yaw})
    return rocks


def run_one(job: dict[str, Any], args: argparse.Namespace, out_root: Path) -> RunResult:
    model = job["model"]
    round_idx = job["round_idx"]
    rock_seed = job["rock_seed"]
    run_plot_dir = out_root / "raw" / f"round_{round_idx:02d}" / model
    run_plot_dir.mkdir(parents=True, exist_ok=True)

    python_exe = str(CHRONO_PYTHON if CHRONO_PYTHON.exists() else Path(sys.executable))
    cmd = [
        python_exe,
        str(SIM_DIR / "launch_decoupled.py"),
        "--model", model,
        "--nn-model", args.nn_model,
        "--terrain", TERRAIN,
        "--path", PATH_NAME,
        "--speed", str(args.speed),
        "--time", str(args.time),
        "--lead-in", str(args.lead_in),
        "--rocks", str(args.rocks),
        "--rock-seed", str(rock_seed),
        "--rock-zone-x", str(args.rock_zone_x[0]), str(args.rock_zone_x[1]),
        "--rock-zone-y", str(args.rock_zone_y[0]), str(args.rock_zone_y[1]),
        "--rock-size", str(args.rock_size[0]), str(args.rock_size[1]),
        "--sim-port", str(job["sim_port"]),
        "--ctrl-port", str(job["ctrl_port"]),
        "--plot-dir", str(run_plot_dir),
        "--rms-time-start", str(args.metric_start),
        "--no-vis",
        "--no-plot",
    ]
    if args.no_noise:
        cmd.append("--no-noise")
    if args.no_imu:
        cmd.append("--no-imu")
    if args.no_path_reindex:
        cmd.append("--no-path-reindex")
    if args.safety_filter:
        cmd.append("--safety-filter")
    if args.dob_ki is not None:
        cmd += ["--dob-ki", str(args.dob_ki)]

    t0 = time.time()
    if args.dry_run:
        print(" ".join(cmd))
        return RunResult(
            model=model, round_idx=round_idx, rock_seed=rock_seed,
            status="dry_run", terrain=TERRAIN, path=PATH_NAME,
            speed_mps=args.speed, sim_time_s=args.time, n_rocks=args.rocks,
            wall_s=0.0,
        )

    if args.rocks > 0 and GLOBAL_COLLISION_LOG.exists():
        GLOBAL_COLLISION_LOG.unlink()

    print(f"[round {round_idx + 1}/{args.rounds}] {model} seed={rock_seed} "
          f"ports={job['sim_port']}/{job['ctrl_port']}", flush=True)
    try:
        proc = subprocess.run(
            cmd,
            cwd=str(SIM_DIR),
            timeout=args.timeout,
            capture_output=True,
            text=True,
        )
        wall_s = time.time() - t0
    except subprocess.TimeoutExpired:
        return RunResult(
            model=model, round_idx=round_idx, rock_seed=rock_seed,
            status="timeout", terrain=TERRAIN, path=PATH_NAME,
            speed_mps=args.speed, sim_time_s=args.time, n_rocks=args.rocks,
            wall_s=time.time() - t0,
        )

    status = "ok" if proc.returncode == 0 else f"exit_{proc.returncode}"
    diag = find_latest_diag(run_plot_dir, t0)
    run_dir = diag.parent if diag else run_plot_dir

    copied_collision = ""
    if (
        args.rocks > 0
        and GLOBAL_COLLISION_LOG.exists()
        and GLOBAL_COLLISION_LOG.stat().st_mtime >= t0 - 1.0
    ):
        dst = run_dir / f"collision_round{round_idx:02d}_{model}.csv"
        shutil.copy2(GLOBAL_COLLISION_LOG, dst)
        copied_collision = str(dst)

    result = RunResult(
        model=model,
        round_idx=round_idx,
        rock_seed=rock_seed,
        status=status,
        terrain=TERRAIN,
        path=PATH_NAME,
        speed_mps=args.speed,
        sim_time_s=args.time,
        n_rocks=args.rocks,
        wall_s=wall_s,
        run_dir=str(run_dir),
        diag_csv=str(diag) if diag else "",
        collision_csv=copied_collision,
    )

    if diag:
        for k, v in parse_diag_csv(diag, args.speed, args.metric_start).items():
            setattr(result, k, v)
    else:
        result.status = "no_diag" if status == "ok" else status

    if copied_collision:
        for k, v in parse_collision_csv(Path(copied_collision)).items():
            setattr(result, k, v)

    if result.status != "ok":
        stderr_tail = (proc.stderr or "")[-500:].replace("\n", " | ")
        stdout_tail = (proc.stdout or "")[-500:].replace("\n", " | ")
        result.notes = f"stderr_tail={stderr_tail}; stdout_tail={stdout_tail}"

    return result


def build_jobs(args: argparse.Namespace) -> list[dict[str, Any]]:
    jobs = []
    idx = 0
    for round_idx in range(args.rounds):
        rock_seed = args.seed + round_idx
        for model in args.models:
            jobs.append({
                "model": model,
                "round_idx": round_idx,
                "rock_seed": rock_seed,
                "sim_port": args.base_port + idx * 10,
                "ctrl_port": args.base_port + idx * 10 + 1,
            })
            idx += 1
    return jobs


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    fields = list(rows[0].keys())
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def summarize(results: list[RunResult]) -> list[dict[str, Any]]:
    rows = [asdict(r) for r in results if r.status == "ok"]
    metrics = [
        "obstacles_hit", "collision_samples", "near_miss_obstacles",
        "near_miss_samples", "min_clearance_m", "avg_abs_cte_m", "rms_cte_m",
        "max_abs_cte_m", "mean_ref_pos_err_m", "rms_ref_pos_err_m",
        "rms_heading_deg", "rms_pose_err", "mean_speed_mps", "speed_ratio",
        "solver_success_pct", "mean_solve_ms", "progress_frac",
    ]
    summary: list[dict[str, Any]] = []
    for model in sorted({r["model"] for r in rows}, key=lambda m: args_model_order(m)):
        sub = [r for r in rows if r["model"] == model]
        row: dict[str, Any] = {
            "model": model,
            "label": MODEL_LABELS.get(model, model),
            "n_ok": len(sub),
        }
        for metric in metrics:
            vals = np.asarray([_float(str(r.get(metric, ""))) for r in sub], dtype=float)
            vals = _finite(vals)
            row[f"{metric}_mean"] = float(np.mean(vals)) if len(vals) else math.nan
            row[f"{metric}_std"] = float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0
        summary.append(row)
    return summary


def args_model_order(model: str) -> int:
    try:
        return list(DEFAULT_MODELS).index(model)
    except ValueError:
        return 999


def _bar_with_std(ax, summary: list[dict[str, Any]], metric: str, ylabel: str, title: str) -> None:
    labels = [r["label"] for r in summary]
    means = [r.get(f"{metric}_mean", math.nan) for r in summary]
    stds = [r.get(f"{metric}_std", 0.0) for r in summary]
    colors = [MODEL_COLORS.get(r["model"], "#666666") for r in summary]
    x = np.arange(len(summary))
    ax.bar(x, means, yerr=stds, capsize=4, color=colors, alpha=0.86)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=20, ha="right")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(axis="y", alpha=0.3)


def plot_hit_counts(summary: list[dict[str, Any]], out_dir: Path) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6))
    _bar_with_std(axes[0], summary, "obstacles_hit", "Unique obstacles hit", "Obstacle Hits")
    _bar_with_std(axes[1], summary, "near_miss_obstacles", "Unique near-miss obstacles", "Near Misses")
    fig.suptitle("Clay right_left Obstacle Avoidance")
    fig.tight_layout()
    path = out_dir / "obstacle_hits.png"
    fig.savefig(path, dpi=220)
    plt.close(fig)


def plot_tracking_kpis(summary: list[dict[str, Any]], out_dir: Path) -> None:
    fig, axes = plt.subplots(2, 3, figsize=(15, 8.5))
    specs = [
        ("avg_abs_cte_m", "m", "Avg |CTE|"),
        ("rms_cte_m", "m", "RMS CTE"),
        ("rms_ref_pos_err_m", "m", "RMS Ref Position Error"),
        ("rms_heading_deg", "deg", "RMS Heading Error"),
        ("rms_pose_err", "m + rad", "RMS Pose Error"),
        ("speed_ratio", "u / u_ref", "Speed Ratio"),
    ]
    for ax, (metric, ylabel, title) in zip(axes.flat, specs):
        _bar_with_std(ax, summary, metric, ylabel, title)
    fig.suptitle("Tracking KPIs Averaged Across Rounds")
    fig.tight_layout()
    path = out_dir / "tracking_kpis.png"
    fig.savefig(path, dpi=220)
    plt.close(fig)


def plot_solver_kpis(summary: list[dict[str, Any]], out_dir: Path) -> None:
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5))
    specs = [
        ("solver_success_pct", "%", "Solver Success"),
        ("mean_solve_ms", "ms", "Mean Solve Time"),
        ("progress_frac", "final x / path x_max", "Path Progress"),
    ]
    for ax, (metric, ylabel, title) in zip(axes.flat, specs):
        _bar_with_std(ax, summary, metric, ylabel, title)
    fig.suptitle("Runtime and Completion KPIs")
    fig.tight_layout()
    path = out_dir / "runtime_kpis.png"
    fig.savefig(path, dpi=220)
    plt.close(fig)


def plot_representative_trajectory(
    results: list[RunResult],
    args: argparse.Namespace,
    out_dir: Path,
) -> None:
    ok = [r for r in results if r.status == "ok" and r.diag_csv]
    if not ok:
        return
    first_round = min(r.round_idx for r in ok)
    round_runs = [r for r in ok if r.round_idx == first_round]
    rock_seed = args.seed + first_round
    rocks = generate_rock_metadata(
        args.rocks,
        tuple(args.rock_zone_x),
        tuple(args.rock_zone_y),
        tuple(args.rock_size),
        rock_seed,
    )

    ref = _read_csv_dicts(ROOT / "paths" / f"{PATH_NAME}.csv")
    ref_x = _arr(ref, "x")
    ref_y = _arr(ref, "y")

    fig, ax = plt.subplots(figsize=(13, 8))
    ax.plot(ref_x, ref_y, "--", color="#111111", lw=2.0, label="Reference path")

    for i, rock in enumerate(rocks):
        rock_patch = Circle(
            (rock["x"], rock["y"]),
            rock["r"],
            facecolor="#7B4A2E",
            edgecolor="#2F1B12",
            alpha=0.8,
            linewidth=0.8,
        )
        ax.add_patch(rock_patch)
        margin_patch = Circle(
            (rock["x"], rock["y"]),
            rock["r"] + 1.5,
            facecolor="none",
            edgecolor="#7B4A2E",
            alpha=0.18,
            linestyle=":",
            linewidth=1.0,
        )
        ax.add_patch(margin_patch)
        if i == 0:
            rock_patch.set_label("Rock")
            margin_patch.set_label("Collision radius")

    for r in sorted(round_runs, key=lambda rr: args_model_order(rr.model)):
        rows = _read_csv_dicts(Path(r.diag_csv))
        x = _arr(rows, "x_fa_true")
        y = _arr(rows, "y_fa_true")
        ax.plot(
            x,
            y,
            lw=2.1,
            alpha=0.9,
            color=MODEL_COLORS.get(r.model, None),
            label=f"{MODEL_LABELS.get(r.model, r.model)} trajectory",
        )

    ax.set_title(f"Representative Trajectories, Round {first_round} (rock seed {rock_seed})")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    fig.tight_layout()
    path = out_dir / "representative_trajectory.png"
    fig.savefig(path, dpi=240)
    plt.close(fig)


def plot_time_history(results: list[RunResult], out_dir: Path) -> None:
    ok = [r for r in results if r.status == "ok" and r.diag_csv]
    if not ok:
        return
    first_round = min(r.round_idx for r in ok)
    round_runs = [r for r in ok if r.round_idx == first_round]

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    for r in sorted(round_runs, key=lambda rr: args_model_order(rr.model)):
        rows = _read_csv_dicts(Path(r.diag_csv))
        t = _arr(rows, "sim_time")
        label = MODEL_LABELS.get(r.model, r.model)
        color = MODEL_COLORS.get(r.model, None)
        axes[0].plot(t, np.abs(_arr(rows, "crosstrack_err")), color=color, label=label)
        axes[1].plot(t, np.abs(_arr(rows, "heading_err_deg")), color=color, label=label)
        axes[2].plot(t, _arr(rows, "u_true"), color=color, label=label)

    axes[0].set_ylabel("|CTE| (m)")
    axes[1].set_ylabel("|Heading| (deg)")
    axes[2].set_ylabel("Speed (m/s)")
    axes[2].set_xlabel("Time (s)")
    for ax in axes:
        ax.grid(True, alpha=0.3)
    axes[0].legend(loc="best")
    fig.suptitle(f"Representative Round Time Histories (round {first_round})")
    fig.tight_layout()
    path = out_dir / "representative_time_history.png"
    fig.savefig(path, dpi=220)
    plt.close(fig)


def print_summary(summary: list[dict[str, Any]]) -> None:
    if not summary:
        print("No successful runs to summarize.")
        return
    print("\nSummary (mean across successful rounds):")
    header = (
        f"{'Model':<16} {'n':>3} {'hits':>7} {'near':>7} "
        f"{'avg|CTE|':>9} {'pose':>8} {'hdg':>8} {'spd':>6} {'solv%':>7}"
    )
    print(header)
    print("-" * len(header))
    for r in summary:
        print(
            f"{r['label']:<16} {int(r['n_ok']):>3d} "
            f"{r['obstacles_hit_mean']:>7.2f} "
            f"{r['near_miss_obstacles_mean']:>7.2f} "
            f"{r['avg_abs_cte_m_mean']:>9.3f} "
            f"{r['rms_pose_err_mean']:>8.3f} "
            f"{r['rms_heading_deg_mean']:>8.2f} "
            f"{r['speed_ratio_mean']:>6.2f} "
            f"{r['solver_success_pct_mean']:>7.1f}"
        )


def main() -> None:
    p = argparse.ArgumentParser(
        description="Benchmark tire-model obstacle avoidance on clay/right_left.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--rounds", type=int, default=3,
                   help="Rounds per model; each round uses a new rock seed shared by all models.")
    p.add_argument("--models", nargs="+", default=list(DEFAULT_MODELS),
                   choices=list(DEFAULT_MODELS),
                   help="MPC tire models to benchmark.")
    p.add_argument("--nn-model", default="paper_v2_mlp_16_4",
                   help="NN tire-model checkpoint name when --model nn is included.")
    p.add_argument("--speed", type=float, default=5.0)
    p.add_argument("--time", type=float, default=30.0)
    p.add_argument("--lead-in", type=float, default=10.0)
    p.add_argument("--metric-start", type=float, default=2.0,
                   help="Ignore early transient before this sim time for tracking KPIs.")
    p.add_argument("--rocks", type=int, default=10)
    p.add_argument("--rock-zone-x", type=float, nargs=2, default=[12.0, 68.0])
    p.add_argument("--rock-zone-y", type=float, nargs=2, default=[-6.0, 6.0])
    p.add_argument("--rock-size", type=float, nargs=2, default=[0.8, 1.8])
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--base-port", type=int, default=17600)
    p.add_argument("--timeout", type=float, default=600.0)
    p.add_argument("--out-dir", type=Path, default=None)
    p.add_argument("--no-noise", action="store_true",
                   help="Disable simulator measurement noise for deterministic runs.")
    p.add_argument("--no-imu", action="store_true",
                   help="Use analytical accel/gyro instead of Chrono sensor IMU.")
    p.add_argument("--no-path-reindex", action="store_true")
    p.add_argument("--safety-filter", action="store_true",
                   help="Enable sim-side DOB-CBF safety filter; off by default to isolate MPC tire model behavior.")
    p.add_argument("--dob-ki", type=float, default=None,
                   help="Override controller throttle DOB gain.")
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_root = args.out_dir or (DEFAULT_OUT_ROOT / timestamp)
    out_root.mkdir(parents=True, exist_ok=True)

    jobs = build_jobs(args)
    print("Obstacle-avoidance tire benchmark")
    print(f"  terrain/path: {TERRAIN}/{PATH_NAME}")
    print(f"  models: {', '.join(args.models)}")
    print(f"  rounds: {args.rounds}, rocks: {args.rocks}, speed: {args.speed:g} m/s")
    print(f"  output: {out_root}")
    print(f"  jobs: {len(jobs)}")

    results: list[RunResult] = []
    t0 = time.time()
    for job in jobs:
        result = run_one(job, args, out_root)
        results.append(result)
        if result.status == "ok":
            print(
                f"  OK {result.model} r{result.round_idx}: "
                f"hits={result.obstacles_hit}, near={result.near_miss_obstacles}, "
                f"avg|CTE|={result.avg_abs_cte_m:.3f}m, "
                f"pose={result.rms_pose_err:.3f}, hdg={result.rms_heading_deg:.2f}deg",
                flush=True,
            )
        else:
            print(f"  {result.status.upper()} {result.model} r{result.round_idx}", flush=True)

    result_rows = [asdict(r) for r in results]
    write_csv(out_root / "per_run_results.csv", result_rows)
    with (out_root / "per_run_results.json").open("w") as f:
        json.dump(result_rows, f, indent=2)

    summary = summarize(results)
    write_csv(out_root / "summary_by_model.csv", summary)
    with (out_root / "summary_by_model.json").open("w") as f:
        json.dump(summary, f, indent=2)

    if not args.dry_run:
        plot_hit_counts(summary, out_root)
        plot_tracking_kpis(summary, out_root)
        plot_solver_kpis(summary, out_root)
        plot_representative_trajectory(results, args, out_root)
        plot_time_history(results, out_root)

    print_summary(summary)
    print(f"\nCompleted in {(time.time() - t0) / 60.0:.1f} min")
    print(f"Results written to: {out_root}")


if __name__ == "__main__":
    main()
