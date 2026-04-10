#!/usr/bin/env python3
"""
Run MPC benchmarks for NN tire models on the same path/terrain.

Launches sim + controller for each model, collects diagnostic summaries
from the generated CSV files, and writes simulation/mpc_benchmark_results.json.

Features:
  - Optional discovery of all loadable checkpoints under nn_models/
  - Flags runs that never progress along the path (brake-only / parked)
  - Parallel runs across CPU cores (unique ZMQ ports + per-model plot dirs)

The JSON file is overwritten with exactly the models selected for this run.

Usage:
    python run_mpc_benchmarks.py -j 4
    python run_mpc_benchmarks.py --discover -j 8
    python run_mpc_benchmarks.py --models mlp_16_4
"""

from __future__ import annotations

import argparse
import concurrent.futures
import csv
import json
import math
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

UTIL_DIR = Path(__file__).parent
PROJECT_ROOT = UTIL_DIR.parent
SIM_DIR = PROJECT_ROOT / "simulation"
PLOTS_DIR = SIM_DIR / "plots"
NN_MODELS_ROOT = PROJECT_ROOT / "nn_models"
BENCH_PLOT_ROOT = PLOTS_DIR / "mpc_benchmark"

ALL_MODELS = {
    # Static MLP
    "mlp_12_2":             "paper_v1_mlp_12_2",
    "mlp_16_4":             "paper_v1_mlp_16_4",
    "mlp_16_8":             "paper_v1_mlp_16_8",
    "mlp_24_12":            "paper_v1_mlp_24_12",
    "mlp_32_16":            "paper_v1_mlp_32_16",
    # Static ResNet
    "resnet_h8_b2":         "paper_v1_resnet_h8_b2",
    "resnet_h16_b2":        "paper_v1_resnet_h16_b2",
    "resnet_h16_b4":        "paper_v1_resnet_h16_b4",
    "resnet_h32_b2":        "paper_v1_resnet_h32_b2",
    # Rate MLP
    "rate_mlp_16_8":        "paper_v1_mlp_rate_16_8",
    "rate_mlp_24_12":       "paper_v1_mlp_rate_24_12",
    # Rate ResNet
    "rate_resnet_h16_b2":   "paper_v1_resnet_rate_h16_b2",
    "rate_resnet_h32_b2":   "paper_v1_resnet_rate_h32_b2",
    # Temporal MLP
    "temp_K3_mlp_16_8":     "paper_v1_mlp_temporal_K3_16_8",
    "temp_K3_mlp_24_12":    "paper_v1_mlp_temporal_K3_24_12",
    "temp_K5_mlp_16_8":     "paper_v1_mlp_temporal_K5_16_8",
    "temp_K5_mlp_24_12":    "paper_v1_mlp_temporal_K5_24_12",
    "temp_K10_mlp_16_8":    "paper_v1_mlp_temporal_K10_16_8",
    "temp_K10_mlp_24_12":   "paper_v1_mlp_temporal_K10_24_12",
    # Temporal ResNet
    "temp_K3_resnet_h16":   "paper_v1_resnet_temporal_K3_h16_b2",
    "temp_K3_resnet_h32":   "paper_v1_resnet_temporal_K3_h32_b2",
    "temp_K5_resnet_h16":   "paper_v1_resnet_temporal_K5_h16_b2",
    "temp_K5_resnet_h32":   "paper_v1_resnet_temporal_K5_h32_b2",
    "temp_K10_resnet_h16":  "paper_v1_resnet_temporal_K10_h16_b2",
    "temp_K10_resnet_h32":  "paper_v1_resnet_temporal_K10_h32_b2",
    # MPC-oriented retrain (static-teacher anchor + timing fixes in controller)
    "mpc_anchor_temp_K5_16_8": "mpc_anchor_temporal_K5_16_8",
    "mpc_anchor_rate_16_8": "mpc_anchor_rate_mlp_16_8",
}

COMMON = dict(
    path="double_lane_change",
    terrain="clay",
    time="30",
    speed="5.0",
    lead_in="10.0",
    sine_wavelength="30.0",
)


def reference_path_x_end(
    path_type: str,
    lead_in: float = 0.0,
    sine_wavelength: float = 30.0,
) -> float:
    """Total path extent in x (m), matching reference_path.generate_path_waypoints."""
    if path_type == "lane_change":
        return 25.0 + lead_in + 40.0
    if path_type == "double_lane_change":
        return 38.0 + lead_in + 40.0
    if path_type == "sinusoidal":
        return lead_in + 5.0 * sine_wavelength
    return 200.0


def discover_nn_models(root: Path | None = None) -> list[tuple[str, str]]:
    """Return [(label, nn_model_relpath), ...] for each loadable checkpoint under root."""
    root = root or NN_MODELS_ROOT
    if not root.is_dir():
        return []
    found: list[tuple[str, str]] = []
    for d in sorted(root.iterdir()):
        if not d.is_dir():
            continue
        if (d / "best_terrain_nn.pt").is_file() and (d / "scalers.pkl").is_file():
            name = d.name
            found.append((name, name))
    return found


def _safe_plot_segment(name: str) -> str:
    return re.sub(r"[^a-zA-Z0-9_.-]+", "_", name)[:120]


def find_latest_run_dir(
    created_after: float,
    search_root: Path,
    terrain: str,
    path_type: str,
) -> Path | None:
    """Newest run directory under search_root matching terrain + path in the folder name."""
    if not search_root.is_dir():
        return None
    candidates = sorted(
        [
            d
            for d in search_root.iterdir()
            if d.is_dir()
            and terrain in d.name
            and path_type in d.name
            and d.stat().st_mtime > created_after
        ],
        key=lambda d: d.stat().st_mtime,
        reverse=True,
    )
    return candidates[0] if candidates else None


def _float_col(rows, key: str) -> np.ndarray:
    """Parse a column to float; missing or empty → NaN."""
    out = []
    for r in rows:
        v = r.get(key, "")
        if v is None or v == "":
            out.append(np.nan)
            continue
        try:
            out.append(float(v))
        except ValueError:
            out.append(np.nan)
    return np.asarray(out, dtype=float)


def _stall_metrics(
    rows: list,
    metrics: dict,
    path_type: str,
    lead_in: float,
    sine_wavelength: float,
) -> dict:
    """Detect brake-only / no meaningful progress along the reference x extent."""
    x_end = reference_path_x_end(path_type, lead_in, sine_wavelength)
    sim_times = np.array([float(r.get("sim_time", 0) or 0) for r in rows])

    x_true = _float_col(rows, "x_fa_true")
    x_meas = _float_col(rows, "x_fa_meas")
    if np.any(np.isfinite(x_true)):
        x_track = x_true
    else:
        x_track = x_meas

    if len(x_track) == 0 or not np.any(np.isfinite(x_track)):
        final_x = 0.0
    else:
        final_x = float(x_track[-1]) if np.isfinite(x_track[-1]) else float(np.nanmax(x_track))

    last_t = float(sim_times[-1]) if len(sim_times) else 0.0
    progress = (final_x / x_end) if x_end > 1e-6 else 0.0

    usr = metrics.get("mean_speed_ratio")
    bd = metrics.get("brake_duty_pct")

    reasons: list[str] = []
    stalled = False

    # Primary: sim ran long enough but vehicle never reached mid-path
    if last_t >= 8.0 and progress < 0.48:
        stalled = True
        reasons.append(f"low_progress({progress:.0%} of {x_end:.0f}m)")

    if last_t >= 10.0 and usr is not None and math.isfinite(usr) and usr < 0.16:
        stalled = True
        reasons.append(f"low_speed_ratio({usr:.2f})")

    if (
        bd is not None
        and math.isfinite(bd)
        and bd > 90.0
        and (usr is None or not math.isfinite(usr) or usr < 0.30)
    ):
        stalled = True
        reasons.append(f"heavy_brake({bd:.0f}%)")

    # Dedupe reasons while preserving order
    seen = set()
    uniq = []
    for r in reasons:
        if r not in seen:
            seen.add(r)
            uniq.append(r)

    return {
        "final_x_m": final_x,
        "path_x_end_m": x_end,
        "progress_frac": progress,
        "stalled": stalled,
        "stall_reason": "; ".join(uniq),
    }


def parse_diagnostic_csv(csv_path: Path, common: dict) -> dict:
    """Parse a diagnostic CSV and compute benchmark metrics."""
    rows: list = []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)

    if not rows:
        return {}

    lead_in = float(common.get("lead_in", "10.0"))
    sine_wl = float(common.get("sine_wavelength", "30.0"))
    path_type = common["path"]

    cte = np.array([float(r.get("crosstrack_err", 0)) for r in rows])
    solver_status = [r.get("solver_status", "") for r in rows]
    solve_times = np.array(
        [float(r.get("solve_time_ms", 0)) / 1000.0 for r in rows if r.get("solve_time_ms")]
    )
    steering = np.array([float(r.get("steering_angle", 0)) for r in rows])
    sim_times = np.array([float(r.get("sim_time", 0)) for r in rows])

    mask = sim_times >= 2.0
    cte_m = cte[mask] if mask.any() else cte
    steer_m = steering[mask] if mask.any() else steering

    total = len(solver_status)
    success_count = sum(1 for s in solver_status if s == "0")
    maxiter_count = sum(1 for s in solver_status if s == "2")
    minstep_count = sum(1 for s in solver_status if s == "3")

    n_final = max(1, len(steer_m) // 5)
    steer_final = steer_m[-n_final:]
    steer_osc = np.degrees(np.std(steer_final)) if len(steer_final) > 1 else 0.0

    metrics: dict = {
        "n_solves": total,
        "avg_cte_m": float(np.mean(np.abs(cte_m))),
        "max_cte_m": float(np.max(np.abs(cte_m))),
        "rms_cte_m": float(np.sqrt(np.mean(cte_m**2))),
        "success_pct": 100.0 * success_count / max(total, 1),
        "maxiter_pct": 100.0 * maxiter_count / max(total, 1),
        "minstep_pct": 100.0 * minstep_count / max(total, 1),
        "steer_osc_deg": round(float(steer_osc), 2),
    }

    if len(solve_times) > 0:
        metrics["mean_solve_ms"] = float(np.mean(solve_times) * 1000)
        metrics["mpc_hz"] = float(1.0 / np.mean(solve_times))

    path_pos = _float_col(rows, "path_pos_err_m")
    path_lat = _float_col(rows, "path_lat_err_m")
    path_lon = _float_col(rows, "path_lon_err_m")
    hpath_deg = _float_col(rows, "heading_path_err_deg")
    u_true = _float_col(rows, "u_true")
    v_ref0 = _float_col(rows, "v_ref_0")
    braking = _float_col(rows, "braking")

    if mask.any():
        path_pos_m = path_pos[mask]
        path_lat_m = path_lat[mask]
        path_lon_m = path_lon[mask]
        hpath_m = hpath_deg[mask]
        u_m = u_true[mask]
        br_m = braking[mask]
        vref = float(v_ref0[0]) if len(v_ref0) and np.isfinite(v_ref0[0]) else float("nan")
    else:
        path_pos_m = path_pos
        path_lat_m = path_lat
        path_lon_m = path_lon
        hpath_m = hpath_deg
        u_m = u_true
        br_m = braking
        vref = float(v_ref0[0]) if len(v_ref0) and np.isfinite(v_ref0[0]) else float("nan")

    if np.any(np.isfinite(path_pos_m)):
        metrics["avg_path_pos_err_m"] = float(np.nanmean(path_pos_m))
        metrics["rms_path_pos_m"] = float(np.sqrt(np.nanmean(path_pos_m**2)))
        metrics["rms_path_lat_m"] = float(np.sqrt(np.nanmean(path_lat_m**2)))
        metrics["rms_path_lon_m"] = float(np.sqrt(np.nanmean(path_lon_m**2)))
        hpath_rad = np.radians(hpath_m)
        ok_pose = np.isfinite(path_pos_m) & np.isfinite(hpath_rad)
        if np.any(ok_pose):
            pr = path_pos_m[ok_pose]
            hr = hpath_rad[ok_pose]
            metrics["rms_pose_m"] = float(np.sqrt(np.mean(pr**2 + hr**2)))
        metrics["rms_heading_path_deg"] = float(np.sqrt(np.nanmean(hpath_m**2)))
    if np.any(np.isfinite(u_m)):
        mu = float(np.nanmean(u_m))
        metrics["mean_u_mps"] = mu
        if np.isfinite(vref) and vref > 1e-6:
            metrics["mean_speed_ratio"] = float(mu / vref)
    if np.any(np.isfinite(br_m)):
        metrics["brake_duty_pct"] = float(100.0 * np.nanmean(br_m > 0.15))

    metrics.update(_stall_metrics(rows, metrics, path_type, lead_in, sine_wl))
    return metrics


def run_one(job: dict) -> dict:
    """Run one benchmark: launch sim+controller, wait, parse CSV."""
    label = job["label"]
    nn_model = job["nn_model"]
    idx = job["idx"]
    total = job["total"]
    sim_port = job["sim_port"]
    ctrl_port = job["ctrl_port"]
    plot_parent = Path(job["plot_parent"])
    common = job["common"]

    print(f"\n{'='*70}")
    print(f"[{idx+1}/{total}] {label} → {nn_model}  (ports {sim_port}/{ctrl_port})")
    print(f"{'='*70}")

    plot_parent.mkdir(parents=True, exist_ok=True)

    cmd = [
        sys.executable,
        str(SIM_DIR / "launch_decoupled.py"),
        "--model",
        "nn",
        "--nn-model",
        nn_model,
        "--path",
        common["path"],
        "--terrain",
        common["terrain"],
        "--time",
        common["time"],
        "--speed",
        common["speed"],
        "--lead-in",
        common.get("lead_in", "10.0"),
        "--sine-wavelength",
        common.get("sine_wavelength", "30.0"),
        "--no-vis",
        "--sim-port",
        str(sim_port),
        "--ctrl-port",
        str(ctrl_port),
        "--plot-dir",
        str(plot_parent),
    ]

    t0 = time.time()
    try:
        proc = subprocess.run(cmd, timeout=420, cwd=str(SIM_DIR))
        wall = time.time() - t0
    except subprocess.TimeoutExpired:
        print(f"  TIMEOUT after 420s!")
        return {
            "label": label,
            "nn_model": nn_model,
            "status": "timeout",
            "idx": idx,
        }

    print(f"  Completed in {wall:.1f}s (exit {proc.returncode})")

    run_dir = find_latest_run_dir(
        created_after=t0,
        search_root=plot_parent,
        terrain=common["terrain"],
        path_type=common["path"],
    )
    if not run_dir:
        print(f"  WARNING: No plot directory found under {plot_parent}")
        return {
            "label": label,
            "nn_model": nn_model,
            "status": "no_output",
            "idx": idx,
        }

    csv_files = list(run_dir.glob("diag_*.csv"))
    if not csv_files:
        print(f"  WARNING: No diagnostic CSV in {run_dir}")
        return {
            "label": label,
            "nn_model": nn_model,
            "status": "no_csv",
            "idx": idx,
        }

    metrics = parse_diagnostic_csv(csv_files[0], common)
    metrics["label"] = label
    metrics["nn_model"] = nn_model
    metrics["wall_time_s"] = round(wall, 1)
    metrics["run_dir"] = str(run_dir)
    metrics["status"] = "ok"
    metrics["idx"] = idx

    if metrics.get("stalled"):
        print(f"  *** STALLED: {metrics.get('stall_reason', '')} ***")

    avg = metrics.get("avg_cte_m")
    hz = metrics.get("mpc_hz")
    suc = metrics.get("success_pct")
    osc = metrics.get("steer_osc_deg")
    app = metrics.get("avg_path_pos_err_m")
    pose = metrics.get("rms_pose_m")
    usr = metrics.get("mean_speed_ratio")
    prog = metrics.get("progress_frac")
    print(f"  Avg CTE: {avg:.4f}m  " if avg is not None else "  Avg CTE: ?  ", end="")
    print(f"pathPos: {app:.3f}m  " if app is not None else "pathPos: ?  ", end="")
    print(f"poseRMS: {pose:.3f}m  " if pose is not None else "poseRMS: ?  ", end="")
    print(f"u/u_ref: {usr:.2f}  " if usr is not None else "u/u_ref: ?  ", end="")
    print(f"prog: {prog:.0%}  " if prog is not None else "prog: ?  ", end="")
    print(f"MPC Hz: {hz:.0f}  " if hz else "MPC Hz: ?  ", end="")
    print(f"Success: {suc:.1f}%  " if suc is not None else "Success: ?  ", end="")
    print(f"Steer osc: {osc:.1f}°" if osc is not None else "Steer osc: ?")

    return metrics


def _run_one_worker(job: dict) -> dict:
    """Top-level wrapper for ProcessPoolExecutor (must be picklable)."""
    return run_one(job)


def print_summary(results: list):
    """Print formatted comparison table."""
    common = COMMON
    print(f"\n\n{'='*130}")
    print("MPC BENCHMARK COMPARISON — Architecture Classes")
    print(
        f"Path: {common['path']}, Terrain: {common['terrain']}, "
        f"Speed: {common['speed']} m/s, Duration: {common['time']}s"
    )
    print(f"{'='*130}")

    fmt = (
        "{:<20s} {:>6s} {:>7s} {:>7s} {:>6s} {:>6s} {:>7s} "
        "{:>6s} {:>6s} {:>6s} {:>5s} {:>5s}"
    )
    print(
        fmt.format(
            "Model",
            "CTE",
            "pathPos",
            "poseRMS",
            "u/ref",
            "prog",
            "MPCHz",
            "Succ%",
            "MaxIt%",
            "Osc°",
            "Time",
            "Stall",
        )
    )
    print("-" * 130)

    for r in sorted(results, key=lambda x: x.get("idx", 0)):
        if r["status"] != "ok":
            print(f"{r['label']:<20s}  *** {r['status']} ***")
            continue

        def fmtn(key, nd=3, default="-"):
            v = r.get(key)
            if v is None or (isinstance(v, float) and not np.isfinite(v)):
                return default
            return f"{v:.{nd}f}"

        stall = "YES" if r.get("stalled") else "no"
        prog_s = f"{r.get('progress_frac', 0):.0%}" if r.get("progress_frac") is not None else "-"

        print(
            fmt.format(
                r["label"][:20],
                f"{r['avg_cte_m']:.3f}",
                fmtn("avg_path_pos_err_m"),
                fmtn("rms_pose_m"),
                fmtn("mean_speed_ratio", 2),
                prog_s,
                f"{r.get('mpc_hz', 0):.0f}",
                f"{r['success_pct']:.0f}",
                f"{r['maxiter_pct']:.0f}",
                f"{r['steer_osc_deg']:.0f}",
                f"{r['wall_time_s']:.0f}s",
                stall,
            )
        )
    print(f"{'='*130}")
    print(
        "CTE = legacy |y−y_ref(x)|; pathPos / poseRMS when columns exist; "
        "u/ref = mean speed / v_ref; prog = final x / path length; "
        "Stall = low progress, low speed ratio, or sustained braking."
    )


def main():
    p = argparse.ArgumentParser(description="MPC NN tire model benchmarks")
    p.add_argument(
        "--models",
        nargs="+",
        default=None,
        help="Keys from built-in paper_v1 map (default: all 25, or all discovered)",
    )
    p.add_argument(
        "--discover",
        action="store_true",
        help=f"Benchmark every subdirectory of {NN_MODELS_ROOT} with "
        "best_terrain_nn.pt + scalers.pkl (ignores --models default)",
    )
    p.add_argument(
        "--discover-prefix",
        default=None,
        metavar="PREFIX",
        help="With --discover: only directories whose name starts with PREFIX (e.g. paper_v2_)",
    )
    p.add_argument(
        "-j",
        "--jobs",
        type=int,
        default=max(1, min(8, (os.cpu_count() or 4))),
        help="Parallel worker processes (default: min(8, CPU count))",
    )
    p.add_argument(
        "--results-json",
        type=Path,
        default=None,
        help="Write benchmark JSON here (default: simulation/mpc_benchmark_results.json)",
    )
    args = p.parse_args()

    if args.discover:
        discovered = discover_nn_models()
        if args.discover_prefix:
            pfx = args.discover_prefix
            discovered = [(a, b) for a, b in discovered if str(b).startswith(pfx)]
            print(f"After prefix filter {pfx!r}: {len(discovered)} model(s)")
        if not discovered:
            print(f"No loadable models found under {NN_MODELS_ROOT}", file=sys.stderr)
            sys.exit(1)
        models = discovered
        print(f"Discovered {len(models)} model(s) under {NN_MODELS_ROOT.name}/")
    else:
        keys = args.models if args.models is not None else list(ALL_MODELS.keys())
        unknown = [k for k in keys if k not in ALL_MODELS]
        if unknown:
            print(f"Unknown model keys: {unknown}", file=sys.stderr)
            sys.exit(1)
        models = [(k, ALL_MODELS[k]) for k in keys]

    n = len(models)
    jobs = []
    for i, (label, nn_model) in enumerate(models):
        plot_seg = _safe_plot_segment(nn_model)
        plot_parent = BENCH_PLOT_ROOT / plot_seg
        jobs.append(
            {
                "label": label,
                "nn_model": nn_model,
                "idx": i,
                "total": n,
                "sim_port": 5600 + i * 10,
                "ctrl_port": 5600 + i * 10 + 1,
                "plot_parent": str(plot_parent.resolve()),
                "common": dict(COMMON),
            }
        )

    BENCH_PLOT_ROOT.mkdir(parents=True, exist_ok=True)

    results: list = []
    if args.jobs <= 1:
        for job in jobs:
            results.append(run_one(job))
    else:
        print(f"Running {n} benchmark(s) with {args.jobs} parallel workers…")
        with concurrent.futures.ProcessPoolExecutor(max_workers=args.jobs) as ex:
            futs = {ex.submit(_run_one_worker, job): job for job in jobs}
            for fut in concurrent.futures.as_completed(futs):
                try:
                    results.append(fut.result())
                except Exception as e:
                    job = futs[fut]
                    print(f"\n*** Worker crash for {job['label']}: {e} ***")
                    results.append(
                        {
                            "label": job["label"],
                            "nn_model": job["nn_model"],
                            "status": f"worker_error: {e}",
                            "idx": job["idx"],
                        }
                    )

    print_summary(results)

    out_path = (args.results_json if args.results_json is not None else UTIL_DIR / "mpc_benchmark_results.json")
    out_path = out_path.resolve()

    def _json_safe(obj):
        if isinstance(obj, dict):
            return {k: _json_safe(v) for k, v in obj.items()}
        if isinstance(obj, list):
            return [_json_safe(v) for v in obj]
        if isinstance(obj, float):
            if not math.isfinite(obj):
                return None
        if isinstance(obj, (np.floating,)):
            x = float(obj)
            return None if not math.isfinite(x) else x
        if isinstance(obj, (np.integer,)):
            return int(obj)
        return obj

    with open(out_path, "w") as f:
        json.dump(_json_safe(results), f, indent=2)
    print(f"\nResults saved to {out_path} ({len(results)} row(s))")


if __name__ == "__main__":
    main()
