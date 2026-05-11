#!/usr/bin/env python3
"""
Batch runner: simulate every (terrain × path × model) combo, collect
diagnostic CSVs, and generate comparison plots — including an NN vs
Pacejka accuracy grid.  Runs multiple simulations in parallel by
assigning each worker a unique ZMQ port pair so they don't collide.

Usage:
    # Default: 3 terrains × 3 paths × 2 models, 3 workers
    python new_diagnostics/run_all_comparisons.py

    # NN only, 3 runs per combo, 4 parallel workers
    python new_diagnostics/run_all_comparisons.py --models nn --runs 3 --workers 4

    # Quick smoke-test
    python new_diagnostics/run_all_comparisons.py \\
        --terrains clay --paths lane_change --runs 1 --workers 1 --time 8
"""

import argparse
import queue
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

TERRAINS = ["clay", "sand", "dirt"]
PATHS    = ["lane_change", "double_lane_change", "sinusoidal"]
MODELS   = ["nn", "linear"]   # "linear" = Pacejka Magic Formula

DIAG_DIR   = Path(__file__).resolve().parent
SCM_TELEOP = DIAG_DIR.parent
SIM_DIR    = SCM_TELEOP / "simulation"
SIM_SCRIPT = SIM_DIR / "launch_decoupled.py"
PLOTS_DIR  = SIM_DIR / "plots"

# Each worker slot i uses ports  BASE+2i  and  BASE+2i+1
BASE_SIM_PORT  = 5560
BASE_CTRL_PORT = 5561

_print_lock = threading.Lock()

def tprint(*a, **kw):
    with _print_lock:
        print(*a, **kw)


# =============================================================================
# Data helpers
# =============================================================================

def find_latest_csv(terrain: str, path_type: str,
                    model_tag: str = "nn") -> Path | None:
    pattern = (f"*_{terrain}_{path_type}_{model_tag}/"
               f"diag_{terrain}_{path_type}_{model_tag}.csv")
    matches = sorted(PLOTS_DIR.glob(pattern), key=lambda p: p.stat().st_mtime)
    return matches[-1] if matches else None


def load_csv(csv_path: Path) -> dict:
    import csv as _csv
    with open(csv_path) as f:
        reader = _csv.DictReader(f)
        rows = list(reader)
    if not rows:
        return {}
    # Drop incomplete rows (DictReader sets missing fields to None)
    n_cols = len(rows[0])
    rows = [r for r in rows if None not in r.values() and len(r) == n_cols]
    if not rows:
        return {}
    data = {}
    for k in rows[0].keys():
        try:
            data[k] = np.array([float(r[k]) for r in rows])
        except (ValueError, KeyError):
            data[k] = np.array([r.get(k, "") for r in rows])
    return data


def load_ref_path_csv(csv_path: Path) -> tuple | None:
    """Return (x, y) arrays from a saved reference_path_*.csv, or None."""
    try:
        d = np.loadtxt(csv_path, delimiter=",", skiprows=1)
        return d[:, 1], d[:, 2]   # columns: s, x, y, psi
    except Exception:
        return None


CRASH_CTE_THRESHOLD    = 4.0   # metres — RMS above this = vehicle left the track
CRASH_ROW_FRACTION     = 0.50  # runs with fewer than this fraction of expected rows = crashed
CONVERGED_CTE_THRESH   = 0.40  # metres — vehicle is "on path" once |CTE| drops below this
CONVERGED_MIN_DURATION = 2.0   # seconds — must stay on path for this long to count as converged


def compute_stats(data: dict, sim_time: float = 15.0, ctrl_hz: float = 15.0) -> dict:
    """
    Return a rich stats dict that separates three phases:

    convergence_time : seconds until the vehicle first enters and stays within
                       CONVERGED_CTE_THRESH for CONVERGED_MIN_DURATION seconds.
                       nan  → never converged (complete failure).

    steady_rms_cte   : RMS CTE computed only AFTER convergence.
                       Measures how well the controller actually tracks once
                       the initial transient is over.  Comparable between NN
                       and Pacejka even when their convergence times differ.
                       nan  → never converged.

    on_path_fraction : fraction of simulation time (after t=2s) where
                       |CTE| < CONVERGED_CTE_THRESH.  Captures how often the
                       vehicle is "usefully on the path" across the whole run.

    converged        : bool — did the controller ever achieve steady-state tracking?
    crashed          : bool — did the vehicle leave the track entirely?
    """
    if "crosstrack_err" not in data or len(data["crosstrack_err"]) == 0:
        return {}

    t_all  = data["sim_time"]
    ct_all = data["crosstrack_err"]
    sp_all = data.get("speed_err", np.array([]))

    # Restrict to post-warmup (t > 2s)
    mask   = t_all > 2.0
    t      = t_all[mask]
    ct     = ct_all[mask]
    sp     = sp_all[mask] if len(sp_all) == len(t_all) else np.array([])
    n_rows = int(mask.sum())

    if n_rows == 0:
        return {}

    # ── crash detection ────────────────────────────────────────────────────────
    expected_min = int(max(1, sim_time * ctrl_hz * CRASH_ROW_FRACTION))
    overall_rms  = float(np.sqrt(np.mean(ct**2)))
    crashed      = (overall_rms > CRASH_CTE_THRESHOLD) or (n_rows < expected_min)

    # ── convergence time ───────────────────────────────────────────────────────
    # Find the first time the vehicle gets within threshold AND stays there for
    # at least CONVERGED_MIN_DURATION seconds.
    on_path    = np.abs(ct) < CONVERGED_CTE_THRESH
    dt_approx  = float(np.median(np.diff(t))) if len(t) > 1 else 0.1
    window     = max(1, int(CONVERGED_MIN_DURATION / dt_approx))

    convergence_time = float("nan")
    steady_start_idx = None
    for i in range(len(on_path) - window + 1):
        if np.all(on_path[i: i + window]):
            convergence_time = float(t[i])
            steady_start_idx = i
            break

    # ── steady-state RMS (only after convergence) ──────────────────────────────
    if steady_start_idx is not None:
        ct_ss        = ct[steady_start_idx:]
        steady_rms   = float(np.sqrt(np.mean(ct_ss**2)))
        steady_avg   = float(np.mean(np.abs(ct_ss)))
        steady_max   = float(np.max(np.abs(ct_ss)))
    else:
        steady_rms = steady_avg = steady_max = float("nan")

    # ── on-path fraction ───────────────────────────────────────────────────────
    on_path_fraction = float(np.mean(on_path))

    return {
        # Overall (kept for backward compat / summary table)
        "rms_cte":          overall_rms,
        "avg_cte":          float(np.mean(np.abs(ct))),
        "max_cte":          float(np.max(np.abs(ct))),
        "mean_dv":          float(np.mean(sp)) if len(sp) else float("nan"),
        "n_rows":           n_rows,
        "crashed":          crashed,
        # Phase-separated
        "convergence_time": convergence_time,   # nan = never converged
        "steady_rms_cte":   steady_rms,         # nan = never converged
        "steady_avg_cte":   steady_avg,
        "steady_max_cte":   steady_max,
        "on_path_frac":     on_path_fraction,   # 0–1
        "converged":        steady_start_idx is not None,
    }


# =============================================================================
# Plotting
# =============================================================================

def generate_plots(data: dict, terrain: str, path_type: str,
                   save_dir: Path, run_idx: int,
                   model: str = "nn",
                   ref_path_csv: Path | None = None):
    """Per-run plots saved under plots/<model>/<terrain>_<path>/."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    save_dir.mkdir(parents=True, exist_ok=True)
    t    = data["sim_time"]
    mask = t > 2.0
    tag  = f"{terrain}/{path_type} [{model}] run {run_idx}"
    stem = f"run{run_idx:02d}"   # no model suffix — folder already encodes it

    # Smooth reference path geometry (from the saved CSV, not from x_ref_0/y_ref_0).
    # x_ref_0/y_ref_0 is just the first point of each MPC solve — it tracks the
    # vehicle's noisy state projection onto the path and looks jagged in plots.
    ref_xy = load_ref_path_csv(ref_path_csv) if ref_path_csv else None

    # --- 1. Actual vs predicted lateral forces (NN runs only) ---
    has_forces = (
        "actual_Fy_front" in data
        and np.any(data["actual_Fy_front"][mask] != 0)
    )
    if has_forces:
        fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
        for ax, side, key_a, key_p in [
            (axes[0], "Front", "actual_Fy_front", "pred_Fy_front"),
            (axes[1], "Rear",  "actual_Fy_rear",  "pred_Fy_rear"),
        ]:
            ax.plot(t, data[key_a], "b-",  alpha=0.7, label="Chrono actual")
            ax.plot(t, data[key_p], "r--", alpha=0.7, label="NN predicted")
            ax.set_ylabel("Fy (N)")
            ax.set_title(f"{side} Axle Lateral Force — {tag}")
            ax.legend(); ax.grid(True, alpha=0.3)
        axes[1].set_xlabel("Time (s)")
        plt.tight_layout()
        plt.savefig(save_dir / f"{stem}_Fy_timeseries.png", dpi=150)
        plt.close()

        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        for ax, side, key_a, key_p, key_alpha in [
            (axes[0], "Front", "actual_Fy_front", "pred_Fy_front", "alpha_f"),
            (axes[1], "Rear",  "actual_Fy_rear",  "pred_Fy_rear",  "alpha_r"),
        ]:
            alpha_deg = np.degrees(data[key_alpha][mask])
            sort_idx  = np.argsort(alpha_deg)
            ax.scatter(alpha_deg, data[key_a][mask], s=4, alpha=0.3,
                       c="blue", label="Chrono actual")
            ax.plot(alpha_deg[sort_idx], data[key_p][mask][sort_idx],
                    "r-", linewidth=2, label="NN predicted")
            ax.set_xlabel("Slip angle (deg)"); ax.set_ylabel("Fy (N)")
            ax.set_title(f"{side}: Fy vs α — {terrain}")
            ax.legend(); ax.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(save_dir / f"{stem}_Fy_vs_alpha.png", dpi=150)
        plt.close()

    # --- 2. Tracking performance ---
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    axes[0].plot(t, data["crosstrack_err"], alpha=0.8)
    axes[0].axhline(0, color="k", linewidth=0.5)
    axes[0].set_ylabel("Cross-track error (m)")
    axes[0].set_title(f"Tracking — {tag}")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(t, data["heading_err_deg"], "g-", alpha=0.8)
    axes[1].axhline(0, color="k", linewidth=0.5)
    axes[1].set_ylabel("Heading error (deg)")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(t, data["u_meas"],  "b-",  alpha=0.8, label="actual u")
    axes[2].plot(t, data["v_ref_0"], "r--", alpha=0.7, label="v_target")
    axes[2].set_ylabel("Speed (m/s)"); axes[2].set_xlabel("Time (s)")
    axes[2].legend(); axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_dir / f"{stem}_tracking.png", dpi=150)
    plt.close()

    # --- 3. XY trajectory ---
    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(data["x_fa_meas"], data["y_fa_meas"], alpha=0.85, label="actual")
    if ref_xy is not None:
        ax.plot(ref_xy[0], ref_xy[1], "r--", alpha=0.6,
                linewidth=1.5, label="reference path")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title(f"XY Trajectory — {tag}")
    ax.axis("equal"); ax.legend(); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(save_dir / f"{stem}_xy.png", dpi=150)
    plt.close()

    tprint(f"    Plots → {save_dir}/{stem}_*.png")


def generate_improvement_heatmap(all_entries: list, save_dir: Path,
                                  terrains: list, paths: list,
                                  sim_time: float = 15.0):
    """
    Three-panel heatmap comparing NN vs Pacejka with phase-separated metrics
    so that startup transients and complete failures don't distort the picture.

    Panel 1 — Steady-state RMS CTE improvement (%)
        Only counted after BOTH controllers have converged (|CTE| < threshold
        for 2+ seconds).  This is the fairest apples-to-apples comparison:
        both controllers are tracking, which is better?
        If Pacejka never converges, the cell is marked "no conv." instead
        of reporting a misleading 93% improvement number.

    Panel 2 — On-path fraction (NN vs Pacejka)
        What fraction of the run is each controller within the CTE threshold?
        Captures both startup transients AND total failures in one honest number.

    Panel 3 — Convergence time  (NN vs Pacejka, seconds)
        How long until each controller first locks onto the path?
        Shows startup penalty cleanly, separated from steady-state quality.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.colors as mcolors
    from collections import defaultdict

    save_dir.mkdir(parents=True, exist_ok=True)

    nr, nc = len(paths), len(terrains)
    path_labels = [p.replace("_", "\n") for p in paths]

    # ── aggregate per (terrain, path, model) across runs ─────────────────────
    agg:    dict = defaultdict(list)
    crashes: dict = defaultdict(int)
    for e in all_entries:
        if e["status"] != "OK":
            continue
        key = (e["terrain"], e["path_type"], e["model"])
        if e.get("crashed", False):
            crashes[key] += 1
        else:
            agg[key].append(e)   # store full entry, not just one number

    def _med(entries, field):
        vals = [e[field] for e in entries if not np.isnan(e.get(field, float("nan")))]
        return float(np.median(vals)) if vals else float("nan")

    def _frac_converged(entries):
        if not entries:
            return float("nan")
        return float(np.mean([1.0 if e.get("converged") else 0.0 for e in entries]))

    # Matrices: rows = paths, cols = terrains
    # Panel 1: steady-state RMS CTE improvement %
    ss_impr   = np.full((nr, nc), np.nan)
    ss_nn     = np.full((nr, nc), np.nan)
    ss_pj     = np.full((nr, nc), np.nan)
    no_conv_pj = np.zeros((nr, nc), dtype=bool)   # Pacejka never converged
    # Panel 2: on-path fraction
    frac_nn   = np.full((nr, nc), np.nan)
    frac_pj   = np.full((nr, nc), np.nan)
    # Panel 3: convergence time
    conv_nn   = np.full((nr, nc), np.nan)
    conv_pj   = np.full((nr, nc), np.nan)
    # crash counts
    cr_nn_m   = np.zeros((nr, nc), dtype=int)
    cr_pj_m   = np.zeros((nr, nc), dtype=int)

    for ri, path_type in enumerate(paths):
        for ci, terrain in enumerate(terrains):
            nn_e  = agg.get((terrain, path_type, "nn"),     [])
            pj_e  = agg.get((terrain, path_type, "linear"), [])
            cr_nn_m[ri, ci] = crashes.get((terrain, path_type, "nn"),     0)
            cr_pj_m[ri, ci] = crashes.get((terrain, path_type, "linear"), 0)

            # steady-state RMS
            nn_ss = _med(nn_e, "steady_rms_cte")
            pj_ss = _med(pj_e, "steady_rms_cte")
            ss_nn[ri, ci] = nn_ss
            ss_pj[ri, ci] = pj_ss
            if not np.isnan(pj_ss) and not np.isnan(nn_ss) and pj_ss > 1e-9:
                ss_impr[ri, ci] = (pj_ss - nn_ss) / pj_ss * 100
            pj_conv_frac = _frac_converged(pj_e)
            no_conv_pj[ri, ci] = (pj_conv_frac < 0.5) if not np.isnan(pj_conv_frac) else True

            # on-path fraction
            frac_nn[ri, ci] = _med(nn_e, "on_path_frac")
            frac_pj[ri, ci] = _med(pj_e, "on_path_frac")

            # convergence time (nan → sim_time if never converged)
            ct_nn = _med(nn_e, "convergence_time")
            ct_pj = _med(pj_e, "convergence_time")
            conv_nn[ri, ci] = ct_nn if not np.isnan(ct_nn) else sim_time
            conv_pj[ri, ci] = ct_pj if not np.isnan(ct_pj) else sim_time

    # ── build figure ──────────────────────────────────────────────────────────
    fig, axes = plt.subplots(1, 3, figsize=(5 * nc + 3, max(4, 2.0 * nr + 2.5)))
    cmap_rg = mcolors.LinearSegmentedColormap.from_list(
        "rg", ["#d32f2f", "#ffffff", "#388e3c"])

    def _draw_heatmap(ax, matrix, vmin, vmax, cmap, title, fmt,
                      annot_override=None):
        im = ax.imshow(matrix, cmap=cmap, vmin=vmin, vmax=vmax, aspect="auto")
        ax.set_xticks(range(nc))
        ax.set_xticklabels([t.capitalize() for t in terrains], fontsize=10)
        ax.set_yticks(range(nr))
        ax.set_yticklabels(path_labels, fontsize=9)
        ax.set_title(title, fontsize=10, fontweight="bold", pad=8)
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        for ri_ in range(nr):
            for ci_ in range(nc):
                v = matrix[ri_, ci_]
                ann = annot_override[ri_, ci_] if annot_override is not None else None
                if ann is None:
                    ann = fmt(v) if not np.isnan(v) else "—"
                bg = im.norm(v) if not np.isnan(v) else 0.5
                color = "white" if abs(bg - 0.5) > 0.3 else "black"
                ax.text(ci_, ri_, ann, ha="center", va="center",
                        fontsize=8.5, color=color, fontweight="bold")

    # ── Panel 1: steady-state improvement ─────────────────────────────────────
    # Build annotation: if Pacejka never converged, show "no conv." instead of %
    ann1 = np.empty((nr, nc), dtype=object)
    ann1[:] = None
    vmax1 = max(float(np.nanmax(np.abs(ss_impr))), 5.0)
    for ri_ in range(nr):
        for ci_ in range(nc):
            v = ss_impr[ri_, ci_]
            cr_n = cr_nn_m[ri_, ci_]
            cr_p = cr_pj_m[ri_, ci_]
            crash_tag = f"\n💥{cr_n}|{cr_p}" if (cr_n or cr_p) else ""
            if no_conv_pj[ri_, ci_]:
                ann1[ri_, ci_] = f"Paj\nno conv.{crash_tag}"
            elif np.isnan(v):
                ann1[ri_, ci_] = f"—{crash_tag}"
            else:
                sign = "▲" if v > 0 else "▼"
                ann1[ri_, ci_] = f"{sign}{abs(v):.1f}%{crash_tag}"

    _draw_heatmap(axes[0], ss_impr, -vmax1, vmax1, cmap_rg,
                  f"Steady-state RMS CTE improvement\n"
                  f"(NN vs Pacejka, after convergence only)\n"
                  f"Green = NN better  |  threshold={CONVERGED_CTE_THRESH}m",
                  lambda v: f"{v:.1f}%", annot_override=ann1)

    # ── Panel 2: on-path fraction ─────────────────────────────────────────────
    # Show both numbers in each cell
    ann2 = np.empty((nr, nc), dtype=object)
    for ri_ in range(nr):
        for ci_ in range(nc):
            nn_f = frac_nn[ri_, ci_]
            pj_f = frac_pj[ri_, ci_]
            nn_s = f"{nn_f*100:.0f}%" if not np.isnan(nn_f) else "—"
            pj_s = f"{pj_f*100:.0f}%" if not np.isnan(pj_f) else "—"
            ann2[ri_, ci_] = f"NN: {nn_s}\nPaj: {pj_s}"
    # Color by NN fraction (green = NN on-path most of the time)
    _draw_heatmap(axes[1], frac_nn, 0.0, 1.0,
                  plt.cm.RdYlGn,
                  f"On-path fraction (|CTE|<{CONVERGED_CTE_THRESH}m)\n"
                  f"Whole run, both controllers shown\n"
                  f"Color = NN fraction",
                  lambda v: f"{v*100:.0f}%", annot_override=ann2)

    # ── Panel 3: convergence time ─────────────────────────────────────────────
    ann3 = np.empty((nr, nc), dtype=object)
    for ri_ in range(nr):
        for ci_ in range(nc):
            t_nn = conv_nn[ri_, ci_]
            t_pj = conv_pj[ri_, ci_]
            nn_s = f"{t_nn:.1f}s" if not np.isnan(t_nn) else "—"
            pj_s = f"{t_pj:.1f}s" if t_pj < sim_time else "never"
            ann3[ri_, ci_] = f"NN: {nn_s}\nPaj: {pj_s}"
    # Color by Pacejka convergence time (red = long/never)
    cmap_conv = mcolors.LinearSegmentedColormap.from_list(
        "conv", ["#388e3c", "#ffeb3b", "#d32f2f"])
    _draw_heatmap(axes[2], conv_pj, 0, sim_time, cmap_conv,
                  f"Time to first converge (s)\n"
                  f"Color = Pacejka (red = never/slow)\n"
                  f"Both values shown per cell",
                  lambda v: f"{v:.1f}s", annot_override=ann3)

    fig.suptitle("NN vs Pacejka — Phase-Separated Comparison\n"
                 "(Panel 1: steady-state only  |  Panel 2: whole-run on-path  |  "
                 "Panel 3: convergence speed)",
                 fontsize=11, fontweight="bold", y=1.01)
    plt.tight_layout()

    out = save_dir / "nn_vs_pacejka_improvement.png"
    plt.savefig(out, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"\n  Improvement heatmap → {out}")


# =============================================================================
# Worker
# =============================================================================

def run_one(job: dict, args) -> dict:
    terrain   = job["terrain"]
    path_type = job["path_type"]
    model     = job["model"]
    run_idx   = job["run_idx"]
    slot      = job["slot"]
    model_tag = "nn" if model == "nn" else "pacejka"
    label     = (f"[{job['global_idx']}/{job['total']}] "
                 f"{terrain}/{path_type}/{model} run {run_idx}")

    sim_port  = BASE_SIM_PORT  + 2 * slot
    ctrl_port = BASE_CTRL_PORT + 2 * slot

    tprint(f"\n{'='*70}")
    tprint(f"{label}  (ports {sim_port}/{ctrl_port})")
    tprint(f"{'='*70}")

    before_csvs = set(PLOTS_DIR.glob(
        f"*_{terrain}_{path_type}_{model_tag}/"
        f"diag_{terrain}_{path_type}_{model_tag}.csv"))

    sim_cmd = [
        sys.executable, str(SIM_SCRIPT),
        "--model",     model,
        "--nn-model",  args.nn_model,
        "--terrain",   terrain,
        "--path",      path_type,
        "--time",      str(args.time),
        "--speed",     str(args.speed),
        "--sim-port",  str(sim_port),
        "--ctrl-port", str(ctrl_port),
        "--no-vis",
    ]

    log_dir = DIAG_DIR / "logs" / f"{terrain}_{path_type}"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / f"run{run_idx:02d}_{model}.log"

    with open(log_file, "w") as lf:
        ret = subprocess.run(sim_cmd, cwd=str(SIM_DIR), stdout=lf, stderr=lf)

    if ret.returncode != 0:
        tprint(f"  *** {label} FAILED (see {log_file}) ***")
        return {"run": run_idx, "terrain": terrain, "path_type": path_type,
                "model": model, "status": "FAILED"}

    time.sleep(0.3)

    after_csvs = set(PLOTS_DIR.glob(
        f"*_{terrain}_{path_type}_{model_tag}/"
        f"diag_{terrain}_{path_type}_{model_tag}.csv"))
    new_csvs = after_csvs - before_csvs
    csv_path = (
        max(new_csvs, key=lambda p: p.stat().st_mtime) if new_csvs
        else find_latest_csv(terrain, path_type, model_tag)
    )

    if csv_path is None or not csv_path.exists():
        tprint(f"  WARNING: {label} — CSV not found")
        return {"run": run_idx, "terrain": terrain, "path_type": path_type,
                "model": model, "status": "NO_CSV"}

    tprint(f"  {label} → {csv_path.relative_to(SCM_TELEOP)}")

    ref_path_csv = next(csv_path.parent.glob("reference_path_*.csv"), None)
    data  = load_csv(csv_path)
    stats = compute_stats(data, sim_time=args.time)
    entry = {"run": run_idx, "terrain": terrain, "path_type": path_type,
             "model": model, "status": "OK", "csv": str(csv_path), **stats}

    if stats:
        crash_flag = "  *** CRASH ***" if stats.get("crashed") else ""
        tprint(f"  {label}  RMS={stats['rms_cte']:.4f}m  "
               f"Avg|CTE|={stats['avg_cte']:.4f}m  "
               f"Max={stats['max_cte']:.4f}m  n={stats['n_rows']}{crash_flag}")

    if not args.no_plot:
        try:
            model_folder = "nn" if model == "nn" else "pacejka"
            plot_dir = DIAG_DIR / "plots" / model_folder / f"{terrain}_{path_type}"
            generate_plots(data, terrain, path_type, plot_dir, run_idx,
                           model=model, ref_path_csv=ref_path_csv)
        except Exception as e:
            tprint(f"  WARNING: {label} plot failed: {e}")

    return entry


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Batch simulate all terrain/path/model combos and compare",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--time",     type=float, default=15.0)
    parser.add_argument("--speed",    type=float, default=8.0)
    parser.add_argument("--runs",     type=int,   default=1,
                        help="Repeated runs per combo (for averaging)")
    parser.add_argument("--workers",  type=int,   default=3,
                        help="Parallel simulations (each needs its own CPU cores)")
    parser.add_argument("--terrains", nargs="+",  default=TERRAINS,
                        choices=TERRAINS)
    parser.add_argument("--paths",    nargs="+",  default=PATHS,
                        choices=PATHS)
    parser.add_argument("--models",   nargs="+",  default=MODELS,
                        choices=MODELS,
                        help="Tire models to compare (nn, linear/Pacejka)")
    parser.add_argument("--nn-model", default="temporal_v1",
                        help="NN model version directory under nn_models/")
    parser.add_argument("--no-plot",  action="store_true",
                        help="Skip plot generation (collect CSVs only)")
    args = parser.parse_args()

    combos = [(t, p, m)
              for t in args.terrains
              for p in args.paths
              for m in args.models]
    total = len(combos) * args.runs

    n_workers = min(args.workers, total)
    print(f"Batch: {len(args.terrains)}T × {len(args.paths)}P × "
          f"{len(args.models)}M × {args.runs} run(s) = {total} simulations")
    print(f"Workers: {n_workers}  |  Sim time: {args.time}s  "
          f"Speed: {args.speed} m/s  Models: {args.models}")
    print(f"Estimated wall time: ~{args.time * total / n_workers / 60:.1f} min\n")

    # Build job list
    jobs = []
    global_idx = 0
    for terrain, path_type, model in combos:
        for run_idx in range(1, args.runs + 1):
            global_idx += 1
            jobs.append({
                "terrain":    terrain,
                "path_type":  path_type,
                "model":      model,
                "run_idx":    run_idx,
                "global_idx": global_idx,
                "total":      total,
                "slot":       -1,
            })

    # Dispatch with a slot pool (each slot has a reserved port pair)
    slot_pool   = queue.Queue()
    for s in range(n_workers):
        slot_pool.put(s)

    all_entries = []
    lock        = threading.Lock()
    threads     = []

    def worker(job):
        entry = run_one(job, args)
        with lock:
            all_entries.append(entry)
        slot_pool.put(job["slot"])

    for job in jobs:
        slot = slot_pool.get()
        job["slot"] = slot
        t = threading.Thread(target=worker, args=(job,), daemon=True)
        threads.append(t)
        t.start()

    for t in threads:
        t.join()

    # -------------------------------------------------------------------------
    # Improvement heatmap (NN vs Pacejka)
    # -------------------------------------------------------------------------
    if not args.no_plot and "nn" in args.models and "linear" in args.models:
        try:
            generate_improvement_heatmap(
                all_entries,
                save_dir  = DIAG_DIR / "plots",
                terrains  = args.terrains,
                paths     = args.paths,
                sim_time  = args.time,
            )
        except Exception as e:
            print(f"  WARNING: heatmap failed: {e}")

    # -------------------------------------------------------------------------
    # Summary table
    # -------------------------------------------------------------------------
    all_entries.sort(key=lambda e: (e["terrain"], e["path_type"],
                                    e["model"], e["run"]))

    print(f"\n{'='*80}")
    print("BATCH SUMMARY")
    print(f"{'='*80}")
    print(f"  {'Terrain':<8}  {'Path':<22}  {'Model':<8}  {'Run':>4}  "
          f"{'RMS CTE':>9}  {'Avg|CTE|':>9}  {'n':>5}  Status")
    print(f"  {'-'*8}  {'-'*22}  {'-'*8}  {'-'*4}  "
          f"{'-'*9}  {'-'*9}  {'-'*5}  {'-'*10}")
    for e in all_entries:
        rms   = f"{e.get('rms_cte', float('nan')):.4f}m"
        avg   = f"{e.get('avg_cte', float('nan')):.4f}m"
        n_r   = str(e.get('n_rows', '?'))
        crash = "  CRASH" if e.get("crashed") else ""
        print(f"  {e['terrain']:<8}  {e['path_type']:<22}  {e['model']:<8}  "
              f"{e['run']:>4}  {rms:>9}  {avg:>9}  {n_r:>5}  {e['status']}{crash}")

    print(f"\nPlots → {DIAG_DIR / 'plots'}/")


if __name__ == "__main__":
    main()
