#!/usr/bin/env python3
"""Paper experiment: human-in-the-loop safety-filter delay rounds.

This is the canonical human-in-the-loop (HIL) obstacle-avoidance benchmark:
a human drives the HMMWV through a rock field while the sim-side safety
filter (none / MPPI / DOB-CBF / NMPC) screens the delayed operator
commands.  Each round delays *both* the operator command path and the
driver POV camera feed -- the command delay models the uplink and the
camera delay models the downlink of the teleoperation link.  Camera delay
is ``--camera-delay-scale`` times the command delay (default 1.0,
symmetric link).

Alternatively, ``--latency-profile-json`` runs every round under a
time-varying 5G-like latency profile (the ``control``/``manual`` channels
drive the command uplink and the ``camera`` channel the asymmetric video
downlink), superseding the fixed-delay sweep.  ``--live-hud`` launches the
Tesla-style HMI overlay (``simulation/hil_hud.py``) on each round's ZMQ
ports so the operator sees the live wheel/throttle (and the filter takeover)
while driving.

The script orchestrates one round at a time, writes raw sim diagnostics,
and summarizes tracking, speed, collision, clearance, and intervention
metrics per (filter, delay) cell.
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (  # noqa: E402
    DEFAULT_NN_MODEL,
    LAUNCHER,
    LOGS_DIR,
    PATH_ROCK_ZONES,
    PROJECT_ROOT,
    TERRAINS,
    ensure_runtime_env,
    parse_collision_csv,
    parse_log_summary,
    parse_shield_csv,
    save_summary_markdown,
    timestamped_result_dir,
    write_manifest,
)

SIM_DIR = PROJECT_ROOT / "simulation"
sys.path.insert(0, str(SIM_DIR))
from reference_path import ReferencePath, generate_path_waypoints  # noqa: E402


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--filters", nargs="+", default=["none", "mppi", "dob_cbf", "nmpc"],
                   choices=["none", "mppi", "nmpc", "dob_cbf"])
    p.add_argument("--delays", nargs="+", type=float, default=[0.0, 0.15, 0.30],
                   help="Operator command-path (uplink) delays in seconds.")
    p.add_argument("--camera-delay-scale", type=float, default=1.0,
                   help="Camera (downlink) delay as a multiple of the command "
                        "delay. 1.0 = symmetric link; >1 models a heavier video "
                        "downlink (the learned 5G profile is approx 1.6).")
    p.add_argument("--latency-profile-json", default="",
                   help="Run every round under a time-varying 5G-like latency "
                        "profile JSON (uplink = control/manual channels, "
                        "asymmetric downlink = camera channel) instead of the "
                        "fixed --delays sweep. Supersedes the constant uplink/"
                        "downlink/teleop delays; collapses to one 5G condition "
                        "per cell.")
    p.add_argument("--live-hud", action="store_true",
                   help="Launch the live HMI overlay (simulation/hil_hud.py) on "
                        "each round's ZMQ ports: a virtual steering wheel "
                        "(commanded ghost vs applied) + throttle bar, so the "
                        "filter takeover is visible live while driving.")
    p.add_argument("--hud-corner", choices=["br", "bl", "tr", "tl"], default="br",
                   help="Screen corner to dock the live HUD overlay into.")
    p.add_argument("--hud-wheel-lock", type=float, default=450.0,
                   help="HUD wheel rotation at full steer; set to half the G29's "
                        "lock-to-lock range to match the physical wheel (default 450).")
    # Real-time tuning for interactive driving (defaults favour real-time over
    # the autonomous-sweep fidelity, since a human must drive these live).
    p.add_argument("--cam-width", type=int, default=1920,
                   help="Driver POV render width (px).")
    p.add_argument("--cam-height", type=int, default=1200,
                   help="Driver POV render height (px). 1200 = 16:10 (default).")
    p.add_argument("--cam-fov", type=float, default=1.05,
                   help="Driver POV camera horizontal FOV (rad, ~1.05=60deg).")
    p.add_argument("--cam-rate", type=float, default=30.0,
                   help="Driver POV camera render rate (Hz).")
    p.add_argument("--cam-fullscreen", action=argparse.BooleanOptionalAction,
                   default=True,
                   help="Display the driver POV fullscreen (renders at cam W x H, "
                        "scaled to the screen). Use --no-cam-fullscreen for a window.")
    p.add_argument("--convoy", type=str, default="",
                   help="Spawn PID-driven traffic for a convoy safety scenario the "
                        "operator must avoid (lead_brake/cut_in/stalled/swerver/convoy/"
                        "platoon/oncoming/double_cut/stop_and_go/jam/overtake/gauntlet).")
    p.add_argument("--traffic-detail", choices=["auto", "mesh", "primitives"],
                   default="auto", help="Traffic render detail (auto stays real-time).")
    p.add_argument("--mesh-resolution", type=float, default=0.12,
                   help="SCM mesh spacing (m). Default 0.12 = real-time for HIL: "
                        "the terrain triangle count dominates the camera render "
                        "cost (1080p@30Hz is RT at 0.12 but ~0.55x at 0.08). The "
                        "autonomous sweeps use 0.08 for force fidelity.")
    p.add_argument("--terrains", nargs="+", default=["clay", "sand"], choices=list(TERRAINS))
    p.add_argument("--paths", nargs="+", default=["sinusoidal", "lane_change"])
    p.add_argument("--speeds", nargs="+", type=float, default=[4.0])
    p.add_argument("--bumpiness", nargs="+", type=int, default=[0, 4])
    p.add_argument("--rounds", type=int, default=1,
                   help="Repeated human rounds per condition.")
    p.add_argument("--base-seed", type=int, default=910)
    p.add_argument("--time", type=float, default=25.0)
    p.add_argument("--lead-in", type=float, default=5.0)
    p.add_argument("--rocks", type=int, default=5)
    p.add_argument("--manual-mode", choices=["g29", "wasd"], default="g29")
    p.add_argument("--vis-mode", choices=["irrlicht", "sensor", "both", "none"], default="sensor")
    p.add_argument("--shield-horizon", type=int, default=12)
    p.add_argument("--mppi-samples", type=int, default=384)
    p.add_argument("--nmpc-iter", type=int, default=6)
    p.add_argument("--safety-buffer", type=float, default=0.25)
    p.add_argument("--auto-start", action="store_true",
                   help="Do not wait for Enter before each round.")
    p.add_argument("--dry-run", action="store_true",
                   help="Only write manifest and command plan; do not launch Chrono.")
    p.add_argument("--timeout", type=float, default=360.0)
    p.add_argument("--base-port", type=int, default=10400)
    p.add_argument("--quick", action="store_true",
                   help="Single short WASD-compatible smoke round.")
    return p.parse_args()


def command_for_round(args: argparse.Namespace, run_dir: Path, idx: int, filter_name: str,
                      delay: float, terrain: str, path: str, speed: float,
                      bump: int, seed: int) -> list[str]:
    sim_port = args.base_port + 2 * idx
    ctrl_port = sim_port + 1
    camera_delay = delay * args.camera_delay_scale
    cmd = [
        sys.executable, "-u", str(LAUNCHER),
        "--terrain", terrain,
        "--path", path,
        "--speed", str(speed),
        "--time", str(args.time),
        "--lead-in", str(args.lead_in),
        "--bumpiness", str(bump),
        "--rocks", str(args.rocks),
        "--rock-seed", str(seed),
        "--sim-port", str(sim_port),
        "--ctrl-port", str(ctrl_port),
        "--vis-mode", args.vis_mode,
        "--manual-honor-time",
        "--sim-diag-csv", str(run_dir / "sim_diag.csv"),
        "--nn-model", DEFAULT_NN_MODEL,
        "--cam-width", str(args.cam_width),
        "--cam-height", str(args.cam_height),
        "--cam-fov", str(args.cam_fov),
        "--cam-rate", str(args.cam_rate),
    ]
    if args.cam_fullscreen:
        cmd.append("--cam-fullscreen")
    if args.convoy:
        cmd += ["--convoy", args.convoy, "--traffic-detail", args.traffic_detail]
    if args.mesh_resolution is not None:
        cmd += ["--mesh-resolution", str(args.mesh_resolution)]
    if args.latency_profile_json:
        # 5G profile drives all channels: control/manual = command uplink,
        # camera = asymmetric video downlink. Supersedes the constant delays.
        cmd += ["--latency-profile-json", args.latency_profile_json,
                "--latency-profile-log", str(run_dir / "latency_profile.csv")]
    else:
        cmd += ["--manual-input-delay", str(delay),
                "--camera-input-delay", str(camera_delay)]
    cmd.append("--wasd" if args.manual_mode == "wasd" else "--manual")
    if args.rocks > 0:
        zone = PATH_ROCK_ZONES.get(path, PATH_ROCK_ZONES["sinusoidal"])
        cmd += [
            "--rock-zone-x", str(zone["x"][0]), str(zone["x"][1]),
            "--rock-zone-y", str(zone["y"][0]), str(zone["y"][1]),
            "--rock-size", "0.8", "1.8",
        ]
    if filter_name != "none":
        cmd += [
            "--safety-filter",
            "--safety-flavor", filter_name,
            "--safety-buffer", str(args.safety_buffer),
            "--shield-horizon", str(args.shield_horizon),
        ]
        if not args.latency_profile_json:
            # In profile mode the sim samples the control channel and feeds
            # the sim-side filter; a fixed --teleop-delay would override it.
            cmd += ["--teleop-delay", str(delay)]
        if filter_name == "mppi":
            cmd += ["--mppi-samples", str(args.mppi_samples)]
        if filter_name == "nmpc":
            cmd += ["--nmpc-iter", str(args.nmpc_iter)]
    return cmd


def parse_sim_diag(path: Path, ref_path_name: str, speed: float, lead_in: float,
                   metric_start: float = 3.0) -> dict[str, float]:
    if not path.exists():
        return {}
    df = pd.read_csv(path)
    if df.empty:
        return {}
    t = pd.to_numeric(df["time"], errors="coerce").to_numpy(dtype=float)
    x = pd.to_numeric(df["x"], errors="coerce").to_numpy(dtype=float)
    y = pd.to_numeric(df["y"], errors="coerce").to_numpy(dtype=float)
    u = pd.to_numeric(df["speed"], errors="coerce").to_numpy(dtype=float)
    mask = np.isfinite(t) & (t >= metric_start)
    if not mask.any():
        mask = np.isfinite(t)
    xp, yp = generate_path_waypoints(ref_path_name, lead_in=lead_in)
    ref = ReferencePath(xp, yp, v_target=speed)
    cte = []
    for xi, yi in zip(x, y):
        if math.isfinite(xi) and math.isfinite(yi):
            cte.append(ref.closest_point_on_path(float(xi), float(yi))["e_lat"])
        else:
            cte.append(math.nan)
    cte_arr = np.asarray(cte, dtype=float)
    cte_m = cte_arr[mask & np.isfinite(cte_arr)]
    u_m = u[mask & np.isfinite(u)]
    progress = math.nan
    good_xy = np.isfinite(x) & np.isfinite(y)
    if np.count_nonzero(good_xy) >= 2:
        progress = float(np.sum(np.hypot(np.diff(x[good_xy]), np.diff(y[good_xy]))))
    clearance = pd.to_numeric(df.get("nearest_clearance_m", pd.Series(dtype=float)), errors="coerce")
    clearance_m = clearance[mask] if len(clearance) == len(df) else clearance
    return {
        "n_samples": int(len(df)),
        "rms_cte_m": float(np.sqrt(np.mean(cte_m ** 2))) if len(cte_m) else math.nan,
        "max_abs_cte_m": float(np.max(np.abs(cte_m))) if len(cte_m) else math.nan,
        "mean_abs_cte_m": float(np.mean(np.abs(cte_m))) if len(cte_m) else math.nan,
        "mean_speed_mps": float(np.mean(u_m)) if len(u_m) else math.nan,
        "speed_ratio": float(np.mean(u_m) / speed) if len(u_m) and speed > 1e-6 else math.nan,
        "progress_m": progress,
        "final_x_m": float(x[good_xy][-1]) if np.count_nonzero(good_xy) else math.nan,
        "final_y_m": float(y[good_xy][-1]) if np.count_nonzero(good_xy) else math.nan,
        "min_clearance_m": float(np.nanmin(clearance_m)) if len(clearance_m) and np.isfinite(clearance_m).any() else math.nan,
    }


def _maybe_launch_hud(args: argparse.Namespace, idx: int, run_dir: Path):
    """Launch the live HMI overlay on this round's ZMQ ports (or return None).

    The HUD only subscribes, so it cannot perturb the sim/controller loops; it
    CONNECTs (late binder is fine) and starts updating once the sim binds.
    """
    if not args.live_hud:
        return None
    run_dir.mkdir(parents=True, exist_ok=True)
    sim_port = args.base_port + 2 * idx
    ctrl_port = sim_port + 1
    hud_log = (run_dir / "hud.log").open("w")
    proc = subprocess.Popen(
        [sys.executable, str(SIM_DIR / "hil_hud.py"),
         "--sim-port", str(sim_port), "--ctrl-port", str(ctrl_port),
         "--corner", args.hud_corner, "--wheel-lock-deg", str(args.hud_wheel_lock)],
        cwd=str(PROJECT_ROOT), stdout=hud_log, stderr=subprocess.STDOUT,
        env=dict(**os.environ),
    )
    proc._hud_log = hud_log  # keep the handle so we can close it on teardown
    return proc


def _stop_hud(proc) -> None:
    if proc is None:
        return
    proc.terminate()
    try:
        proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
        proc.kill()
    log = getattr(proc, "_hud_log", None)
    if log is not None:
        log.close()


def run_round(cmd: list[str], run_dir: Path, timeout: float) -> tuple[int, float, str]:
    ensure_runtime_env()
    run_dir.mkdir(parents=True, exist_ok=True)
    log_path = run_dir / "run.log"
    # Route the sim-side collision logger and the safety-filter/shield loggers
    # into this run's dir (same mechanism the parallel sweeps use), so each
    # round's cbf/mppi/nmpc shield CSV + collision log land here -- otherwise
    # they fall back to a shared global dir and the intrusiveness metrics
    # (mean_abs_dsteer/dthrottle, intervention rate) are lost.
    env = dict(os.environ)
    env["HIL_RUN_LOG_DIR"] = str(run_dir)
    t0 = time.time()
    with log_path.open("w") as f:
        try:
            proc = subprocess.run(
                cmd,
                cwd=str(PROJECT_ROOT),
                stdout=f,
                stderr=subprocess.STDOUT,
                timeout=timeout,
                env=env,
            )
            rc = proc.returncode
        except subprocess.TimeoutExpired:
            rc = -9
            f.write(f"\nTIMEOUT after {timeout:.1f}s\n")
    return rc, time.time() - t0, log_path.read_text(errors="replace")


def collect_global_logs(run_dir: Path, created_after: float) -> tuple[str, str]:
    """Resolve the per-run collision + shield log paths.

    With HIL_RUN_LOG_DIR set in run_round, the sim/safety loggers write these
    straight into run_dir; fall back to the shared global LOGS_DIR for any that
    an older path still drops there.
    """
    collision_csv = ""
    shield_csv = ""
    for name in ("collision_log.csv", "cbf_filter_log.csv", "mppi_shield_log.csv", "nmpc_shield_log.csv"):
        local = run_dir / name
        if local.exists():
            src = local
        else:
            src = LOGS_DIR / name
            if not (src.exists() and src.stat().st_mtime >= created_after - 2.0):
                continue
            shutil.copy2(src, run_dir / name)
            src = run_dir / name
        if name == "collision_log.csv":
            collision_csv = str(src)
        elif not shield_csv:
            shield_csv = str(src)
    return collision_csv, shield_csv


def plot_figures(results_csv: Path, out_dir: Path) -> None:
    df = pd.read_csv(results_csv)
    ok = df[df["status"] == "ok"].copy()
    if ok.empty:
        return
    for col in ("rms_cte_m", "speed_ratio", "collisions",
                "min_clearance_m", "intervention_rate_pct"):
        if col not in ok.columns:
            ok[col] = math.nan
    fig_dir = out_dir / "figures"
    for col in ("mean_abs_dsteer", "mean_abs_dthrottle"):
        if col not in ok.columns:
            ok[col] = math.nan
    summary = ok.groupby(["filter", "delay_s"], sort=False).agg(
        rms_cte=("rms_cte_m", "mean"),
        collisions=("collisions", "mean"),
        clearance=("min_clearance_m", "mean"),
        speed_ratio=("speed_ratio", "mean"),
        intervention=("intervention_rate_pct", "mean"),
        dsteer=("mean_abs_dsteer", "mean"),
        dthrottle=("mean_abs_dthrottle", "mean"),
    ).reset_index()

    fig, axes = plt.subplots(2, 3, figsize=(14, 7.5))
    for filter_name, sub in summary.groupby("filter", sort=False):
        axes[0, 0].plot(sub["delay_s"], sub["collisions"], marker="o", label=filter_name)
        axes[0, 1].plot(sub["delay_s"], sub["clearance"], marker="o", label=filter_name)
        axes[0, 2].plot(sub["delay_s"], sub["intervention"], marker="o", label=filter_name)
        axes[1, 0].plot(sub["delay_s"], sub["dsteer"], marker="o", label=filter_name)
        axes[1, 1].plot(sub["delay_s"], sub["dthrottle"], marker="o", label=filter_name)
        axes[1, 2].plot(sub["delay_s"], sub["rms_cte"], marker="o", label=filter_name)
    labels = [
        ("Unique obstacles hit (lower better)", "Safety"),
        ("Minimum clearance (m, higher better)", "Safety margin"),
        ("Intervention rate (%)", "Intrusiveness: how often"),
        ("Mean |Δ steer| (filtered − operator)", "Intrusiveness: how much"),
        ("Mean |Δ throttle| (filtered − operator)", "Intrusiveness: how much"),
        ("RMS CTE (m, post-shield trajectory)", "Tracking under shield"),
    ]
    for ax, (label, title) in zip(axes.flat, labels):
        ax.set_xlabel("Operator command delay (s)")
        ax.set_ylabel(label)
        ax.set_title(title, fontsize=9.5)
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle("Human-in-the-loop delay compensation rounds: safety × intrusiveness")
    fig.tight_layout()
    fig.savefig(fig_dir / "human_delay_compensation_summary.png", dpi=220)
    plt.close(fig)

    pivot = ok.pivot_table(index="delay_s", columns="filter", values="collisions", aggfunc="mean")
    fig, ax = plt.subplots(figsize=(1.45 * len(pivot.columns) + 4, 3.6))
    im = ax.imshow(pivot.values, aspect="auto", cmap="RdYlGn_r")
    ax.set_xticks(range(len(pivot.columns)))
    ax.set_xticklabels(pivot.columns)
    ax.set_yticks(range(len(pivot.index)))
    ax.set_yticklabels([f"{v:.2f}" for v in pivot.index])
    for i in range(pivot.shape[0]):
        for j in range(pivot.shape[1]):
            v = pivot.values[i, j]
            if math.isfinite(v):
                ax.text(j, i, f"{v:.1f}", ha="center", va="center", fontsize=9)
    ax.set_xlabel("Filter")
    ax.set_ylabel("Delay (s)")
    ax.set_title("Mean unique obstacles hit")
    fig.colorbar(im, ax=ax, fraction=0.045)
    fig.tight_layout()
    fig.savefig(fig_dir / "human_delay_collision_heatmap.png", dpi=220)
    plt.close(fig)


def main() -> None:
    args = parse_args()
    if args.quick:
        args.filters = ["none"]
        args.delays = [0.0]
        args.terrains = ["clay"]
        args.paths = ["sinusoidal"]
        args.speeds = [4.0]
        args.bumpiness = [0]
        args.rounds = 1
        args.time = min(args.time, 8.0)
        args.manual_mode = "wasd"

    prof_tag = ""
    if args.latency_profile_json:
        args.latency_profile_json = str(
            Path(args.latency_profile_json).expanduser().resolve())
        prof_tag = Path(args.latency_profile_json).stem
        if args.delays != [0.0]:
            print(f"[5G] latency profile '{prof_tag}' supersedes the --delays "
                  "sweep; collapsing to one 5G condition per cell.")
        args.delays = [0.0]

    out_dir = timestamped_result_dir("human_delay_compensation_rounds")
    write_manifest(out_dir, args, "Human-in-the-loop manual delay compensation rounds.")
    print(f"Output: {out_dir}")

    planned = []
    idx = 0
    for filter_name in args.filters:
        for delay in args.delays:
            for terrain in args.terrains:
                for path in args.paths:
                    for speed in args.speeds:
                        for bump in args.bumpiness:
                            for rep in range(args.rounds):
                                seed = args.base_seed + rep
                                cell = prof_tag if prof_tag else f"delay{delay:.2f}"
                                run_dir = out_dir / "raw" / (
                                    f"{idx:04d}_{filter_name}_{cell}_{terrain}_{path}_v{speed:g}_b{bump}_r{rep}"
                                )
                                cmd = command_for_round(args, run_dir, idx, filter_name, delay, terrain, path, speed, bump, seed)
                                planned.append((idx, filter_name, delay, terrain, path, speed, bump, seed, run_dir, cmd))
                                idx += 1

    plan_rows = [
        {
            "idx": i, "filter": f, "delay_s": d, "terrain": te, "path": pa,
            "speed_mps": sp, "bumpiness": bu, "seed": se,
            "run_dir": str(rd), "command": " ".join(cmd),
        }
        for i, f, d, te, pa, sp, bu, se, rd, cmd in planned
    ]
    pd.DataFrame(plan_rows).to_csv(out_dir / "round_plan.csv", index=False)
    if args.dry_run:
        print(f"Dry run wrote command plan: {out_dir / 'round_plan.csv'}")
        return

    rows: list[dict] = []
    total = len(planned)
    for i, filter_name, delay, terrain, path, speed, bump, seed, run_dir, cmd in planned:
        for name in ("collision_log.csv", "mppi_shield_log.csv", "nmpc_shield_log.csv", "cbf_filter_log.csv"):
            p = LOGS_DIR / name
            if p.exists():
                p.unlink()
        print(f"\n[{i + 1}/{total}] filter={filter_name} delay={delay:.2f}s "
              f"{terrain}/{path} v={speed:g} b={bump}")
        print(f"Raw output: {run_dir}")
        if not args.auto_start:
            input("Press Enter when the driver is ready for this round...")
        hud_proc = _maybe_launch_hud(args, i, run_dir)
        created_after = time.time()
        try:
            rc, wall_s, text = run_round(cmd, run_dir, args.timeout)
        finally:
            _stop_hud(hud_proc)
        collision_csv, shield_csv = collect_global_logs(run_dir, created_after)
        row = {
            "experiment": "human_delay_compensation_rounds",
            "filter": filter_name,
            "variant": f"{filter_name}_delay{delay:.2f}",
            "delay_s": delay,
            "camera_delay_s": delay * args.camera_delay_scale,
            "latency_profile": prof_tag,
            "terrain": terrain,
            "path": path,
            "speed_mps": speed,
            "bumpiness": bump,
            "seed": seed,
            "run_dir": str(run_dir),
            "rc": rc,
            "wall_s": wall_s,
            "status": "ok" if rc == 0 else f"exit_{rc}",
            "sim_diag_csv": str(run_dir / "sim_diag.csv"),
            "collision_csv": collision_csv,
            "shield_csv": shield_csv,
        }
        row.update(parse_log_summary(text))
        row.update(parse_sim_diag(run_dir / "sim_diag.csv", path, speed, args.lead_in))
        row.update(parse_collision_csv(Path(collision_csv) if collision_csv else None))
        row.update(parse_shield_csv(Path(shield_csv) if shield_csv else None))
        rows.append(row)
        pd.DataFrame(rows).to_csv(out_dir / "results.csv", index=False)
        print(f"    {row['status']}: collisions={row.get('collisions', 0)} "
              f"rms_cte={row.get('rms_cte_m', math.nan):.3f} "
              f"speed_ratio={row.get('speed_ratio', math.nan):.2f}")

    results_csv = out_dir / "results.csv"
    results_df = pd.DataFrame(rows)
    results_df.to_csv(results_csv, index=False)
    # Metric columns are only present when at least one run produced them
    # (e.g. intervention_rate_pct needs a shield run). Backfill any missing
    # metric column with NaN so the aggregation works for any filter subset.
    # Intrusiveness metrics: intervention rate and per-tick |Δcmd| are what
    # the paper argues about for the latency-aware HIL filter. Backfill any
    # column that no run produced (e.g. dsteer is shield-only) with NaN.
    for col in ("rms_cte_m", "speed_ratio", "collisions",
                "min_clearance_m", "intervention_rate_pct",
                "mean_abs_dsteer", "mean_abs_dthrottle", "near_misses"):
        if col not in results_df.columns:
            results_df[col] = math.nan
    summary = results_df.groupby(["filter", "delay_s"], sort=False).agg(
        n_runs=("status", "count"),
        n_ok=("status", lambda s: int((s == "ok").sum())),
        rms_cte_m_mean=("rms_cte_m", "mean"),
        rms_cte_m_std=("rms_cte_m", "std"),
        speed_ratio_mean=("speed_ratio", "mean"),
        speed_ratio_std=("speed_ratio", "std"),
        collisions_mean=("collisions", "mean"),
        collisions_std=("collisions", "std"),
        near_misses_mean=("near_misses", "mean"),
        min_clearance_m_mean=("min_clearance_m", "mean"),
        min_clearance_m_std=("min_clearance_m", "std"),
        intervention_rate_pct_mean=("intervention_rate_pct", "mean"),
        intervention_rate_pct_std=("intervention_rate_pct", "std"),
        mean_abs_dsteer_mean=("mean_abs_dsteer", "mean"),
        mean_abs_dthrottle_mean=("mean_abs_dthrottle", "mean"),
    ).reset_index()
    summary.to_csv(out_dir / "summary_by_filter_delay.csv", index=False)
    save_summary_markdown(
        out_dir,
        "Human Delay Compensation Rounds",
        summary,
        [
            "Noise policy: sensor noise enabled in every run.",
            (f"Delay policy: time-varying 5G latency profile `{prof_tag}` on "
             "every round -- control/manual channels = command uplink, camera "
             "channel = asymmetric video downlink (supersedes fixed delays).")
            if prof_tag else
            ("Delay policy: each round delays both the operator command path "
             "(`--manual-input-delay`, plus `--teleop-delay` so the predictive "
             "filter horizon is delay-aware) and the driver POV camera feed "
             "(`--camera-input-delay`)."),
            f"Camera delay = {args.camera_delay_scale:g} x command delay "
            "(--camera-delay-scale; 1.0 = symmetric link)."
            if not prof_tag else
            "Live HMI overlay (hil_hud.py) attached per round."
            if args.live_hud else "Constant-delay sweep mode.",
        ],
    )
    plot_figures(results_csv, out_dir)
    print(f"Done: {out_dir}")


if __name__ == "__main__":
    main()
