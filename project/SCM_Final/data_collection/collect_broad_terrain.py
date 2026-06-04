#!/usr/bin/env python3
"""Broad multi-axis trace collector for the terrain estimator.

Each LHS terrain cell is exercised under many combinations of:
  * excitation mode   — scripted open-loop OR closed-loop NMPC tracking
  * cruise-bias seed  — scripted only, 4 levels covering ~3/5/7/9 m/s
  * target speed      — closed-loop only, {4, 6, 8} m/s
  * reference path    — closed-loop only, {sinusoidal, lane_change,
                                            double_lane_change}
  * bumpiness         — {0, 2, 4} (Perlin terrain elevation noise)

For each scenario the collector writes one CSV in the trainer's standard
format (same columns as ``collect_rich_excitation.py``).

The closed-loop variant launches BOTH ``chrono_sim_node.py`` and
``acados_mpc_controller_node.py`` as subprocesses; the collector itself
runs an additional passive ``VehicleState`` subscriber and never
publishes commands — the controller is in charge of the actuators.

Designed for parallel execution via ``ProcessPoolExecutor``: each task
strides its ZMQ ports by ``4*idx`` and gets its own acados build cache
(``ACADOS_UNIQUE_BUILD_DIR=1``) so cold codegen never races.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

REPO = Path(__file__).resolve().parent.parent
SIM = REPO / "simulation"
if str(SIM) not in sys.path:
    sys.path.insert(0, str(SIM))

from hil_messages import (                                     # noqa: E402
    ControlCommand, SimStatus, VehicleState,
    ZMQPublisher, ZMQSubscriber,
    sim_sub_endpoint, ctrl_pub_endpoint, ctrl_sub_endpoint,
)

DEFAULT_OUT = REPO / "data" / "terrain_estimator" / "traces_broad_v7"
# broad_v3 LHS yamls cover n ∈ [0.30, 1.30], phi ∈ [10, 30] across 200 cells.
DEFAULT_YAML_DIR = REPO / "config" / "terrain_yamls" / "broad_v3"

# ---- CSV schema mirrors the trainer's expectation -------------------------
CSV_HEADER = [
    "t", "terrain", "n_true", "throttle_cmd", "steer_cmd",
    "u", "v", "omega", "ax", "ay",
    "az", "omega_x", "omega_y",
    "wheel_omega_fl", "wheel_omega_fr",
    "wheel_omega_rl", "wheel_omega_rr",
    "steering_angle",
]

PATHS_CLOSED_LOOP = ("sinusoidal", "lane_change", "double_lane_change")
SPEEDS_CLOSED_LOOP = (4.0, 6.0, 8.0)
BUMPS = (0, 2, 4)
CRUISE_BIAS_SEEDS = (0, 1, 2, 3)   # maps to bias_levels in scripted_excitation


# =========================================================================
# Scenario
# =========================================================================
@dataclass(frozen=True)
class Scenario:
    """Pickle-friendly per-run description."""
    idx: int
    label: str
    mode: str               # "scripted" | "closed_loop"
    lhs_label: str
    n_true: float
    phi_true: float
    terrain_yaml: str
    preset_proxy: str       # closest of clay/dirt/sand
    bumpiness: int

    # scripted only
    cruise_bias_seed: int

    # closed-loop only
    speed: float
    path: str
    sine_amplitude: float
    sine_wavelength: float

    # runtime
    sim_port: int
    ctrl_port: int
    duration: float
    out_csv: str
    out_log: str


# =========================================================================
# Scripted excitation (lifted from collect_rich_excitation.py)
# =========================================================================
def _scripted_cmd(t_rel: float, seed: int) -> Tuple[float, float, float]:
    """Return (throttle, steer_cmd, brake) for the open-loop probe."""
    ph = 0.13 * seed
    bias_levels = (0.0, 0.15, 0.30, 0.45)
    cb = bias_levels[seed % 4]

    def cl(x):
        return float(np.clip(x, 0.0, 1.0))

    if t_rel < 4.0:
        throttle = cl(0.35 + cb)
        steer = 0.25 * math.sin(2 * math.pi * (t_rel + ph) / 3.0)
        brake = 0.0
    elif t_rel < 10.0:
        throttle = cl(0.40 + cb)
        steer = 0.75 * math.sin(2 * math.pi * (t_rel + ph) / 2.2)
        brake = 0.0
    elif t_rel < 16.0:
        throttle = cl(0.85 + cb * 0.3)
        steer = 0.30 * math.sin(2 * math.pi * (t_rel + ph) / 2.8)
        brake = 0.0
    elif t_rel < 22.0:
        throttle = cl(0.05 + cb * 0.4)
        steer = 0.80 * math.sin(2 * math.pi * (t_rel + ph) / 1.9)
        brake = 0.35 if (t_rel - 16.0) < 3.0 else 0.0
    else:
        tc = t_rel - 22.0
        f = 0.3 + (1.5 - 0.3) * (tc / 8.0)
        steer = 0.55 * math.sin(2 * math.pi * f * tc + ph)
        throttle = cl(0.35 + cb + 0.25 *
                      math.sin(2 * math.pi * (tc + ph) / 3.5))
        brake = 0.0

    return throttle, steer, brake


# =========================================================================
# Per-row writer
# =========================================================================
def _row_from_state(t_rel: float, preset: str, n_true: float,
                    throttle: float, steer_cmd: float,
                    msg: VehicleState) -> list:
    return [
        f"{t_rel:.4f}", preset, f"{n_true:.3f}",
        f"{throttle:.3f}", f"{steer_cmd:.4f}",
        f"{msg.u:.4f}", f"{msg.v:.4f}", f"{msg.omega:.4f}",
        f"{msg.ax:.4f}", f"{msg.ay:.4f}",
        f"{msg.az:.4f}", f"{msg.omega_x:.4f}", f"{msg.omega_y:.4f}",
        f"{msg.wheel_omega_fl:.4f}", f"{msg.wheel_omega_fr:.4f}",
        f"{msg.wheel_omega_rl:.4f}", f"{msg.wheel_omega_rr:.4f}",
        f"{msg.steering_angle:.4f}",
    ]


# =========================================================================
# Scripted run
# =========================================================================
def _run_scripted(scn: Scenario) -> dict:
    sim_script = SIM / "chrono_sim_node.py"
    sim_cmd = [
        sys.executable, "-u", str(sim_script),
        "--time", f"{scn.duration + 5.0}", "--speed", "5",
        "--terrain", scn.preset_proxy, "--path", "sinusoidal",
        "--vis-mode", "none",
        "--sim-port", str(scn.sim_port),
        "--ctrl-host", "localhost",
        "--ctrl-port", str(scn.ctrl_port),
        "--no-wait-for-controller",
        "--terrain-config", scn.terrain_yaml,
        "--bumpiness", str(scn.bumpiness),
    ]
    log_f = open(scn.out_log, "w")
    sim_proc = subprocess.Popen(sim_cmd, stdout=log_f, stderr=subprocess.STDOUT)

    state_sub = ZMQSubscriber(sim_sub_endpoint("localhost", scn.sim_port))
    ctrl_pub = ZMQPublisher(ctrl_pub_endpoint(scn.ctrl_port))
    time.sleep(2.0)

    rows: list[list] = []
    seq = 0
    t0_sim: Optional[float] = None
    timeout_count = 0
    ok = False

    try:
        while True:
            res = state_sub.recv(timeout_ms=500)
            if res is None:
                timeout_count += 1
                if timeout_count > 30:
                    break
                continue
            timeout_count = 0
            topic, msg = res
            if isinstance(msg, SimStatus) and msg.event == "stop":
                break
            if not isinstance(msg, VehicleState):
                continue

            t = float(msg.time)
            if t0_sim is None:
                t0_sim = t
            t_rel = t - t0_sim
            throttle, steer_cmd, brake = _scripted_cmd(
                t_rel, scn.cruise_bias_seed)
            cmd = ControlCommand(
                time=t, wall_time=time.time(), seq=seq,
                steering=steer_cmd, throttle=throttle, braking=brake,
                delta=0.0, acceleration=0.0,
                delta_dot=0.0, jerk=0.0,
            )
            ctrl_pub.send(cmd)
            seq += 1
            rows.append(_row_from_state(
                t_rel, scn.preset_proxy, scn.n_true,
                throttle, steer_cmd, msg))
            if t_rel >= scn.duration:
                ok = True
                break
    finally:
        state_sub.close()
        ctrl_pub.close()
        sim_proc.terminate()
        try:
            sim_proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            sim_proc.kill()
            sim_proc.wait()
        log_f.close()

    if ok and len(rows) >= 200:
        with open(scn.out_csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(CSV_HEADER)
            w.writerows(rows)
        return {"label": scn.label, "ok": True, "rows": len(rows),
                "mode": "scripted"}
    return {"label": scn.label, "ok": False, "rows": len(rows),
            "mode": "scripted"}


# =========================================================================
# Closed-loop run
# =========================================================================
def _run_closed_loop(scn: Scenario) -> dict:
    sim_script = SIM / "chrono_sim_node.py"
    ctrl_script = SIM / "acados_mpc_controller_node.py"

    sim_cmd = [
        sys.executable, "-u", str(sim_script),
        "--time", f"{scn.duration + 6.0}", "--speed", str(scn.speed),
        "--terrain", scn.preset_proxy,
        "--path", scn.path,
        "--sine-amplitude", str(scn.sine_amplitude),
        "--sine-wavelength", str(scn.sine_wavelength),
        "--vis-mode", "none",
        "--sim-port", str(scn.sim_port),
        "--ctrl-host", "localhost",
        "--ctrl-port", str(scn.ctrl_port),
        "--lead-in", "5.0",
        "--terrain-config", scn.terrain_yaml,
        "--bumpiness", str(scn.bumpiness),
    ]
    ctrl_cmd = [
        sys.executable, "-u", str(ctrl_script),
        "--model", "nn",
        "--nn-model", "vehicle_rate_64_32_lhs",
        "--sim-host", "localhost",
        "--sim-port", str(scn.sim_port),
        "--ctrl-port", str(scn.ctrl_port),
        "--speed", str(scn.speed),
    ]

    env = os.environ.copy()
    # Share the acados build cache across every closed-loop run. The serial
    # prewarm (first closed-loop scenario in main()) populates this cache;
    # all subsequent runs find it warm and skip ~25s of codegen. UNIQUE
    # build dirs would force cold codegen on every run because each
    # controller subprocess gets a fresh PID.
    env.pop("ACADOS_UNIQUE_BUILD_DIR", None)
    env.setdefault("ACADOS_SOURCE_DIR", str(Path.home() /
                                            "Documents/sbel/acados"))

    log_f = open(scn.out_log, "w")
    sim_proc = subprocess.Popen(sim_cmd, stdout=log_f,
                                 stderr=subprocess.STDOUT, env=env)
    # Give the sim a small head-start so the controller's ready ping
    # lands on a live socket.
    time.sleep(0.5)
    ctrl_proc = subprocess.Popen(ctrl_cmd, stdout=log_f,
                                  stderr=subprocess.STDOUT, env=env)

    state_sub = ZMQSubscriber(sim_sub_endpoint("localhost", scn.sim_port))
    cmd_sub = ZMQSubscriber(ctrl_sub_endpoint("localhost", scn.ctrl_port))
    # Allow extra time for acados codegen on cold workers.
    cold_wait = 30.0
    rows: list[list] = []
    t0_sim: Optional[float] = None
    timeout_count = 0
    ok = False
    last_throttle = 0.0
    last_steer_cmd = 0.0
    wait_start = time.time()

    try:
        while True:
            # Drain any newer ControlCommand (CONFLATE keeps only latest).
            cres = cmd_sub.recv(timeout_ms=0)
            if cres is not None:
                _, cmsg = cres
                if isinstance(cmsg, ControlCommand):
                    thr = float(cmsg.throttle) if math.isfinite(
                        cmsg.throttle) else last_throttle
                    brk = float(cmsg.braking) if math.isfinite(
                        cmsg.braking) else 0.0
                    last_throttle = float(np.clip(thr - brk, -1.0, 1.0))
                    if math.isfinite(cmsg.steering):
                        last_steer_cmd = float(np.clip(
                            cmsg.steering, -1.0, 1.0))

            timeout_ms = 1000 if (time.time() - wait_start) < cold_wait else 500
            res = state_sub.recv(timeout_ms=timeout_ms)
            if res is None:
                timeout_count += 1
                if timeout_count > 60:
                    break
                continue
            timeout_count = 0
            topic, msg = res
            if isinstance(msg, SimStatus) and msg.event == "stop":
                break
            if not isinstance(msg, VehicleState):
                continue
            t = float(msg.time)
            if t0_sim is None:
                t0_sim = t
            t_rel = t - t0_sim
            rows.append(_row_from_state(
                t_rel, scn.preset_proxy, scn.n_true,
                last_throttle, last_steer_cmd, msg))
            if t_rel >= scn.duration:
                ok = True
                break
    finally:
        state_sub.close()
        cmd_sub.close()
        for proc in (ctrl_proc, sim_proc):
            proc.terminate()
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
        log_f.close()

    if ok and len(rows) >= 200:
        with open(scn.out_csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(CSV_HEADER)
            w.writerows(rows)
        return {"label": scn.label, "ok": True, "rows": len(rows),
                "mode": "closed_loop"}
    return {"label": scn.label, "ok": False, "rows": len(rows),
            "mode": "closed_loop"}


def _worker(scn: Scenario) -> dict:
    if Path(scn.out_csv).exists() and Path(scn.out_csv).stat().st_size > 4096:
        return {"label": scn.label, "ok": True, "rows": -1,
                "mode": scn.mode, "skipped": True}
    t0 = time.time()
    try:
        if scn.mode == "scripted":
            r = _run_scripted(scn)
        else:
            r = _run_closed_loop(scn)
    except Exception as e:
        r = {"label": scn.label, "ok": False, "rows": 0,
             "mode": scn.mode, "error": str(e)}
    r["wall_s"] = round(time.time() - t0, 1)
    return r


# =========================================================================
# Scenario building
# =========================================================================
def _parse_label(yaml_path: Path) -> Tuple[str, float, float]:
    """Decode a rich-LHS YAML filename → (label, n_true, phi_true)."""
    stem = yaml_path.stem            # rich0007_n0612_phi241
    parts = stem.split("_")
    n_milli = int(parts[1][1:])
    phi_deci = int(parts[2][3:])
    return stem, n_milli / 1000.0, phi_deci / 10.0


def _closest_preset(n_val: float) -> str:
    # Same mapping convention as the v5 collector (clay≈0.5, dirt≈0.7, sand≈1.1)
    if n_val < 0.6:
        return "clay"
    if n_val < 0.9:
        return "dirt"
    return "sand"


def build_scenarios(yaml_paths: List[Path], out_dir: Path,
                    duration: float, base_port: int,
                    sine_amp: float, sine_wave: float,
                    skip_existing_scripted_bump0: bool) -> List[Scenario]:
    """20 scenarios per LHS cell → 180 stable cells × 20 = 3600 traces.

    Per cell:
      • 4 scripted, bump=0 (cruise-bias 0..3)
      • 4 scripted, bump=4 (cruise-bias 0..3)
      • 9 closed-loop, bump=0 (3 speeds × 3 paths)
      • 3 closed-loop, bump=4 (3 speeds × sinusoidal)
    """
    scenarios: List[Scenario] = []
    idx = 0
    for yp in yaml_paths:
        label, n_true, phi_true = _parse_label(yp)
        preset = _closest_preset(n_true)

        # --- scripted, bump=0 × 4 cruise-bias seeds ---
        for seed in CRUISE_BIAS_SEEDS:
            tag = f"{label}_scr_b0_s{seed}"
            scenarios.append(Scenario(
                idx=idx, label=tag, mode="scripted",
                lhs_label=label, n_true=n_true, phi_true=phi_true,
                terrain_yaml=str(yp), preset_proxy=preset,
                bumpiness=0, cruise_bias_seed=seed,
                speed=5.0, path="sinusoidal",
                sine_amplitude=sine_amp, sine_wavelength=sine_wave,
                sim_port=base_port + 4 * idx, ctrl_port=base_port + 4 * idx + 1,
                duration=duration,
                out_csv=str(out_dir / f"{tag}.csv"),
                out_log=str(out_dir / "logs" / f"{tag}.log"),
            ))
            idx += 1

        # --- scripted, bump=4 × 4 cruise-bias seeds ---
        for seed in CRUISE_BIAS_SEEDS:
            tag = f"{label}_scr_b4_s{seed}"
            scenarios.append(Scenario(
                idx=idx, label=tag, mode="scripted",
                lhs_label=label, n_true=n_true, phi_true=phi_true,
                terrain_yaml=str(yp), preset_proxy=preset,
                bumpiness=4, cruise_bias_seed=seed,
                speed=5.0, path="sinusoidal",
                sine_amplitude=sine_amp, sine_wavelength=sine_wave,
                sim_port=base_port + 4 * idx, ctrl_port=base_port + 4 * idx + 1,
                duration=duration,
                out_csv=str(out_dir / f"{tag}.csv"),
                out_log=str(out_dir / "logs" / f"{tag}.log"),
            ))
            idx += 1

        # --- closed-loop bump=0: 3 speeds × 3 paths ---
        for path in PATHS_CLOSED_LOOP:
            for speed in SPEEDS_CLOSED_LOOP:
                tag = f"{label}_cl_{path[:4]}_v{int(round(speed))}_b0"
                scenarios.append(Scenario(
                    idx=idx, label=tag, mode="closed_loop",
                    lhs_label=label, n_true=n_true, phi_true=phi_true,
                    terrain_yaml=str(yp), preset_proxy=preset,
                    bumpiness=0, cruise_bias_seed=0,
                    speed=speed, path=path,
                    sine_amplitude=sine_amp, sine_wavelength=sine_wave,
                    sim_port=base_port + 4 * idx,
                    ctrl_port=base_port + 4 * idx + 1,
                    duration=duration,
                    out_csv=str(out_dir / f"{tag}.csv"),
                    out_log=str(out_dir / "logs" / f"{tag}.log"),
                ))
                idx += 1

        # --- closed-loop bump=4: 3 speeds × sinusoidal ---
        for speed in SPEEDS_CLOSED_LOOP:
            tag = f"{label}_cl_sinu_v{int(round(speed))}_b4"
            scenarios.append(Scenario(
                idx=idx, label=tag, mode="closed_loop",
                lhs_label=label, n_true=n_true, phi_true=phi_true,
                terrain_yaml=str(yp), preset_proxy=preset,
                bumpiness=4, cruise_bias_seed=0,
                speed=speed, path="sinusoidal",
                sine_amplitude=sine_amp, sine_wavelength=sine_wave,
                sim_port=base_port + 4 * idx, ctrl_port=base_port + 4 * idx + 1,
                duration=duration,
                out_csv=str(out_dir / f"{tag}.csv"),
                out_log=str(out_dir / "logs" / f"{tag}.log"),
            ))
            idx += 1

    return scenarios


# =========================================================================
# Main
# =========================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--yaml-dir", type=Path, default=DEFAULT_YAML_DIR)
    ap.add_argument("--max-cells", type=int, default=None,
                    help="Cap on number of LHS cells (defaults: all *.yaml).")
    ap.add_argument("--duration", type=float, default=25.0)
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--base-port", type=int, default=27000)
    ap.add_argument("--smoke", type=int, default=0,
                    help="If >0, run only this many scenarios (sampled "
                         "round-robin across cells/modes) and stop.")
    ap.add_argument("--no-skip-existing-scripted-bump0", action="store_true",
                    help="Re-collect scripted+bump=0 traces even if "
                         "traces_vertical_v5 already has them.")
    ap.add_argument("--sine-amplitude", type=float, default=2.0)
    ap.add_argument("--sine-wavelength", type=float, default=30.0)
    ap.add_argument("--n-min", type=float, default=0.40,
                    help="Filter LHS yamls with n below this floor (SCM "
                         "physics is unstable for very low n).")
    args = ap.parse_args()

    out_dir: Path = args.out
    (out_dir / "logs").mkdir(parents=True, exist_ok=True)

    yaml_paths_all = sorted(args.yaml_dir.glob("*.yaml"))
    # SCM physics is unstable below n≈0.40 (vehicle CG diverges). Filter to
    # the stable simulable range before building scenarios.
    yaml_paths = [p for p in yaml_paths_all if _parse_label(p)[1] >= args.n_min]
    skipped_low_n = len(yaml_paths_all) - len(yaml_paths)
    if skipped_low_n:
        print(f"[broad] filtered out {skipped_low_n} LHS cells "
              f"with n < {args.n_min} (SCM unstable)")
    if args.max_cells is not None:
        # Sub-sample uniformly to keep n-coverage even.
        if len(yaml_paths) > args.max_cells:
            stride = len(yaml_paths) / args.max_cells
            yaml_paths = [yaml_paths[int(i * stride)]
                          for i in range(args.max_cells)]
    if not yaml_paths:
        print(f"No LHS YAMLs at {args.yaml_dir}", file=sys.stderr)
        sys.exit(1)

    scenarios = build_scenarios(
        yaml_paths,
        out_dir=out_dir,
        duration=args.duration,
        base_port=args.base_port,
        sine_amp=args.sine_amplitude,
        sine_wave=args.sine_wavelength,
        skip_existing_scripted_bump0=not args.no_skip_existing_scripted_bump0,
    )

    if args.smoke and args.smoke < len(scenarios):
        scenarios = scenarios[:args.smoke]
        print(f"[smoke] truncated to {len(scenarios)} scenarios")

    n_scripted = sum(1 for s in scenarios if s.mode == "scripted")
    n_cl = sum(1 for s in scenarios if s.mode == "closed_loop")
    print(f"[broad] {len(scenarios)} scenarios "
          f"(scripted={n_scripted}, closed_loop={n_cl}) "
          f"across {len(yaml_paths)} LHS cells")
    print(f"[broad] output dir: {out_dir}")

    manifest_path = out_dir / "manifest.csv"
    write_header = not manifest_path.exists()

    t_start = time.time()
    completed: List[dict] = []
    with manifest_path.open("a", newline="") as mf:
        mw = csv.writer(mf)
        if write_header:
            mw.writerow(["label", "mode", "lhs_label",
                         "n_true", "phi_true", "speed", "path",
                         "bumpiness", "cruise_bias_seed",
                         "ok", "rows", "wall_s", "csv"])
        # Cache prewarm: closed-loop runs codegen acados on first call. Run
        # one closed-loop scenario solo so workers don't race on /tmp dirs.
        first_cl_idx = next(
            (i for i, s in enumerate(scenarios) if s.mode == "closed_loop"),
            None,
        )
        prewarm_done = False
        if first_cl_idx is not None and len(scenarios) > 1:
            print("[broad] prewarming acados cache (1 closed-loop scn)...",
                  flush=True)
            scn = scenarios[first_cl_idx]
            r = _worker(scn)
            completed.append(r)
            mw.writerow([
                scn.label, scn.mode, scn.lhs_label,
                f"{scn.n_true:.3f}", f"{scn.phi_true:.2f}",
                f"{scn.speed:.1f}", scn.path,
                scn.bumpiness, scn.cruise_bias_seed,
                int(r["ok"]), r["rows"], r.get("wall_s", -1.0), scn.out_csv,
            ])
            mf.flush()
            tag = "✓" if r["ok"] else "✗"
            print(f"  [prewarm] {tag} {scn.label} rows={r['rows']} "
                  f"wall={r.get('wall_s', -1):.1f}s", flush=True)
            scenarios = scenarios[:first_cl_idx] + scenarios[first_cl_idx + 1:]
            prewarm_done = True

        with ProcessPoolExecutor(max_workers=args.workers) as ex:
            futs = {ex.submit(_worker, scn): scn for scn in scenarios}
            for fut in as_completed(futs):
                scn = futs[fut]
                try:
                    r = fut.result()
                except Exception as e:
                    r = {"label": scn.label, "ok": False, "rows": 0,
                         "mode": scn.mode, "error": str(e), "wall_s": -1.0}
                completed.append(r)
                mw.writerow([
                    scn.label, scn.mode, scn.lhs_label,
                    f"{scn.n_true:.3f}", f"{scn.phi_true:.2f}",
                    f"{scn.speed:.1f}", scn.path,
                    scn.bumpiness, scn.cruise_bias_seed,
                    int(r["ok"]), r["rows"], r.get("wall_s", -1.0), scn.out_csv,
                ])
                mf.flush()
                done = len(completed)
                ok_rate = sum(1 for x in completed if x["ok"]) / max(done, 1)
                elapsed = (time.time() - t_start) / 60.0
                tag = "✓" if r["ok"] else "✗"
                print(f"  [{done:4d}/{len(scenarios) + (1 if prewarm_done else 0)}]"
                      f" {tag} {scn.mode:<11s} {scn.label[:38]:<38s}  "
                      f"rows={r['rows']:>4} "
                      f"wall={r.get('wall_s', -1):5.1f}s "
                      f"ok_rate={ok_rate:.0%} elapsed={elapsed:.1f}min",
                      flush=True)

    n_ok = sum(1 for r in completed if r["ok"])
    print(f"\n[broad] done — {n_ok}/{len(completed)} ok, "
          f"{(time.time() - t_start) / 60.0:.1f} min wall")


if __name__ == "__main__":
    main()
