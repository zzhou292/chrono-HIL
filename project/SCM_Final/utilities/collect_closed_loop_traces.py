#!/usr/bin/env python3
"""Collect labeled vehicle-state traces *with the MPC closing the loop*.

The original ``collect_terrain_traces.py`` drives the vehicle with an
open-loop sinusoidal steer command + constant throttle.  The learned
terrain estimator trained on that data then jitters in closed-loop MPC
because the windowed-feature distribution is different (MPC modulates
throttle to track speed, steering tracks the path rather than tracing a
pure sine, lateral excitation is shape-tied to the path, etc.).

This script fixes the train/eval gap by:

1.  Launching ``chrono_sim_node`` headless (with a custom YAML or preset
    terrain), exactly as the open-loop collector does;
2.  Launching ``acados_mpc_controller_node`` as a separate subprocess so
    it generates real MPC commands;
3.  Subscribing to the sim's ``VehicleState`` topic *and* the controller's
    ``ControlCommand`` topic, joining them on simulation time, and
    writing a CSV in the same column layout the trainer already
    consumes.

The written CSVs land in ``data/terrain_traces_closedloop/`` so they
remain easy to mix-or-match with the open-loop set during retraining.
"""

from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from itertools import product
from pathlib import Path
from typing import Optional, Dict

ROOT = Path(__file__).resolve().parent.parent
SIM_DIR = ROOT / "simulation"
DATA_DIR = ROOT / "data"
sys.path.insert(0, str(SIM_DIR))

from hil_messages import (
    ControlCommand, SimStatus, VehicleState,
    ZMQSubscriber,
    sim_sub_endpoint, ctrl_sub_endpoint,
)
from collect_rich_excitation import build_lhs_specs


TRUE_N = {"clay": 0.5, "dirt": 0.7, "sand": 1.1}

CSV_HEADER = [
    "t", "terrain", "n_true", "throttle_cmd", "steer_cmd",
    "u", "v", "omega", "ax", "ay",
    "wheel_omega_fl", "wheel_omega_fr",
    "wheel_omega_rl", "wheel_omega_rr",
    "steering_angle",
]


def _conda_run(args: list[str]) -> list[str]:
    """Run with the interpreter that launched this collector."""
    return [sys.executable, *args]


def collect_one_closed_loop(
    *,
    terrain: str,                 # preset name (clay/dirt/sand) — controls
                                  # MPC initial NN-CBF lookup; the actual
                                  # SCM physics is set by the YAML if given
    n_true: float,                # ground-truth n written into the CSV
    throttle_cap: float,          # passed through as MPC v_target proxy
    duration: float,
    sim_port: int,
    ctrl_port: int,
    out_csv: Path,
    terrain_yaml: Optional[Path] = None,
    speed: float = 5.0,
    path: str = "sinusoidal",
    lead_in: float = 5.0,
    sine_amplitude: float = 2.0,
    sine_wavelength: float = 30.0,
    seed: int = 0,
    nn_model: str = "vehicle_rate_64_32_lhs",
) -> bool:

    if out_csv.exists() and out_csv.stat().st_size > 4096:
        print(f"  [skip] {out_csv.name} already populated")
        return True
    out_csv.parent.mkdir(parents=True, exist_ok=True)

    sim_script = SIM_DIR / "chrono_sim_node.py"
    ctrl_script = SIM_DIR / "acados_mpc_controller_node.py"

    sim_args = [
        str(sim_script),
        "--time", str(duration + 8.0),
        "--speed", str(speed),
        "--terrain", terrain, "--path", path,
        "--lead-in", str(lead_in),
        "--vis-mode", "none",
        "--sim-port", str(sim_port),
        "--ctrl-host", "localhost",
        "--ctrl-port", str(ctrl_port),
    ]
    if terrain_yaml is not None:
        sim_args += ["--terrain-config", str(terrain_yaml)]
    if path == "sinusoidal":
        sim_args += ["--sine-amplitude", str(sine_amplitude),
                     "--sine-wavelength", str(sine_wavelength)]

    ctrl_args = [
        str(ctrl_script),
        "--time", str(duration + 8.0),
        "--speed", str(speed),
        "--terrain", terrain, "--path", path,
        "--lead-in", str(lead_in),
        "--model", "nn", "--nn-model", nn_model,
        "--sim-host", "localhost",
        "--sim-port", str(sim_port),
        "--ctrl-port", str(ctrl_port),
        "--no-csv", "--no-plot",
    ]
    if path == "sinusoidal":
        ctrl_args += ["--sine-amplitude", str(sine_amplitude),
                      "--sine-wavelength", str(sine_wavelength)]

    print(f"  launching sim+ctrl: terrain={terrain}  n_true={n_true:.2f}  "
          f"speed={speed}  yaml={terrain_yaml}")
    sim_proc = subprocess.Popen(_conda_run(sim_args),
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)
    time.sleep(2.0)   # let sim bind sockets
    ctrl_proc = subprocess.Popen(_conda_run(ctrl_args),
                                 stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)

    state_sub = ZMQSubscriber(sim_sub_endpoint("localhost", sim_port))
    cmd_sub = ZMQSubscriber(ctrl_sub_endpoint("localhost", ctrl_port))
    time.sleep(1.5)

    rows: list[list] = []
    last_cmd: Optional[ControlCommand] = None
    t0_sim: Optional[float] = None
    last_log_t = 0.0
    timeout_count = 0

    try:
        # Drive a short busy loop pulling messages from both sockets.
        while True:
            # Drain any pending control commands first (cheap).
            while True:
                cmd_res = cmd_sub.recv(timeout_ms=0)
                if cmd_res is None:
                    break
                _, cmsg = cmd_res
                if isinstance(cmsg, ControlCommand):
                    last_cmd = cmsg

            res = state_sub.recv(timeout_ms=500)
            if res is None:
                timeout_count += 1
                if timeout_count > 30:
                    print("    [warn] no state messages — aborting run")
                    return False
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

            thr = float(last_cmd.throttle) if last_cmd is not None else 0.0
            steer = float(last_cmd.steering) if last_cmd is not None else 0.0

            rows.append([
                f"{t_rel:.4f}", terrain, f"{n_true:.3f}",
                f"{thr:.3f}", f"{steer:.4f}",
                f"{msg.u:.4f}", f"{msg.v:.4f}", f"{msg.omega:.4f}",
                f"{msg.ax:.4f}", f"{msg.ay:.4f}",
                f"{msg.wheel_omega_fl:.4f}", f"{msg.wheel_omega_fr:.4f}",
                f"{msg.wheel_omega_rl:.4f}", f"{msg.wheel_omega_rr:.4f}",
                f"{msg.steering_angle:.4f}",
            ])

            if t_rel - last_log_t > 5.0:
                print(f"    t={t_rel:5.1f}s  u={msg.u:.2f} v={msg.v:+.2f} "
                      f"omega={msg.omega:+.2f}  thr_cmd={thr:.2f}  "
                      f"steer_cmd={steer:+.2f}")
                last_log_t = t_rel

            if t_rel >= duration:
                break

    finally:
        state_sub.close()
        cmd_sub.close()
        for proc, name in ((ctrl_proc, "ctrl"), (sim_proc, "sim")):
            proc.terminate()
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()

    if len(rows) < 100:
        print(f"    [warn] only {len(rows)} rows captured — discarding")
        return False
    # Drop rows from before the controller actually started publishing
    # (steer_cmd will be exactly 0.0000 in that prefix).
    while rows and rows[0][4] == "0.0000" and rows[0][3] == "0.000":
        rows.pop(0)
    if len(rows) < 100:
        print(f"    [warn] only {len(rows)} rows after ctrl-warmup trim — discarding")
        return False

    with out_csv.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_HEADER)
        w.writerows(rows)
    print(f"    [ok] saved {len(rows)} rows -> {out_csv.name}")
    return True


@dataclass(frozen=True)
class ClosedLoopTask:
    idx: int
    total: int
    label: str
    terrain: str
    n_true: float
    phi_true: Optional[float]
    terrain_yaml: Optional[str]
    speed: float
    sine_amplitude: float
    sine_wavelength: float
    seed: int
    duration: float
    lead_in: float
    sim_port: int
    ctrl_port: int
    out_csv: str
    nn_model: str


def _run_one_task(task: ClosedLoopTask) -> Dict:
    os.environ.setdefault("ACADOS_UNIQUE_BUILD_DIR", "1")
    t0 = time.time()
    ok = collect_one_closed_loop(
        terrain=task.terrain,
        n_true=task.n_true,
        throttle_cap=0.65,
        duration=task.duration,
        sim_port=task.sim_port,
        ctrl_port=task.ctrl_port,
        out_csv=Path(task.out_csv),
        terrain_yaml=Path(task.terrain_yaml) if task.terrain_yaml else None,
        speed=task.speed,
        lead_in=task.lead_in,
        sine_amplitude=task.sine_amplitude,
        sine_wavelength=task.sine_wavelength,
        seed=task.seed,
        nn_model=task.nn_model,
    )
    return {
        "label": task.label,
        "n_true": task.n_true,
        "phi_true": task.phi_true,
        "speed": task.speed,
        "amplitude": task.sine_amplitude,
        "wavelength": task.sine_wavelength,
        "seed": task.seed,
        "lead_in": task.lead_in,
        "ok": bool(ok),
        "csv": task.out_csv,
        "yaml": task.terrain_yaml or "",
        "wall_s": round(time.time() - t0, 1),
    }


def _write_manifest_row(writer: csv.writer, result: Dict) -> None:
    writer.writerow([
        result["label"], f"{result['n_true']:.4f}",
        "" if result["phi_true"] is None else f"{result['phi_true']:.4f}",
        f"{result['speed']:.3f}", f"{result['amplitude']:.3f}",
        f"{result['wavelength']:.3f}", result["seed"],
        f"{result['lead_in']:.3f}", int(result["ok"]),
        result["csv"], result["yaml"], result["wall_s"],
    ])


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out", default=str(
        Path(__file__).parent.parent / "data" / "terrain_traces_closedloop"))
    p.add_argument("--terrains", nargs="+",
                   default=["clay", "dirt", "sand"],
                   help="Preset terrains to run (also map to ground truth n)")
    p.add_argument("--speeds", type=float, nargs="+", default=[4.0, 5.5, 7.0],
                   help="MPC target speeds (m/s)")
    p.add_argument("--sine-amplitudes", type=float, nargs="+",
                   default=[1.5, 2.5])
    p.add_argument("--sine-wavelengths", type=float, nargs="+",
                   default=[25.0, 35.0])
    p.add_argument("--duration", type=float, default=25.0)
    p.add_argument("--lead-in", type=float, default=5.0,
                   help="Straight lead-in distance passed to sim and MPC.")
    p.add_argument("--seeds", type=int, nargs="+", default=[0])
    p.add_argument("--sim-port-base", type=int, default=31000)
    p.add_argument("--nn-model", default="vehicle_rate_64_32_lhs")
    p.add_argument("--mode", choices=["presets", "lhs-n-phi"], default="presets",
                   help="Collect canonical preset traces or an LHS (n, phi) grid "
                        "using custom terrain YAMLs.")
    p.add_argument("--yaml-dir", type=Path, default=DATA_DIR / "terrain_yamls_closedloop_joint")
    p.add_argument("--lhs-cells", type=int, default=24)
    p.add_argument("--lhs-seed", type=int, default=23)
    p.add_argument("--n-range", type=float, nargs=2, default=[0.45, 1.15])
    p.add_argument("--phi-range", type=float, nargs=2, default=[12.0, 32.0])
    p.add_argument("--workers", type=int, default=6,
                   help="Parallel Chrono workers after one cache-prewarm run.")
    args = p.parse_args()

    if not os.environ.get("ACADOS_SOURCE_DIR"):
        p.error("ACADOS_SOURCE_DIR must point at the acados source tree "
                "before collecting closed-loop traces.")

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    args.yaml_dir.mkdir(parents=True, exist_ok=True)

    if args.mode == "lhs-n-phi":
        specs = build_lhs_specs(
            args.lhs_cells, tuple(args.n_range), tuple(args.phi_range),
            seed=args.lhs_seed,
        )
        terrain_specs = []
        for label, preset_proxy, n_true, phi_true, cfg in specs:
            yaml_path = args.yaml_dir / f"{label}.yaml"
            if not yaml_path.exists():
                import yaml
                yaml_path.write_text(yaml.safe_dump(cfg, sort_keys=False))
            terrain_specs.append((label, preset_proxy, n_true, phi_true, yaml_path))
    else:
        terrain_specs = [
            (terr, terr, TRUE_N[terr], None, None)
            for terr in args.terrains
        ]

    combos = list(product(terrain_specs, args.speeds,
                          args.sine_amplitudes, args.sine_wavelengths,
                          args.seeds))
    print(f"[cl-collect] {len(combos)} runs total -> {out_dir}")

    tasks = []
    for idx, ((label, terr, n_true, phi_true, yaml_path), sp, amp, wl, seed) in enumerate(combos):
        name = (f"cl_{label}_v{int(round(sp*10)):03d}"
            f"_amp{int(round(amp*10)):02d}"
            f"_wl{int(round(wl)):02d}_seed{seed}.csv")
        tasks.append(ClosedLoopTask(
            idx=idx + 1,
            total=len(combos),
            label=label,
            terrain=terr,
            n_true=n_true,
            phi_true=phi_true,
            terrain_yaml="" if yaml_path is None else str(yaml_path),
            speed=sp,
            sine_amplitude=amp,
            sine_wavelength=wl,
            seed=seed,
            duration=args.duration,
            lead_in=args.lead_in,
            sim_port=args.sim_port_base + 2 * idx,
            ctrl_port=args.sim_port_base + 2 * idx + 1,
            out_csv=str(out_dir / name),
            nn_model=args.nn_model,
        ))

    manifest_path = out_dir / "manifest.csv"
    write_header = not manifest_path.exists()
    header = ["label", "n_true", "phi_true", "speed", "amplitude",
              "wavelength", "seed", "lead_in", "ok", "csv", "yaml",
              "wall_s"]
    with manifest_path.open("a", newline="") as mf:
        mw = csv.writer(mf)
        if write_header:
            mw.writerow(header)

        if not tasks:
            print("\n[cl-collect] no runs requested")
            return

        print(f"\n[prewarm] [{tasks[0].idx}/{tasks[0].total}] "
              f"{Path(tasks[0].out_csv).name}")
        results = [_run_one_task(tasks[0])]
        _write_manifest_row(mw, results[0])
        mf.flush()

        remaining = tasks[1:]
        if remaining:
            with ProcessPoolExecutor(max_workers=max(int(args.workers), 1)) as ex:
                futures = {ex.submit(_run_one_task, task): task for task in remaining}
                for fut in as_completed(futures):
                    task = futures[fut]
                    try:
                        result = fut.result()
                    except Exception as exc:
                        result = {
                            "label": task.label,
                            "n_true": task.n_true,
                            "phi_true": task.phi_true,
                            "speed": task.speed,
                            "amplitude": task.sine_amplitude,
                            "wavelength": task.sine_wavelength,
                            "seed": task.seed,
                            "lead_in": task.lead_in,
                            "ok": False,
                            "csv": task.out_csv,
                            "yaml": task.terrain_yaml or "",
                            "wall_s": -1.0,
                        }
                        print(f"  [warn] {Path(task.out_csv).name}: {exc}")
                    results.append(result)
                    _write_manifest_row(mw, result)
                    mf.flush()
                    print(f"[cl-collect] {len(results):4d}/{len(tasks)}  "
                          f"{Path(result['csv']).name}  ok={result['ok']}  "
                          f"wall={result['wall_s']:5.1f}s", flush=True)

    print("\n[cl-collect] done")


if __name__ == "__main__":
    main()
