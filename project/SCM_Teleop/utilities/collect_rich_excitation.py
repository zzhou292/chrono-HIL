#!/usr/bin/env python3
"""Collect vehicle traces with *rich excitation* for terrain-parameter ID.

Each trace runs through a scripted schedule that intentionally probes
different regions of the friction cone:

  Phase 0 ( 0 -  4 s): gentle cruise, low sin steer  → baseline
  Phase 1 ( 4 - 10 s): aggressive sin steer, nominal throttle → lateral load
  Phase 2 (10 - 16 s): throttle burst, mild steer   → longitudinal load
  Phase 3 (16 - 22 s): hard brake + aggressive steer → combined slip
  Phase 4 (22 - 32 s): random-walk throttle + steer (per-trace seed)

This breaks the (n, phi) identifiability degeneracy that constant-throttle /
single-amplitude steering leaves behind.  Used together with the LHS
(n, phi) sweep to retrain the joint estimator.
"""

from __future__ import annotations

import argparse
import csv
import math
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from itertools import product
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent
SIM_DIR = ROOT / "simulation"
sys.path.insert(0, str(SIM_DIR))

from hil_messages import (                                      # noqa: E402
    ControlCommand, SimStatus, VehicleState,
    ZMQPublisher, ZMQSubscriber,
    sim_sub_endpoint, ctrl_pub_endpoint,
)
from collect_diverse_terrains import interp_terrain, closest_preset  # noqa: E402

DATA_DIR = ROOT / "data"
DEFAULT_OUT = DATA_DIR / "terrain_traces_rich"
DEFAULT_YAML = DATA_DIR / "terrain_yamls_rich"

CSV_HEADER = [
    "t", "terrain", "n_true", "throttle_cmd", "steer_cmd",
    "u", "v", "omega", "ax", "ay",
    "wheel_omega_fl", "wheel_omega_fr",
    "wheel_omega_rl", "wheel_omega_rr",
    "steering_angle",
]


def scripted_excitation(t: float, seed: int, rng: np.random.Generator
                        ) -> Tuple[float, float, float]:
    """Return (throttle, steer_cmd, braking) at sim time ``t``.

    Piecewise schedule keyed off ``t`` with a per-seed phase offset.
    Random-walk phase uses a persistent ``rng`` (caller supplies one per run).
    """
    ph = 0.13 * seed

    if t < 4.0:
        # cruise
        throttle = 0.35
        steer = 0.25 * math.sin(2 * math.pi * (t + ph) / 3.0)
        brake = 0.0
    elif t < 10.0:
        # aggressive lateral
        throttle = 0.40
        steer = 0.75 * math.sin(2 * math.pi * (t + ph) / 2.2)
        brake = 0.0
    elif t < 16.0:
        # longitudinal burst
        throttle = 0.85
        steer = 0.30 * math.sin(2 * math.pi * (t + ph) / 2.8)
        brake = 0.0
    elif t < 22.0:
        # hard brake + aggressive steer (combined slip)
        throttle = 0.05
        steer = 0.80 * math.sin(2 * math.pi * (t + ph) / 1.9)
        brake = 0.35 if (t - 16.0) < 3.0 else 0.0
    else:
        # Bounded chirp: steering frequency sweeps 0.3→1.5 Hz over 8s
        # with slowly varying throttle (smooth square-ish).
        t_c = t - 22.0
        f = 0.3 + (1.5 - 0.3) * (t_c / 8.0)
        steer = 0.55 * math.sin(2 * math.pi * f * t_c + ph)
        throttle = 0.35 + 0.25 * math.sin(2 * math.pi * (t_c + ph) / 3.5)
        brake = 0.0

    return throttle, steer, brake


def collect_one_rich(*, label: str, preset_proxy: str,
                     n_true: float, phi_true: float,
                     yaml_path: Path, out_csv: Path,
                     seed: int, duration: float,
                     sim_port: int, ctrl_port: int) -> bool:
    """Launch sim and record a rich-excitation trace.  Return True on success."""

    if out_csv.exists() and out_csv.stat().st_size > 4096:
        print(f"  [skip] {out_csv.name} already populated")
        return True

    out_csv.parent.mkdir(parents=True, exist_ok=True)

    sim_script = SIM_DIR / "chrono_sim_node.py"
    base_args = [
        str(sim_script),
        "--time", str(duration + 5.0), "--speed", "5",
        "--terrain", preset_proxy, "--path", "sinusoidal",
        "--vis-mode", "none",
        "--sim-port", str(sim_port),
        "--ctrl-host", "localhost",
        "--ctrl-port", str(ctrl_port),
        "--no-wait-for-controller",
        "--terrain-config", str(yaml_path),
    ]
    # Use the interpreter that is already running this collector (the `sim`
    # conda env). The previous hard-coded conda path was machine-specific
    # and broke on any host whose user is not `kyle`.
    sim_cmd = [sys.executable, *base_args]

    print(f"  launching sim: {preset_proxy} n={n_true:.2f} "
          f"phi={phi_true:.1f} seed={seed}")
    sim_proc = subprocess.Popen(sim_cmd,
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL)

    state_sub = ZMQSubscriber(sim_sub_endpoint("localhost", sim_port))
    ctrl_pub = ZMQPublisher(ctrl_pub_endpoint(ctrl_port))
    time.sleep(2.0)

    rng = np.random.default_rng(seed * 1009 + 17)
    rows: list[list] = []
    seq = 0
    t0_sim: Optional[float] = None
    timeout_count = 0

    try:
        while True:
            res = state_sub.recv(timeout_ms=500)
            if res is None:
                timeout_count += 1
                if timeout_count > 30:
                    print("    [warn] no messages — aborting run")
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

            throttle, steer_cmd, brake = scripted_excitation(t_rel, seed, rng)
            cmd = ControlCommand(
                time=t, wall_time=time.time(), seq=seq,
                steering=steer_cmd, throttle=throttle, braking=brake,
                delta=0.0, acceleration=0.0,
                delta_dot=0.0, jerk=0.0,
            )
            ctrl_pub.send(cmd)
            seq += 1

            rows.append([
                f"{t_rel:.4f}", preset_proxy, f"{n_true:.3f}",
                f"{throttle:.3f}", f"{steer_cmd:.4f}",
                f"{msg.u:.4f}", f"{msg.v:.4f}", f"{msg.omega:.4f}",
                f"{msg.ax:.4f}", f"{msg.ay:.4f}",
                f"{msg.wheel_omega_fl:.4f}", f"{msg.wheel_omega_fr:.4f}",
                f"{msg.wheel_omega_rl:.4f}", f"{msg.wheel_omega_rr:.4f}",
                f"{msg.steering_angle:.4f}",
            ])

            if t_rel >= duration:
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

    if len(rows) < 200:
        print(f"    [warn] only {len(rows)} rows — discarding")
        return False

    with out_csv.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(CSV_HEADER)
        w.writerows(rows)
    print(f"    [ok] saved {len(rows)} rows → {out_csv.name}")
    return True


# ------------------------------------------------------------------ specs --
def build_lhs_specs(n_cells: int,
                    n_range: Tuple[float, float],
                    phi_range: Tuple[float, float],
                    seed: int = 0
                    ) -> List[Tuple[str, str, float, float, Dict]]:
    """LHS in (n, phi)."""
    rng = np.random.default_rng(seed)
    n_lo, n_hi = n_range
    p_lo, p_hi = phi_range
    n_strata = (np.arange(n_cells) + rng.random(n_cells)) / n_cells
    p_strata = (np.arange(n_cells) + rng.random(n_cells)) / n_cells
    rng.shuffle(p_strata)
    n_vals = n_lo + n_strata * (n_hi - n_lo)
    p_vals = p_lo + p_strata * (p_hi - p_lo)
    specs = []
    for i, (n_val, phi_val) in enumerate(zip(n_vals, p_vals)):
        base = interp_terrain(float(n_val))
        cfg = dict(base)
        cfg["friction_angle"] = float(phi_val)
        cfg["elastic_stiffness"] = 2e8
        cfg["damping"] = 3e4
        cfg["description"] = f"rich_{i:04d}_n{n_val:.3f}_phi{phi_val:.1f}"
        label = (f"rich{i:04d}"
                 f"_n{int(round(n_val * 1000)):04d}"
                 f"_phi{int(round(phi_val * 10)):03d}")
        specs.append((label, closest_preset(float(n_val)),
                      float(n_val), float(phi_val), cfg))
    return specs


def run_job(**kw) -> Dict:
    t0 = time.time()
    ok = collect_one_rich(**kw)
    return {"label": kw["label"], "n": kw["n_true"], "phi": kw["phi_true"],
            "seed": kw["seed"], "ok": bool(ok),
            "wall_s": round(time.time() - t0, 1),
            "csv": str(kw["out_csv"])}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--yaml-dir", type=Path, default=DEFAULT_YAML)
    ap.add_argument("--lhs-cells", type=int, default=200)
    ap.add_argument("--lhs-seed", type=int, default=13)
    # Focus on the identifiability-degenerate zone.
    ap.add_argument("--n-range", type=float, nargs=2,
                    default=[0.55, 0.95])
    ap.add_argument("--phi-range", type=float, nargs=2,
                    default=[12.0, 28.0])
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1])
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--sim-port-base", type=int, default=34000)
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    args.yaml_dir.mkdir(parents=True, exist_ok=True)

    specs = build_lhs_specs(args.lhs_cells, tuple(args.n_range),
                            tuple(args.phi_range), seed=args.lhs_seed)
    combos = list(product(specs, args.seeds))
    print(f"[rich] {len(specs)} LHS cells × {len(args.seeds)} seeds "
          f"= {len(combos)} runs, duration={args.duration}s")

    manifest_path = args.out / "manifest.csv"
    write_header = not manifest_path.exists()

    jobs = []
    for idx, ((label, prox, n_true, phi_true, cfg), seed) in enumerate(combos):
        yp = args.yaml_dir / f"{label}.yaml"
        if not yp.exists():
            yp.write_text(yaml.safe_dump(cfg, sort_keys=False))
        out_csv = args.out / f"{label}_seed{seed}.csv"
        port = args.sim_port_base + idx * 4
        jobs.append({
            "label": label, "preset_proxy": prox,
            "n_true": n_true, "phi_true": phi_true,
            "yaml_path": yp, "out_csv": out_csv, "seed": seed,
            "duration": args.duration,
            "sim_port": port, "ctrl_port": port + 1,
        })

    t_start = time.time()
    with ThreadPoolExecutor(max_workers=args.workers) as ex:
        futures = {ex.submit(run_job, **j): j for j in jobs}
        results = []
        with manifest_path.open("a", newline="") as mf:
            w = csv.writer(mf)
            if write_header:
                w.writerow(["label", "n_true", "phi_true",
                            "seed", "ok", "wall_s", "csv"])
            for fut in as_completed(futures):
                try:
                    r = fut.result()
                except Exception as e:
                    j = futures[fut]
                    r = {"label": j["label"], "n": j["n_true"],
                         "phi": j["phi_true"], "seed": j["seed"],
                         "ok": False, "wall_s": -1.0,
                         "csv": str(j["out_csv"]), "err": str(e)}
                results.append(r)
                w.writerow([r["label"], f"{r['n']:.3f}", f"{r['phi']:.2f}",
                            r["seed"], int(r["ok"]), r["wall_s"], r["csv"]])
                done = len(results)
                ok_rate = sum(1 for x in results if x["ok"]) / max(done, 1)
                elapsed = (time.time() - t_start) / 60.0
                print(f"[rich] {done:4d}/{len(jobs)}  {r['label']:<24s}  "
                      f"n={r['n']:.2f} phi={r['phi']:.1f} seed={r['seed']}  "
                      f"ok={r['ok']}  wall={r['wall_s']:5.1f}s  "
                      f"ok_rate={ok_rate:.0%} elapsed={elapsed:.1f}min",
                      flush=True)

    n_ok = sum(1 for r in results if r["ok"])
    print(f"\n[rich] done — {n_ok}/{len(results)} successful, "
          f"{(time.time() - t_start) / 60:.1f} min wall")
    return 0 if n_ok == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
