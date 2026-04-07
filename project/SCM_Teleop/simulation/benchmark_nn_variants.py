#!/usr/bin/env python3
"""
Benchmark multiple NN model variants in closed-loop MPC.

Runs decoupled sim + ACADOS controller for:
- path: sinusoidal
- terrains: clay, sand, dirt
- bumpiness: fixed (default 5)
- repeats: N runs per (terrain, model) to average KPIs

KPIs parsed from controller stdout:
- Mean solve (ms)
- Effective rate (Hz)
- Avg |CTE| (m)

This is designed for paper tables: outputs a CSV summary.
"""

from __future__ import annotations

import argparse
import csv
import re
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from statistics import mean
from queue import Queue


SCRIPT_DIR = Path(__file__).resolve().parent


_RE_MEAN_SOLVE = re.compile(r"Mean solve:\s+([0-9.]+)\s+ms")
_RE_EFF_RATE = re.compile(r"Effective rate:\s+([0-9.]+)\s+Hz")
_RE_AVG_CTE = re.compile(r"Avg \|CTE\|:\s+([0-9.]+)\s+m")


def run_one(model_name: str, terrain: str, bumpiness: int, sim_time: float, speed: float,
            sine_amp: float, sine_wl: float, sim_port: int, ctrl_port: int) -> dict | None:
    sim_cmd = [
        sys.executable, str(SCRIPT_DIR / "chrono_sim_node.py"),
        "--time", str(sim_time),
        "--speed", str(speed),
        "--terrain", terrain,
        "--bumpiness", str(bumpiness),
        "--path", "sinusoidal",
        "--sine-amplitude", str(sine_amp),
        "--sine-wavelength", str(sine_wl),
        "--vis-mode", "none",
        "--wait-for-controller", "120",
        "--sim-port", str(sim_port),
        "--ctrl-port", str(ctrl_port),
        "--no-noise",
    ]

    ctrl_cmd = [
        sys.executable, str(SCRIPT_DIR / "acados_mpc_controller_node.py"),
        "--model", "nn",
        "--nn-model", model_name,
        "--terrain", terrain,
        "--time", str(sim_time),
        "--speed", str(speed),
        "--path", "sinusoidal",
        "--sine-amplitude", str(sine_amp),
        "--sine-wavelength", str(sine_wl),
        "--sim-port", str(sim_port),
        "--ctrl-port", str(ctrl_port),
        "--rms-time-start", "5.0",
        "--no-plot",
        "--no-csv",
    ]

    sim_proc = None
    ctrl_proc = None
    try:
        ctrl_proc = subprocess.Popen(ctrl_cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        time.sleep(0.3)
        sim_proc = subprocess.Popen(sim_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        sim_proc.wait()
        # Allow compile + warmup + full sim horizon with margin.
        out, _ = ctrl_proc.communicate(timeout=max(180, int(sim_time) + 150))

        m1 = _RE_MEAN_SOLVE.search(out)
        m2 = _RE_EFF_RATE.search(out)
        m3 = _RE_AVG_CTE.search(out)
        if not (m1 and m2 and m3):
            return None

        return {
            "mean_solve_ms": float(m1.group(1)),
            "effective_hz": float(m2.group(1)),
            "avg_abs_cte_m": float(m3.group(1)),
        }
    except Exception:
        return None
    finally:
        for p in (sim_proc, ctrl_proc):
            if p is not None and p.poll() is None:
                p.kill()
                try:
                    p.wait(timeout=5)
                except Exception:
                    pass


def run_job(job: dict, port_pool: Queue) -> dict:
    sim_port, ctrl_port = port_pool.get()
    try:
        res = run_one(
            model_name=job["model"],
            terrain=job["terrain"],
            bumpiness=job["bumpiness"],
            sim_time=job["time"],
            speed=job["speed"],
            sine_amp=job["sine_amplitude"],
            sine_wl=job["sine_wavelength"],
            sim_port=sim_port,
            ctrl_port=ctrl_port,
        )
        return {
            "model": job["model"],
            "terrain": job["terrain"],
            "run_idx": job["run_idx"],
            "result": res,
        }
    finally:
        port_pool.put((sim_port, ctrl_port))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--models", nargs="+", required=True, help="NN model directory names under nn_models/")
    p.add_argument("--runs", type=int, default=5)
    p.add_argument("--time", type=float, default=30.0)
    p.add_argument("--speed", type=float, default=5.0)
    p.add_argument("--bumpiness", type=int, default=5)
    p.add_argument("--sine-amplitude", type=float, default=2.0)
    p.add_argument("--sine-wavelength", type=float, default=30.0)
    p.add_argument("--out", default="paper_figures/nn_variant_benchmark.csv")
    p.add_argument("--workers", type=int, default=4, help="Parallel worker count (default: 4)")
    p.add_argument("--base-port", type=int, default=16600, help="Base port for worker port-pairs")
    args = p.parse_args()

    terrains = ["clay", "sand", "dirt"]
    out_path = (SCRIPT_DIR.parent / args.out).resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    jobs = []
    for model in args.models:
        for terrain in terrains:
            for r in range(args.runs):
                jobs.append({
                    "model": model,
                    "terrain": terrain,
                    "run_idx": r,
                    "bumpiness": args.bumpiness,
                    "time": args.time,
                    "speed": args.speed,
                    "sine_amplitude": args.sine_amplitude,
                    "sine_wavelength": args.sine_wavelength,
                })

    print(f"[bench] total jobs={len(jobs)} workers={args.workers}")
    port_pool: Queue = Queue()
    for i in range(args.workers):
        sim_port = args.base_port + 2 * i
        ctrl_port = sim_port + 1
        port_pool.put((sim_port, ctrl_port))

    grouped = {(m, t): {"mean_solve_ms": [], "effective_hz": [], "avg_abs_cte_m": []}
               for m in args.models for t in terrains}

    with ThreadPoolExecutor(max_workers=args.workers) as ex:
        futs = [ex.submit(run_job, j, port_pool) for j in jobs]
        for fut in as_completed(futs):
            out = fut.result()
            model, terrain, run_idx = out["model"], out["terrain"], out["run_idx"]
            res = out["result"]
            if res is None:
                print(f"[bench] {model} {terrain} run {run_idx+1}/{args.runs}: FAILED")
                continue
            print(f"[bench] {model} {terrain} run {run_idx+1}/{args.runs}: "
                  f"solve={res['mean_solve_ms']:.2f}ms hz={res['effective_hz']:.1f} "
                  f"cte={res['avg_abs_cte_m']:.3f}m")
            for k in grouped[(model, terrain)]:
                grouped[(model, terrain)][k].append(res[k])

    rows = []
    for model in args.models:
        for terrain in terrains:
            vals = grouped[(model, terrain)]
            if not vals["mean_solve_ms"]:
                rows.append({
                    "model": model,
                    "terrain": terrain,
                    "runs_ok": 0,
                    "mean_solve_ms": "",
                    "effective_hz": "",
                    "avg_abs_cte_m": "",
                })
                continue
            rows.append({
                "model": model,
                "terrain": terrain,
                "runs_ok": len(vals["mean_solve_ms"]),
                "mean_solve_ms": mean(vals["mean_solve_ms"]),
                "effective_hz": mean(vals["effective_hz"]),
                "avg_abs_cte_m": mean(vals["avg_abs_cte_m"]),
            })

    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    print(f"✓ wrote {out_path}")


if __name__ == "__main__":
    main()

