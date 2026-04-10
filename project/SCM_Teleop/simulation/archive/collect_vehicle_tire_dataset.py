#!/usr/bin/env python3
"""
Parallel full-vehicle tire dataset collection (MPC-aligned inputs, sim force labels).

Spawns multiple ``launch_decoupled.py`` subprocesses with unique ZMQ ports and
``scenario_id`` values. **MPC uses TMeasy tires by default** (stable closed-loop
teacher); SCM/Chrono still supplies force labels in the CSV. Override with ``--model``.

Each run writes a shard CSV whose inputs are produced by
``tire_input_features.py`` / the controller (bicycle α, IMU-based F̂z, κ̂, δ̇).

Requires the same environment as a normal sim (e.g. ``conda activate chrono``).

Example (fixed preset)::

    cd simulation
    conda activate chrono
    python collect_vehicle_tire_dataset.py --runs 12 --jobs 4 \\
        --output ../data/vehicle_mpc/clay_dlc_train.csv \\
        --terrain clay --path double_lane_change --time 28 --speed 5 --lead-in 10 \\
        --base-port 5700

Example (LHS over ``TRAINING_RANGES_V6`` soil hull — covers clay/sand/dirt presets)::

    python collect_vehicle_tire_dataset.py --runs 64 --jobs 4 \\
        --terrain-mode lhs --lhs-seed 42 \\
        --output ../data/vehicle_mpc/lhs_v6_vehicle.csv \\
        --path double_lane_change --time 25 --speed 5 --lead-in 10

Soil parameters come **only** from per-run YAML (LHS). ``--terrain`` is still passed to
``launch_decoupled`` because the CLI requires one of ``sand|clay|dirt``; it does **not**
select soil when ``--terrain-config`` is set—the sim uses the YAML and publishes
``terrain_preset: custom`` in config.

Then train (rows are logged at the **MPC step**, ~0.1 s — pass ``--record-dt 0.1`` to
temporal/rate/ResNet-temporal scripts; default 0.005 is for rig CSVs)::

    cd ../nn_training
    python train_temporal_nn.py --data ../data/vehicle_mpc/clay_dlc_train.csv \\
        --record-dt 0.1 --dt-nn 0.1 \\
        --output_dir ../nn_models/vehicle_temporal_K5_16_8 --K 5 --hidden 16 8
"""

from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import yaml

from param_consistency import generate_lhs_terrain_yaml_dicts


def _default_jobs() -> int:
    return max(1, min(8, (os.cpu_count() or 4)))


def _run_one_shard(job: dict) -> dict:
    """Picklable worker: one launch_decoupled run → one shard CSV."""
    launch = Path(job["launch_script"])
    cwd = Path(job["cwd"])
    shard = Path(job["shard_path"])
    shard.parent.mkdir(parents=True, exist_ok=True)
    if shard.exists():
        shard.unlink()

    cmd = [
        sys.executable,
        str(launch),
        "--terrain",
        job["terrain"],
        "--path",
        job["path"],
        "--time",
        str(job["time"]),
        "--speed",
        str(job["speed"]),
        "--lead-in",
        str(job["lead_in"]),
        "--sine-amplitude",
        str(job["sine_amplitude"]),
        "--sine-wavelength",
        str(job["sine_wavelength"]),
        "--model",
        job["model"],
        "--kappa",
        job["kappa"],
        "--sim-port",
        str(job["sim_port"]),
        "--ctrl-port",
        str(job["ctrl_port"]),
        "--plot-dir",
        job["plot_dir"],
        "--no-vis",
        "--no-plot",
        "--log-tire-csv",
        str(shard),
        "--log-scenario-id",
        str(job["scenario_id"]),
    ]
    if job["model"] == "nn":
        cmd.extend(["--nn-model", job["nn_model"]])
    tcp = job.get("terrain_config_path")
    if tcp:
        cmd.extend(["--terrain-config", str(tcp)])
    if job.get("no_delay_comp"):
        cmd.append("--no-delay-comp")
    if job.get("no_lat_transfer"):
        cmd.append("--no-lat-transfer")
    if job.get("no_csv", True):
        cmd.append("--no-csv")
    rocks = int(job.get("rocks", 0))
    if rocks > 0 and job.get("rock_seed") is not None:
        cmd.extend(["--rocks", str(rocks)])
        cmd.extend(["--rock-seed", str(job["rock_seed"])])

    t0 = time.time()
    try:
        proc = subprocess.run(
            cmd,
            cwd=str(cwd),
            timeout=float(job.get("timeout", 600)),
            env=os.environ.copy(),
        )
        dt = time.time() - t0
        ok = proc.returncode == 0 and shard.is_file() and shard.stat().st_size > 100
        return {
            "scenario_id": job["scenario_id"],
            "returncode": proc.returncode,
            "shard": str(shard),
            "ok": ok,
            "seconds": dt,
        }
    except subprocess.TimeoutExpired:
        return {
            "scenario_id": job["scenario_id"],
            "returncode": -1,
            "shard": str(shard),
            "ok": False,
            "seconds": time.time() - t0,
        }


def merge_shard_csvs(shard_paths: list[Path], output: Path) -> int:
    """Concatenate shards; single header. Returns row count (excluding header)."""
    output.parent.mkdir(parents=True, exist_ok=True)
    header = None
    nrows = 0
    with open(output, "w", newline="") as fout:
        w = csv.writer(fout)
        for sp in sorted(shard_paths):
            if not sp.is_file() or sp.stat().st_size < 50:
                continue
            with open(sp, newline="") as fin:
                r = csv.reader(fin)
                try:
                    h = next(r)
                except StopIteration:
                    continue
                if header is None:
                    header = h
                    w.writerow(h)
                elif h != header:
                    raise ValueError(f"Header mismatch in {sp}")
                for row in r:
                    w.writerow(row)
                    nrows += 1
    return nrows


def main():
    p = argparse.ArgumentParser(description="Parallel vehicle tire CSV collection")
    p.add_argument("--runs", type=int, required=True, help="Number of scenarios (shards)")
    p.add_argument("--jobs", type=int, default=None, help="Parallel workers (default: min(8, CPU))")
    p.add_argument("--output", type=Path, required=True, help="Merged CSV path")
    p.add_argument(
        "--shard-dir",
        type=Path,
        default=None,
        help="Directory for per-scenario shards (default: <output>_shards/)",
    )
    p.add_argument("--base-scenario-id", type=int, default=0, help="First scenario_id")
    p.add_argument("--base-port", type=int, default=5700, help="First sim port (ctrl = sim+1)")
    p.add_argument("--timeout", type=float, default=600, help="Per-run subprocess timeout (s)")
    p.add_argument(
        "--terrain-mode",
        choices=["preset", "lhs"],
        default="preset",
        help="preset: single --terrain for all runs; lhs: Latin hypercube over "
        "TRAINING_RANGES_V6 soil params (see param_consistency.py)",
    )
    p.add_argument(
        "--lhs-seed",
        type=int,
        default=0,
        help="RNG seed for LHS terrain generation (terrain-mode=lhs)",
    )
    p.add_argument(
        "--terrain",
        default="clay",
        choices=["sand", "clay", "dirt"],
        help="preset mode: this soil for every run. lhs mode: ignored for physics (YAML defines "
        "soil); must still be a valid sim preset name for argparse/launch_decoupled.",
    )
    p.add_argument(
        "--path",
        default="double_lane_change",
        choices=["lane_change", "double_lane_change", "sinusoidal"],
    )
    p.add_argument("--time", type=float, default=25.0)
    p.add_argument("--speed", type=float, default=5.0)
    p.add_argument("--lead-in", type=float, default=10.0)
    p.add_argument("--sine-amplitude", type=float, default=2.0)
    p.add_argument("--sine-wavelength", type=float, default=30.0)
    p.add_argument(
        "--model",
        default="tmeasy",
        choices=["nn", "pacejka", "tmeasy", "linear"],
        help="MPC internal tire model for closed-loop collection (default: tmeasy)",
    )
    p.add_argument(
        "--nn-model",
        default="paper_v1_mlp_16_4",
        help="When --model nn: checkpoint directory under nn_models/",
    )
    p.add_argument("--kappa", default="zero", choices=["zero", "approx"])
    p.add_argument("--no-delay-comp", action="store_true")
    p.add_argument("--no-lat-transfer", action="store_true")
    p.add_argument("--keep-shards", action="store_true", help="Do not delete shard CSVs after merge")
    p.add_argument(
        "--keep-lhs-yamls",
        action="store_true",
        help="Keep per-run terrain YAML files (lhs mode; default: delete after merge unless --keep-shards)",
    )
    p.add_argument("--rocks", type=int, default=0, help="If >0, vary --rock-seed per scenario")
    args = p.parse_args()

    script_dir = Path(__file__).parent.resolve()
    launch_script = script_dir / "launch_decoupled.py"
    if not launch_script.is_file():
        print(f"Missing {launch_script}", file=sys.stderr)
        sys.exit(1)

    jobs_n = args.jobs if args.jobs is not None else _default_jobs()
    shard_dir = args.shard_dir
    if shard_dir is None:
        shard_dir = args.output.parent / f"{args.output.stem}_shards"
    shard_dir = shard_dir.resolve()
    shard_dir.mkdir(parents=True, exist_ok=True)

    plot_root = shard_dir / "_plots"
    plot_root.mkdir(parents=True, exist_ok=True)

    terrain_yaml_paths: list[Path] = []
    if args.terrain_mode == "lhs":
        lhs_dir = shard_dir / "lhs_terrain_yaml"
        lhs_dir.mkdir(parents=True, exist_ok=True)
        configs = generate_lhs_terrain_yaml_dicts(args.runs, seed=args.lhs_seed)
        for i, cfg in enumerate(configs):
            sid = args.base_scenario_id + i
            ypath = lhs_dir / f"scenario_{sid:05d}.yaml"
            with open(ypath, "w") as yf:
                yaml.safe_dump(cfg, yf, default_flow_style=False, sort_keys=False)
            terrain_yaml_paths.append(ypath)
        print(
            f"LHS terrain: {args.runs} samples in TRAINING_RANGES_V6 hull "
            f"(seed={args.lhs_seed}) → {lhs_dir}"
        )

    jobs = []
    for i in range(args.runs):
        sid = args.base_scenario_id + i
        sim_port = args.base_port + i * 10
        ctrl_port = sim_port + 1
        shard_path = shard_dir / f"scenario_{sid:05d}.csv"
        plot_dir = str(plot_root / f"run_{sid:05d}")
        rock_seed = (args.base_scenario_id * 1000 + sid) if args.rocks > 0 else None
        job = {
                "launch_script": str(launch_script),
                "cwd": str(script_dir),
                "shard_path": str(shard_path),
                "scenario_id": sid,
                "sim_port": sim_port,
                "ctrl_port": ctrl_port,
                "plot_dir": plot_dir,
                "terrain": args.terrain,
                "path": args.path,
                "time": args.time,
                "speed": args.speed,
                "lead_in": args.lead_in,
                "sine_amplitude": args.sine_amplitude,
                "sine_wavelength": args.sine_wavelength,
                "model": args.model,
                "nn_model": args.nn_model,
                "kappa": args.kappa,
                "no_delay_comp": args.no_delay_comp,
                "no_lat_transfer": args.no_lat_transfer,
                "no_csv": True,
                "timeout": args.timeout,
                "rocks": args.rocks,
                "rock_seed": rock_seed,
        }
        if args.terrain_mode == "lhs":
            job["terrain_config_path"] = str(terrain_yaml_paths[i])
        jobs.append(job)

    print(f"Collecting {args.runs} scenario(s) with {jobs_n} worker(s)…")
    print(f"  Shards: {shard_dir}")
    print(f"  Merged: {args.output.resolve()}")

    results = []
    with ProcessPoolExecutor(max_workers=jobs_n) as ex:
        futs = {ex.submit(_run_one_shard, j): j for j in jobs}
        for fut in as_completed(futs):
            results.append(fut.result())

    ok_n = sum(1 for r in results if r["ok"])
    print(f"Finished: {ok_n}/{len(results)} shards OK")
    for r in sorted(results, key=lambda x: x["scenario_id"]):
        st = "ok" if r["ok"] else f"FAIL(rc={r['returncode']})"
        print(f"  scenario {r['scenario_id']:5d}  {st:12s}  {r['seconds']:.1f}s  {r['shard']}")

    shards = sorted(shard_dir.glob("scenario_*.csv"))
    nrows = merge_shard_csvs(shards, args.output.resolve())
    print(f"Merged {len(shards)} file(s) → {nrows} data rows (+ header) → {args.output}")

    if not args.keep_shards:
        for sp in shards:
            try:
                sp.unlink()
            except OSError:
                pass
        if args.terrain_mode == "lhs" and not args.keep_lhs_yamls:
            for yp in terrain_yaml_paths:
                try:
                    yp.unlink()
                except OSError:
                    pass
            try:
                lhs_d = shard_dir / "lhs_terrain_yaml"
                if lhs_d.is_dir() and not any(lhs_d.iterdir()):
                    lhs_d.rmdir()
            except OSError:
                pass

    if ok_n < args.runs:
        sys.exit(1)


if __name__ == "__main__":
    main()
