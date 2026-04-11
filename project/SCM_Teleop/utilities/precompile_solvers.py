#!/usr/bin/env python3
"""
Pre-compile ACADOS MPC solvers for all models used in paper benchmarks.

Terrain does NOT affect the ACADOS build — only the NN model (or
analytical tire type) determines the compiled solver.  So we build once
per unique model and the benchmark runs reuse the cached binaries.

Usage:
    # Compile all models (analytical + all 25 NN)
    python precompile_solvers.py

    # Only analytical models
    python precompile_solvers.py --only analytical

    # Only NN models
    python precompile_solvers.py --only nn

    # Specific model(s)
    python precompile_solvers.py --models paper_v1_mlp_16_4 paper_v1_resnet_h16_b2
"""

from __future__ import annotations

import argparse
import concurrent.futures
import os
import sys
import time
from pathlib import Path

# Force unbuffered stdout so output appears immediately under conda run
os.environ["PYTHONUNBUFFERED"] = "1"
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(line_buffering=True)

# ── Models (must match run_paper_benchmarks.py) ──────────────────────
ANALYTICAL_TYPES = ["pacejka", "tmeasy", "linear"]

ALL_NN_MODELS = {
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
    # Temporal K=3
    "temp_K3_mlp_16_8":     "paper_v1_mlp_temporal_K3_16_8",
    "temp_K3_mlp_24_12":    "paper_v1_mlp_temporal_K3_24_12",
    "temp_K3_resnet_h16":   "paper_v1_resnet_temporal_K3_h16_b2",
    "temp_K3_resnet_h32":   "paper_v1_resnet_temporal_K3_h32_b2",
    # Temporal K=5
    "temp_K5_mlp_16_8":     "paper_v1_mlp_temporal_K5_16_8",
    "temp_K5_mlp_24_12":    "paper_v1_mlp_temporal_K5_24_12",
    "temp_K5_resnet_h16":   "paper_v1_resnet_temporal_K5_h16_b2",
    "temp_K5_resnet_h32":   "paper_v1_resnet_temporal_K5_h32_b2",
    # Temporal K=10
    "temp_K10_mlp_16_8":    "paper_v1_mlp_temporal_K10_16_8",
    "temp_K10_mlp_24_12":   "paper_v1_mlp_temporal_K10_24_12",
    "temp_K10_resnet_h16":  "paper_v1_resnet_temporal_K10_h16_b2",
    "temp_K10_resnet_h32":  "paper_v1_resnet_temporal_K10_h32_b2",
}

# Solver construction parameters (must match acados_mpc_controller_node.py)
DT_MPC = 0.1
N_HORIZON = 30

# Add simulation/ to sys.path for local imports
_UTIL_DIR = Path(__file__).parent
_PROJECT_ROOT = _UTIL_DIR.parent
_SIM_DIR = _PROJECT_ROOT / "simulation"
sys.path.insert(0, str(_SIM_DIR))


def build_analytical(tire_model: str) -> dict:
    """Build (or verify cache for) an analytical tire model solver."""
    from acados_mpc_solver import AcadosMPC

    build_dir = Path(f"/tmp/acados_mpc_{tire_model}")
    t0 = time.time()
    try:
        mpc = AcadosMPC(
            nn_tire_model=None,
            dt=DT_MPC,
            N=N_HORIZON,
            lateral_load_transfer=True,
            kappa_mode="zero",
            tire_model=tire_model,
            build_dir=build_dir,
        )
        elapsed = time.time() - t0
        return {"model": tire_model, "status": "ok", "time_s": elapsed,
                "build_dir": str(build_dir)}
    except Exception as e:
        elapsed = time.time() - t0
        return {"model": tire_model, "status": "error", "time_s": elapsed,
                "error": str(e)}


def build_nn(label: str, nn_model_name: str) -> dict:
    """Build (or verify cache for) an NN tire model solver."""
    from nn_tire_model import load_nn_tire_model
    from param_consistency import get_terrain_preset, terrain_preset_to_internal
    from acados_mpc_solver import AcadosMPC

    model_dir = _PROJECT_ROOT / "nn_models" / nn_model_name

    if not model_dir.exists():
        return {"model": label, "nn_model": nn_model_name,
                "status": "missing", "error": f"Not found: {model_dir}"}

    # Any terrain works — terrain doesn't affect the build.
    # Use sand as a dummy for loading the NN (it needs terrain params
    # for input normalization, but the compiled solver is terrain-agnostic).
    tp = get_terrain_preset("sand")
    terrain_params = terrain_preset_to_internal(tp)

    t0 = time.time()
    try:
        nn_tire = load_nn_tire_model(str(model_dir), terrain_params)
        t_load = time.time() - t0

        safe_model_tag = nn_model_name.replace("/", "_")
        build_dir = Path(f"/tmp/acados_mpc_{safe_model_tag}")

        mpc = AcadosMPC(
            nn_tire_model=nn_tire,
            dt=DT_MPC,
            N=N_HORIZON,
            lateral_load_transfer=True,
            kappa_mode="zero",
            tire_model="nn",
            build_dir=build_dir,
        )
        elapsed = time.time() - t0
        return {"model": label, "nn_model": nn_model_name,
                "status": "ok", "time_s": elapsed, "load_s": t_load,
                "n_params": nn_tire.n_params, "model_type": nn_tire.model_type,
                "build_dir": str(build_dir)}
    except Exception as e:
        elapsed = time.time() - t0
        return {"model": label, "nn_model": nn_model_name,
                "status": "error", "time_s": elapsed, "error": str(e)}


def main():
    p = argparse.ArgumentParser(description="Pre-compile ACADOS solvers")
    p.add_argument("--only", choices=["analytical", "nn"],
                   help="Only compile one category")
    p.add_argument("--models", nargs="+",
                   help="Specific NN model names to compile (e.g. paper_v1_mlp_16_4)")
    p.add_argument("-j", "--workers", type=int, default=1,
                   help="Parallel compilation workers (default 1)")
    args = p.parse_args()

    results = []
    total_t0 = time.time()

    # Analytical models
    if args.only != "nn" and not args.models:
        print(f"{'=' * 60}")
        print(f"  Analytical models ({len(ANALYTICAL_TYPES)})")
        print(f"{'=' * 60}")
        for i, tm in enumerate(ANALYTICAL_TYPES):
            print(f"  [{i + 1}/{len(ANALYTICAL_TYPES)}] {tm} ...", end=" ", flush=True)
            r = build_analytical(tm)
            results.append(r)
            if r["status"] == "ok":
                print(f"OK ({r['time_s']:.1f}s)")
            else:
                print(f"FAILED: {r['error']}")
        print()

    # NN models
    if args.only != "analytical":
        if args.models:
            # Specific models requested
            nn_items = [(name, name) for name in args.models]
        else:
            nn_items = list(ALL_NN_MODELS.items())

        print(f"{'=' * 60}")
        print(f"  NN models ({len(nn_items)})  workers={args.workers}")
        print(f"{'=' * 60}")

        def _fmt_result(nn_name, r):
            if r["status"] == "ok":
                cached = r["time_s"] < 2.0
                tag = " (cached)" if cached else ""
                return (f"{nn_name}  OK  "
                        f"{r.get('n_params', '?')} params  "
                        f"({r['time_s']:.1f}s){tag}")
            elif r["status"] == "missing":
                return f"{nn_name}  MISSING: {r['error']}"
            else:
                return f"{nn_name}  FAILED: {r['error']}"

        if args.workers <= 1:
            for i, (label, nn_name) in enumerate(nn_items):
                print(f"  [{i + 1}/{len(nn_items)}] {nn_name} ...",
                      end=" ", flush=True)
                r = build_nn(label, nn_name)
                results.append(r)
                print(_fmt_result(nn_name, r).split(nn_name, 1)[1].strip())
        else:
            # Print all jobs upfront so user can see what's queued
            for i, (label, nn_name) in enumerate(nn_items):
                print(f"  [{i + 1}/{len(nn_items)}] {nn_name}  ... queued")
            print()

            with concurrent.futures.ProcessPoolExecutor(
                    max_workers=args.workers) as pool:
                futures = {
                    pool.submit(build_nn, label, nn_name): (i, label, nn_name)
                    for i, (label, nn_name) in enumerate(nn_items)
                }
                done_count = 0
                for fut in concurrent.futures.as_completed(futures):
                    done_count += 1
                    idx, label, nn_name = futures[fut]
                    try:
                        r = fut.result()
                    except Exception as e:
                        r = {"model": label, "nn_model": nn_name,
                             "status": "error", "time_s": 0, "error": str(e)}
                    results.append(r)
                    elapsed = time.time() - total_t0
                    print(f"  [{done_count}/{len(nn_items)}] "
                          f"{_fmt_result(nn_name, r)}  "
                          f"(elapsed {elapsed:.0f}s)",
                          flush=True)
        print()

    # Summary
    total_elapsed = time.time() - total_t0
    ok_count = sum(1 for r in results if r["status"] == "ok")
    err_count = sum(1 for r in results if r["status"] == "error")
    miss_count = sum(1 for r in results if r["status"] == "missing")

    print(f"{'=' * 60}")
    print(f"  Summary: {ok_count} OK, {err_count} errors, {miss_count} missing")
    print(f"  Total time: {total_elapsed:.1f}s ({total_elapsed / 60:.1f} min)")
    print(f"{'=' * 60}")

    if err_count > 0:
        print("\nFailed models:")
        for r in results:
            if r["status"] == "error":
                print(f"  {r['model']}: {r['error']}")

    sys.exit(1 if err_count > 0 else 0)


if __name__ == "__main__":
    main()
