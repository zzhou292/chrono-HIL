#!/usr/bin/env python3
"""Generate a denser (n, phi) grid of soils, run Chrono open-loop steering
sweeps in parallel, and dump labelled CSV traces.

This breaks the n–phi correlation that the canonical clay/dirt/sand presets
(and the existing diverse-terrain set, which only perturbs phi by ±5° at
fixed preset n) impose on the trained estimator.

Outputs
-------
* YAMLs in        ``data/terrain_yamls_grid/``
* CSV traces in   ``data/terrain_traces_grid/``
* Manifest CSV    ``data/terrain_traces_grid/manifest.csv``
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from itertools import product
from pathlib import Path
from typing import Dict, List, Tuple

import yaml

ROOT = Path(__file__).resolve().parent.parent  # SCM_Teleop/
SIM_DIR = ROOT / "simulation"
sys.path.insert(0, str(SIM_DIR))

from collect_terrain_traces import collect_one  # noqa: E402
from collect_diverse_terrains import interp_terrain, closest_preset  # noqa: E402

DATA_DIR = ROOT / "data"
DEFAULT_OUT = DATA_DIR / "terrain_traces_grid"
DEFAULT_YAML = DATA_DIR / "terrain_yamls_grid"


def build_specs(n_levels: List[float], phi_levels: List[float]
                ) -> List[Tuple[str, str, float, float, Dict[str, float]]]:
    """Return list of (label, preset_proxy, n_true, phi_true, params).

    Cartesian product of ``n_levels`` and ``phi_levels``.
    """
    specs = []
    for n_val in n_levels:
        base = interp_terrain(float(n_val))
        for phi_val in phi_levels:
            cfg = dict(base)
            cfg["friction_angle"] = float(phi_val)
            cfg["elastic_stiffness"] = 2e8
            cfg["damping"] = 3e4
            cfg["description"] = f"grid_n{n_val:.2f}_phi{phi_val:.0f}"
            label = (f"grid_n{int(round(n_val * 100)):03d}"
                     f"_phi{int(round(phi_val)):02d}")
            specs.append((label, closest_preset(n_val),
                          float(n_val), float(phi_val), cfg))
    return specs


def build_lhs_specs(n_cells: int, n_range: Tuple[float, float],
                    phi_range: Tuple[float, float], seed: int = 0
                    ) -> List[Tuple[str, str, float, float, Dict[str, float]]]:
    """Latin-hypercube sample (n, phi) in the given ranges.

    Each cell becomes a labelled YAML with parameters interpolated along the
    canonical n-manifold but with overridden friction angle.
    """
    import numpy as np
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
        cfg["description"] = (f"lhs_{i:04d}_n{n_val:.3f}_phi{phi_val:.1f}")
        label = (f"lhs{i:04d}"
                 f"_n{int(round(n_val * 1000)):04d}"
                 f"_phi{int(round(phi_val * 10)):03d}")
        specs.append((label, closest_preset(float(n_val)),
                      float(n_val), float(phi_val), cfg))
    return specs


def run_job(*, label: str, preset_proxy: str, n_true: float, phi_true: float,
            yaml_path: Path, out_csv: Path,
            throttle: float, steer_amp: float, steer_period: float,
            seed: int, duration: float,
            sim_port: int, ctrl_port: int) -> Dict:
    t0 = time.time()
    ok = collect_one(
        terrain=preset_proxy, throttle=throttle, steer_amp=steer_amp,
        steer_period=steer_period, seed=seed, duration=duration,
        sim_port=sim_port, ctrl_port=ctrl_port,
        out_csv=out_csv, python_exe="conda",
        terrain_yaml=yaml_path, n_true_override=n_true,
    )
    return {"label": label, "n": n_true, "phi": phi_true,
            "thr": throttle, "amp": steer_amp, "seed": seed,
            "ok": bool(ok), "wall_s": round(time.time() - t0, 1),
            "csv": str(out_csv), "yaml": str(yaml_path)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--yaml-dir", type=Path, default=DEFAULT_YAML)
    ap.add_argument("--mode", choices=["grid", "lhs"], default="grid")
    ap.add_argument("--n-levels", type=float, nargs="+",
                    default=[0.50, 0.65, 0.80, 0.95, 1.10])
    ap.add_argument("--phi-levels", type=float, nargs="+",
                    default=[12.0, 18.0, 24.0, 30.0, 35.0])
    ap.add_argument("--lhs-cells", type=int, default=256)
    ap.add_argument("--n-range", type=float, nargs=2,
                    default=[0.45, 1.20])
    ap.add_argument("--phi-range", type=float, nargs=2,
                    default=[8.0, 38.0])
    ap.add_argument("--lhs-seed", type=int, default=0)
    ap.add_argument("--throttles", type=float, nargs="+", default=[0.50])
    ap.add_argument("--steer-amps", type=float, nargs="+", default=[0.5])
    ap.add_argument("--steer-period", type=float, default=3.0)
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1])
    ap.add_argument("--duration", type=float, default=22.0)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--sim-port-base", type=int, default=27000)
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    args.yaml_dir.mkdir(parents=True, exist_ok=True)

    if args.mode == "lhs":
        specs = build_lhs_specs(args.lhs_cells,
                                tuple(args.n_range),
                                tuple(args.phi_range),
                                seed=args.lhs_seed)
    else:
        specs = build_specs(args.n_levels, args.phi_levels)
    combos = list(product(specs, args.throttles, args.steer_amps, args.seeds))
    print(f"[grid] mode={args.mode}  {len(specs)} (n, phi) cells "
          f"× {len(args.throttles)} thr × {len(args.steer_amps)} amp "
          f"× {len(args.seeds)} seeds  =  {len(combos)} runs")

    manifest_path = args.out / "manifest.csv"
    write_header = not manifest_path.exists()

    jobs = []
    for idx, ((label, prox, n_true, phi_true, cfg), thr, amp, seed) \
            in enumerate(combos):
        yp = args.yaml_dir / f"{label}.yaml"
        if not yp.exists():
            yp.write_text(yaml.safe_dump(cfg, sort_keys=False))
        out_csv = args.out / (
            f"{label}_thr{int(round(thr * 100)):02d}"
            f"_amp{int(round(amp * 100)):02d}_seed{seed}.csv"
        )
        port = args.sim_port_base + idx * 4
        jobs.append({
            "label": label, "preset_proxy": prox,
            "n_true": n_true, "phi_true": phi_true,
            "yaml_path": yp, "out_csv": out_csv,
            "throttle": thr, "steer_amp": amp,
            "steer_period": args.steer_period, "seed": seed,
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
                            "throttle", "steer_amp", "seed",
                            "ok", "wall_s", "csv"])
            for fut in as_completed(futures):
                try:
                    r = fut.result()
                except Exception as e:
                    j = futures[fut]
                    r = {"label": j["label"], "n": j["n_true"],
                         "phi": j["phi_true"], "thr": j["throttle"],
                         "amp": j["steer_amp"], "seed": j["seed"],
                         "ok": False, "wall_s": -1.0,
                         "csv": str(j["out_csv"]), "yaml": str(j["yaml_path"]),
                         "err": str(e)}
                results.append(r)
                w.writerow([r["label"], f"{r['n']:.3f}", f"{r['phi']:.2f}",
                            f"{r['thr']:.2f}", f"{r['amp']:.2f}", r["seed"],
                            int(r["ok"]), r["wall_s"], r["csv"]])
                done = len(results)
                ok_rate = sum(1 for x in results if x["ok"]) / max(done, 1)
                elapsed = (time.time() - t_start) / 60.0
                print(f"[grid] {done:3d}/{len(jobs)}  {r['label']:<22s}  "
                      f"thr={r['thr']:.2f} seed={r['seed']}  "
                      f"ok={r['ok']}  wall={r['wall_s']:5.1f}s  "
                      f"ok_rate={ok_rate:.0%}  elapsed={elapsed:.1f}min",
                      flush=True)

    n_ok = sum(1 for r in results if r["ok"])
    print(f"\n[grid] done — {n_ok}/{len(results)} runs successful, "
          f"{(time.time() - t_start) / 60:.1f} min wall-time")
    return 0 if n_ok == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
