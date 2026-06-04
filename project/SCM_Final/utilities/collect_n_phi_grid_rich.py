#!/usr/bin/env python3
"""Cartesian (n, phi) grid sweep using the *rich-excitation* command pattern.

This is the rich-excitation counterpart of ``collect_n_phi_grid.py`` -- it
re-uses the Cartesian spec builder (``build_specs``) but drives each cell
through the same piecewise scripted command sequence the joint-regressor
training corpus uses (``collect_rich_excitation.collect_one_rich``). Running
the grid here lets us isolate the (n, phi) coverage question from the
fixed-pattern-excitation confound of ``collect_n_phi_grid.py``.

Outputs
-------
* YAMLs in   ``data/terrain_yamls_grid_rich/``
* Traces in  ``data/terrain_traces_grid_rich/``
* Manifest   ``data/terrain_traces_grid_rich/manifest.csv``
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "utilities"))
sys.path.insert(0, str(ROOT / "simulation"))

from collect_rich_excitation import collect_one_rich   # noqa: E402
from collect_n_phi_grid import build_specs             # noqa: E402

DATA_DIR = ROOT / "data"
DEFAULT_OUT = DATA_DIR / "terrain_traces_grid_rich"
DEFAULT_YAML = DATA_DIR / "terrain_yamls_grid_rich"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    ap.add_argument("--yaml-dir", type=Path, default=DEFAULT_YAML)
    ap.add_argument("--n-levels", type=float, nargs="+",
                    default=[0.50, 0.65, 0.80, 0.95, 1.10])
    ap.add_argument("--phi-levels", type=float, nargs="+",
                    default=[12.0, 18.0, 24.0, 30.0, 35.0])
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1])
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--sim-port-base", type=int, default=29000)
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    args.yaml_dir.mkdir(parents=True, exist_ok=True)

    specs = build_specs(args.n_levels, args.phi_levels)
    combos = [(s, seed) for s in specs for seed in args.seeds]
    print(f"[grid-rich] {len(specs)} (n, phi) cells x {len(args.seeds)} seeds"
          f" = {len(combos)} runs")

    manifest_path = args.out / "manifest.csv"
    write_header = not manifest_path.exists()

    jobs = []
    for idx, ((label, prox, n_true, phi_true, cfg), seed) in enumerate(combos):
        yp = args.yaml_dir / f"{label}.yaml"
        if not yp.exists():
            yp.write_text(yaml.safe_dump(cfg, sort_keys=False))
        out_csv = args.out / f"{label}_seed{seed}.csv"
        port = args.sim_port_base + idx * 4
        jobs.append(dict(
            label=label, preset_proxy=prox,
            n_true=n_true, phi_true=phi_true,
            yaml_path=yp, out_csv=out_csv,
            seed=seed, duration=args.duration,
            sim_port=port, ctrl_port=port + 1))

    t_start = time.time()
    with ThreadPoolExecutor(max_workers=args.workers) as ex:
        futures = {ex.submit(collect_one_rich, **j): j for j in jobs}
        done = 0
        with manifest_path.open("a", newline="") as mf:
            w = csv.writer(mf)
            if write_header:
                w.writerow(["label", "n_true", "phi_true", "seed",
                            "ok", "wall_s", "csv"])
            for fut in as_completed(futures):
                j = futures[fut]
                t0 = time.time()
                try:
                    ok = bool(fut.result())
                except Exception as exc:
                    print(f"  [err] {j['label']} seed={j['seed']}: {exc}")
                    ok = False
                done += 1
                w.writerow([j["label"], f"{j['n_true']:.3f}",
                            f"{j['phi_true']:.2f}", j["seed"], int(ok),
                            round(time.time() - t0, 1), str(j["out_csv"])])
                print(f"[grid-rich] {done:3d}/{len(jobs)}  {j['label']:<20s}"
                      f"  seed={j['seed']}  ok={ok}"
                      f"  elapsed={(time.time()-t_start)/60.0:.1f}min")
    print(f"[grid-rich] done -- {done} runs in "
          f"{(time.time()-t_start)/60.0:.1f} min")


if __name__ == "__main__":
    main()
