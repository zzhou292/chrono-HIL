#!/usr/bin/env python3
"""Generate random *unseen* SCM soils for closed-loop estimator tests.

Each soil is sampled by:
  - drawing a random target n in a configurable range,
  - interpolating along the (clay → dirt → sand) preset manifold to get
    the on-manifold seed for (Kphi, Kc, cohesion, friction_angle,
    janosi_shear),
  - then *off-manifold* perturbing cohesion (×0.5–1.5) and friction
    angle (±5°) so the resulting parameter vector is not on the
    interpolation curve the estimator was trained around.

Soils are written as ``terrain1.yaml`` / ``terrain2.yaml`` / … in
``data/terrain_yamls_random/`` and a manifest CSV is produced for
downstream consumers.
"""

from __future__ import annotations

import argparse
import csv
import json
import random
import sys
from pathlib import Path
from typing import Dict, List

import yaml

ROOT = Path(__file__).resolve().parent.parent
SIM_DIR = ROOT / "simulation"
sys.path.insert(0, str(SIM_DIR))

from collect_diverse_terrains import interp_terrain, closest_preset
from param_consistency import TERRAIN_PRESETS


def sample_random_terrain(
    n_low: float, n_high: float, rng: random.Random,
) -> Dict[str, float]:
    n = rng.uniform(n_low, n_high)
    cfg = interp_terrain(n)

    # Off-manifold cohesion perturbation: 0.5×–1.5× the on-manifold value
    coh_scale = rng.uniform(0.5, 1.5)
    cfg["cohesion"] = float(cfg["cohesion"]) * coh_scale

    # Off-manifold friction-angle perturbation: ±5° (clamped to a sane range)
    phi_delta = rng.uniform(-5.0, 5.0)
    cfg["friction_angle"] = float(
        max(8.0, min(40.0, cfg["friction_angle"] + phi_delta))
    )

    cfg["elastic_stiffness"] = 2e8
    cfg["damping"] = 3e4
    cfg["description"] = (
        f"random_n{n:.3f}_coh{coh_scale:.2f}_phi{phi_delta:+.1f}"
    )
    cfg["_sample_meta"] = {
        "target_n": n,
        "coh_scale": coh_scale,
        "phi_delta_deg": phi_delta,
    }
    return cfg


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out-dir", default=str(
        Path(__file__).parent.parent / "data" / "terrain_yamls_random"))
    p.add_argument("--n-terrains", type=int, default=6)
    p.add_argument("--n-low", type=float, default=0.50,
                   help="Lower bound for random n")
    p.add_argument("--n-high", type=float, default=1.10,
                   help="Upper bound for random n")
    p.add_argument("--seed", type=int, default=42)
    args = p.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    rng = random.Random(args.seed)

    manifest_rows: List[Dict] = []
    for i in range(1, args.n_terrains + 1):
        cfg = sample_random_terrain(args.n_low, args.n_high, rng)
        meta = cfg.pop("_sample_meta")
        label = f"terrain{i}"
        yaml_path = out_dir / f"{label}.yaml"
        # `yaml.safe_dump` rejects np floats but we already cast to float.
        yaml_path.write_text(yaml.safe_dump(cfg, sort_keys=False))
        proxy = closest_preset(meta["target_n"])
        manifest_rows.append({
            "label": label,
            "yaml": str(yaml_path),
            "true_n": round(meta["target_n"], 4),
            "coh_scale": round(meta["coh_scale"], 3),
            "phi_delta_deg": round(meta["phi_delta_deg"], 2),
            "preset_proxy": proxy,
            "Kphi": round(cfg["Kphi"], 1),
            "Kc": round(cfg["Kc"], 1),
            "cohesion": round(cfg["cohesion"], 1),
            "friction_angle": round(cfg["friction_angle"], 2),
            "janosi_shear": round(cfg["janosi_shear"], 4),
        })
        print(f"[gen] {label}  n={meta['target_n']:.3f}  "
              f"coh×{meta['coh_scale']:.2f}  phi{meta['phi_delta_deg']:+.1f}°  "
              f"proxy={proxy}")

    manifest_path = out_dir / "manifest.csv"
    with manifest_path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(manifest_rows[0].keys()))
        w.writeheader()
        w.writerows(manifest_rows)
    (out_dir / "manifest.json").write_text(
        json.dumps(manifest_rows, indent=2))
    print(f"\n[gen] wrote {len(manifest_rows)} terrains to {out_dir}")
    print(f"[gen] manifest: {manifest_path}")


if __name__ == "__main__":
    main()
