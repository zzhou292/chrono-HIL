#!/usr/bin/env python3
"""
Load each NN tire checkpoint under nn_models/ and run numeric CasADi forward passes.

Usage:
  python smoke_test_nn_models.py --prefix paper_v2_
  python smoke_test_nn_models.py --prefix paper_v2_ --json ../nn_models/paper_v2_smoke_results.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent
SIM_DIR = PROJECT_DIR / "simulation"
NN_MODELS = PROJECT_DIR / "nn_models"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--prefix", default="", help="Only test dirs whose name starts with this")
    p.add_argument("--json", type=Path, default=None, help="Write results JSON")
    args = p.parse_args()

    sys.path.insert(0, str(SIM_DIR))
    from nn_tire_model import load_nn_tire_model
    from param_consistency import TERRAIN_PRESETS

    preset = TERRAIN_PRESETS["clay"]
    terrain = {
        "Kphi": float(preset["Kphi"]),
        "Kc": float(preset["Kc"]),
        "n": float(preset["n"]),
        "c": float(preset["cohesion"]),
        "phi": float(preset["friction_angle"]),
        "k": float(preset["janosi_shear"]),
    }

    dirs = sorted(d for d in NN_MODELS.iterdir() if d.is_dir() and (d / "best_terrain_nn.pt").is_file())
    if args.prefix:
        dirs = [d for d in dirs if d.name.startswith(args.prefix)]

    if not dirs:
        print(f"No model dirs under {NN_MODELS} with prefix {args.prefix!r}", file=sys.stderr)
        sys.exit(1)

    results = []
    failed = 0
    for d in dirs:
        rec = {"name": d.name, "ok": False, "error": None, "Fx": None, "Fy": None}
        try:
            model = load_nn_tire_model(d, terrain)
            K = int(getattr(model, "temporal_K", 1) or 1)
            hist = np.zeros(max(0, K - 1) * 5, dtype=np.float64) if K > 1 else None
            rates = np.zeros(3, dtype=np.float64) if getattr(model, "rate_augmented", False) else None
            Fx, Fy = model.predict_numeric(
                0.05,
                5000.0,
                5.0,
                0.0,
                steering_rate=0.2,
                terrain_params=terrain,
                hist=hist,
                rates=rates,
            )
            if not (np.isfinite(Fx) and np.isfinite(Fy)):
                raise ValueError(f"non-finite Fx={Fx}, Fy={Fy}")
            rec["ok"] = True
            rec["Fx"] = float(Fx)
            rec["Fy"] = float(Fy)
            print(f"OK  {d.name:45s}  Fx={Fx:10.2f}  Fy={Fy:10.2f}")
        except Exception as e:
            failed += 1
            rec["error"] = str(e)
            print(f"FAIL {d.name}: {e}")
        results.append(rec)

    summary = {"n_ok": sum(1 for r in results if r["ok"]), "n_fail": failed, "models": results}
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(json.dumps(summary, indent=2))
        print(f"Wrote {args.json}")

    if failed:
        sys.exit(1)
    print(f"All {len(results)} load/CasADi smoke test(s) passed.")


if __name__ == "__main__":
    main()
