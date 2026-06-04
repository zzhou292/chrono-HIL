#!/usr/bin/env python3
"""collect_lhs_training_scms.py
================================

Generate a uniform-LHS sweep over **both** soil parameters *and*
excitation parameters (steering amplitude, steering period, target
cruise speed) and run each scenario through the Chrono SCM HMMWV
runner. The resulting logs feed
``train_vehicle_fy_surrogate.py`` so the surrogate sees a broad
operating-point distribution rather than a single sinusoidal-steer
shape.

CLAUDE.md §8 compliance: every axis (Kphi, Kc, n, cohesion, friction
angle, janosi shear, steer amplitude, steer period, target speed) is
sampled uniformly over its documented box; no narrowing toward a
canonical preset or controller-conditioned operating point.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation"))
import param_consistency  # noqa
from param_consistency import generate_lhs_terrain_yaml_dicts  # noqa


# Widened LHS box for the *training* sweep only. The canonical clay /
# dirt / sand soils sit on the EDGES of the deployed TRAINING_RANGES_V6
# box (clay: Kphi at 5 % of range, janosi at the 0 % floor), so a
# surrogate trained on that box has to EXTRAPOLATE to reach them and
# fails on clay/loam. Widening the box (never narrowing — CLAUDE.md §8)
# turns those soils into interior points the surrogate interpolates to.
# The override is applied ONLY to this collector; the global
# TRAINING_RANGES_V6 (which documents the deployed rig models and the
# terrain-window MLP) is restored on exit.
_WIDENED_BOX = {
    "bekker_Kphi":   (0.3e6, 4.0e6),     # clay 0.69e6 → ~11 % (was 5 %)
    "bekker_n":      (0.35, 1.35),       # clay 0.5 → 15 %, sand 1.1 → 75 %
    "mohr_cohesion": (300.0, 20700.0),   # dirt 1700 / sand 1000 more interior
    "janosi_shear":  (0.008, 0.028),     # clay 0.01 → 10 %, dirt/sand 0.025 → 85 %
}


def _lhs_with_overrides(n_samples: int, seed: int,
                        overrides: dict):
    """Generate an LHS sweep with ``overrides`` temporarily merged into
    ``TRAINING_RANGES_V6``. Restores the global on exit so deployed
    models' documented ranges are untouched."""
    orig = dict(param_consistency.TRAINING_RANGES_V6)
    param_consistency.TRAINING_RANGES_V6 = {**orig, **overrides}
    try:
        return generate_lhs_terrain_yaml_dicts(n_samples, seed=seed)
    finally:
        param_consistency.TRAINING_RANGES_V6 = orig


@dataclass(frozen=True)
class Task:
    idx: int
    yaml_path: str
    log_path: str
    steer_amp: float
    steer_period: float
    target_speed: float
    open_loop_throttle: float       # < 0 means use PI cruise instead


def _worker(t: Task) -> Dict[str, float]:
    cmd = [
        sys.executable, "-u",
        str(ROOT / "data_collection" / "run_dallas_scm.py"),
        "--terrain-config", t.yaml_path,
        "--steer-amp-rad", f"{t.steer_amp:.3f}",
        "--steer-period",  f"{t.steer_period:.3f}",
        "--time", "30", "--lead-in", "2",
        "--output", t.log_path,
    ]
    if t.open_loop_throttle >= 0:
        cmd += ["--open-loop-throttle", f"{t.open_loop_throttle:.3f}"]
    else:
        cmd += ["--target-speed", f"{t.target_speed:.3f}"]
    out = subprocess.run(cmd, cwd=str(ROOT),
                          capture_output=True, text=True, timeout=300)
    ok = out.returncode == 0 and Path(t.log_path).exists()
    return {"idx": t.idx, "ok": int(ok),
            "stderr": out.stderr[-200:] if not ok else ""}


def main() -> int:
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("--n", type=int, default=300)
    p.add_argument("--workers", type=int, default=8)
    p.add_argument("--seed", type=int, default=7,
                   help="LHS seed — distinct from the benchmark seed (42) "
                        "so the training scenarios are genuinely disjoint "
                        "from the held-out lhs100 benchmark.")
    p.add_argument("--out-dir", default="data/dallas_scm/lhs_train300")
    p.add_argument("--widened-box", action="store_true",
                   help="Sample soils from the widened box so the canonical "
                        "clay/dirt/sand presets are interior (not edge) "
                        "points. Recommended for the deployed surrogate.")
    args = p.parse_args()

    out_dir = ROOT / args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    # Soil LHS. With --widened-box, sample from the enlarged box so the
    # canonical presets are interior; otherwise use the deployed
    # TRAINING_RANGES_V6 box verbatim.
    overrides = _WIDENED_BOX if args.widened_box else {}
    soils = _lhs_with_overrides(args.n, args.seed, overrides)
    # Manoeuvre LHS — independent uniform over each axis. Amplitude
    # widened to [0.20, 0.65] rad so the surrogate sees up to the
    # Buzhardt-style amp 0.60 used by the deployed window-MLP's
    # training (and bracketing the Dallas paper118 0.50 rad default).
    # Half of the scenarios run the Buzhardt-style scripted open-loop
    # constant throttle in [0.55, 0.85] (matches the deployed window-
    # MLP's training profile); the other half run PI cruise at
    # target_speed in [3.5, 7.0] m/s.
    rng = np.random.default_rng(args.seed + 1)
    amp   = rng.uniform(0.20, 0.65, args.n)
    per   = rng.uniform(2.0,  5.0, args.n)
    speed = rng.uniform(3.5,  7.0, args.n)
    ol_throttle = rng.uniform(0.55, 0.85, args.n)
    use_ol = rng.random(args.n) < 0.5
    ol_throttle = np.where(use_ol, ol_throttle, -1.0)   # -1 = PI cruise

    tasks: List[Task] = []
    for i, soil in enumerate(soils):
        yaml_path = out_dir / f"scn_{i:03d}.yaml"
        log_path  = out_dir / f"scn_{i:03d}.npz"
        if not yaml_path.exists():
            cfg = {k: float(v) for k, v in soil.items()
                    if isinstance(v, (int, float))}
            cfg["__steer_amp"] = float(amp[i])
            cfg["__steer_period"] = float(per[i])
            cfg["__target_speed"] = float(speed[i])
            with open(yaml_path, "w") as f:
                json.dump(cfg, f, indent=2)
        tasks.append(Task(idx=i, yaml_path=str(yaml_path),
                           log_path=str(log_path),
                           steer_amp=float(amp[i]),
                           steer_period=float(per[i]),
                           target_speed=float(speed[i]),
                           open_loop_throttle=float(ol_throttle[i])))

    print(f"Running {len(tasks)} LHS scenarios on {args.workers} workers...")
    done = 0
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        futs = {ex.submit(_worker, t): t for t in tasks}
        for fut in as_completed(futs):
            r = fut.result()
            done += 1
            tag = "OK " if r["ok"] else "FAIL"
            print(f"  [{done:3d}/{len(tasks)}] {tag} idx={r['idx']}"
                  + (f"  {r['stderr']}" if not r["ok"] else ""),
                  flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
