#!/usr/bin/env python3
"""Collect vehicle-domain tire-force traces using the corrected open-loop diagnostic.

Each run launches `diag_force_match.py` for one terrain and writes a detailed CSV
containing actual axle forces, per-wheel slips/loads, and NN predictions under
the true terrain preset. These traces are suitable for training the
vehicle-domain force models in `simulation/force_residual_adapter.py`.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parent.parent


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--terrains", nargs="+", default=["clay", "dirt", "sand"])
    p.add_argument("--repeats", type=int, default=1)
    p.add_argument("--nn-model", default="factored_v1_resnet_h32_b2_sim")
    p.add_argument("--init-terrain", default="dirt")
    p.add_argument("--time", type=float, default=10.0)
    p.add_argument("--speed", type=float, default=5.0)
    p.add_argument("--speeds", default="", help="Comma-separated override sweep of speeds")
    p.add_argument("--steer-amp", type=float, default=0.5)
    p.add_argument("--steer-amps", default="", help="Comma-separated override sweep of steering amplitudes")
    p.add_argument("--steer-period", type=float, default=3.0)
    p.add_argument("--steer-periods", default="", help="Comma-separated override sweep of steering periods")
    p.add_argument("--out-dir", default="logs/vehicle_force_traces")
    p.add_argument("--base-port", type=int, default=7900)
    p.add_argument("--python", default=sys.executable,
                   help="Python executable to use for diag_force_match.py")
    args = p.parse_args()

    def _parse_sweep(raw: str, fallback: float) -> list[float]:
        vals = [x.strip() for x in raw.split(",") if x.strip()]
        if not vals:
            return [float(fallback)]
        return [float(x) for x in vals]

    out_dir = (ROOT / args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    speeds = _parse_sweep(args.speeds, args.speed)
    steer_amps = _parse_sweep(args.steer_amps, args.steer_amp)
    steer_periods = _parse_sweep(args.steer_periods, args.steer_period)

    run_idx = 0
    for rep in range(args.repeats):
        for terrain in args.terrains:
            for speed in speeds:
                for steer_amp in steer_amps:
                    for steer_period in steer_periods:
                        sim_port = args.base_port + 2 * run_idx
                        ctrl_port = sim_port + 1
                        csv_path = out_dir / (
                            f"trace_{terrain}"
                            f"_spd{speed:.1f}"
                            f"_amp{steer_amp:.2f}"
                            f"_per{steer_period:.2f}"
                            f"_rep{rep:02d}_{args.nn_model}.csv"
                        )
                        cmd = [
                            args.python,
                            str(ROOT / "new_diagnostics" / "diag_force_match.py"),
                            terrain,
                            "--nn-model", args.nn_model,
                            "--init-terrain", args.init_terrain,
                            "--time", str(args.time),
                            "--speed", str(speed),
                            "--steer-amp", str(steer_amp),
                            "--steer-period", str(steer_period),
                            "--sim-port", str(sim_port),
                            "--ctrl-port", str(ctrl_port),
                            "--output-csv", str(csv_path),
                        ]
                        print(
                            "[collect] "
                            f"terrain={terrain} rep={rep} speed={speed:.1f} "
                            f"amp={steer_amp:.2f} period={steer_period:.2f} "
                            f"-> {csv_path.name}"
                        )
                        result = subprocess.run(cmd, cwd=str(ROOT))
                        if result.returncode != 0:
                            print(
                                "[collect] FAILED "
                                f"terrain={terrain} rep={rep} speed={speed:.1f} "
                                f"amp={steer_amp:.2f} period={steer_period:.2f} "
                                f"rc={result.returncode}"
                            )
                            return int(result.returncode)
                        run_idx += 1

    print(f"[collect] wrote traces to {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
