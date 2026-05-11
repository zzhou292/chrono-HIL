#!/usr/bin/env python3
"""Launch the factored static SCM collector with recommended defaults."""

from __future__ import annotations

import argparse
import math
import subprocess
from pathlib import Path


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("num_samples", type=int)
    p.add_argument("output_csv")
    p.add_argument("--build-dir", default=str(Path(__file__).resolve().parents[2] / "build"))
    p.add_argument("--terrain-bank-size", type=int, default=0)
    p.add_argument("--threads", type=int, default=0)
    p.add_argument("--batch-size", type=int, default=0)
    p.add_argument("--sequential", action="store_true")
    args = p.parse_args()

    build_dir = Path(args.build_dir).resolve()
    exe = build_dir / "bin" / "collect_static_data"
    if not exe.exists():
        exe = build_dir / "collect_static_data"
    if not exe.exists():
        raise FileNotFoundError(f"collect_static_data not found under {build_dir}")

    terrain_bank_size = (
        args.terrain_bank_size
        if args.terrain_bank_size > 0
        else max(16, int(round(math.sqrt(args.num_samples))))
    )

    cmd = [
        str(exe),
        str(args.num_samples),
        args.output_csv,
        "--factored",
        "--terrain-bank-size",
        str(terrain_bank_size),
    ]
    if args.threads > 0:
        cmd.extend(["--threads", str(args.threads)])
    if args.batch_size > 0:
        cmd.extend(["--batch-size", str(args.batch_size)])
    if args.sequential:
        cmd.append("--sequential")

    print("Running:", " ".join(cmd))
    return subprocess.call(cmd)


if __name__ == "__main__":
    raise SystemExit(main())
