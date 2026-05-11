#!/usr/bin/env python3
"""Sanity check the analytical Bekker tire model on the three terrain
presets across realistic operating ranges."""

import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from bekker_tire_model import (
    TerrainParams, TireGeometry, lateral_force, bekker_axle_forces,
)
from param_consistency import TERRAIN_PRESETS, terrain_preset_to_internal

geom = TireGeometry(radius=0.47, width=0.30)

print(f"Tire geom: r={geom.radius}m, b={geom.width}m\n")

for terrain_name in ("clay", "dirt", "sand"):
    p = terrain_preset_to_internal(TERRAIN_PRESETS[terrain_name])
    terr = TerrainParams(
        Kphi=p["Kphi"], Kc=p["Kc"], n=p["n"],
        c=p["c"], phi=np.radians(p["phi"]), k=p["k"],
    )
    print(f"=== {terrain_name} (n={p['n']}, phi={p['phi']:.0f}deg, c={p['c']}) ===")
    print(f"{'alpha':>8} {'Fz':>8} {'Fx':>8} {'Fy':>8} {'h':>6}")
    for alpha in (0.02, 0.05, 0.10, 0.20):
        for Fz in (5500.0, 6500.0, 7500.0):
            Fx, Fy, h = lateral_force(alpha, 0.05, Fz / 2.0, 5.0, geom, terr)
            print(f"{alpha:8.3f} {Fz:8.0f} {Fx:8.1f} {Fy:8.1f} {h:6.4f}")
    print()

# Sweep n through manifold to verify monotonic Fy growth.
print("\n=== Fy(n) sweep at alpha=0.10, Fz=6500/2 ===")
print(f"{'n':>5} {'clay':>10} {'dirt':>10} {'sand':>10}")
for n in np.arange(0.4, 1.3, 0.1):
    row = [f"{n:5.2f}"]
    # Build a "preset-adjacent" terrain at this n (use clay base, just modify n).
    base = terrain_preset_to_internal(TERRAIN_PRESETS["clay"])
    terr = TerrainParams(
        Kphi=base["Kphi"], Kc=base["Kc"], n=float(n),
        c=base["c"], phi=np.radians(base["phi"]), k=base["k"],
    )
    _, fy, _ = lateral_force(0.10, 0.05, 6500.0 / 2.0, 5.0, geom, terr)
    row.append(f"{fy:10.1f}")
    base = terrain_preset_to_internal(TERRAIN_PRESETS["dirt"])
    terr = TerrainParams(
        Kphi=base["Kphi"], Kc=base["Kc"], n=float(n),
        c=base["c"], phi=np.radians(base["phi"]), k=base["k"],
    )
    _, fy, _ = lateral_force(0.10, 0.05, 6500.0 / 2.0, 5.0, geom, terr)
    row.append(f"{fy:10.1f}")
    base = terrain_preset_to_internal(TERRAIN_PRESETS["sand"])
    terr = TerrainParams(
        Kphi=base["Kphi"], Kc=base["Kc"], n=float(n),
        c=base["c"], phi=np.radians(base["phi"]), k=base["k"],
    )
    _, fy, _ = lateral_force(0.10, 0.05, 6500.0 / 2.0, 5.0, geom, terr)
    row.append(f"{fy:10.1f}")
    print(" ".join(row))
