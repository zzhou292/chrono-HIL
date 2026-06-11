#!/usr/bin/env python3
"""Calibrate a SINGLE GLOBAL Pacejka + TMeasy parameter set to SCM.

Fair-baseline calibration for the tire-model comparison: rather than the
rigid-road .tir defaults (mu=0.74, B=8.77), fit one global parameter set to
the pooled SCM tire-rig force data across the whole soil box (one set for all
terrains, since the controller has no per-terrain knowledge). The resulting
defaults live in ``simulation/analytical_tire_models.py``.

The SCM peak lateral friction is ~0.32 (median; 0.18 clay-like to 0.47
sand-like), so the calibrated mu (~0.42) is far below a rigid road and the
analytical model no longer over-corners. A single global set still cannot
follow terrain-to-terrain force variation -- that residual gap is what the NN
surrogate closes, and reporting against this *calibrated* baseline (not the
rigid-road strawman) is what makes the comparison fair.
"""
from __future__ import annotations
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.optimize import least_squares

DATA = Path(__file__).resolve().parents[1] / "data" / "tire_rig" / "scm_static_100k_v4.csv"


def main():
    df = pd.read_csv(DATA)
    m = df[df.slip_ratio.abs() < 0.05]               # pure-cornering subset
    a = m.slip_angle.abs().to_numpy()
    rcoef = (m.Fy.abs() / m.Fz).to_numpy()           # |Fy/Fz| magnitude
    Fy = m.Fy.abs().to_numpy()

    # Pacejka magnitude: |Fy/Fz| = mu*sin(C*atan(B*a - E*(B*a - atan(B*a))))
    def pac(p, x):
        B, C, E, mu = p; Bx = B * x
        return mu * np.sin(C * np.arctan(Bx - E * (Bx - np.arctan(Bx))))
    rp = least_squares(lambda p: pac(p, a) - rcoef, [6, 1.5, 0.4, 0.35],
                       bounds=([1, 1.0, 0.0, 0.1], [20, 2.0, 1.0, 1.2]))
    B, C, E, mu = rp.x

    # TMeasy magnitude (balanced load, per-tire = 0.5*Fym*sin(min(pi/2*a/am, pi/2)))
    def tm(p, x):
        Fym, am = p; hp = np.pi / 2
        return 0.5 * Fym * np.sin(np.minimum(hp * x / max(am, 1e-4), hp))
    rt = least_squares(lambda p: tm(p, a) - Fy, [6000, 0.15], bounds=([1000, 0.03], [20000, 0.40]))
    Fym, am = rt.x

    # SCM peak lateral friction by soil firmness (saturated, |alpha|>0.25)
    big = m[m.slip_angle.abs() > 0.25]
    print(f"rows fit: {len(m)} (pure-cornering subset of {len(df)})")
    print(f"\nSCM peak |Fy|/Fz (saturated):  overall median={np.median((big.Fy.abs()/big.Fz)):.2f}")
    for lo, hi, lbl in [(0, 15, 'clay-like'), (15, 25, 'dirt-like'), (25, 40, 'sand-like')]:
        s = big[(big.mohr_friction >= np.radians(lo)) & (big.mohr_friction < np.radians(hi))]
        if len(s):
            print(f"  phi[{lo:2d},{hi:2d})deg ({lbl:9s}) peak|Fy/Fz|={np.median((s.Fy.abs()/s.Fz)):.2f}")
    print(f"\nleast-squares Pacejka fit:  B={B:.2f} C={C:.3f} E={E:.3f}  mu={mu:.3f}")
    print(f"least-squares TMeasy fit:   Fym={Fym:.0f} N alpha_m={am:.3f} rad ({np.degrees(am):.1f} deg)")
    print(f"\nADOPTED in simulation/analytical_tire_models.py: PACEJKA_MU = {mu:.2f}")
    print("  (single global peak friction; magic-formula / TMeasy shape factors kept at")
    print("   standard values -- a full shape refit collapses cornering stiffness and")
    print("   cripples closed-loop tracking, unfairly sandbagging the baselines.)")


if __name__ == "__main__":
    main()
