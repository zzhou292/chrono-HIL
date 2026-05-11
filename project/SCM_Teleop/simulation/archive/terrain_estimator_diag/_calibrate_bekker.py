#!/usr/bin/env python3
"""Calibrate a global Bekker-vs-SCM scale factor using vehicle-frame Fy data.

The reduced-order Bekker model from Dallas et al. (2020) systematically
under-predicts SCM forces because it omits dynamic and bulldozing effects.
Dallas restores agreement via terrain-specific ``g1, g2, g3`` correction
functions; for the terrain estimator we only need the *relative* response of
``Fy`` to ``n`` to be correct, so a single scalar gain is sufficient and
keeps the model fully analytical and physically interpretable.

Reads ``data/vehicle_forces/vehicle_*.csv`` (each row carries true terrain
parameters and the SCM ground-truth ``Fy``), runs the analytical Bekker
model on the same operating point, and computes the least-squares gain
``g`` minimising ``∑ (Fy_scm − g · Fy_bekker)^2``.
"""

import math
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).parent))

from bekker_tire_model import TerrainParams, TireGeometry, lateral_force


CSV_FILES = {
    "clay": Path(__file__).parent.parent / "data" / "vehicle_forces" / "vehicle_clay.csv",
    "dirt": Path(__file__).parent.parent / "data" / "vehicle_forces" / "vehicle_dirt.csv",
    "sand": Path(__file__).parent.parent / "data" / "vehicle_forces" / "vehicle_sand.csv",
}

geom = TireGeometry(radius=0.47, width=0.30)


def predict_row(row) -> float:
    terr = TerrainParams(
        Kphi=float(row.bekker_Kphi),
        Kc=float(row.bekker_Kc),
        n=float(row.bekker_n),
        c=float(row.mohr_cohesion),
        phi=float(row.mohr_friction),       # already in radians
        k=float(row.janosi_shear),
    )
    _, fy, _ = lateral_force(
        slip_angle=float(row.slip_angle),
        slip_ratio=float(row.slip_ratio),
        load_W=max(float(row.vertical_load), 1.0),
        velocity=max(abs(float(row.velocity)), 0.5),
        geom=geom,
        terrain=terr,
    )
    return fy


def calibrate(name, csv_path):
    df = pd.read_csv(csv_path)
    # Restrict to operating ranges where Bekker is valid (skip near-zero alpha
    # and full-saturation slip).  Also subsample for speed.
    # Restrict to driving-envelope operating points (low longitudinal slip,
    # moderate lateral slip, sensible loads, normal vehicle speeds).
    df = df[(df.slip_angle.abs() > 0.02) & (df.slip_angle.abs() < 0.30)
            & (df.slip_ratio.abs() < 0.15)
            & (df.velocity > 2.0)
            & (df.vertical_load.between(3000, 8500))]
    df = df.sample(n=min(len(df), 1500), random_state=0)

    Fy_scm = df.Fy.values
    Fy_bek = np.array([predict_row(row) for row in df.itertuples()])

    # Filter degenerate Bekker outputs (sinkage solver failures).
    mask = (np.isfinite(Fy_bek)) & (np.abs(Fy_bek) > 5.0)
    Fy_scm = Fy_scm[mask]
    Fy_bek = Fy_bek[mask]
    if len(Fy_scm) < 50:
        print(f"  {name}: insufficient samples after filtering")
        return None

    # SCM "Fy" follows the rig sign convention (opposite of bicycle-model
    # convention).  Flip Bekker prediction to match before fitting.
    Fy_bek_signed = -Fy_bek

    # Least-squares scale: g = <Fy_scm, Fy_bek> / <Fy_bek, Fy_bek>
    g_ls = float(np.dot(Fy_scm, Fy_bek_signed) / np.dot(Fy_bek_signed, Fy_bek_signed))
    rmse_pre  = float(np.sqrt(np.mean((Fy_scm - Fy_bek_signed)**2)))
    rmse_post = float(np.sqrt(np.mean((Fy_scm - g_ls * Fy_bek_signed)**2)))
    corr      = float(np.corrcoef(Fy_scm, Fy_bek_signed)[0, 1])
    print(f"  {name:>5s}: N={len(Fy_scm):4d}  g={g_ls:.3f}  "
          f"corr={corr:+.3f}  rmse pre={rmse_pre:7.0f} N  post={rmse_post:7.0f} N")
    return g_ls


print("Per-terrain Bekker → SCM calibration (single gain):")
gains = {}
for name, path in CSV_FILES.items():
    if path.exists():
        g = calibrate(name, path)
        if g is not None:
            gains[name] = g


# ─── Joint v-dependent calibration: Fy_scm = g * v^a * Fy_bekker ────────
print("\n\nJoint velocity-dependent calibration (all terrains pooled):")
all_scm, all_bek, all_v = [], [], []
for name, path in CSV_FILES.items():
    if not path.exists():
        continue
    df = pd.read_csv(path)
    df = df[(df.slip_angle.abs() > 0.02) & (df.slip_angle.abs() < 0.30)
            & (df.slip_ratio.abs() < 0.15)
            & (df.velocity > 2.0)
            & (df.vertical_load.between(3000, 8500))]
    df = df.sample(n=min(len(df), 1500), random_state=0)
    Fy_bek = np.array([predict_row(row) for row in df.itertuples()])
    mask = np.isfinite(Fy_bek) & (np.abs(Fy_bek) > 5.0)
    all_scm.append(df.Fy.values[mask])
    all_bek.append(-Fy_bek[mask])         # rig sign convention flip
    all_v.append(df.velocity.values[mask])

Fy_scm = np.concatenate(all_scm)
Fy_bek = np.concatenate(all_bek)
v      = np.concatenate(all_v)

# Restrict to same-sign samples (Bekker either matches or is opposite, fit
# only where signs agree to keep regression sensible).
sign_ok = np.sign(Fy_scm) == np.sign(Fy_bek)
Fy_scm = Fy_scm[sign_ok]
Fy_bek = Fy_bek[sign_ok]
v      = v[sign_ok]
print(f"  N samples after sign filter: {len(Fy_scm)}")

# Log-linear fit: log|Fy_scm / Fy_bek| = log g + a log v
ratio = Fy_scm / Fy_bek
mask = (ratio > 0.05) & (ratio < 20.0)
log_ratio = np.log(ratio[mask])
log_v     = np.log(v[mask])

A = np.vstack([np.ones_like(log_v), log_v]).T
coef, _, _, _ = np.linalg.lstsq(A, log_ratio, rcond=None)
log_g, a = coef
g0 = float(math.exp(log_g))
print(f"  Best fit: Fy_scm ≈ {g0:.3f} * v^{a:+.3f} * Fy_bekker")
# Evaluation:
Fy_pred = g0 * np.power(v[mask], a) * Fy_bek[mask]
rmse_pre  = float(np.sqrt(np.mean((Fy_scm[mask] - Fy_bek[mask])**2)))
rmse_post = float(np.sqrt(np.mean((Fy_scm[mask] - Fy_pred)**2)))
print(f"  RMSE pre={rmse_pre:.0f} N → post={rmse_post:.0f} N")
