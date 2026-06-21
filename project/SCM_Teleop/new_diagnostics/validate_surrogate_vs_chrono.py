#!/usr/bin/env python3
"""
validate_surrogate_vs_chrono.py
================================

Compare an NN tire surrogate's per-wheel Fy predictions to ground-truth
Chrono closed-loop data captured by ``diag_force_match.py``.

The diag CSVs in ``logs/diag_force_match_<terrain>_<model>_*.csv`` already
record per-wheel slip angle, Fz, and longitudinal slip for every physics
step of a closed-loop run, *and* the actual per-wheel Fy that Chrono
computed.  This script re-runs an NN through those same per-wheel inputs
and reports correlation, RMS error, sign agreement, and the
prediction-magnitude vs ground-truth range.

Why this matters
----------------
The static-MLP/ResNet surrogates were trained on a *tire test rig*
(constant slip, measure steady-state Fy).  R² on the rig test set
exceeds 0.98, but those steady-state forces are a *very* different
distribution from what an actual SCM-driven vehicle experiences:
sinkage rate, suspension load transfer, terrain heterogeneity, and
fast slip-angle transients all dominate the closed-loop force trace
and are absent from the rig dataset.

On the diag CSV captured during a clay closed-loop run, the
``paper_v2_mlp_16_4`` model has *near-zero* per-wheel Fy correlation
with actual Chrono Fy (~0.1).  The predicted force *range* is also
markedly smaller than the actual range.  This explains why the
predictive safety shields (MPPI / NMPC) struggle relative to DOB-CBF:
they reason about a dynamics model that doesn't actually match
closed-loop physics.

Usage
-----
    python new_diagnostics/validate_surrogate_vs_chrono.py \
        --diag logs/diag_force_match_clay_factored_v1_resnet_h32_b2_sim_v3.csv \
        --models paper_v2_mlp_16_4 paper_v2_resnet_h32_b2

To capture a fresh diag against a current model, see
``new_diagnostics/diag_force_match.py``.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / 'simulation'))

from param_consistency import get_terrain_preset, terrain_preset_to_internal  # noqa: E402
from nn_tire_model import load_nn_tire_model                                 # noqa: E402


WHEELS = ['front_left', 'front_right', 'rear_left', 'rear_right']


def score_model(model_dir: Path, df: pd.DataFrame, terrain_params: dict) -> dict:
    nn = load_nn_tire_model(str(model_dir), terrain_params)
    rate_aug = getattr(nn, 'rate_augmented', False)
    out = {'name': model_dir.name}
    rms_avg = 0.0
    for wheel in WHEELS:
        a, p = [], []
        for _, row in df.iterrows():
            alpha = float(row[f'{wheel}_slip_angle'])
            Fz = float(row[f'{wheel}_Fz'])
            kappa = float(row[f'{wheel}_long_slip'])
            u = float(row['u'])
            try:
                kwargs = {'kappa': kappa}
                if rate_aug:
                    kwargs['steering_rate'] = float(row['steering_rate'])
                    kwargs['rates'] = np.array([
                        float(row.get(f'{wheel}_d_slip_ratio', 0.0)),
                        float(row.get(f'{wheel}_d_slip_angle', 0.0)),
                        float(row.get('du', 0.0)),
                    ])
                _, fy = nn.predict_numeric(alpha, Fz, u, **kwargs)
            except Exception:
                continue
            a.append(float(row[f'{wheel}_fy']))
            p.append(fy)
        a, p = np.array(a), np.array(p)
        rms = float(np.sqrt(np.mean((a - p) ** 2))) if len(a) else float('nan')
        rms_avg += rms / len(WHEELS)
        if a.std() > 0 and p.std() > 0:
            corr = float(np.corrcoef(a, p)[0, 1])
        else:
            corr = float('nan')
        sign_agree = (float(np.mean(np.sign(a) == np.sign(p))) * 100
                       if len(a) else float('nan'))
        out[f'{wheel}_corr'] = corr
        out[f'{wheel}_rms'] = rms
        out[f'{wheel}_sign_pct'] = sign_agree
        out[f'{wheel}_pred_max'] = float(np.max(np.abs(p))) if len(p) else 0.0
        out[f'{wheel}_actual_max'] = float(np.max(np.abs(a))) if len(a) else 0.0
    out['rms_avg'] = rms_avg
    return out


def main():
    ap = argparse.ArgumentParser(__doc__)
    ap.add_argument('--diag', type=Path, required=True,
                    help='Path to a logs/diag_force_match_*.csv with '
                         'per-wheel ground-truth slip and Fy from a closed-loop run.')
    ap.add_argument('--terrain', default=None,
                    choices=['clay', 'sand', 'dirt'],
                    help='Terrain preset for the model.  Defaults to the '
                         'true_terrain column in the diag CSV.')
    ap.add_argument('--models', nargs='+', required=True,
                    help='Model directory names under nn_models/')
    args = ap.parse_args()

    df = pd.read_csv(args.diag).sort_values('time').reset_index(drop=True)
    # Compute slip-rate columns for rate-augmented models
    dt = df['time'].diff().median()
    for w in WHEELS:
        df[f'{w}_d_slip_angle'] = df[f'{w}_slip_angle'].diff() / dt
        df[f'{w}_d_slip_ratio'] = df[f'{w}_long_slip'].diff() / dt
    df['du'] = df['u'].diff() / dt
    df = df.dropna(subset=[f'{WHEELS[0]}_d_slip_angle']).reset_index(drop=True)

    terrain = args.terrain or str(df['true_terrain'].iloc[0])
    tp = terrain_preset_to_internal(get_terrain_preset(terrain))
    print(f"Diag: {args.diag.name}   ({len(df)} rows after diff, dt={dt:.4f}s)")
    print(f"Terrain: {terrain}\n")

    print(f"{'Model':<32s}  "
          f"{'corr_FL':>8} {'corr_FR':>8} {'corr_RL':>8} {'corr_RR':>8}  "
          f"{'avg RMS (N)':>12}  {'sign agree':>11}")
    for m in args.models:
        mdir = REPO / 'nn_models' / m
        if not mdir.exists():
            print(f"  [skip] {m}: not found")
            continue
        r = score_model(mdir, df, tp)
        sign_pct = float(np.mean([r[f'{w}_sign_pct'] for w in WHEELS]))
        print(f"{r['name']:<32s}  "
              f"{r['front_left_corr']:>+8.2f} {r['front_right_corr']:>+8.2f} "
              f"{r['rear_left_corr']:>+8.2f} {r['rear_right_corr']:>+8.2f}  "
              f"{r['rms_avg']:>12.0f}  {sign_pct:>10.0f}%")

    # Print a "model-prediction range" line so the user can see whether
    # the surrogate is consistently *under-predicting* the magnitude of
    # closed-loop forces (the more diagnostic signal than correlation
    # alone — a model with the right shape but wrong gain shows up
    # here).
    print()
    for w in WHEELS:
        actual_max = float(np.max(np.abs(df[f'{w}_fy'])))
        print(f"  actual_{w}_max |Fy| = {actual_max:.0f} N")


if __name__ == '__main__':
    main()
