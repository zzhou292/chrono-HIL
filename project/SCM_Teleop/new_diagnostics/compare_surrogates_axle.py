#!/usr/bin/env python3
"""
compare_surrogates_axle.py
==========================

Fair per-*axle* validation of one or more NN tire surrogates against a
closed-loop Chrono diagnostic CSV (e.g. ``logs/diag_force_match_*.csv``).

Why this script in addition to ``validate_surrogate_vs_chrono.py``?
The new ``closed_loop_v1_*`` models are trained on bicycle-model
*axle-averaged* inputs (front slip / front Fz), whereas the rig-trained
``paper_v2_*`` models were trained per-wheel.  Feeding per-wheel data
to the new model is unfair; this script averages each axle's
slip / Fz / kappa across left+right wheels and queries the NN once
per axle, then compares the predicted *axle* Fy (×2 for the pair) to
the actual axle Fy (sum of left + right wheel Fy) — which is what
the surrogate dynamics actually use.

Usage::

    python new_diagnostics/compare_surrogates_axle.py \\
        --diag logs/diag_force_match_clay_factored_v1_resnet_h32_b2_sim_v3.csv \\
        --models paper_v2_mlp_16_4 closed_loop_v1_mlp_16_4 \\
                 closed_loop_v1_mlp_32_16 closed_loop_v1_mlp_temporal_K4_16_8
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


def score(model_dir: Path, df: pd.DataFrame, terrain_params: dict) -> dict:
    nn = load_nn_tire_model(str(model_dir), terrain_params)
    front_actual = (df['front_left_fy'] + df['front_right_fy']).to_numpy()
    rear_actual = (df['rear_left_fy'] + df['rear_right_fy']).to_numpy()
    front_pred = np.zeros(len(df))
    rear_pred = np.zeros(len(df))
    # Bicycle-model averaging — same operating point the surrogate
    # dynamics rollout actually feeds the NN.
    for i, row in df.iterrows():
        alpha_f = 0.5 * (row['front_left_slip_angle'] + row['front_right_slip_angle'])
        alpha_r = 0.5 * (row['rear_left_slip_angle']  + row['rear_right_slip_angle'])
        Fz_f = 0.5 * (row['front_left_Fz']  + row['front_right_Fz'])
        Fz_r = 0.5 * (row['rear_left_Fz']  + row['rear_right_Fz'])
        kappa_f = 0.5 * (row['front_left_long_slip']  + row['front_right_long_slip'])
        kappa_r = 0.5 * (row['rear_left_long_slip']  + row['rear_right_long_slip'])
        u = max(float(row['u']), 0.5)
        try:
            _, fy_f = nn.predict_numeric(alpha_f, Fz_f, u, kappa=kappa_f)
            _, fy_r = nn.predict_numeric(alpha_r, Fz_r, u, kappa=kappa_r)
        except Exception:
            fy_f = fy_r = float('nan')
        front_pred[i] = 2 * fy_f
        rear_pred[i] = 2 * fy_r

    out = {'name': model_dir.name}
    for label, a, p in [('front', front_actual, front_pred),
                         ('rear', rear_actual, rear_pred)]:
        mask = np.isfinite(p) & np.isfinite(a)
        if mask.sum() < 10:
            out[f'{label}_corr'] = float('nan')
            out[f'{label}_rms'] = float('nan')
            out[f'{label}_sign'] = float('nan')
            continue
        a, p = a[mask], p[mask]
        out[f'{label}_corr'] = float(np.corrcoef(a, p)[0, 1]) if a.std() and p.std() else float('nan')
        out[f'{label}_rms'] = float(np.sqrt(np.mean((a - p) ** 2)))
        out[f'{label}_sign'] = float(np.mean(np.sign(a) == np.sign(p)) * 100)
        out[f'{label}_actual_max_abs'] = float(np.max(np.abs(a)))
        out[f'{label}_pred_max_abs'] = float(np.max(np.abs(p)))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--diag', type=Path, required=True)
    ap.add_argument('--models', nargs='+', required=True)
    ap.add_argument('--terrain', default=None,
                    choices=['clay', 'sand', 'dirt'])
    args = ap.parse_args()

    df = pd.read_csv(args.diag).sort_values('time').reset_index(drop=True)
    terrain = args.terrain or str(df['true_terrain'].iloc[0])
    tp = terrain_preset_to_internal(get_terrain_preset(terrain))
    print(f'Diag: {args.diag.name}  ({len(df)} rows, terrain={terrain})\n')

    print(f"{'Model':<40s}  {'F corr':>8} {'F rms (N)':>10} {'F sign%':>8} {'F pred|max|':>11} "
          f"{'R corr':>8} {'R rms (N)':>10} {'R sign%':>8} {'R pred|max|':>11}")
    for name in args.models:
        mdir = REPO / 'nn_models' / name
        if not mdir.exists():
            print(f"  [skip] {name}: not found")
            continue
        r = score(mdir, df, tp)
        print(f"{r['name']:<40s}  "
              f"{r['front_corr']:>+8.2f} {r['front_rms']:>10.0f} {r['front_sign']:>8.0f} "
              f"{r['front_pred_max_abs']:>11.0f}  "
              f"{r['rear_corr']:>+8.2f} {r['rear_rms']:>10.0f} {r['rear_sign']:>8.0f} "
              f"{r['rear_pred_max_abs']:>11.0f}")
    # Ground-truth reference
    front_actual = (df['front_left_fy'] + df['front_right_fy']).to_numpy()
    rear_actual = (df['rear_left_fy'] + df['rear_right_fy']).to_numpy()
    print(f"\nGround-truth |Fy_axle| max: front={np.max(np.abs(front_actual)):.0f} N, "
          f"rear={np.max(np.abs(rear_actual)):.0f} N")


if __name__ == '__main__':
    main()
