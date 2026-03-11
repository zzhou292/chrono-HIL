#!/usr/bin/env python3
"""
Benchmark: Linear vs NN MPC across random soil parameters.
Samples terrain within the NN training range and compares controllers.
"""

import numpy as np
import sys
import json
import time as time_mod
from pathlib import Path
from datetime import datetime

sys.path.append(str(Path(__file__).parent))

from param_consistency import TRAINING_RANGES_V6 as TRAINING_RANGES


def sample_terrain(rng):
    """Sample terrain parameters uniformly from the NN training range."""
    import math
    return {
        'Kphi':  rng.uniform(*TRAINING_RANGES['bekker_Kphi']),
        'Kc':    rng.uniform(*TRAINING_RANGES['bekker_Kc']),
        'n':     rng.uniform(*TRAINING_RANGES['bekker_n']),
        'cohesion':       rng.uniform(*TRAINING_RANGES['mohr_cohesion']),
        # v6 mohr_friction is in radians; convert to degrees for terrain config
        'friction_angle': math.degrees(rng.uniform(*TRAINING_RANGES['mohr_friction'])),
        'janosi_shear':   rng.uniform(*TRAINING_RANGES['janosi_shear']),
    }


def terrain_label(t):
    """Short human-readable label for a terrain config."""
    return (f"Kphi={t['Kphi']:.1e} n={t['n']:.2f} "
            f"c={t['cohesion']:.0f} phi={t['friction_angle']:.0f}")


def run_one(controller_type, terrain_config, path_type, sim_time, speed,
            kappa_mode='zero', quiet=True):
    """
    Run a single simulation and return metrics dict.
    Imports are done inside the function so each call gets a fresh module state.
    """
    from dallas_chrono_demo import run_simulation
    
    old_stdout = sys.stdout
    if quiet:
        sys.stdout = open('/dev/null', 'w')
    try:
        rms = run_simulation(
            controller_type=controller_type,
            visualize=False,
            sim_time=sim_time,
            path_type=path_type,
            soft_terrain=True,
            debug=False,
            nn_scale=1.0,
            nn_sign=1,
            terrain_config=terrain_config,
            v_target=speed,
            kappa_mode=kappa_mode,
        )
    except Exception as e:
        rms = None
    finally:
        if quiet:
            sys.stdout.close()
            sys.stdout = old_stdout
    
    return rms


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Soil parameter sweep benchmark')
    parser.add_argument('--n-soils', type=int, default=8, help='Number of random soils')
    parser.add_argument('--path', type=str, default='lane_change',
                        choices=['lane_change', 'double_lane_change', 'sinusoidal'])
    parser.add_argument('--time', type=float, default=12.0, help='Sim time per run (s)')
    parser.add_argument('--speed', type=float, default=5.0, help='Target speed (m/s)')
    parser.add_argument('--seed', type=int, default=42, help='Random seed')
    parser.add_argument('--quiet', action='store_true', default=True, help='Suppress per-run output')
    parser.add_argument('--verbose', action='store_true', help='Show per-run output')
    parser.add_argument('--kappa', type=str, default='zero', choices=['zero', 'approx'],
                        help='Longitudinal slip mode: zero (pure lateral) or approx (ax-based)')
    args = parser.parse_args()

    quiet = not args.verbose
    rng = np.random.default_rng(args.seed)
    
    # Generate soil configs
    soils = [sample_terrain(rng) for _ in range(args.n_soils)]
    
    print("=" * 90)
    print(f"SOIL PARAMETER SWEEP: Linear vs NN MPC")
    print(f"  Path: {args.path}, Speed: {args.speed} m/s, Sim time: {args.time}s")
    print(f"  Soils: {args.n_soils} (seed={args.seed}), kappa_mode: {args.kappa}")
    print("=" * 90)
    
    results = []
    total_start = time_mod.time()
    
    for i, soil in enumerate(soils):
        label = terrain_label(soil)
        print(f"\n--- Soil {i+1}/{args.n_soils}: {label} ---")
        
        # Run linear
        t0 = time_mod.time()
        print(f"  Running LINEAR...", end='', flush=True)
        rms_linear = run_one('linear', soil, args.path, args.time, args.speed, args.kappa, quiet)
        t_linear = time_mod.time() - t0
        print(f" RMS={rms_linear:.3f}m ({t_linear:.0f}s)" if rms_linear else f" FAILED ({t_linear:.0f}s)")
        
        # Run NN
        t0 = time_mod.time()
        print(f"  Running NN...    ", end='', flush=True)
        rms_nn = run_one('nn', soil, args.path, args.time, args.speed, args.kappa, quiet)
        t_nn = time_mod.time() - t0
        print(f" RMS={rms_nn:.3f}m ({t_nn:.0f}s)" if rms_nn else f" FAILED ({t_nn:.0f}s)")
        
        results.append({
            'soil_idx': i,
            'soil': soil,
            'label': label,
            'rms_linear': rms_linear,
            'rms_nn': rms_nn,
            'time_linear': t_linear,
            'time_nn': t_nn,
        })
    
    total_elapsed = time_mod.time() - total_start
    
    # Summary table
    print("\n" + "=" * 90)
    print("RESULTS SUMMARY")
    print("=" * 90)
    print(f"{'Soil':<45} {'Linear':>10} {'NN':>10} {'Winner':>10} {'Improv':>10}")
    print("-" * 90)
    
    nn_wins = 0
    linear_wins = 0
    ties = 0
    improvements = []
    
    for r in results:
        rl = r['rms_linear']
        rn = r['rms_nn']
        
        if rl is None or rn is None:
            winner = 'FAIL'
            improv_str = '—'
        elif abs(rl - rn) < 0.005:
            winner = 'TIE'
            improv_str = '—'
            ties += 1
        elif rn < rl:
            winner = 'NN'
            improv = (rl - rn) / rl * 100
            improv_str = f'+{improv:.0f}%'
            nn_wins += 1
            improvements.append(improv)
        else:
            winner = 'LINEAR'
            improv = (rn - rl) / rn * 100
            improv_str = f'-{improv:.0f}%'
            linear_wins += 1
            improvements.append(-improv)
        
        rl_str = f'{rl:.3f}m' if rl is not None else 'FAIL'
        rn_str = f'{rn:.3f}m' if rn is not None else 'FAIL'
        print(f"{r['label']:<45} {rl_str:>10} {rn_str:>10} {winner:>10} {improv_str:>10}")
    
    # Aggregate statistics
    valid = [r for r in results if r['rms_linear'] is not None and r['rms_nn'] is not None]
    if valid:
        avg_linear = np.mean([r['rms_linear'] for r in valid])
        avg_nn = np.mean([r['rms_nn'] for r in valid])
        med_linear = np.median([r['rms_linear'] for r in valid])
        med_nn = np.median([r['rms_nn'] for r in valid])
        
        print("-" * 90)
        print(f"{'MEAN':<45} {avg_linear:>9.3f}m {avg_nn:>9.3f}m")
        print(f"{'MEDIAN':<45} {med_linear:>9.3f}m {med_nn:>9.3f}m")
        print(f"\nNN wins: {nn_wins}, Linear wins: {linear_wins}, Ties: {ties}")
        if improvements:
            print(f"Mean improvement when NN wins: {np.mean([x for x in improvements if x > 0]):.1f}%"
                  if any(x > 0 for x in improvements) else "")
        print(f"Total time: {total_elapsed:.0f}s ({total_elapsed/60:.1f} min)")
    
    # Save results
    out_path = Path(__file__).parent / f'benchmark_{args.path}_{datetime.now():%Y%m%d_%H%M%S}.json'
    save_data = {
        'config': {'path': args.path, 'speed': args.speed, 'sim_time': args.time,
                    'seed': args.seed, 'n_soils': args.n_soils},
        'results': [{k: v for k, v in r.items() if k != 'soil'} | {'soil': r['soil']}
                    for r in results],
        'summary': {
            'nn_wins': nn_wins, 'linear_wins': linear_wins, 'ties': ties,
            'mean_rms_linear': float(avg_linear) if valid else None,
            'mean_rms_nn': float(avg_nn) if valid else None,
        }
    }
    with open(out_path, 'w') as f:
        json.dump(save_data, f, indent=2, default=str)
    print(f"\nResults saved to: {out_path}")


if __name__ == '__main__':
    main()
