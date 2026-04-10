#!/usr/bin/env python3
"""
Evaluate and compare all trained NN models.

Loads each model's test_metrics.json, and optionally benchmarks CasADi
symbolic evaluation speed (simulating what the MPC solver does).

Usage:
  python evaluate_models.py [--benchmark]
"""

import json
import sys
import time
import argparse
from pathlib import Path

import numpy as np
import torch

NN_MODELS_DIR = Path(__file__).parent.parent / 'nn_models'


def collect_metrics():
    """Gather test metrics from all model directories.

    Handles two metric file formats:
      - Unified (test_metrics.json): {train: {r2, rmse}, val: ..., test: ...}
      - Legacy rate (metrics.json):  {r2_Fx, r2_Fy, mae_Fx, mae_Fy, ...}
    """
    results = []
    for model_dir in sorted(NN_MODELS_DIR.iterdir()):
        if not model_dir.is_dir():
            continue

        # Try unified format first, then legacy
        metrics_file = model_dir / 'test_metrics.json'
        legacy_file = model_dir / 'metrics.json'
        m = None
        if metrics_file.exists():
            with open(metrics_file) as f:
                m = json.load(f)
        elif legacy_file.exists():
            with open(legacy_file) as f:
                m = json.load(f)
        else:
            continue

        # Extract test R² and RMSE — three known formats:
        #  1) Unified list:  test.r2 = [fx, fy],  test.rmse = [fx, fy]
        #  2) Flat keys:     test.r2_fx, test.r2_fy, test.rmse_fx, test.rmse_fy
        #  3) Legacy top-level: r2_Fx, r2_Fy  (old rate metrics.json)
        test = m.get('test', {})
        if test and 'r2' in test and isinstance(test['r2'], list):
            r2 = test.get('r2', [None, None])
            rmse = test.get('rmse', [None, None])
        elif test and 'r2_fx' in test:
            r2 = [test.get('r2_fx'), test.get('r2_fy')]
            rmse = [test.get('rmse_fx'), test.get('rmse_fy')]
        else:
            r2 = [m.get('r2_Fx'), m.get('r2_Fy')]
            rmse = [None, None]

        arch = m.get('architecture', {})

        # Fallback: read checkpoint metadata if not in metrics file
        ckpt_path = model_dir / 'best_terrain_nn.pt'
        ckpt_meta = {}
        if ckpt_path.exists():
            try:
                ckpt = torch.load(ckpt_path, map_location='cpu', weights_only=False)
                if isinstance(ckpt, dict):
                    ckpt_meta = ckpt
            except Exception:
                pass

        n_params = (arch.get('n_params') or arch.get('total_params')
                    or m.get('n_params') or ckpt_meta.get('n_params'))
        arch_type = (arch.get('type') or arch.get('architecture_type')
                     or ckpt_meta.get('architecture_type') or 'mlp')
        hidden = (arch.get('hidden_sizes') or arch.get('hidden_dim')
                  or m.get('hidden_sizes') or ckpt_meta.get('hidden_sizes')
                  or ckpt_meta.get('hidden_dim'))
        n_blocks = arch.get('n_blocks', ckpt_meta.get('n_blocks', '-'))
        temporal_K = arch.get('temporal_K', ckpt_meta.get('temporal_K', 1))
        rate_aug = arch.get('rate_augmented', ckpt_meta.get('rate_augmented', False))

        results.append({
            'name': model_dir.name,
            'arch': arch_type,
            'hidden': hidden,
            'n_blocks': n_blocks,
            'K': temporal_K,
            'rate_augmented': rate_aug,
            'n_params': n_params,
            'r2_fx': r2[0],
            'r2_fy': r2[1],
            'rmse_fx': rmse[0] if rmse else None,
            'rmse_fy': rmse[1] if rmse else None,
        })
    return results


def benchmark_casadi(results):
    """Benchmark CasADi symbolic evaluation for each model."""
    try:
        import casadi as ca
    except ImportError:
        print("CasADi not available — skipping timing benchmark")
        return results

    sys.path.insert(0, str(Path(__file__).parent.parent / 'simulation'))
    sys.path.insert(0, str(Path(__file__).parent.parent / 'nn_training'))

    from nn_tire_model import load_nn_tire_model

    # Dummy terrain params (typical clay)
    terrain = dict(Kphi=692000, Kc=13200, n=0.7, c=4140, phi=22.5, k=0.015)

    for r in results:
        model_dir = NN_MODELS_DIR / r['name']
        model_path = model_dir / 'best_terrain_nn.pt'
        scaler_path = model_dir / 'scalers.pkl'
        if not model_path.exists() or not scaler_path.exists():
            r['mpc_mean_ms'] = None
            continue

        try:
            nn_ca = load_nn_tire_model(model_dir, terrain)

            # Warm up
            for _ in range(5):
                nn_ca.predict_numeric(0.1, 3000, 5.0, 0.0)

            # Benchmark: 200 scalar evaluations
            times = []
            for _ in range(200):
                alpha = np.random.uniform(-0.6, 0.6)
                Fz = np.random.uniform(1500, 7500)
                u = np.random.uniform(2, 10)
                kap = np.random.uniform(-0.5, 0.5)
                t0 = time.perf_counter()
                nn_ca.predict_numeric(alpha, Fz, u, kap)
                times.append((time.perf_counter() - t0) * 1000)

            r['mpc_mean_ms'] = np.mean(times)
            r['mpc_p95_ms'] = np.percentile(times, 95)
        except Exception as e:
            print(f"  WARNING: benchmark failed for {r['name']}: {e}")
            r['mpc_mean_ms'] = None
            r['mpc_p95_ms'] = None

    return results


def print_table(results):
    """Print a formatted comparison table."""
    # Sort by R² Fx descending
    results.sort(key=lambda x: (x.get('r2_fx') or 0), reverse=True)

    print("\n" + "=" * 120)
    print("MODEL COMPARISON — Terrain NN Force Prediction")
    print("=" * 130)
    fmt = "{:<35s} {:>8s} {:>6s} {:>3s} {:>5s} {:>7s} {:>8s} {:>8s} {:>9s} {:>9s} {:>8s}"
    print(fmt.format("Model", "Arch", "Hidden", "K", "Rate",  "Params",
                      "R²(Fx)", "R²(Fy)", "RMSE(Fx)", "RMSE(Fy)", "CasADi"))
    print("-" * 130)

    for r in results:
        h = str(r['hidden']) if r['hidden'] else '?'
        blk = f"x{r['n_blocks']}" if r['n_blocks'] != '-' else ''
        arch_str = f"{r['arch']}{blk}" if r['arch'] == 'resnet' else r['arch']
        rate_str = 'yes' if r.get('rate_augmented') else '-'
        params = str(r['n_params']) if r['n_params'] else '?'
        r2fx = f"{r['r2_fx']:.4f}" if r['r2_fx'] else '?'
        r2fy = f"{r['r2_fy']:.4f}" if r['r2_fy'] else '?'
        rmfx = f"{r['rmse_fx']:.0f}N" if r['rmse_fx'] else '?'
        rmfy = f"{r['rmse_fy']:.0f}N" if r['rmse_fy'] else '?'
        timing = f"{r['mpc_mean_ms']:.2f}ms" if r.get('mpc_mean_ms') else '-'
        print(fmt.format(r['name'], arch_str, h, str(r['K']), rate_str,
                          params, r2fx, r2fy, rmfx, rmfy, timing))

    print("=" * 130)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--benchmark', action='store_true',
                   help='Run CasADi evaluation timing benchmark')
    args = p.parse_args()

    results = collect_metrics()
    if not results:
        print("No models found in", NN_MODELS_DIR)
        return

    if args.benchmark:
        results = benchmark_casadi(results)

    print_table(results)

    # Save results
    out_path = NN_MODELS_DIR / 'comparison_results.json'
    with open(out_path, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\nResults saved to {out_path}")


if __name__ == '__main__':
    main()
