#!/usr/bin/env python3
"""
Benchmark NN vs Pacejka tire models across terrain presets and path types.

Runs both models on all combinations of terrains and paths, generating:
- Error bar plots comparing RMS tracking error
- Summary table with statistics and % improvement

Supports parallel execution across multiple CPU cores.
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
import time
import sys
import multiprocessing as mp
from functools import partial
from pathlib import Path
from datetime import datetime

# Import run_simulation from the main demo script
from dallas_chrono_demo import (run_simulation, TERRAIN_PRESETS, 
                                 check_sinusoidal_feasibility, suggest_feasible_sine_params)


# Path configurations: (name, path_type, sine_amplitude, sine_wavelength)
# NOTE: For sinusoids, max curvature κ = A*(2π/λ)², min turning radius R = 1/κ
# HMMWV min turning radius ≈ 6m, so wavelength must be ≥ 2π*sqrt(A*6) for feasibility
PATH_CONFIGS = [
    ('lane_change', 'lane_change', 2.0, 30.0),
    ('double_lane_change', 'double_lane_change', 2.0, 30.0),
    ('sine_gentle', 'sinusoidal', 1.5, 40.0),    # R=27m, very easy
    ('sine_medium', 'sinusoidal', 2.0, 30.0),    # R=11m, moderate
    ('sine_tight', 'sinusoidal', 2.0, 24.0),     # R=7.2m, challenging but feasible (was 2.5/20 = infeasible!)
]

# Base terrain types (excludes soft/hard aliases)
TERRAIN_TYPES = ['sand', 'clay', 'dirt']


def validate_path_configs():
    """
    Validate all sinusoidal paths are physically feasible for the HMMWV.
    Prints a summary table and returns True if all paths are feasible.
    """
    print("\n  Path Feasibility Check:")
    print("  " + "-" * 55)
    print(f"  {'Path':<20} {'Amp':>6} {'λ':>6} {'R_req':>8} {'Status':>12}")
    print("  " + "-" * 55)
    
    all_feasible = True
    for name, path_type, amp, wl in PATH_CONFIGS:
        if path_type == 'sinusoidal':
            feasible, req_R, ach_R, margin = check_sinusoidal_feasibility(amp, wl)
            status = "✓ OK" if feasible else f"✗ need λ≥{suggest_feasible_sine_params(amp):.0f}m"
            if not feasible:
                all_feasible = False
            print(f"  {name:<20} {amp:>5.1f}m {wl:>5.0f}m {req_R:>7.1f}m {status:>12}")
        else:
            print(f"  {name:<20} {'n/a':>6} {'n/a':>6} {'n/a':>8} {'✓ OK':>12}")
    
    print("  " + "-" * 55)
    print(f"  Vehicle min turning radius: ~6.0m (HMMWV, δ_max=0.5 rad)")
    
    if not all_feasible:
        print("\n  ⚠ Some paths are INFEASIBLE - results will show poor tracking for both models!")
    print()
    
    return all_feasible


def run_single_simulation(args_tuple):
    """
    Worker function for parallel execution.
    Must be a top-level function for multiprocessing to pickle it.
    
    Args:
        args_tuple: (model, terrain, path_name, path_type, sine_amp, sine_wl, 
                     sim_time, v_target, run_idx, measurement_noise, use_closest_point,
                     rms_time_start, rms_time_end)
    
    Returns:
        (terrain, path_name, model, run_idx, rms_error or None)
    """
    import sys
    import os
    
    (model, terrain, path_name, path_type, sine_amp, sine_wl, 
     sim_time, v_target, run_idx, measurement_noise, use_closest_point,
     rms_time_start, rms_time_end) = args_tuple
    
    # Seed RNG uniquely per worker using run_idx and hash of job params
    # This ensures different noise realizations for each run
    seed = hash((terrain, path_name, model, run_idx)) % (2**31)
    np.random.seed(seed)
    
    try:
        # Suppress stdout from individual simulations in parallel mode
        devnull = open(os.devnull, 'w')
        old_stdout = sys.stdout
        sys.stdout = devnull
        
        try:
            rms = run_simulation(
                controller_type=model,
                visualize=False,  # No visualization in parallel mode
                sim_time=sim_time,
                path_type=path_type,
                terrain_preset=terrain,
                v_target=v_target,
                sine_amplitude=sine_amp,
                sine_wavelength=sine_wl,
                measurement_noise=measurement_noise,
                use_closest_point=use_closest_point,
                rms_time_start=rms_time_start,
                rms_time_end=rms_time_end,
                debug=False
            )
        finally:
            sys.stdout = old_stdout
            devnull.close()
        
        return (terrain, path_name, model, run_idx, rms)
    except Exception as e:
        return (terrain, path_name, model, run_idx, None)


def run_benchmark_parallel(n_runs=20, sim_time=30.0, v_target=5.0, n_workers=None,
                           measurement_noise=None, use_closest_point=True,
                           rms_time_start=5.0, rms_time_end=25.0):
    """
    Run benchmark in parallel using multiprocessing.
    
    Args:
        n_runs: Number of runs per (model, terrain, path) combination
        sim_time: Simulation time per run (s)
        v_target: Target speed (m/s)
        n_workers: Number of parallel workers (default: CPU count - 1)
        measurement_noise: Dict with noise std devs or True for defaults
        use_closest_point: If True, use closest-point path re-indexing for sinusoidal
        rms_time_start: Start time for RMS calculation (excludes startup transients)
        rms_time_end: End time for RMS calculation (excludes final transients)
    
    Returns:
        results: Dict[(terrain, path_name)][model] -> list of RMS errors
    """
    terrains = TERRAIN_TYPES
    models = ['linear', 'nn']
    
    # Default to leaving one core free for system
    if n_workers is None:
        n_workers = max(1, mp.cpu_count() - 1)
    
    # Build list of all jobs
    jobs = []
    for terrain in terrains:
        for path_name, path_type, sine_amp, sine_wl in PATH_CONFIGS:
            for model in models:
                for run_idx in range(n_runs):
                    jobs.append((
                        model, terrain, path_name, path_type, sine_amp, sine_wl,
                        sim_time, v_target, run_idx, measurement_noise, use_closest_point,
                        rms_time_start, rms_time_end
                    ))
    
    total_jobs = len(jobs)
    
    noise_str = "ON" if measurement_noise else "OFF"
    reindex_str = "ON" if use_closest_point else "OFF"
    rms_window_str = f"[{rms_time_start:.0f}s, {rms_time_end:.0f}s]" if rms_time_start else "full"
    print(f"\n{'='*70}")
    print(f"TIRE MODEL BENCHMARK (Parallel)")
    print(f"{'='*70}")
    print(f"  Terrains: {', '.join(terrains)}")
    print(f"  Paths: {', '.join([p[0] for p in PATH_CONFIGS])}")
    print(f"  Models: Pacejka (linear), NN")
    print(f"  Runs per combination: {n_runs}")
    print(f"  Speed: {v_target} m/s, Time: {sim_time}s")
    print(f"  RMS window: {rms_window_str}")
    print(f"  Measurement noise: {noise_str}")
    print(f"  Path re-indexing: {reindex_str}")
    print(f"  Total simulations: {total_jobs}")
    print(f"  Workers: {n_workers}")
    print(f"{'='*70}\n")
    
    t_start = time.time()
    
    # Initialize results structure
    results = {}
    for terrain in terrains:
        for path_name, _, _, _ in PATH_CONFIGS:
            results[(terrain, path_name)] = {'linear': [], 'nn': []}
    
    # Run in parallel
    completed = 0
    last_pct = 0
    
    print("Progress: ", end='', flush=True)
    
    with mp.Pool(processes=n_workers) as pool:
        for result in pool.imap_unordered(run_single_simulation, jobs):
            terrain, path_name, model, run_idx, rms = result
            
            if rms is not None:
                results[(terrain, path_name)][model].append(rms)
            
            completed += 1
            pct = int(100 * completed / total_jobs)
            if pct >= last_pct + 5:
                print(f"{pct}%", end=' ', flush=True)
                last_pct = pct
    
    print("Done!")
    
    elapsed = time.time() - t_start
    print(f"\n[Benchmark completed in {elapsed/60:.1f} minutes]")
    print(f"  Effective parallelism: {total_jobs * sim_time / elapsed:.1f}x")
    
    return results


def run_benchmark_sequential(n_runs=20, sim_time=30.0, v_target=5.0, visualize=False,
                              measurement_noise=None, use_closest_point=True,
                              rms_time_start=5.0, rms_time_end=25.0):
    """
    Run benchmark sequentially (original implementation).
    """
    terrains = TERRAIN_TYPES
    models = ['linear', 'nn']
    
    results = {}
    for terrain in terrains:
        for path_name, _, _, _ in PATH_CONFIGS:
            results[(terrain, path_name)] = {m: [] for m in models}
    
    total_runs = len(terrains) * len(PATH_CONFIGS) * len(models) * n_runs
    
    noise_str = "ON" if measurement_noise else "OFF"
    reindex_str = "ON" if use_closest_point else "OFF"
    rms_window_str = f"[{rms_time_start:.0f}s, {rms_time_end:.0f}s]" if rms_time_start else "full"
    print(f"\n{'='*70}")
    print(f"TIRE MODEL BENCHMARK (Sequential)")
    print(f"{'='*70}")
    print(f"  Terrains: {', '.join(terrains)}")
    print(f"  Paths: {', '.join([p[0] for p in PATH_CONFIGS])}")
    print(f"  Models: Pacejka (linear), NN")
    print(f"  Runs per combination: {n_runs}")
    print(f"  Speed: {v_target} m/s, Time: {sim_time}s")
    print(f"  RMS window: {rms_window_str}")
    print(f"  Measurement noise: {noise_str}")
    print(f"  Path re-indexing: {reindex_str}")
    print(f"  Total runs: {total_runs}")
    print(f"{'='*70}\n")
    
    t_start = time.time()
    
    for terrain in terrains:
        print(f"\n{'='*60}")
        print(f"TERRAIN: {terrain.upper()}")
        print(f"{'='*60}")
        
        for path_name, path_type, sine_amp, sine_wl in PATH_CONFIGS:
            print(f"\n  [PATH: {path_name}]")
            
            for model in models:
                model_name = 'Pacejka' if model == 'linear' else 'NN'
                print(f"    {model_name}: ", end='', flush=True)
                
                run_errors = []
                for i in range(n_runs):
                    try:
                        rms = run_simulation(
                            controller_type=model,
                            visualize=visualize,
                            sim_time=sim_time,
                            path_type=path_type,
                            terrain_preset=terrain,
                            v_target=v_target,
                            sine_amplitude=sine_amp,
                            sine_wavelength=sine_wl,
                            measurement_noise=measurement_noise,
                            use_closest_point=use_closest_point,
                            rms_time_start=rms_time_start,
                            rms_time_end=rms_time_end,
                            debug=False
                        )
                        if rms is not None:
                            run_errors.append(rms)
                            print(f".", end='', flush=True)
                    except Exception as e:
                        print(f"X", end='', flush=True)
                
                results[(terrain, path_name)][model] = run_errors
                
                if run_errors:
                    mean_rms = np.mean(run_errors)
                    std_rms = np.std(run_errors)
                    print(f" {mean_rms:.4f} ± {std_rms:.4f} m")
                else:
                    print(f" FAILED")
    
    elapsed = time.time() - t_start
    print(f"\n[Benchmark completed in {elapsed/60:.1f} minutes]")
    
    return results


def compute_statistics(results):
    """Compute summary statistics for each (terrain, path, model) combination."""
    stats = {}
    
    for key, models in results.items():
        stats[key] = {}
        for model, errors in models.items():
            if errors:
                stats[key][model] = {
                    'mean': np.mean(errors),
                    'std': np.std(errors),
                    'min': np.min(errors),
                    'max': np.max(errors),
                    'median': np.median(errors),
                    'n': len(errors)
                }
            else:
                stats[key][model] = None
    
    return stats


def compute_improvement(stats):
    """Compute % improvement of NN over Pacejka for each combination."""
    improvements = {}
    
    for key, models in stats.items():
        pacejka = models.get('linear')
        nn = models.get('nn')
        
        if pacejka and nn:
            improvement = (pacejka['mean'] - nn['mean']) / pacejka['mean'] * 100
            improvements[key] = {
                'improvement_pct': improvement,
                'pacejka_mean': pacejka['mean'],
                'nn_mean': nn['mean'],
                'pacejka_std': pacejka['std'],
                'nn_std': nn['std']
            }
        else:
            improvements[key] = None
    
    return improvements


def print_summary_table(stats, improvements):
    """Print formatted summary table."""
    print(f"\n{'='*100}")
    print("SUMMARY TABLE: RMS Tracking Error (meters)")
    print(f"{'='*100}")
    print(f"{'Terrain':<10} | {'Path':<18} | {'Pacejka (mean±std)':<20} | {'NN (mean±std)':<20} | {'Improvement':<12}")
    print(f"{'-'*100}")
    
    # Group by terrain for readability
    terrains = TERRAIN_TYPES
    path_names = [p[0] for p in PATH_CONFIGS]
    
    for terrain in terrains:
        for i, path_name in enumerate(path_names):
            key = (terrain, path_name)
            pacejka = stats[key].get('linear')
            nn = stats[key].get('nn')
            imp = improvements.get(key)
            
            if pacejka:
                pacejka_str = f"{pacejka['mean']:.4f} ± {pacejka['std']:.4f}"
            else:
                pacejka_str = "N/A"
            
            if nn:
                nn_str = f"{nn['mean']:.4f} ± {nn['std']:.4f}"
            else:
                nn_str = "N/A"
            
            if imp:
                if imp['improvement_pct'] > 0:
                    imp_str = f"+{imp['improvement_pct']:.1f}%"
                else:
                    imp_str = f"{imp['improvement_pct']:.1f}%"
            else:
                imp_str = "N/A"
            
            # Only show terrain name on first row of group
            terrain_str = terrain if i == 0 else ""
            print(f"{terrain_str:<10} | {path_name:<18} | {pacejka_str:<20} | {nn_str:<20} | {imp_str:<12}")
        
        print(f"{'-'*100}")
    
    # Overall summaries
    print(f"\n{'='*70}")
    print("AGGREGATE STATISTICS")
    print(f"{'='*70}")
    
    # By terrain
    print("\nBy Terrain:")
    for terrain in terrains:
        terrain_imps = [improvements[(terrain, p[0])]['improvement_pct'] 
                        for p in PATH_CONFIGS if improvements.get((terrain, p[0]))]
        if terrain_imps:
            print(f"  {terrain}: {np.mean(terrain_imps):+.1f}% avg improvement")
    
    # By path
    print("\nBy Path Type:")
    for path_name, _, _, _ in PATH_CONFIGS:
        path_imps = [improvements[(t, path_name)]['improvement_pct'] 
                     for t in terrains if improvements.get((t, path_name))]
        if path_imps:
            print(f"  {path_name}: {np.mean(path_imps):+.1f}% avg improvement")
    
    # Overall
    all_imps = [imp['improvement_pct'] for imp in improvements.values() if imp]
    if all_imps:
        print(f"\nOverall Average: {np.mean(all_imps):+.1f}%")
        print(f"  (Positive = NN better, Negative = Pacejka better)")


def plot_error_bars(results, stats, improvements, output_path='benchmark_results.png'):
    """Create comprehensive error bar plots."""
    terrains = TERRAIN_TYPES
    path_names = [p[0] for p in PATH_CONFIGS]
    n_terrains = len(terrains)
    n_paths = len(path_names)
    
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    
    # ===== Plot 1: Grouped by terrain (aggregate over paths) =====
    ax1 = axes[0, 0]
    terrain_pacejka_means = []
    terrain_pacejka_stds = []
    terrain_nn_means = []
    terrain_nn_stds = []
    
    for terrain in terrains:
        p_means = [stats[(terrain, p[0])]['linear']['mean'] 
                   for p in PATH_CONFIGS if stats.get((terrain, p[0]), {}).get('linear')]
        n_means = [stats[(terrain, p[0])]['nn']['mean'] 
                   for p in PATH_CONFIGS if stats.get((terrain, p[0]), {}).get('nn')]
        
        terrain_pacejka_means.append(np.mean(p_means) if p_means else 0)
        terrain_nn_means.append(np.mean(n_means) if n_means else 0)
        terrain_pacejka_stds.append(np.std(p_means) if p_means else 0)
        terrain_nn_stds.append(np.std(n_means) if n_means else 0)
    
    x = np.arange(n_terrains)
    width = 0.35
    
    ax1.bar(x - width/2, terrain_pacejka_means, width, yerr=terrain_pacejka_stds,
            label='Pacejka', color='#2196F3', capsize=5, alpha=0.8)
    ax1.bar(x + width/2, terrain_nn_means, width, yerr=terrain_nn_stds,
            label='NN', color='#4CAF50', capsize=5, alpha=0.8)
    
    ax1.set_xlabel('Terrain Type', fontsize=12)
    ax1.set_ylabel('Mean RMS Error (m)', fontsize=12)
    ax1.set_title('By Terrain (averaged over paths)', fontsize=14)
    ax1.set_xticks(x)
    ax1.set_xticklabels([t.capitalize() for t in terrains])
    ax1.legend()
    ax1.grid(axis='y', alpha=0.3)
    
    # ===== Plot 2: Grouped by path type (aggregate over terrains) =====
    ax2 = axes[0, 1]
    path_pacejka_means = []
    path_pacejka_stds = []
    path_nn_means = []
    path_nn_stds = []
    
    for path_name, _, _, _ in PATH_CONFIGS:
        p_means = [stats[(t, path_name)]['linear']['mean'] 
                   for t in terrains if stats.get((t, path_name), {}).get('linear')]
        n_means = [stats[(t, path_name)]['nn']['mean'] 
                   for t in terrains if stats.get((t, path_name), {}).get('nn')]
        
        path_pacejka_means.append(np.mean(p_means) if p_means else 0)
        path_nn_means.append(np.mean(n_means) if n_means else 0)
        path_pacejka_stds.append(np.std(p_means) if p_means else 0)
        path_nn_stds.append(np.std(n_means) if n_means else 0)
    
    x2 = np.arange(n_paths)
    
    ax2.bar(x2 - width/2, path_pacejka_means, width, yerr=path_pacejka_stds,
            label='Pacejka', color='#2196F3', capsize=5, alpha=0.8)
    ax2.bar(x2 + width/2, path_nn_means, width, yerr=path_nn_stds,
            label='NN', color='#4CAF50', capsize=5, alpha=0.8)
    
    ax2.set_xlabel('Path Type', fontsize=12)
    ax2.set_ylabel('Mean RMS Error (m)', fontsize=12)
    ax2.set_title('By Path (averaged over terrains)', fontsize=14)
    ax2.set_xticks(x2)
    ax2.set_xticklabels([p[0].replace('_', '\n') for p in PATH_CONFIGS], fontsize=9)
    ax2.legend()
    ax2.grid(axis='y', alpha=0.3)
    
    # ===== Plot 3: Improvement by terrain =====
    ax3 = axes[1, 0]
    terrain_improvements = []
    for terrain in terrains:
        imps = [improvements[(terrain, p[0])]['improvement_pct'] 
                for p in PATH_CONFIGS if improvements.get((terrain, p[0]))]
        terrain_improvements.append(np.mean(imps) if imps else 0)
    
    colors = ['#4CAF50' if imp > 0 else '#F44336' for imp in terrain_improvements]
    bars = ax3.bar(x, terrain_improvements, color=colors, alpha=0.8)
    ax3.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
    ax3.set_xlabel('Terrain Type', fontsize=12)
    ax3.set_ylabel('NN Improvement (%)', fontsize=12)
    ax3.set_title('NN vs Pacejka Improvement by Terrain', fontsize=14)
    ax3.set_xticks(x)
    ax3.set_xticklabels([t.capitalize() for t in terrains])
    ax3.grid(axis='y', alpha=0.3)
    
    for bar, imp in zip(bars, terrain_improvements):
        va = 'bottom' if imp >= 0 else 'top'
        offset = 3 if imp >= 0 else -3
        ax3.annotate(f'{imp:+.1f}%', xy=(bar.get_x() + bar.get_width()/2, bar.get_height()),
                    xytext=(0, offset), textcoords='offset points', ha='center', va=va, fontsize=10)
    
    # ===== Plot 4: Improvement by path =====
    ax4 = axes[1, 1]
    path_improvements = []
    for path_name, _, _, _ in PATH_CONFIGS:
        imps = [improvements[(t, path_name)]['improvement_pct'] 
                for t in terrains if improvements.get((t, path_name))]
        path_improvements.append(np.mean(imps) if imps else 0)
    
    colors = ['#4CAF50' if imp > 0 else '#F44336' for imp in path_improvements]
    bars = ax4.bar(x2, path_improvements, color=colors, alpha=0.8)
    ax4.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
    ax4.set_xlabel('Path Type', fontsize=12)
    ax4.set_ylabel('NN Improvement (%)', fontsize=12)
    ax4.set_title('NN vs Pacejka Improvement by Path', fontsize=14)
    ax4.set_xticks(x2)
    ax4.set_xticklabels([p[0].replace('_', '\n') for p in PATH_CONFIGS], fontsize=9)
    ax4.grid(axis='y', alpha=0.3)
    
    for bar, imp in zip(bars, path_improvements):
        va = 'bottom' if imp >= 0 else 'top'
        offset = 3 if imp >= 0 else -3
        ax4.annotate(f'{imp:+.1f}%', xy=(bar.get_x() + bar.get_width()/2, bar.get_height()),
                    xytext=(0, offset), textcoords='offset points', ha='center', va=va, fontsize=10)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\nPlot saved to: {output_path}")
    
    # Create detailed heatmap
    plot_heatmap(stats, improvements, output_path.replace('.png', '_heatmap.png'))
    
    try:
        plt.show()
    except:
        pass


def plot_heatmap(stats, improvements, output_path):
    """Create heatmap of improvement percentages."""
    terrains = TERRAIN_TYPES
    path_names = [p[0] for p in PATH_CONFIGS]
    
    # Build improvement matrix
    imp_matrix = np.zeros((len(terrains), len(path_names)))
    for i, terrain in enumerate(terrains):
        for j, path_name in enumerate(path_names):
            imp = improvements.get((terrain, path_name))
            imp_matrix[i, j] = imp['improvement_pct'] if imp else 0
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Custom colormap: red for negative, green for positive
    from matplotlib.colors import TwoSlopeNorm
    vmax = max(abs(imp_matrix.min()), abs(imp_matrix.max()), 1)
    norm = TwoSlopeNorm(vmin=-vmax, vcenter=0, vmax=vmax)
    
    im = ax.imshow(imp_matrix, cmap='RdYlGn', norm=norm, aspect='auto')
    
    # Labels
    ax.set_xticks(np.arange(len(path_names)))
    ax.set_yticks(np.arange(len(terrains)))
    ax.set_xticklabels([p.replace('_', '\n') for p in path_names])
    ax.set_yticklabels([t.capitalize() for t in terrains])
    
    # Annotate cells
    for i in range(len(terrains)):
        for j in range(len(path_names)):
            val = imp_matrix[i, j]
            color = 'white' if abs(val) > vmax * 0.5 else 'black'
            ax.text(j, i, f'{val:+.1f}%', ha='center', va='center', color=color, fontsize=10)
    
    ax.set_xlabel('Path Type', fontsize=12)
    ax.set_ylabel('Terrain', fontsize=12)
    ax.set_title('NN Improvement over Pacejka (%)\n(Green = NN better, Red = Pacejka better)', fontsize=14)
    
    plt.colorbar(im, ax=ax, label='Improvement (%)')
    plt.tight_layout()
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"Heatmap saved to: {output_path}")


def save_raw_data(results, output_path='benchmark_raw_data.npz'):
    """Save raw benchmark data for later analysis."""
    data = {}
    for (terrain, path_name), models in results.items():
        for model, errors in models.items():
            key = f"{terrain}_{path_name}_{model}"
            data[key] = np.array(errors) if errors else np.array([])
    
    np.savez(output_path, **data)
    print(f"Raw data saved to: {output_path}")


def main():
    parser = argparse.ArgumentParser(description="Benchmark NN vs Pacejka tire models")
    parser.add_argument('--runs', type=int, default=20, 
                        help='Number of runs per combination')
    parser.add_argument('--time', type=float, default=30.0,
                        help='Simulation time per run (s, default: 30)')
    parser.add_argument('--rms-start', type=float, default=5.0,
                        help='Start time for RMS calculation (s, default: 5)')
    parser.add_argument('--rms-end', type=float, default=25.0,
                        help='End time for RMS calculation (s, default: 25)')
    parser.add_argument('--speed', type=float, default=5.0,
                        help='Target speed (m/s)')
    parser.add_argument('--workers', '-j', type=int, default=None,
                        help='Number of parallel workers (default: CPU count - 1)')
    parser.add_argument('--sequential', action='store_true',
                        help='Run sequentially instead of parallel (slower but allows --vis)')
    parser.add_argument('--vis', action='store_true',
                        help='Enable visualization (requires --sequential)')
    parser.add_argument('--no-noise', action='store_true',
                        help='Disable measurement noise (on by default: x,y:1.2m, psi,omega:0.0175rad, u,v:0.25m/s)')
    parser.add_argument('--no-reindex', action='store_true',
                        help='Disable closest-point path re-indexing (test old x-projection method)')
    parser.add_argument('--output', type=str, default='benchmark_results.png',
                        help='Output plot filename')
    parser.add_argument('--quick', action='store_true',
                        help='Quick test: 3 runs, 15s sim time, [3-12]s RMS window')
    
    args = parser.parse_args()
    
    if args.quick:
        n_runs = 3
        sim_time = 15.0
        rms_time_start = 3.0
        rms_time_end = 12.0
        print("[QUICK MODE: 3 runs, 15s sim time, RMS [3-12]s]")
    else:
        n_runs = args.runs
        sim_time = args.time
        rms_time_start = args.rms_start
        rms_time_end = args.rms_end
    
    # Visualization requires sequential mode
    if args.vis and not args.sequential:
        print("Warning: --vis requires --sequential. Enabling sequential mode.")
        args.sequential = True
    
    # Measurement noise ON by default (use --no-noise to disable)
    measurement_noise = None if args.no_noise else True
    
    # Path re-indexing (default ON, --no-reindex disables)
    use_closest_point = not args.no_reindex
    
    # Validate all paths are feasible for the vehicle
    validate_path_configs()
    
    # Run benchmark
    if args.sequential:
        results = run_benchmark_sequential(
            n_runs=n_runs,
            sim_time=sim_time,
            v_target=args.speed,
            visualize=args.vis,
            measurement_noise=measurement_noise,
            use_closest_point=use_closest_point,
            rms_time_start=rms_time_start,
            rms_time_end=rms_time_end
        )
    else:
        results = run_benchmark_parallel(
            n_runs=n_runs,
            sim_time=sim_time,
            v_target=args.speed,
            n_workers=args.workers,
            measurement_noise=measurement_noise,
            use_closest_point=use_closest_point,
            rms_time_start=rms_time_start,
            rms_time_end=rms_time_end
        )
    
    # Compute statistics
    stats = compute_statistics(results)
    improvements = compute_improvement(stats)
    
    # Print summary table
    print_summary_table(stats, improvements)
    
    # Create timestamped output directory
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    noise_tag = "noise" if measurement_noise else "nonoise"
    run_dir = Path("benchmark_runs") / f"{ts}_{noise_tag}"
    run_dir.mkdir(parents=True, exist_ok=True)
    print(f"\nOutput directory: {run_dir}/")
    
    # Create plots
    plot_path = str(run_dir / args.output)
    plot_error_bars(results, stats, improvements, plot_path)
    
    # Save raw data
    save_raw_data(results, output_path=str(run_dir / 'benchmark_raw_data.npz'))
    
    print("\nBenchmark complete!")


if __name__ == "__main__":
    main()
