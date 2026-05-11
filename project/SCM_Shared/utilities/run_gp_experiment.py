#!/usr/bin/env python3
"""Run dynamics-GP learning experiment: 1 baseline + 10 GP rounds, then plot."""
import subprocess, re, sys, shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SIM_DIR = ROOT / "simulation"
FIG_DIR = ROOT / "my_paper" / "paper_figures"
CONDA_BIN = "/home/kyle/miniconda3/bin/conda"
FIG_DIR.mkdir(parents=True, exist_ok=True)

COMMON = [
    CONDA_BIN, "run", "--no-capture-output", "-n", "sim", "python",
    str(SIM_DIR / "launch_decoupled.py"),
    "--model", "nn", "--terrain", "clay", "--path", "sinusoidal",
    "--time", "20", "--speed", "8.0", "--no-vis", "--no-plot",
    "--vel-filter-tau", "0.05",   # EMA filter on noisy [u,v,omega] — critical for GP target SNR
]
GP_ARGS = [
    "--dynamics-gp",
    "--dynamics-gp-gain", "0.5",
    "--gp-noise-var", "0.5",
]

def extract(text, pattern, group=1, default=float('nan')):
    m = re.search(pattern, text)
    return float(m.group(group)) if m else default

def run_one(round_num, extra_args=None):
    cmd = COMMON + (extra_args or [])
    print(f"=== Round {round_num} ({'GP' if extra_args else 'baseline'}) ===", flush=True)
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=300, cwd=ROOT)
    out = r.stdout + r.stderr

    cte_avg = extract(out, r'Avg path pos err:\s+([\d.]+)')
    cte_rms = extract(out, r'Avg path pos err:.*RMS\s+([\d.]+)')
    cte_max = extract(out, r'Max.*CTE.*:\s+([\d.]+)')
    pred_pos = extract(out, r'Mean pos:\s+([\d.]+)')
    pred_u = extract(out, r'Mean \|u\|:\s+([\d.]+)')
    pred_v = extract(out, r'Mean \|v\|:\s+([\d.]+)')
    pred_omega = float('nan')
    # robust omega extraction
    for line in out.splitlines():
        if 'rad/s' in line and 'Mean' in line:
            m = re.search(r'([\d.]+)\s+rad/s', line)
            if m:
                pred_omega = float(m.group(1))
                break
    mean_speed = extract(out, r'Mean speed:\s+([\d.]+)')
    dyn_ind = extract(out, r'DynGP inducing points:\s+(\d+)', default=0)

    row = dict(round=round_num, cte_avg=cte_avg, cte_rms=cte_rms,
               cte_max=cte_max, pred_pos=pred_pos, pred_u=pred_u,
               pred_v=pred_v, pred_omega=pred_omega, mean_speed=mean_speed,
               dyn_ind=int(dyn_ind))
    print(f"  CTE={cte_avg:.4f}m  pred_pos={pred_pos:.4f}m  "
          f"pred_u={pred_u:.3f}  pred_v={pred_v:.3f}  pred_w={pred_omega:.4f}  "
          f"DynGP={int(dyn_ind)}", flush=True)
    return row

# --- Setup ---
# Clear cached solver
acados_dir = Path("/tmp/acados_mpc_static_mlp")
if acados_dir.exists():
    shutil.rmtree(acados_dir)

# Clear dynamics-GP state
gp_dir = ROOT / "data" / "gp_residual"
p = gp_dir / "dynamics_gp_state.npz"
if p.exists():
    p.unlink()
print("Dynamics GP state cleared.\n", flush=True)

# --- Run ---
results = []

# Baseline
results.append(run_one(0))

# 10 GP rounds
for i in range(1, 11):
    results.append(run_one(i, GP_ARGS))

# --- Print table ---
print("\n" + "=" * 90)
print(f"{'Round':>5} {'CTE_avg':>8} {'CTE_rms':>8} {'pred_pos':>9} "
      f"{'pred_u':>7} {'pred_v':>7} {'pred_ω':>7} {'speed':>6} {'DynGP':>5}")
print("-" * 90)
for r in results:
    label = "base" if r['round'] == 0 else f"DGP-{r['round']}"
    print(f"{label:>5} {r['cte_avg']:8.4f} {r['cte_rms']:8.4f} {r['pred_pos']:9.4f} "
          f"{r['pred_u']:7.3f} {r['pred_v']:7.3f} {r['pred_omega']:7.4f} "
          f"{r['mean_speed']:6.2f} {r['dyn_ind']:5d}")

# --- Plot ---
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

rounds = [r['round'] for r in results]
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('Dynamics-GP Learning Experiment (clay sinusoidal, 8 m/s, 20s)', fontsize=14, fontweight='bold')

# CTE
ax = axes[0, 0]
ax.plot(rounds, [r['cte_avg'] for r in results], 'bo-', lw=2, ms=8, label='Avg CTE')
ax.plot(rounds, [r['cte_rms'] for r in results], 'rs--', lw=1.5, ms=6, label='RMS CTE')
ax.axhline(results[0]['cte_avg'], color='b', ls=':', alpha=0.4, label=f'Baseline avg={results[0]["cte_avg"]:.3f}m')
ax.set_xlabel('Round')
ax.set_ylabel('Cross-Track Error (m)')
ax.set_title('Path Tracking Error')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)
ax.set_xticks(rounds)
ax.set_xticklabels(['base'] + [str(i) for i in range(1, 11)])

# Prediction errors
ax = axes[0, 1]
ax.plot(rounds, [r['pred_pos'] for r in results], 'ko-', lw=2, ms=8, label='pos (m)')
ax.plot(rounds, [r['pred_u'] for r in results], 'g^-', lw=1.5, ms=6, label='|u| (m/s)')
ax.plot(rounds, [r['pred_v'] for r in results], 'm+-', lw=1.5, ms=6, label='|v| (m/s)')
ax.plot(rounds, [r['pred_omega'] for r in results], 'cx-', lw=1.5, ms=6, label='|ω| (rad/s)')
ax.set_xlabel('Round')
ax.set_ylabel('1-Step Prediction Error')
ax.set_title('Model Prediction Residuals')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)
ax.set_xticks(rounds)
ax.set_xticklabels(['base'] + [str(i) for i in range(1, 11)])

# GP inducing points
ax = axes[1, 0]
ax.plot(rounds, [r['dyn_ind'] for r in results], 'bs-', lw=2, ms=8, label='Dynamics GP')
ax.set_xlabel('Round')
ax.set_ylabel('Inducing Points')
ax.set_title('Dynamics GP Model Size')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3)
ax.set_xticks(rounds)
ax.set_xticklabels(['base'] + [str(i) for i in range(1, 11)])

# Improvement %
ax = axes[1, 1]
base_cte = results[0]['cte_avg']
base_pred = results[0]['pred_pos']
cte_imp = [(1 - r['cte_avg'] / base_cte) * 100 for r in results]
pred_imp = [(1 - r['pred_pos'] / base_pred) * 100 for r in results]
ax.bar([x - 0.15 for x in rounds], cte_imp, width=0.3, color='steelblue', label='CTE improvement %')
ax.bar([x + 0.15 for x in rounds], pred_imp, width=0.3, color='coral', label='Pred. error improvement %')
ax.axhline(0, color='k', lw=0.5)
ax.set_xlabel('Round')
ax.set_ylabel('Improvement (%)')
ax.set_title('Improvement vs Baseline')
ax.legend(fontsize=9)
ax.grid(True, alpha=0.3, axis='y')
ax.set_xticks(rounds)
ax.set_xticklabels(['base'] + [str(i) for i in range(1, 11)])

fig.tight_layout()
out_path = FIG_DIR / "gp_experiment_results.png"
fig.savefig(str(out_path), dpi=150)
print(f"\nPlot saved to {out_path}")
