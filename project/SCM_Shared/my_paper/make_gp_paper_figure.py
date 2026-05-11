#!/usr/bin/env python3
"""
Generate paper figure for Online Residual Learning contribution.

Uses validated multi-run statistics (already collected, not re-simulated here).

Clay terrain results (clay, sinusoidal, 8 m/s, 20s, 5-run each for baseline + GP):
  - Baseline (no GP):       0.335 +/- 0.063 m   (n=5)
  - GP only (1 train round + eval): 0.136 +/- 0.026 m   (n=5)  → -59%
  - GP + speed scheduling (1 round): 0.091 m              (n=1)  → -73%

Dirt terrain results (dirt, sinusoidal, 8 m/s, 20s):
  - Baseline (no GP):       0.244 m   (n=1)
  - GP, no terrain gate:    0.578 m   (n=1)  → +137% (BAD)
  - GP + terrain gate:      0.230 m   (n=1)  → -6%   (gate works)
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

OUT = Path(__file__).resolve().parent / "paper_figures" / "gp_paper_figure.png"
OUT.parent.mkdir(parents=True, exist_ok=True)

# ── Data ──────────────────────────────────────────────────────────────────────
CLAY_METHODS  = ["Baseline\n(no GP)", "Dynamics GP\n(1 round)", "GP + Speed\nScheduling"]
CLAY_CTE      = [0.335, 0.136, 0.091]
CLAY_ERR      = [0.063, 0.026, 0.0]     # std; 0 where n=1

DIRT_METHODS  = ["Baseline\n(no GP)", "GP, No Gate", "GP + Terrain\nGate"]
DIRT_CTE      = [0.244, 0.578, 0.230]
DIRT_ERR      = [0.0,   0.0,   0.0]

# ── Colors ────────────────────────────────────────────────────────────────────
CLAY_COLORS = ["#4878CF", "#6ACC65", "#D65F5F"]   # blue, green, red-orange
DIRT_COLORS = ["#4878CF", "#D65F5F", "#6ACC65"]

# ── Figure ────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(1, 2, figsize=(11, 5.5))
fig.suptitle("Online Residual GP Adapter — Lateral CTE Comparison\n"
             "(clay / dirt, sinusoidal path, 8 m/s, 20 s)",
             fontsize=13, fontweight='bold')

def make_bar_group(ax, methods, ctes, errs, colors, title, baseline_idx=0):
    x = np.arange(len(methods))
    bars = ax.bar(x, ctes, width=0.55, color=colors, edgecolor='white', linewidth=0.8, zorder=3)

    # error caps only where n > 1
    for xi, (cte, err) in enumerate(zip(ctes, errs)):
        if err > 0:
            ax.errorbar(xi, cte, yerr=err, fmt='none',
                        ecolor='black', capsize=6, capthick=1.5, elinewidth=1.5, zorder=4)

    # dashed baseline reference
    ax.axhline(ctes[baseline_idx], color='#4878CF', ls='--', lw=1.2, alpha=0.6, zorder=2)

    # improvement labels
    base = ctes[baseline_idx]
    for xi, (bar, cte) in enumerate(zip(bars, ctes)):
        pct = (cte - base) / base * 100
        label = f"{cte:.3f} m"
        if xi != baseline_idx:
            sign = "+" if pct > 0 else ""
            label += f"\n({sign}{pct:.0f}%)"
        ax.text(xi, cte + max(ctes) * 0.03, label,
                ha='center', va='bottom', fontsize=9.5, fontweight='bold')

    ax.set_xticks(x)
    ax.set_xticklabels(methods, fontsize=10)
    ax.set_ylabel("Mean Lateral CTE (m)", fontsize=11)
    ax.set_title(title, fontsize=12, fontweight='bold')
    ax.set_ylim(0, max(ctes) * 1.35)
    ax.grid(True, axis='y', alpha=0.3, zorder=0)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

make_bar_group(axes[0], CLAY_METHODS, CLAY_CTE, CLAY_ERR, CLAY_COLORS,
               "Clay Terrain\n(soft soil, large SCM nonlinearities)")
make_bar_group(axes[1], DIRT_METHODS, DIRT_CTE, DIRT_ERR, DIRT_COLORS,
               "Dirt Terrain\n(firm soil, NN well-calibrated)")

# n-sample notes
axes[0].text(0.02, 0.98, "n=5 runs (bars show mean ± std)\nn=1 run for GP+Speed",
             transform=axes[0].transAxes, fontsize=8, va='top',
             bbox=dict(boxstyle='round,pad=0.3', fc='wheat', alpha=0.5))
axes[1].text(0.02, 0.98, "n=1 run each",
             transform=axes[1].transAxes, fontsize=8, va='top',
             bbox=dict(boxstyle='round,pad=0.3', fc='wheat', alpha=0.5))

fig.tight_layout()
fig.savefig(str(OUT), dpi=200, bbox_inches='tight')
print(f"Saved: {OUT}")
