#!/usr/bin/env python3
"""
Phase 1+2 Diagnostic: NN sensitivity to n AND force reconstruction accuracy.

1. Sweep n at multiple operating points to map Fy(n)
2. Check if sensor-reconstructed forces match NN predictions
3. Identify where the UKF's observability breaks down

Run from SCM_Teleop root:
  conda run -n sim python diag_nn_sensitivity.py
"""

import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(PROJECT_ROOT / "simulation"))

from simulation.terrain_parameter_estimator import TerrainParameterEstimator
from simulation.param_consistency import (
    TERRAIN_PRESETS, HMMWV_VEHICLE_PARAMS, terrain_preset_to_internal,
)


def main():
    est = TerrainParameterEstimator(
        model_dir=str(PROJECT_ROOT / "nn_models" / "paper_v2_mlp_16_4")
    )

    vp = HMMWV_VEHICLE_PARAMS
    M = vp["M"]
    Izz = vp["Izz"]
    Lf = vp["Lf"]
    Lr = vp["Lr"]
    L = vp["L"]

    n_vals = np.linspace(0.5, 1.1, 31)

    # Typical operating points during sinusoidal open-loop driving
    op_points = [
        {"label": "gentle turn", "kappa": 0.02, "alpha": -0.10, "u": 4.5, "Fz": 6000.0, "sr": 0.1},
        {"label": "moderate turn", "kappa": 0.02, "alpha": -0.20, "u": 4.0, "Fz": 6500.0, "sr": 0.3},
        {"label": "sharp turn", "kappa": 0.03, "alpha": -0.35, "u": 3.5, "Fz": 6500.0, "sr": 0.5},
        {"label": "straight accel", "kappa": 0.10, "alpha": -0.02, "u": 5.0, "Fz": 6000.0, "sr": 0.0},
        {"label": "rear axle gentle", "kappa": 0.02, "alpha": 0.05, "u": 4.5, "Fz": 5500.0, "sr": 0.0},
        {"label": "rear axle moderate", "kappa": 0.02, "alpha": 0.12, "u": 4.0, "Fz": 5500.0, "sr": 0.0},
    ]

    fig, axes = plt.subplots(3, 2, figsize=(14, 12))
    fig.suptitle("NN Fy Sensitivity to Bekker n — Multiple Operating Points", fontsize=14)

    for idx, op in enumerate(op_points):
        ax = axes[idx // 2, idx % 2]
        Fy_vals = []
        Fx_vals = []
        for n in n_vals:
            Fx, Fy = est._nn_FxFy(
                kappa=op["kappa"], alpha=op["alpha"],
                u=op["u"], Fz=op["Fz"], sr=op["sr"], n_val=n)
            Fx_vals.append(Fx)
            Fy_vals.append(Fy)

        Fy_arr = np.array(Fy_vals)
        Fx_arr = np.array(Fx_vals)

        ax.plot(n_vals, Fy_arr, 'b-', linewidth=2, label='Fy (per-wheel)')
        ax.plot(n_vals, Fx_arr, 'r--', linewidth=2, label='Fx (per-wheel)')

        # Mark preset terrain values
        for name, preset in TERRAIN_PRESETS.items():
            ax.axvline(preset["n"], color='gray', linestyle=':', alpha=0.7)
            ax.text(preset["n"], ax.get_ylim()[0] if ax.get_ylim()[0] != 0 else min(Fy_arr),
                    f"  {name}", fontsize=8, rotation=90, va='bottom')

        fy_range = max(Fy_arr) - min(Fy_arr)
        ax.set_title(f"{op['label']} (α={op['alpha']:.2f}, κ={op['kappa']:.2f}, "
                     f"u={op['u']:.1f})\nFy range: {fy_range:.0f} N", fontsize=10)
        ax.set_xlabel("Bekker n")
        ax.set_ylabel("Force (N)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("plots/nn_sensitivity_to_n.png", dpi=150)
    print("Saved: plots/nn_sensitivity_to_n.png")

    # === Part 2: Axle-level force sensitivity (what the UKF actually compares) ===
    print("\n" + "="*80)
    print("AXLE-LEVEL FORCE SENSITIVITY (what the UKF grid filter compares)")
    print("="*80)

    # At each terrain preset, compute axle Fy_f, Fy_r and derived ay, omega_dot
    for terrain_name, preset in TERRAIN_PRESETS.items():
        true_n = preset["n"]
        internal = terrain_preset_to_internal(preset)

        # Typical operating conditions for this terrain
        # Clay has lower speeds, sand has higher
        u_typical = {"clay": 3.5, "dirt": 4.5, "sand": 5.0}[terrain_name]
        alpha_f_typical = -0.15
        alpha_r_typical = 0.08
        kappa_typical = 0.02

        print(f"\n--- {terrain_name.upper()} (true n={true_n}) ---")
        print(f"Operating: u={u_typical}, alpha_f={alpha_f_typical}, alpha_r={alpha_r_typical}")
        print(f"{'n':>5} | {'Fy_f_pw':>9} | {'Fy_r_pw':>9} | {'Fy_f_axle':>10} | {'Fy_r_axle':>10} | {'ay':>8} | {'wd':>8}")
        print("-" * 75)

        for n_val in [0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1]:
            Fx_f, Fy_f_pw = est._nn_FxFy(kappa_typical, alpha_f_typical,
                                           u_typical, 6500.0, 0.3, n_val)
            Fx_r, Fy_r_pw = est._nn_FxFy(kappa_typical, alpha_r_typical,
                                           u_typical, 5500.0, 0.0, n_val)

            # Axle forces (x2 for both wheels, negated per the estimator convention)
            Fy_f_axle = -2.0 * Fy_f_pw
            Fy_r_axle = -2.0 * Fy_r_pw

            # What these forces would produce as inertial measurements
            ay = (Fy_f_axle + Fy_r_axle) / M
            omega_dot = (Lf * Fy_f_axle - Lr * Fy_r_axle) / Izz

            print(f"{n_val:5.2f} | {Fy_f_pw:9.1f} | {Fy_r_pw:9.1f} | "
                  f"{Fy_f_axle:10.1f} | {Fy_r_axle:10.1f} | {ay:8.3f} | {omega_dot:8.4f}")

    # === Part 3: Check gradient magnitude for UKF convergence ===
    print("\n" + "="*80)
    print("GRADIENT dFy/dn AT EACH TERRAIN (key for UKF observability)")
    print("="*80)

    for terrain_name, preset in TERRAIN_PRESETS.items():
        true_n = preset["n"]
        u_typical = {"clay": 3.5, "dirt": 4.5, "sand": 5.0}[terrain_name]

        dn = 0.01
        n_lo = max(true_n - dn, 0.5)
        n_hi = min(true_n + dn, 1.1)

        _, Fy_f_lo = est._nn_FxFy(0.02, -0.15, u_typical, 6500.0, 0.3, n_lo)
        _, Fy_f_hi = est._nn_FxFy(0.02, -0.15, u_typical, 6500.0, 0.3, n_hi)
        _, Fy_r_lo = est._nn_FxFy(0.02, 0.08, u_typical, 5500.0, 0.0, n_lo)
        _, Fy_r_hi = est._nn_FxFy(0.02, 0.08, u_typical, 5500.0, 0.0, n_hi)

        dFyf_dn = (Fy_f_hi - Fy_f_lo) / (n_hi - n_lo)
        dFyr_dn = (Fy_r_hi - Fy_r_lo) / (n_hi - n_lo)

        # Axle-level
        dFyf_axle_dn = -2.0 * dFyf_dn
        dFyr_axle_dn = -2.0 * dFyr_dn
        day_dn = (dFyf_axle_dn + dFyr_axle_dn) / M
        dwd_dn = (Lf * dFyf_axle_dn - Lr * dFyr_axle_dn) / Izz

        print(f"{terrain_name:>5}: dFy_f/dn = {dFyf_axle_dn:+.1f} N, "
              f"dFy_r/dn = {dFyr_axle_dn:+.1f} N, "
              f"day/dn = {day_dn:+.4f} m/s², "
              f"dωd/dn = {dwd_dn:+.6f} rad/s²")

    # === Part 4: Log-likelihood landscape ===
    print("\n" + "="*80)
    print("LOG-LIKELIHOOD LANDSCAPE (Bayesian grid filter perspective)")
    print("="*80)

    fig2, axes2 = plt.subplots(1, 3, figsize=(16, 5))
    fig2.suptitle("Log-Likelihood Landscape for Terrain Identification", fontsize=14)

    for idx, (terrain_name, preset) in enumerate(TERRAIN_PRESETS.items()):
        true_n = preset["n"]
        u_typical = {"clay": 3.5, "dirt": 4.5, "sand": 5.0}[terrain_name]

        # "True" axle forces at this terrain
        _, Fy_f_true = est._nn_FxFy(0.02, -0.15, u_typical, 6500.0, 0.3, true_n)
        _, Fy_r_true = est._nn_FxFy(0.02, 0.08, u_typical, 5500.0, 0.0, true_n)
        Fy_f_axle_true = -2.0 * Fy_f_true
        Fy_r_axle_true = -2.0 * Fy_r_true
        y_true = np.array([Fy_f_axle_true, Fy_r_axle_true])

        # Log-likelihood at each n
        loglik = []
        R_diag = np.array([500.0**2, 500.0**2])  # conservative noise
        for n_val in n_vals:
            _, Fy_f_pred = est._nn_FxFy(0.02, -0.15, u_typical, 6500.0, 0.3, n_val)
            _, Fy_r_pred = est._nn_FxFy(0.02, 0.08, u_typical, 5500.0, 0.0, n_val)
            Fy_f_axle_pred = -2.0 * Fy_f_pred
            Fy_r_axle_pred = -2.0 * Fy_r_pred
            y_pred = np.array([Fy_f_axle_pred, Fy_r_axle_pred])

            innov = y_true - y_pred
            ll = -0.5 * np.sum(innov**2 / R_diag)
            loglik.append(ll)

        loglik = np.array(loglik)
        # Normalize for plotting
        loglik_norm = loglik - np.max(loglik)

        ax = axes2[idx]
        ax.plot(n_vals, np.exp(loglik_norm), 'b-', linewidth=2)
        ax.axvline(true_n, color='green', linestyle='--', linewidth=2, label=f'True n={true_n}')
        ax.set_title(f"{terrain_name.upper()}")
        ax.set_xlabel("Bekker n")
        ax.set_ylabel("Relative Likelihood")
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Print peak
        peak_n = n_vals[np.argmax(loglik)]
        print(f"{terrain_name}: Peak likelihood at n={peak_n:.3f} (true={true_n}), "
              f"Fy_f_axle_true={Fy_f_axle_true:.0f}N, Fy_r_axle_true={Fy_r_axle_true:.0f}N")

    plt.tight_layout()
    plt.savefig("plots/nn_loglikelihood_landscape.png", dpi=150)
    print("Saved: plots/nn_loglikelihood_landscape.png")


if __name__ == "__main__":
    main()
