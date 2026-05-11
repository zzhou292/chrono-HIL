#!/usr/bin/env python3
"""
Task 5: Compare actual Chrono tire forces with NN predictions.
==============================================================
Reads tire_force_log.json and state_history_log.json produced by
launch_decoupled.py's force-instrumented simulation loop, then
computes NN predictions at the same states and plots comparisons.

Usage:
    1. Run a simulation:
       python launch_decoupled.py --controller nn --terrain clay --sim-time 15
    2. Plot comparison:
       python compare_forces.py --model ../nn_models/v6_sweep_16_4/best_terrain_nn.pt \
                                --scaler ../nn_models/v6_sweep_16_4/scalers.pkl \
                                --terrain clay
"""

import argparse
import json
import sys
import numpy as np
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "simulation"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "nn_training"))

from param_consistency import TERRAIN_PRESETS, HMMWV_VEHICLE_PARAMS


def load_nn_predictor(model_path, scaler_path, terrain):
    """Load NN model, return a callable: (slip_angle, Fz, velocity, kappa, sr) -> (Fx, Fy)."""
    import torch, pickle
    from train_terrain_nn import TerrainNN

    checkpoint = torch.load(model_path, weights_only=False, map_location="cpu")
    state_dict = checkpoint.get("model_state_dict", checkpoint) if isinstance(checkpoint, dict) else checkpoint
    hidden_sizes = checkpoint.get("hidden_sizes") if isinstance(checkpoint, dict) else None

    if any(k.startswith("layer") and not k.startswith("layers.") for k in state_dict):
        old_to_new = {}
        idx = 0
        while f"layer{idx+1}.weight" in state_dict:
            old_to_new[f"layer{idx+1}.weight"] = f"layers.{idx}.weight"
            old_to_new[f"layer{idx+1}.bias"] = f"layers.{idx}.bias"
            idx += 1
        state_dict = {old_to_new.get(k, k): v for k, v in state_dict.items()}

    if hidden_sizes is None:
        layer_ids = sorted(set(int(k.split(".")[1]) for k in state_dict if k.startswith("layers.")))
        hidden_sizes = [state_dict[f"layers.{i}.weight"].shape[0] for i in layer_ids[:-1]]

    model = TerrainNN(input_size=11, output_size=2, hidden_sizes=hidden_sizes)
    model.load_state_dict(state_dict)
    model.eval()

    with open(scaler_path, "rb") as f:
        scalers = pickle.load(f)
    scaler_X = scalers["X"]
    scaler_y = scalers["y"]

    phi_mean = scaler_X.mean_[9]
    is_v6 = phi_mean < 1.0

    def predict(slip_angle, Fz, velocity, kappa=0.0, steering_rate=0.0):
        if is_v6:
            phi_rad = np.radians(terrain["phi"])
            x_raw = np.array([[kappa, slip_angle, velocity, Fz, steering_rate,
                               terrain["Kphi"], terrain["Kc"], terrain["n"],
                               terrain["c"], phi_rad, terrain["k"]]])
        else:
            x_raw = np.array([[Fz, slip_angle, kappa, 0.0, velocity,
                               terrain["Kphi"], terrain["Kc"], terrain["n"],
                               terrain["c"], terrain["phi"], terrain["k"]]])
        x_scaled = scaler_X.transform(x_raw)
        with torch.no_grad():
            y_scaled = model(torch.tensor(x_scaled, dtype=torch.float32)).numpy()
        y_out = scaler_y.inverse_transform(y_scaled)
        return float(y_out[0, 0]), float(y_out[0, 1])

    return predict, is_v6


def main():
    parser = argparse.ArgumentParser(description="Compare actual vs NN tire forces")
    parser.add_argument("--model", type=str, required=True)
    parser.add_argument("--scaler", type=str, required=True)
    parser.add_argument("--terrain", type=str, default="clay")
    parser.add_argument("--force-log", type=str, default=None,
                        help="Path to tire_force_log.json (default: auto)")
    parser.add_argument("--state-log", type=str, default=None,
                        help="Path to state_history_log.json (default: auto)")
    parser.add_argument("--save-dir", type=str, default=None)
    args = parser.parse_args()

    diag_dir = Path(__file__).parent
    force_log = Path(args.force_log) if args.force_log else diag_dir / "tire_force_log.json"
    state_log = Path(args.state_log) if args.state_log else diag_dir / "state_history_log.json"

    if not force_log.exists():
        print(f"ERROR: {force_log} not found. Run a simulation first with force logging enabled.")
        print(f"  python simulation/launch_decoupled.py --controller nn --terrain {args.terrain} --sim-time 15")
        sys.exit(1)

    # Load data
    with open(force_log) as f:
        forces = json.load(f)
    with open(state_log) as f:
        states = json.load(f)

    print(f"Loaded {len(forces)} force records, {len(states)} state records")

    # Setup terrain
    tp = TERRAIN_PRESETS[args.terrain]
    terrain = {
        "Kphi": tp["Kphi"], "Kc": tp["Kc"], "n": tp["n"],
        "c": tp["cohesion"], "phi": tp["friction_angle"], "k": tp["janosi_shear"],
    }
    vp = HMMWV_VEHICLE_PARAMS
    L = vp["Lf"] + vp["Lr"]
    M = vp["M"]
    h_cg = vp.get("h_cg", 0.65)
    T = vp.get("T", 1.8194)
    Fz_f_wheel_static = M * 9.81 * vp["Lr"] / L / 2.0
    Fz_r_wheel_static = M * 9.81 * vp["Lf"] / L / 2.0

    predict, is_v6 = load_nn_predictor(args.model, args.scaler, terrain)
    print(f"Model format: {'v6' if is_v6 else 'v3'}")

    # Extract time series from force log
    times_f = np.array([r['time'] for r in forces])

    # Actual forces: sum left+right per axle (global frame)
    actual_Fy_front = np.array([r['front_left_Fy'] + r['front_right_Fy'] for r in forces])
    actual_Fy_rear = np.array([r['rear_left_Fy'] + r['rear_right_Fy'] for r in forces])
    actual_Fx_front = np.array([r['front_left_Fx'] + r['front_right_Fx'] for r in forces])
    actual_Fx_rear = np.array([r['rear_left_Fx'] + r['rear_right_Fx'] for r in forces])
    actual_Fz_front = np.array([r['front_left_Fz'] + r['front_right_Fz'] for r in forces])
    actual_Fz_rear = np.array([r['rear_left_Fz'] + r['rear_right_Fz'] for r in forces])

    # Actual slip angles from Chrono (average left+right)
    actual_alpha_f = np.array([(r['front_left_slip_angle'] + r['front_right_slip_angle']) / 2 for r in forces])
    actual_alpha_r = np.array([(r['rear_left_slip_angle'] + r['rear_right_slip_angle']) / 2 for r in forces])

    # Merge state history onto force time grid (interpolate)
    times_s = np.array([s['time'] for s in states])
    u_s = np.array([s['u'] for s in states])
    v_s = np.array([s['v'] for s in states])
    omega_s = np.array([s['omega'] for s in states])
    delta_s = np.array([s['delta'] for s in states])
    psi_s = np.array([s['psi'] for s in states])
    ax_s = np.array([s.get('ax', 0.0) for s in states])

    u_interp = np.interp(times_f, times_s, u_s)
    v_interp = np.interp(times_f, times_s, v_s)
    omega_interp = np.interp(times_f, times_s, omega_s)
    delta_interp = np.interp(times_f, times_s, delta_s)
    ax_interp = np.interp(times_f, times_s, ax_s)

    # Dynamic Fz with longitudinal load transfer (per wheel mean)
    Fz_f_dyn = (M * 9.81 * vp["Lr"] - M * ax_interp * h_cg) / L / 2.0
    Fz_r_dyn = (M * 9.81 * vp["Lf"] + M * ax_interp * h_cg) / L / 2.0

    # Lateral load transfer: ΔFz = M * ay * h_cg / T / 2 (per wheel)
    ay_interp = u_interp * omega_interp  # centripetal
    dFz = M * ay_interp * h_cg / T / 2.0

    # Per-wheel Fz with lateral load transfer
    Fz_f_outer = np.minimum(Fz_f_dyn + dFz, Fz_f_dyn * 1.9)
    Fz_f_inner = np.maximum(Fz_f_dyn - dFz, Fz_f_dyn * 0.1)
    Fz_r_outer = np.minimum(Fz_r_dyn + dFz, Fz_r_dyn * 1.9)
    Fz_r_inner = np.maximum(Fz_r_dyn - dFz, Fz_r_dyn * 0.1)

    # Compute bicycle-model slip angles
    Lf, Lr = vp["Lf"], vp["Lr"]
    u_safe = np.maximum(np.abs(u_interp), 0.5)
    alpha_f_bike = delta_interp - np.arctan2(v_interp + Lf * omega_interp, u_safe)
    alpha_r_bike = -np.arctan2(v_interp - Lr * omega_interp, u_safe)

    # NN predictions at each time step — static Fz (old baseline)
    nn_Fy_front_static = np.zeros(len(times_f))
    nn_Fy_rear_static = np.zeros(len(times_f))
    nn_Fx_front = np.zeros(len(times_f))
    nn_Fx_rear = np.zeros(len(times_f))

    for i in range(len(times_f)):
        Fx_fw, Fy_fw = predict(alpha_f_bike[i], Fz_f_wheel_static, u_safe[i])
        Fx_rw, Fy_rw = predict(alpha_r_bike[i], Fz_r_wheel_static, u_safe[i])
        nn_Fy_front_static[i] = -2.0 * Fy_fw
        nn_Fy_rear_static[i] = -2.0 * Fy_rw
        nn_Fx_front[i] = 2.0 * Fx_fw
        nn_Fx_rear[i] = 2.0 * Fx_rw

    # NN predictions at each time step — dynamic Fz with lateral + longitudinal load transfer
    nn_Fy_front = np.zeros(len(times_f))
    nn_Fy_rear = np.zeros(len(times_f))

    for i in range(len(times_f)):
        # Front axle: outer + inner wheel
        _, Fy_fo = predict(alpha_f_bike[i], Fz_f_outer[i], u_safe[i])
        _, Fy_fi = predict(alpha_f_bike[i], Fz_f_inner[i], u_safe[i])
        # Rear axle: outer + inner wheel
        _, Fy_ro = predict(alpha_r_bike[i], Fz_r_outer[i], u_safe[i])
        _, Fy_ri = predict(alpha_r_bike[i], Fz_r_inner[i], u_safe[i])
        nn_Fy_front[i] = -(Fy_fo + Fy_fi)
        nn_Fy_rear[i] = -(Fy_ro + Fy_ri)

    # NN predictions using ACTUAL Chrono slip angles (for direct comparison)
    nn_Fy_front_actual_alpha = np.zeros(len(times_f))
    nn_Fy_rear_actual_alpha = np.zeros(len(times_f))
    for i in range(len(times_f)):
        _, Fy_fo = predict(actual_alpha_f[i], Fz_f_outer[i], u_safe[i])
        _, Fy_fi = predict(actual_alpha_f[i], Fz_f_inner[i], u_safe[i])
        _, Fy_ro = predict(actual_alpha_r[i], Fz_r_outer[i], u_safe[i])
        _, Fy_ri = predict(actual_alpha_r[i], Fz_r_inner[i], u_safe[i])
        nn_Fy_front_actual_alpha[i] = -(Fy_fo + Fy_fi)
        nn_Fy_rear_actual_alpha[i] = -(Fy_ro + Fy_ri)

    # Statistics
    # Filter out first 2 seconds (settling)
    mask = times_f > 2.0
    print(f"\n{'='*60}")
    print(f"Force Comparison Statistics (t > 2.0s)")
    print(f"{'='*60}")

    for name, actual, nn_dyn, nn_static, nn_actual in [
        ("Front Fy", actual_Fy_front, nn_Fy_front, nn_Fy_front_static, nn_Fy_front_actual_alpha),
        ("Rear Fy", actual_Fy_rear, nn_Fy_rear, nn_Fy_rear_static, nn_Fy_rear_actual_alpha),
    ]:
        err_dyn = actual[mask] - nn_dyn[mask]
        err_static = actual[mask] - nn_static[mask]
        err_actual = actual[mask] - nn_actual[mask]
        print(f"\n  {name}:")
        print(f"    Actual range: [{actual[mask].min():.0f}, {actual[mask].max():.0f}] N")
        print(f"    NN (dynamic Fz) range: [{nn_dyn[mask].min():.0f}, {nn_dyn[mask].max():.0f}] N")
        print(f"    NN (static Fz)  range: [{nn_static[mask].min():.0f}, {nn_static[mask].max():.0f}] N")
        print(f"    RMS error (dynamic Fz): {np.sqrt(np.mean(err_dyn**2)):.0f} N")
        print(f"    RMS error (static Fz):  {np.sqrt(np.mean(err_static**2)):.0f} N")
        print(f"    Mean error (dynamic Fz): {np.mean(err_dyn):.0f} N")
        print(f"    Mean error (static Fz):  {np.mean(err_static):.0f} N")
        print(f"    RMS error (actual α, dyn Fz): {np.sqrt(np.mean(err_actual**2)):.0f} N")

    # Also compare slip angles
    alpha_err_f = actual_alpha_f[mask] - alpha_f_bike[mask]
    alpha_err_r = actual_alpha_r[mask] - alpha_r_bike[mask]
    print(f"\n  Slip angle comparison (actual Chrono vs bicycle model):")
    print(f"    Front: RMS = {np.degrees(np.sqrt(np.mean(alpha_err_f**2))):.2f}°, "
          f"mean = {np.degrees(np.mean(alpha_err_f)):.2f}°")
    print(f"    Rear: RMS = {np.degrees(np.sqrt(np.mean(alpha_err_r**2))):.2f}°, "
          f"mean = {np.degrees(np.mean(alpha_err_r)):.2f}°")

    # Normal force comparison
    print(f"\n  Normal force (actual Chrono vs model):")
    print(f"    Front actual: [{actual_Fz_front[mask].min():.0f}, {actual_Fz_front[mask].max():.0f}] N, "
          f"mean={actual_Fz_front[mask].mean():.0f} N")
    print(f"    Front static: {2*Fz_f_wheel_static:.0f} N")
    print(f"    Front dyn (long+lat xfer): outer [{Fz_f_outer[mask].min():.0f}, {Fz_f_outer[mask].max():.0f}], "
          f"inner [{Fz_f_inner[mask].min():.0f}, {Fz_f_inner[mask].max():.0f}] N")
    print(f"    Rear actual: [{actual_Fz_rear[mask].min():.0f}, {actual_Fz_rear[mask].max():.0f}] N, "
          f"mean={actual_Fz_rear[mask].mean():.0f} N")
    print(f"    Rear static: {2*Fz_r_wheel_static:.0f} N")
    print(f"    Rear dyn (long+lat xfer): outer [{Fz_r_outer[mask].min():.0f}, {Fz_r_outer[mask].max():.0f}], "
          f"inner [{Fz_r_inner[mask].min():.0f}, {Fz_r_inner[mask].max():.0f}] N")
    print(f"    Rear dynamic (load xfer): [{2*Fz_r_dyn[mask].min():.0f}, {2*Fz_r_dyn[mask].max():.0f}] N, "
          f"mean={2*Fz_r_dyn[mask].mean():.0f} N")

    # =========================================================================
    # Plots
    # =========================================================================
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        from datetime import datetime as _dt
        _ts = _dt.now().strftime("%Y%m%d_%H%M%S")
        base_dir = Path(args.save_dir) if args.save_dir else diag_dir / "plots"
        save_dir = base_dir / f"{_ts}_{args.terrain}"
        save_dir.mkdir(parents=True, exist_ok=True)

        # --- Plot 1: Fy time series ---
        fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

        ax = axes[0]
        ax.plot(times_f, actual_Fy_front, 'b-', alpha=0.7, label='Chrono actual')
        ax.plot(times_f, nn_Fy_front, 'r--', alpha=0.7, label='NN (dynamic Fz)')
        ax.plot(times_f, nn_Fy_front_static, 'g:', alpha=0.7, label='NN (static Fz)')
        ax.set_ylabel('Fy (N)')
        ax.set_title('Front Axle Lateral Force: Actual vs NN')
        ax.legend()
        ax.grid(True, alpha=0.3)

        ax = axes[1]
        ax.plot(times_f, actual_Fy_rear, 'b-', alpha=0.7, label='Chrono actual')
        ax.plot(times_f, nn_Fy_rear, 'r--', alpha=0.7, label='NN (dynamic Fz)')
        ax.plot(times_f, nn_Fy_rear_static, 'g:', alpha=0.7, label='NN (static Fz)')
        ax.set_ylabel('Fy (N)')
        ax.set_xlabel('Time (s)')
        ax.set_title('Rear Axle Lateral Force: Actual vs NN')
        ax.legend()
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(save_dir / "task5_Fy_timeseries.png", dpi=150)
        print(f"\n  Saved: {save_dir / 'task5_Fy_timeseries.png'}")
        plt.close()

        # --- Plot 2: Fy vs slip angle scatter ---
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))

        ax = axes[0]
        ax.scatter(np.degrees(actual_alpha_f[mask]), actual_Fy_front[mask],
                   s=5, alpha=0.3, c='blue', label='Chrono actual')
        # Sort for clean line
        sort_idx = np.argsort(alpha_f_bike[mask])
        ax.plot(np.degrees(alpha_f_bike[mask][sort_idx]), nn_Fy_front[mask][sort_idx],
                'r-', linewidth=2, label='NN prediction')
        ax.set_xlabel('Slip Angle (deg)')
        ax.set_ylabel('Fy front axle (N)')
        ax.set_title('Front: Fy vs Slip Angle')
        ax.legend()
        ax.grid(True, alpha=0.3)

        ax = axes[1]
        ax.scatter(np.degrees(actual_alpha_r[mask]), actual_Fy_rear[mask],
                   s=5, alpha=0.3, c='blue', label='Chrono actual')
        sort_idx = np.argsort(alpha_r_bike[mask])
        ax.plot(np.degrees(alpha_r_bike[mask][sort_idx]), nn_Fy_rear[mask][sort_idx],
                'r-', linewidth=2, label='NN prediction')
        ax.set_xlabel('Slip Angle (deg)')
        ax.set_ylabel('Fy rear axle (N)')
        ax.set_title('Rear: Fy vs Slip Angle')
        ax.legend()
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(save_dir / "task5_Fy_vs_alpha.png", dpi=150)
        print(f"  Saved: {save_dir / 'task5_Fy_vs_alpha.png'}")
        plt.close()

        # --- Plot 3: Slip angle comparison ---
        fig, axes = plt.subplots(2, 1, figsize=(14, 6), sharex=True)

        ax = axes[0]
        ax.plot(times_f, np.degrees(actual_alpha_f), 'b-', alpha=0.7, label='Chrono actual')
        ax.plot(times_f, np.degrees(alpha_f_bike), 'r--', alpha=0.7, label='Bicycle model')
        ax.set_ylabel('α_f (deg)')
        ax.set_title('Front Slip Angle: Chrono vs Bicycle Model')
        ax.legend()
        ax.grid(True, alpha=0.3)

        ax = axes[1]
        ax.plot(times_f, np.degrees(actual_alpha_r), 'b-', alpha=0.7, label='Chrono actual')
        ax.plot(times_f, np.degrees(alpha_r_bike), 'r--', alpha=0.7, label='Bicycle model')
        ax.set_ylabel('α_r (deg)')
        ax.set_xlabel('Time (s)')
        ax.set_title('Rear Slip Angle: Chrono vs Bicycle Model')
        ax.legend()
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(save_dir / "task5_slip_angles.png", dpi=150)
        print(f"  Saved: {save_dir / 'task5_slip_angles.png'}")
        plt.close()

        # --- Plot 4: Normal force comparison (per-wheel: outer vs inner) ---
        fig, axes = plt.subplots(2, 1, figsize=(14, 6), sharex=True)

        ax = axes[0]
        ax.plot(times_f, actual_Fz_front / 2, 'b-', alpha=0.5, label='Chrono actual (mean/wheel)')
        ax.axhline(Fz_f_wheel_static, color='g', linestyle=':', label=f'Static ({Fz_f_wheel_static:.0f} N)')
        ax.plot(times_f, Fz_f_outer, 'r-', alpha=0.7, label='Outer wheel')
        ax.plot(times_f, Fz_f_inner, 'orange', alpha=0.7, label='Inner wheel')
        ax.set_ylabel('Fz front per-wheel (N)')
        ax.set_title('Front: Per-Wheel Normal Force (with lateral load transfer)')
        ax.legend()
        ax.grid(True, alpha=0.3)

        ax = axes[1]
        ax.plot(times_f, actual_Fz_rear / 2, 'b-', alpha=0.5, label='Chrono actual (mean/wheel)')
        ax.axhline(Fz_r_wheel_static, color='g', linestyle=':', label=f'Static ({Fz_r_wheel_static:.0f} N)')
        ax.plot(times_f, Fz_r_outer, 'r-', alpha=0.7, label='Outer wheel')
        ax.plot(times_f, Fz_r_inner, 'orange', alpha=0.7, label='Inner wheel')
        ax.set_ylabel('Fz rear per-wheel (N)')
        ax.set_xlabel('Time (s)')
        ax.set_title('Rear: Per-Wheel Normal Force (with lateral load transfer)')
        ax.legend()
        ax.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.savefig(save_dir / "task5_normal_forces.png", dpi=150)
        print(f"  Saved: {save_dir / 'task5_normal_forces.png'}")
        plt.close()

        # --- Plot 5: Error histograms ---
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))

        err_f_dyn = actual_Fy_front[mask] - nn_Fy_front[mask]
        err_r_dyn = actual_Fy_rear[mask] - nn_Fy_rear[mask]
        err_f_static = actual_Fy_front[mask] - nn_Fy_front_static[mask]
        err_r_static = actual_Fy_rear[mask] - nn_Fy_rear_static[mask]

        ax = axes[0, 0]
        ax.hist(err_f_dyn, bins=50, alpha=0.7, color='red')
        ax.axvline(np.mean(err_f_dyn), color='k', linestyle='--', label=f'Mean={np.mean(err_f_dyn):.0f} N')
        ax.set_xlabel('Fy error (N)')
        ax.set_ylabel('Count')
        ax.set_title(f'Front Fy Error - Dynamic Fz (RMS={np.sqrt(np.mean(err_f_dyn**2)):.0f} N)')
        ax.legend()

        ax = axes[0, 1]
        ax.hist(err_f_static, bins=50, alpha=0.7, color='green')
        ax.axvline(np.mean(err_f_static), color='k', linestyle='--', label=f'Mean={np.mean(err_f_static):.0f} N')
        ax.set_xlabel('Fy error (N)')
        ax.set_ylabel('Count')
        ax.set_title(f'Front Fy Error - Static Fz (RMS={np.sqrt(np.mean(err_f_static**2)):.0f} N)')
        ax.legend()

        ax = axes[1, 0]
        ax.hist(err_r_dyn, bins=50, alpha=0.7, color='red')
        ax.axvline(np.mean(err_r_dyn), color='k', linestyle='--', label=f'Mean={np.mean(err_r_dyn):.0f} N')
        ax.set_xlabel('Fy error (N)')
        ax.set_ylabel('Count')
        ax.set_title(f'Rear Fy Error - Dynamic Fz (RMS={np.sqrt(np.mean(err_r_dyn**2)):.0f} N)')
        ax.legend()

        ax = axes[1, 1]
        ax.hist(err_r_static, bins=50, alpha=0.7, color='green')
        ax.axvline(np.mean(err_r_static), color='k', linestyle='--', label=f'Mean={np.mean(err_r_static):.0f} N')
        ax.set_xlabel('Fy error (N)')
        ax.set_ylabel('Count')
        ax.set_title(f'Rear Fy Error - Static Fz (RMS={np.sqrt(np.mean(err_r_static**2)):.0f} N)')
        ax.legend()

        plt.tight_layout()
        plt.savefig(save_dir / "task5_error_histogram.png", dpi=150)
        print(f"  Saved: {save_dir / 'task5_error_histogram.png'}")
        plt.close()

        print(f"\n  All plots saved to {save_dir}/")

    except ImportError:
        print("  matplotlib not available — skipping plots")


if __name__ == "__main__":
    main()
