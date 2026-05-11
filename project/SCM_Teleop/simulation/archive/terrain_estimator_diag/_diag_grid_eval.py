#!/usr/bin/env python3
"""Replay a single observe() step and print the grid evaluation: y_grid for
each n_val, the inferred y_obs from sensors, and the per-n quadratic residuals.
"""

import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from terrain_parameter_estimator import TerrainParameterEstimator
from param_consistency import terrain_preset_to_internal, get_terrain_preset

model_dir = Path(__file__).parent.parent / "nn_models" / "paper_v2_mlp_16_4"
init_terrain = terrain_preset_to_internal(get_terrain_preset("clay"))
est = TerrainParameterEstimator(
    model_dir=str(model_dir),
    initial_terrain=init_terrain,
    use_measured_tire_ops=True,
    update_interval=10,
)

# Simulate operating conditions taken from the dirt run debug logs (turn).
# v_lat ≈ 0.20, omega ≈ 0.21, ay_meas ≈ 1.61, wd_meas ≈ 0.18 m/s², u ≈ 8.8
# delta typical ≈ 0.08 rad on dirt
u_meas = 8.5
v_lat = 0.20
omega = 0.21
ay_imu = 1.61
omega_dot = 0.18
delta = 0.10
Fz_f = 6500.0
Fz_r = 6000.0
alpha_f = -np.arctan2(v_lat + 1.593 * omega, u_meas) + delta
alpha_r = -np.arctan2(v_lat - 1.709 * omega, u_meas)
print(f"alpha_f={alpha_f:.4f}, alpha_r={alpha_r:.4f}")

# Synthesize wheel ops dict that openloop populates
wheel_ops = {
    "front_left_long_slip": 0.05,
    "front_right_long_slip": 0.05,
    "rear_left_long_slip": 0.05,
    "rear_right_long_slip": 0.05,
    "front_left_slip_angle": alpha_f,
    "front_right_slip_angle": alpha_f,
    "rear_left_slip_angle": alpha_r,
    "rear_right_slip_angle": alpha_r,
    "front_left_Fz": Fz_f,
    "front_right_Fz": Fz_f,
    "rear_left_Fz": Fz_r,
    "rear_right_Fz": Fz_r,
}

# Run observe to set internal state
est.observe(
    kappa=0.05, alpha_f=alpha_f, alpha_r=alpha_r,
    u=u_meas, Fz_f=Fz_f, Fz_r=Fz_r, sr=0.0,
    ay_imu=ay_imu, omega_dot=omega_dot,
    omega=omega, v_lateral=v_lat,
    kappa_f=0.05, kappa_r=0.05,
    wheel_ops=wheel_ops,
)

# Now manually call grid evaluation
y_obs = est._observed_axle_forces(ay_imu, omega_dot)
print(f"y_obs (axle forces) = {y_obs}")
M = est._M
print(f"  Fy_f_obs = {y_obs[0]:.1f} N => ay_f contribution = {y_obs[0]/M:.3f} m/s²")
print(f"  Fy_r_obs = {y_obs[1]:.1f} N => ay_r contribution = {y_obs[1]/M:.3f} m/s²")
print(f"  Total ay = {(y_obs[0]+y_obs[1])/M:.3f}")

print("\nGrid evaluation:")
print(f"{'n':>5} {'Fy_f':>10} {'Fy_r':>10} {'ay_pred':>10} {'wd_pred':>10} {'|innov|':>10}")
Rinv = np.linalg.inv(est._R_axle)
for n_val in est._n_grid:
    state_i = np.array([v_lat, omega, n_val])
    y_pred = est._axle_forces(state_i, u_meas)
    innov = y_obs - np.array(y_pred)
    quad = float(innov @ Rinv @ innov)
    ay = (y_pred[0] + y_pred[1]) / M
    wd = (1.593 * y_pred[0] - 1.709 * y_pred[1]) / 3570.0
    print(f"{n_val:5.2f} {y_pred[0]:10.1f} {y_pred[1]:10.1f} {ay:10.3f} {wd:10.3f} {quad:10.2f}")
