#!/usr/bin/env python3
"""Replay several operating points and trace the running posterior weights."""

import numpy as np
import sys, math
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

# Build a sequence of (steer, t) snapshots that emulate the dirt sinusoid
np.random.seed(0)
T = 12.0
dt_obs = 0.1
n_steps = int(T / dt_obs)
M = est._M
Izz = est._Izz
Lf = est._Lf
Lr = est._Lr
L = Lf + Lr

# True NN underpredicts truth by ~2x; emulate with a known scale gap.
# Use the NN at n=0.7 as ground truth, scaled by 2.0.
def true_axle_forces(alpha_f, alpha_r, u, Fz_f, Fz_r, n_true=0.7):
    _, fy_f = est._nn_FxFy(0.05, alpha_f, u, Fz_f, 0.0, n_true)
    _, fy_r = est._nn_FxFy(0.05, alpha_r, u, Fz_r, 0.0, n_true)
    Fy_f = -2.0 * fy_f * 2.0  # 2x scale-up to mimic vehicle sim measurement
    Fy_r = -2.0 * fy_r * 2.0
    return Fy_f, Fy_r

u = 8.5
v = 0.0
omega = 0.0
for k in range(n_steps):
    t = k * dt_obs
    delta = 0.10 * math.sin(2 * math.pi * t / 3.0)
    alpha_f = -math.atan2(v + Lf * omega, u) + delta
    alpha_r = -math.atan2(v - Lr * omega, u)
    Fy_f, Fy_r = true_axle_forces(alpha_f, alpha_r, u, 6500.0, 6000.0)
    ay = (Fy_f + Fy_r) / M
    wd = (Lf * Fy_f - Lr * Fy_r) / Izz
    # Update v and omega via simple integration (won't track perfectly but ok)
    v += dt_obs * (ay - u * omega)
    omega += dt_obs * wd

    wheel_ops = {
        "front_left_long_slip": 0.05, "front_right_long_slip": 0.05,
        "rear_left_long_slip": 0.05, "rear_right_long_slip": 0.05,
        "front_left_slip_angle": alpha_f, "front_right_slip_angle": alpha_f,
        "rear_left_slip_angle": alpha_r, "rear_right_slip_angle": alpha_r,
        "front_left_Fz": 6500.0, "front_right_Fz": 6500.0,
        "rear_left_Fz": 6000.0, "rear_right_Fz": 6000.0,
    }
    if abs(ay) >= 0.3:
        est.observe(
            kappa=0.05, alpha_f=alpha_f, alpha_r=alpha_r,
            u=u, Fz_f=6500.0, Fz_r=6000.0, sr=0.0,
            ay_imu=ay, omega_dot=wd, omega=omega, v_lateral=v,
            kappa_f=0.05, kappa_r=0.05, wheel_ops=wheel_ops,
        )
    if k % 10 == 9:
        # Inspect posterior
        agg = est._n_prior_logw.copy()
        for ll in est._n_loglik_hist:
            agg += ll
        from terrain_parameter_estimator import _logsumexp
        logw = agg - _logsumexp(agg)
        w = np.exp(logw)
        n_post = float(np.sum(w * est._n_grid))
        n_argmax = est._n_grid[int(np.argmax(w))]
        print(f"t={t:5.2f} hist_len={len(est._n_loglik_hist):3d} "
              f"n_post={n_post:.3f} n_argmax={n_argmax:.3f} "
              f"w[0.5]={w[int(np.argmin(np.abs(est._n_grid-0.5)))]:.3f} "
              f"w[0.7]={w[int(np.argmin(np.abs(est._n_grid-0.7)))]:.3f} "
              f"w[1.1]={w[-1]:.3f}")
