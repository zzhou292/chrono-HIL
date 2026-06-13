#!/usr/bin/env python3
"""Terrain-aware, state-aware, kinematically-feasible speed-profile generation.

Replaces the static curvature-only speed reference (a geometric heuristic that is
blind to live terrain grip, the current state, and the longitudinal/lateral
coupling) with a proper quasi-steady-state time-optimal profile over a
friction-circle (g-g) budget:

    maximise speed along the path  s.t.
      (ax / ax_max)^2 + (ay / ay_max)^2 <= 1     (combined-grip g-g circle)
      ay = v^2 * kappa(s)                         (lateral demand in a corner)
      v(s0) = u0                                  (state-aware start)
      v <= v_cap                                  (driver/cruise intent ceiling)

The grip limits ax_max, ay_max, ax_brake are supplied LIVE from the learned
tyre surrogate at the current estimate n_hat (and per-axle vertical load), so on
soft soil the profile automatically asks for less speed -- it never commands a
speed the terrain cannot support, which is the failure mode of the curvature
heuristic. A forward pass enforces the available tractive acceleration and a
backward pass enforces braking, both inside the g-g circle, so the resulting
profile is feasible for the vehicle to follow.

This is a decoupled trajectory-generation layer: it produces v_ref(s) that the
tracking NMPC then follows, the standard architecture in time-optimal driving
(speed-profile optimisation + tracking control). Pure NumPy, no Chrono/CasADi
dependency, so it is cheap enough to recompute every control step.
"""
from __future__ import annotations

import math
import numpy as np


def _wrap(a: np.ndarray) -> np.ndarray:
    """Wrap angles to (-pi, pi]."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def horizon_curvature(x_ref: np.ndarray, y_ref: np.ndarray,
                      psi_ref: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Per-segment arc length ds and signed curvature kappa over a horizon.

    kappa_k = dpsi_k / ds_k (rad/m). Returned arrays have length len-1 (segments),
    aligned so kappa[k] is the curvature leaving node k.
    """
    x_ref = np.asarray(x_ref, dtype=float)
    y_ref = np.asarray(y_ref, dtype=float)
    psi_ref = np.asarray(psi_ref, dtype=float)
    dx = np.diff(x_ref)
    dy = np.diff(y_ref)
    ds = np.hypot(dx, dy)
    ds = np.maximum(ds, 1e-3)
    dpsi = _wrap(np.diff(psi_ref))
    kappa = dpsi / ds
    return ds, kappa


def gg_speed_profile(
    x_ref: np.ndarray,
    y_ref: np.ndarray,
    psi_ref: np.ndarray,
    u0: float,
    *,
    ay_max: float,
    ax_accel: float,
    ax_brake: float,
    v_cap: float,
    v_floor: float = 0.5,
) -> np.ndarray:
    """Quasi-steady-state time-optimal speed profile over a reference horizon.

    Args:
        x_ref, y_ref, psi_ref: horizon reference nodes (length N+1).
        u0: current measured forward speed (state-aware start), m/s.
        ay_max: max lateral acceleration the terrain supports, m/s^2 (>0).
        ax_accel: max tractive (forward) acceleration, m/s^2 (>0).
        ax_brake: max braking deceleration magnitude, m/s^2 (>0).
        v_cap: upper speed ceiling (driver/cruise intent), m/s.
        v_floor: never command below this (keeps the vehicle moving), m/s.

    Returns:
        v_ref array (length N+1) -- feasible, terrain/curvature-aware speeds.
    """
    n_nodes = len(x_ref)
    if n_nodes < 2:
        return np.full(n_nodes, max(v_floor, min(u0, v_cap)), dtype=float)

    ay_max = max(float(ay_max), 0.2)
    ax_accel = max(float(ax_accel), 0.05)
    ax_brake = max(float(ax_brake), 0.1)
    v_cap = max(float(v_cap), v_floor)

    ds, kappa = horizon_curvature(x_ref, y_ref, psi_ref)  # length N
    abs_k = np.abs(kappa)

    # 1) Lateral-grip cap per node: v_lat = sqrt(ay_max / |kappa|), capped at v_cap.
    #    Node curvature = max of the adjacent segment curvatures (conservative).
    seg_v_lat = np.sqrt(ay_max / np.maximum(abs_k, 1e-6))
    v_lat = np.empty(n_nodes)
    v_lat[0] = seg_v_lat[0]
    v_lat[-1] = seg_v_lat[-1]
    v_lat[1:-1] = np.minimum(seg_v_lat[:-1], seg_v_lat[1:])
    v_lat = np.minimum(v_lat, v_cap)

    # node curvature for the g-g available-accel term
    k_node = np.empty(n_nodes)
    k_node[0] = abs_k[0]
    k_node[-1] = abs_k[-1]
    k_node[1:-1] = np.maximum(abs_k[:-1], abs_k[1:])

    def gg_long(v, k, a_long_max):
        """Longitudinal accel available inside the g-g circle at speed v, curvature k."""
        ay = (v * v) * k
        frac = 1.0 - (ay / ay_max) ** 2
        return a_long_max * math.sqrt(frac) if frac > 0.0 else 0.0

    v = v_lat.copy()
    # 2) Forward pass: enforce tractive-acceleration feasibility ALONG THE PATH.
    #    This is a position-indexed profile: it starts from the grip cap at the
    #    current position (v_lat[0]), NOT the vehicle's current speed u0 -- the
    #    tracking NMPC reconciles the actual u0 toward this profile. (Clamping
    #    v[0]=u0 anchors the whole reference to the current slow speed and kills
    #    the acceleration intent, which over-conservatively stalls the vehicle.)
    for k in range(n_nodes - 1):
        a = gg_long(v[k], k_node[k], ax_accel)
        v_next = math.sqrt(max(v[k] * v[k] + 2.0 * a * ds[k], 0.0))
        v[k + 1] = min(v[k + 1], v_next)

    # 3) Backward pass: limit by braking deceleration into upcoming slow points.
    for k in range(n_nodes - 2, -1, -1):
        a = gg_long(v[k + 1], k_node[k + 1], ax_brake)
        v_prev = math.sqrt(max(v[k + 1] * v[k + 1] + 2.0 * a * ds[k], 0.0))
        v[k] = min(v[k], v_prev)

    return np.clip(v, v_floor, v_cap)


def terrain_grip_limits(
    nn_tire,
    *,
    n_terrain: float,
    terrain_params: dict,
    Fz_front_axle: float,
    Fz_rear_axle: float,
    u: float,
    mass: float,
    alpha_peak: float = 0.30,
    kappa_peak: float = 0.15,
    ax_actuator_max: float = 1.9,
    ax_actuator_min: float = -2.6,
    mu_floor: float = 0.12,
) -> tuple[float, float, float]:
    """Live terrain grip limits (ay_max, ax_accel, ax_brake) from the surrogate.

    Queries the learned tyre model at a moderate peak slip for the available
    lateral force (-> ay_max) and longitudinal force (-> ax_accel/brake) at the
    current n_hat and per-axle load. Falls back to a Coulomb mu*g floor if the
    surrogate is unavailable. All accelerations are clamped to the actuator box.
    """
    g = 9.81
    if nn_tire is None:
        mu = max(math.tan(math.radians(float(terrain_params.get("phi", 20.0)))), mu_floor)
        ay = mu * g
        return ay, min(ax_actuator_max, ay), min(abs(ax_actuator_min), ay)

    u_eval = float(max(abs(u), 0.5))
    Fz_f = float(max(Fz_front_axle / 2.0, 100.0))
    Fz_r = float(max(Fz_rear_axle / 2.0, 100.0))
    try:
        # lateral: |Fy| at peak slip angle, both axles (2 wheels each)
        _, Fy_f = nn_tire.predict_numeric(alpha=alpha_peak, Fz=Fz_f, u=u_eval, kappa=0.0,
                                          n_terrain=n_terrain, steering_rate=0.0,
                                          terrain_params=terrain_params)
        _, Fy_r = nn_tire.predict_numeric(alpha=alpha_peak, Fz=Fz_r, u=u_eval, kappa=0.0,
                                          n_terrain=n_terrain, steering_rate=0.0,
                                          terrain_params=terrain_params)
        Fy_total = 2.0 * (abs(float(Fy_f)) + abs(float(Fy_r)))
        # longitudinal traction: +Fx at peak drive slip
        Fx_dr_f, _ = nn_tire.predict_numeric(alpha=0.0, Fz=Fz_f, u=u_eval, kappa=kappa_peak,
                                             n_terrain=n_terrain, steering_rate=0.0,
                                             terrain_params=terrain_params)
        Fx_dr_r, _ = nn_tire.predict_numeric(alpha=0.0, Fz=Fz_r, u=u_eval, kappa=kappa_peak,
                                             n_terrain=n_terrain, steering_rate=0.0,
                                             terrain_params=terrain_params)
        Fx_drive = 2.0 * (max(float(Fx_dr_f), 0.0) + max(float(Fx_dr_r), 0.0))
        # braking: |Fx| at peak brake slip
        Fx_br_f, _ = nn_tire.predict_numeric(alpha=0.0, Fz=Fz_f, u=u_eval, kappa=-kappa_peak,
                                             n_terrain=n_terrain, steering_rate=0.0,
                                             terrain_params=terrain_params)
        Fx_br_r, _ = nn_tire.predict_numeric(alpha=0.0, Fz=Fz_r, u=u_eval, kappa=-kappa_peak,
                                             n_terrain=n_terrain, steering_rate=0.0,
                                             terrain_params=terrain_params)
        Fx_brake = 2.0 * (abs(float(Fx_br_f)) + abs(float(Fx_br_r)))
    except Exception:
        mu = max(math.tan(math.radians(float(terrain_params.get("phi", 20.0)))), mu_floor)
        ay = mu * g
        return ay, min(ax_actuator_max, ay), min(abs(ax_actuator_min), ay)

    m = float(max(mass, 1.0))
    ay_max = max(Fy_total / m, mu_floor * g)
    ax_accel = min(ax_actuator_max, max(Fx_drive / m, 0.1))
    ax_brake = min(abs(ax_actuator_min), max(Fx_brake / m, 0.3))
    return float(ay_max), float(ax_accel), float(ax_brake)
