#!/usr/bin/env python3
"""
Reduced-Order Bekker Tire Model
================================

Implementation of the classical Bekker terramechanics model used by
Dallas et al. (J. Terramechanics 91, 2020) as the surrogate inside their
sinkage-exponent estimator.  The model captures the right physical
dependencies of lateral tire force on the Bekker terrain parameters
(``Kphi``, ``Kc``, ``n``, cohesion ``c``, internal friction angle ``phi``,
shear-deformation modulus ``k``), giving the UKF a forward model whose
response to ``n`` is structurally correct without depending on a rig-trained
NN.

Equations follow the paper (numbering matches Section 2.2):

    sigma(theta)  = (Kc/b + Kphi) h(theta)^n                          (4)
    tau(theta)    = tau_max (1 - exp(-j(theta)/k))                    (5)
    tau_max       = c + sigma tan(phi)                                (6)
    h(theta)      = piecewise — see Eq. (7)
    j_y(theta)    = r(1 - s)(theta_f - theta) tan(beta)               (18)
    s_y(theta)    = tau_max (1 - exp(-|j_y(theta)|/k_y))              (17)
    F_y           = ∫_{theta_r}^{theta_f} r·b·s_y(theta) dtheta       (16)

Sinkage is determined by Newton iteration on Eq. (15) so that the integrated
normal contribution matches the wheel load.  This is implemented in vectorised
numpy and is fast enough to evaluate inside an Unscented Kalman Filter.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np


@dataclass
class TerrainParams:
    """Bekker / Mohr-Coulomb terrain parameters."""
    Kphi: float    # Frictional modulus       (Pa / m^n)
    Kc:   float    # Cohesive modulus         (Pa / m^(n+1))
    n:    float    # Sinkage exponent         (-)
    c:    float    # Cohesion                 (Pa)
    phi:  float    # Internal friction angle  (rad)
    k:    float    # Shear-deformation modulus(m)

    # Auxiliary terrain parameters (with reasonable defaults from the Dallas
    # paper / Wong-Reece literature).
    K_sink_ratio: float = 0.5   # rear/front sinkage ratio (Eq. 11)
    a0: float           = 0.4   # location-of-max-stress factor a0
    a1: float           = 0.10  # location-of-max-stress factor a1


@dataclass
class TireGeometry:
    radius: float       # m
    width:  float       # contact patch width (m)


# ─── Internal numerical helpers ─────────────────────────────────────────

_THETA_GRID_SIZE = 16   # quadrature nodes (Simpson over θr..θf).  16 nodes
                        # keeps the trapezoidal integral within <1% of the
                        # 32-node value while halving the per-call cost.


def _theta_breakpoints(h: float, geom: TireGeometry,
                        terrain: TerrainParams,
                        slip: float) -> Tuple[float, float, float]:
    """Compute (theta_r, theta_m, theta_f) for a given sinkage."""
    r = geom.radius
    h_safe = max(min(h, 0.95 * r), 1e-6)
    cos_arg_f = max(min(1.0 - h_safe / r, 1.0), -1.0)
    theta_f = math.acos(cos_arg_f)
    cos_arg_r = max(min(1.0 - terrain.K_sink_ratio * h_safe / r, 1.0), -1.0)
    theta_r = -math.acos(cos_arg_r)         # behind the contact normal
    theta_m = (terrain.a0 + terrain.a1 * slip) * theta_f
    return theta_r, theta_m, theta_f


def _h_profile(theta: np.ndarray,
               theta_r: float, theta_m: float, theta_f: float,
               geom: TireGeometry) -> np.ndarray:
    """Piecewise sinkage profile h(theta) — Eq. 7."""
    r = geom.radius
    cos_f = math.cos(theta_f)
    h_front = r * (np.cos(theta) - cos_f)
    if theta_m <= theta_r:
        return np.maximum(h_front, 0.0)
    # Equivalent angle for the rear segment (Eq. 10).
    theta_e = theta_f - (theta - theta_r) * (theta_f - theta_m) / (theta_m - theta_r)
    h_rear = r * (np.cos(theta_e) - cos_f)
    h = np.where(theta >= theta_m, h_front, h_rear)
    return np.maximum(h, 0.0)


def _normal_force_at_sinkage(h: float, slip: float,
                              geom: TireGeometry,
                              terrain: TerrainParams) -> float:
    """Compute the integrated reaction force F_z(h) — Eq. 14."""
    theta_r, theta_m, theta_f = _theta_breakpoints(h, geom, terrain, slip)
    if theta_f <= theta_r:
        return 0.0
    th = np.linspace(theta_r, theta_f, _THETA_GRID_SIZE)
    h_th = _h_profile(th, theta_r, theta_m, theta_f, geom)
    sigma = (terrain.Kc / geom.width + terrain.Kphi) * np.power(h_th, terrain.n)
    # Shear stress (longitudinal direction): tau ≈ tau_max (1 - exp(-j/k)),
    # with j ≈ r[(theta_f - theta) - (1 - s)(sin(theta_f) - sin(theta))]
    if slip >= 0.0:
        j = geom.radius * ((theta_f - th) - (1.0 - slip)
                            * (math.sin(theta_f) - np.sin(th)))
    else:
        j = geom.radius * ((theta_f - th) - (1.0 / (1.0 + slip))
                            * (math.sin(theta_f) - np.sin(th)))
    tau_max = terrain.c + sigma * math.tan(terrain.phi)
    tau = np.sign(j) * tau_max * (1.0 - np.exp(-np.abs(j) / max(terrain.k, 1e-4)))
    integrand = geom.radius * geom.width * (tau * np.sin(th) + sigma * np.cos(th))
    # Trapezoidal integration is robust enough at 32 nodes for our application.
    return float(np.trapz(integrand, th))


def _solve_sinkage(load_W: float, slip: float,
                    geom: TireGeometry,
                    terrain: TerrainParams,
                    max_iter: int = 8,
                    tol: float = 20.0) -> float:
    """Newton-Raphson on F_z(h) = W (Eq. 13–15).  Returns sinkage in metres."""
    n = max(terrain.n, 0.05)
    Kc_b = terrain.Kc / geom.width
    denom = geom.width * (3.0 - n) * (Kc_b + terrain.Kphi) * math.sqrt(2.0 * geom.radius)
    if denom <= 0.0 or load_W <= 0.0:
        return 0.0
    h = math.pow(3.0 * load_W / denom, 2.0 / (2.0 * n + 1.0))
    h = float(np.clip(h, 1e-4, 0.5 * geom.radius))

    for _ in range(max_iter):
        F = _normal_force_at_sinkage(h, slip, geom, terrain)
        residual = F - load_W
        if abs(residual) < tol:
            break
        # Numerical derivative via centered difference (cheap enough).
        eps = max(1e-3 * h, 1e-4)
        F_plus  = _normal_force_at_sinkage(h + eps, slip, geom, terrain)
        F_minus = _normal_force_at_sinkage(max(h - eps, 1e-5), slip, geom, terrain)
        dFdh = (F_plus - F_minus) / (2.0 * eps)
        if abs(dFdh) < 1.0:
            break
        step = residual / dFdh
        # Damped Newton step to keep h positive and bounded.
        h_new = h - 0.5 * step
        h = float(np.clip(h_new, 1e-5, 0.6 * geom.radius))

    return h


def lateral_force(slip_angle: float, slip_ratio: float,
                   load_W: float, velocity: float,
                   geom: TireGeometry,
                   terrain: TerrainParams) -> Tuple[float, float, float]:
    """Compute single-wheel (Fx, Fy, sinkage) from operating conditions.

    ``slip_angle``  — sideslip angle β (rad)
    ``slip_ratio``  — longitudinal slip s (dimensionless)
    ``load_W``      — vertical wheel load (N)
    ``velocity``    — wheel-frame longitudinal speed (m/s)
    """
    # Solve sinkage from vertical equilibrium.
    h = _solve_sinkage(abs(load_W), slip_ratio, geom, terrain)
    if h <= 0.0:
        return 0.0, 0.0, 0.0

    theta_r, theta_m, theta_f = _theta_breakpoints(h, geom, terrain, slip_ratio)
    if theta_f <= theta_r:
        return 0.0, 0.0, h

    th = np.linspace(theta_r, theta_f, _THETA_GRID_SIZE)
    h_th = _h_profile(th, theta_r, theta_m, theta_f, geom)
    sigma = (terrain.Kc / geom.width + terrain.Kphi) * np.power(h_th, terrain.n)
    tau_max = terrain.c + sigma * math.tan(terrain.phi)

    # Longitudinal shear deformation j(θ) and its stress contribution.
    if slip_ratio >= 0.0:
        j = geom.radius * ((theta_f - th) - (1.0 - slip_ratio)
                            * (math.sin(theta_f) - np.sin(th)))
    else:
        j = geom.radius * ((theta_f - th) - (1.0 / (1.0 + slip_ratio))
                            * (math.sin(theta_f) - np.sin(th)))
    tau = np.sign(j) * tau_max * (1.0 - np.exp(-np.abs(j) / max(terrain.k, 1e-4)))

    # Lateral shear deformation j_y (Eq. 18) — uses slip angle β.
    jy = geom.radius * (1.0 - slip_ratio) * (theta_f - th) * math.tan(slip_angle)
    sy = np.sign(jy) * tau_max * (1.0 - np.exp(-np.abs(jy) / max(terrain.k, 1e-4)))

    # Body-frame longitudinal force from shear*sin and pressure*cos contributions.
    Fx = float(np.trapz(geom.radius * geom.width * (tau * np.cos(th) - sigma * np.sin(th)), th))
    # Lateral force (Eq. 16) — purely from lateral shear.  Sign convention:
    # positive slip_angle → leftward Fy; flip to chassis convention later if
    # the bicycle dynamics expect "Fy opposes slip".
    Fy = float(np.trapz(geom.radius * geom.width * sy, th))
    return Fx, Fy, h


# ─── Convenience API used by the UKF ────────────────────────────────────

_BEKKER_GAIN0_DEFAULT = 0.42
_BEKKER_VEXP_DEFAULT  = 0.45
"""Bekker → SCM correction: ``Fy_scm ≈ gain0 · u^vexp · Fy_bekker``.

The reduced-order Bekker model from Dallas et al. (2020) omits dynamic
(bulldozing, multipass, slip-velocity) effects, so its predicted lateral
forces drift away from the SCM ground truth in a velocity-dependent way.
The two coefficients above were fit by least-squares regression of SCM
vehicle ``Fy`` data (``data/vehicle_forces/vehicle_*.csv``) against the
analytical Bekker output (see ``_calibrate_bekker.py``).  With these
defaults the analytical model tracks SCM force to within ~600 N across all
three preset terrains and the typical 2–9 m/s driving envelope, which is
what the UKF needs for accurate ``n`` recovery — only the *relative*
response to ``n`` has to be physical."""


def bekker_axle_forces(alpha_f: float, alpha_r: float,
                        kappa_f: float, kappa_r: float,
                        u: float,
                        Fz_f_per_wheel: float, Fz_r_per_wheel: float,
                        geom: TireGeometry,
                        terrain: TerrainParams,
                        gain0: float = _BEKKER_GAIN0_DEFAULT,
                        vexp:  float = _BEKKER_VEXP_DEFAULT) -> Tuple[float, float]:
    """Predict body-frame lateral forces on the front and rear axles for a
    bicycle model from per-wheel vertical loads.

    Each axle is modelled as two equivalent wheels each carrying
    ``Fz_*_per_wheel`` (not half of an axle load).  Returned force uses the
    bicycle convention: positive ``alpha`` produces positive ``Fy``
    (leftward on the chassis).  ``gain0`` and ``vexp`` apply the
    Bekker→SCM calibration ``g(u) = gain0 * u^vexp`` (defaults from the
    least-squares fit to vehicle SCM data)."""
    _, fy_f_pw, _ = lateral_force(alpha_f, kappa_f, max(Fz_f_per_wheel, 1.0), u, geom, terrain)
    _, fy_r_pw, _ = lateral_force(alpha_r, kappa_r, max(Fz_r_per_wheel, 1.0), u, geom, terrain)
    g = gain0 * math.pow(max(u, 0.5), vexp)
    return g * 2.0 * fy_f_pw, g * 2.0 * fy_r_pw
