#!/usr/bin/env python3
"""Reproduce two UKF terrain-estimator papers.

  (1) Buzhardt & Tallapragada (ACC 2022): parameter-only UKF on a
      5-DOF bicycle + half-car model. Reproduces their Fig 5 / Table I
      across six (soil × steering) scenarios.

  (2) Dallas et al. (arXiv 1908.00130 v2 / IEEE TVT 2021):
      state-augmented UKF on a 3-DOF bicycle model. Reproduces their
      Fig 8 / Table VII for clay and sandy-loam.

Both validators share the same Bekker-Wong terramechanics core and
the same vehicle-dynamics simulator (which always runs the full
half-car model as ground truth — the UKFs differ only in their
estimation model and observation channels).

* longitudinal / lateral / yaw : bicycle (Eqs 1-3 of the paper)
* vertical / pitch              : half-car (Eqs 6-7 of the paper)

The tyre-terrain interaction follows the Bekker-Wong terramechanics
model with sinkage h, contact angles (ϑ_r, ϑ_f) and stress
distributions σ, τ_x, τ_y (Eqs 8-20 of the paper).

The UKF formulation is the parameter-only variant of
Wan & Van der Merwe (Eqs 22-31). State = [n], measurements =
[a_x, a_y, a_z, ω_y, ω_z] with the sensor noise floors from the paper.

The script runs the six scenarios of Fig. 5
(clay/sand × {zero, fast-osc, slow-osc} steering, H_0 ∈ {0.01, 0.05} m),
saves a 2-row × 3-col figure to ``my_paper/paper_figures/ukf_validation.png``,
and prints the per-scenario mean-square error against Table I.

Run::

    python deliverables/ukf_paper_validation.py
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


# =============================================================================
# Bekker-Wong wheel-terrain interaction (paper eqs 8-20)
# =============================================================================

@dataclass(frozen=True)
class SoilParams:
    """Bekker-Mohr soil parameters."""
    kc: float          # Pa / m^(n-1)
    kphi: float        # Pa / m^n
    n: float           # sinkage exponent  (the unknown the UKF estimates)
    c: float           # cohesion, Pa
    phi: float         # friction angle, rad
    kx: float          # Janosi shear modulus, longitudinal, m
    ky: float          # Janosi shear modulus, lateral, m
    a0: float = 0.4    # ϑ_m = (a0 + a1*s) * ϑ_f
    a1: float = 0.0
    b0: float = 0.0    # ϑ_r = (b0 + b1*s) * ϑ_f
    b1: float = 0.0


# Paper / Wong & Reece values. Clay matches the SCM_Final clay preset;
# sand has n=0.8 instead of the 1.1 preset to match Buzhardt's choice.
SOIL_CLAY = SoilParams(kc=13.2e3, kphi=692.2e3, n=0.5,
                       c=4.14e3, phi=math.radians(13.0),
                       kx=0.025, ky=0.025)
SOIL_SAND = SoilParams(kc=0.9e3,  kphi=1523.4e3, n=0.8,
                       c=1.04e3, phi=math.radians(28.0),
                       kx=0.025, ky=0.025)


_MANIFOLD_PRESETS = None


def manifold_soil_from_n(n_val: float) -> "SoilParams":
    """Interpolate the *full* Bekker-Mohr soil vector along the canonical
    clay->dirt->sand preset manifold as a function of n.

    Without this the UKF freezes five of six soil parameters at the start
    preset and only estimates n, so when the true soil moves off that preset
    (e.g. across a clay->sand transition, or simply soft vs firm) the Fy
    surrogate is queried with an inconsistent soil vector and the n estimate
    is biased. Mapping all six parameters from n along the preset manifold
    matches how the deployed window-MLP reconstructs soil from its n output
    (``learned_terrain_estimator._terrain_params_for_n``).
    """
    global _MANIFOLD_PRESETS
    if _MANIFOLD_PRESETS is None:
        import sys
        sim_dir = Path(__file__).resolve().parents[1] / "simulation"
        if str(sim_dir) not in sys.path:
            sys.path.insert(0, str(sim_dir))
        from param_consistency import TERRAIN_PRESETS
        _MANIFOLD_PRESETS = sorted(
            ((float(p["n"]), p) for p in TERRAIN_PRESETS.values()),
            key=lambda kv: kv[0])
    pts = _MANIFOLD_PRESETS
    nv = min(max(float(n_val), pts[0][0]), pts[-1][0])
    for i in range(len(pts) - 1):
        n0, p0 = pts[i]
        n1, p1 = pts[i + 1]
        if nv <= n1:
            w = 0.0 if n1 == n0 else (nv - n0) / (n1 - n0)
            lerp = lambda key: (1.0 - w) * float(p0[key]) + w * float(p1[key])
            return SoilParams(
                kc=lerp("Kc"), kphi=lerp("Kphi"), n=float(n_val),
                c=lerp("cohesion"), phi=math.radians(lerp("friction_angle")),
                kx=lerp("janosi_shear"), ky=lerp("janosi_shear"))
    p = pts[-1][1]
    return SoilParams(kc=float(p["Kc"]), kphi=float(p["Kphi"]), n=float(n_val),
                      c=float(p["cohesion"]),
                      phi=math.radians(float(p["friction_angle"])),
                      kx=float(p["janosi_shear"]), ky=float(p["janosi_shear"]))


@dataclass(frozen=True)
class WheelGeom:
    r: float = 0.35   # rolling radius, m
    b: float = 0.25   # effective width, m


WHEEL = WheelGeom()


def _bekker_wheel_forces(
    *,
    Fz_required: float,
    vl: float,
    vc: float,
    omega: float,
    soil: SoilParams,
    wheel: WheelGeom = WHEEL,
    n_integration: int = 24,
) -> Tuple[float, float, float, float]:
    """Return (Fx, Fy, Fz_achieved, h_f) for one wheel.

    ``Fz_required`` is the dynamic normal reaction the vehicle model
    demands at this wheel; ``h_f`` (max sinkage) is found by Newton-
    Raphson so that the integral of normal stress equals
    ``Fz_required`` (Eq 20 = N in the paper).
    """
    vl = max(abs(vl), 0.3)
    s = max(min((wheel.r * omega - vl) / max(wheel.r * omega, 1e-3), 0.6), 0.005)
    # Some authors take s = 0.0 when omega = vl/r; keep a small floor so
    # the shear law is differentiable.
    beta = math.atan2(vc, vl) if abs(vl) > 1e-3 else 0.0

    a0, a1, b0, b1 = soil.a0, soil.a1, soil.b0, soil.b1

    def _forces_at_hf(hf: float):
        hf = max(min(hf, wheel.r * 0.9), 1e-4)
        theta_f = math.acos(max(min(1.0 - hf / wheel.r, 1.0), -1.0))
        theta_m = (a0 + a1 * s) * theta_f
        theta_r = (b0 + b1 * s) * theta_f
        # Numerical integration ϑ ∈ [ϑ_r, ϑ_f]
        thetas = np.linspace(theta_r, theta_f, n_integration)
        Fx = Fy = Fz = 0.0
        if theta_f <= theta_r + 1e-9:
            return 0.0, 0.0, 0.0
        dtheta = thetas[1] - thetas[0]
        for theta in thetas:
            # Effective sinkage h(ϑ) (Eq 11 of the paper; piecewise)
            if theta >= theta_m:
                h = wheel.r * (math.cos(theta) - math.cos(theta_f))
            else:
                # Eq 15 — equivalent contact angle for the rear branch
                if theta_m > theta_r:
                    frac = (theta - theta_r) / (theta_m - theta_r)
                    theta_e = theta_f - frac * (theta_f - theta_m)
                else:
                    theta_e = theta_f
                h = wheel.r * (math.cos(theta_e) - math.cos(theta_f))
            h = max(h, 0.0)
            sigma = (soil.kc / wheel.b + soil.kphi) * (h ** soil.n)

            jx = wheel.r * ((theta_f - theta) - (1.0 - s) * (math.sin(theta_f) - math.sin(theta)))
            jy = wheel.r * (1.0 - s) * (theta_f - theta) * math.tan(beta)
            tau_x = (soil.c + sigma * math.tan(soil.phi)) * (1.0 - math.exp(-abs(jx) / soil.kx))
            tau_x = math.copysign(tau_x, jx)
            tau_y = (soil.c + sigma * math.tan(soil.phi)) * (1.0 - math.exp(-abs(jy) / soil.ky))
            tau_y = math.copysign(tau_y, jy)

            Fx += wheel.r * wheel.b * (tau_x * math.cos(theta) - sigma * math.sin(theta)) * dtheta
            Fy += -wheel.r * wheel.b * tau_y * dtheta
            Fz += wheel.r * wheel.b * (tau_x * math.sin(theta) + sigma * math.cos(theta)) * dtheta
        return Fx, Fy, Fz

    # Newton-Raphson on h_f so that Fz_predicted == Fz_required.
    hf = 0.03
    for _ in range(15):
        Fx, Fy, Fz_pred = _forces_at_hf(hf)
        err = Fz_pred - Fz_required
        if abs(err) < 50.0:
            break
        # finite-difference derivative
        Fx_p, Fy_p, Fz_p = _forces_at_hf(hf + 1e-4)
        dFz_dh = (Fz_p - Fz_pred) / 1e-4 if Fz_p != Fz_pred else 1e6
        if abs(dFz_dh) < 1e3:
            dFz_dh = math.copysign(1e3, dFz_dh)
        hf = hf - err / dFz_dh
        hf = max(min(hf, wheel.r * 0.5), 1e-4)
    Fx, Fy, Fz_pred = _forces_at_hf(hf)
    return float(Fx), float(Fy), float(Fz_pred), float(hf)


# =============================================================================
# 5-DOF vehicle model (bicycle + half-car) — paper §II-A
# =============================================================================


@dataclass(frozen=True)
class Vehicle:
    # Match Chrono's HMMWV_Full vehicle (chassis ~2086 kg + wheels →
    # total ~2370 kg, ~5.8 kN per wheel). This puts per-wheel Fz inside
    # the NN tyre surrogate's training distribution (the closed-loop
    # LHS data has ``vertical_load`` mean ≈ 6.3 kN per wheel). Earlier
    # iterations used a 5500 kg HMMWV-class number which is 2× the real
    # Chrono HMMWV and pushed the NN ~2× out of distribution.
    m: float = 2370.0      # total mass, kg (Chrono HMMWV_Full)
    # Chrono HMMWV chassis Iz = 3570 kg·m²; adding 4×(70 kg)×(1.65 m)² ≈
    # 760 for the wheel mass gives total ≈ 4330. Lf / Lr taken from
    # Chrono spindle positions (axle 0 at x=+1.59, axle 1 at x=−1.71).
    Iz: float = 4330.0     # yaw inertia, kg·m²
    Iy: float = 3700.0     # pitch inertia, kg·m²
    Lf: float = 1.59       # CG → front axle, m
    Lr: float = 1.71       # CG → rear axle, m
    track: float = 1.819   # left-to-right wheel spacing, m (Chrono HMMWV)
    kf: float = 180000.0   # front suspension stiffness, N/m
    kr: float = 180000.0   # rear  suspension stiffness, N/m
    cf: float =   8000.0   # front suspension damping, N·s/m
    cr: float =   8000.0   # rear  suspension damping, N·s/m
    mw: float =     60.0   # unsprung wheel mass, kg
    g:  float = 9.81


@dataclass(frozen=True)
class TerrainProfile:
    """H(X, Y) = H_0 * sin²(0.5 X) * cos(1.5 Y), Eq 21 of the paper."""
    H0: float = 0.05

    def height(self, X: float, Y: float) -> float:
        return self.H0 * (math.sin(0.5 * X) ** 2) * math.cos(1.5 * Y)

    def height_dot(self, X: float, Y: float, Xdot: float, Ydot: float) -> float:
        # dH/dt = H0 * [ d/dX(sin²(0.5X)) * dX/dt * cos(1.5Y)
        #                + sin²(0.5X) * d/dY(cos(1.5Y)) * dY/dt ]
        dHdX = self.H0 * math.sin(0.5 * X) * math.cos(0.5 * X) * math.cos(1.5 * Y)
        dHdY = -1.5 * self.H0 * (math.sin(0.5 * X) ** 2) * math.sin(1.5 * Y)
        return dHdX * Xdot + dHdY * Ydot


def vehicle_derivatives(
    state: np.ndarray,
    inputs: Tuple[float, float],     # (delta, Fu)
    soil: SoilParams,
    terr: TerrainProfile,
    veh: Vehicle = Vehicle(),
    *,
    half_car: bool = True,
) -> Tuple[np.ndarray, Dict[str, float]]:
    """Return ẋ for the 10-state coupled model and diag observations.

    State layout::

        X, Y, psi, u, v, omega, z, zdot, theta, thetadot
        0  1  2    3  4  5      6  7     8      9

    Inputs: ``delta`` (front steer, rad), ``Fu`` (longitudinal forcing, N).
    """
    X, Y, psi, u, v, omega, z, zdot, theta, thetadot = state
    delta, Fu = inputs

    cospsi, sinpsi = math.cos(psi), math.sin(psi)
    costh,  sinth  = math.cos(theta), math.sin(theta)

    if half_car:
        # Wheel positions (global) and terrain elevation under each wheel.
        Xf = X + veh.Lf * cospsi
        Yf = Y + veh.Lf * sinpsi
        Xr = X - veh.Lr * cospsi
        Yr = Y - veh.Lr * sinpsi
        Xfdot = u * cospsi - v * sinpsi - veh.Lf * omega * sinpsi
        Yfdot = u * sinpsi + v * cospsi + veh.Lf * omega * cospsi
        Xrdot = u * cospsi - v * sinpsi + veh.Lr * omega * sinpsi
        Yrdot = u * sinpsi + v * cospsi - veh.Lr * omega * cospsi

        zfg = terr.height(Xf, Yf)
        zrg = terr.height(Xr, Yr)
        zfg_dot = terr.height_dot(Xf, Yf, Xfdot, Yfdot)
        zrg_dot = terr.height_dot(Xr, Yr, Xrdot, Yrdot)

        # Per-axle vertical translation (z_f, z_r) — Eq just under (7).
        zf = z + veh.Lf * sinth
        zr = z - veh.Lr * sinth
        zf_dot = zdot + veh.Lf * costh * thetadot
        zr_dot = zdot - veh.Lr * costh * thetadot

        # Dynamic normal reaction at each axle (Eq 8 of the paper).
        N_f = 0.5 * veh.m * veh.g - veh.kf * (zf - zfg) - veh.cf * (zf_dot - zfg_dot)
        N_r = 0.5 * veh.m * veh.g - veh.kr * (zr - zrg) - veh.cr * (zr_dot - zrg_dot)
        N_f = max(N_f, 50.0)
        N_r = max(N_r, 50.0)
    else:
        # Bicycle-only: vertical dynamics neglected, static normal load.
        zfg = zrg = 0.0
        zfg_dot = zrg_dot = 0.0
        zf = zr = 0.0
        zf_dot = zr_dot = 0.0
        N_f = 0.5 * veh.m * veh.g
        N_r = 0.5 * veh.m * veh.g

    # Local longitudinal / cornering velocities at each axle (body frame).
    # vl,f = u + 0 (small-angle tyre frame), vc,f = v + Lf*omega - tyre side
    vl_f = max(u, 0.5)
    vc_f = v + veh.Lf * omega
    vl_r = max(u, 0.5)
    vc_r = v - veh.Lr * omega
    # Project into the steered tyre frame for the front axle.
    vl_f_t = vl_f * math.cos(delta) + vc_f * math.sin(delta)
    vc_f_t = -vl_f * math.sin(delta) + vc_f * math.cos(delta)

    # Wheel angular velocity: paper notes it isn't tracked, slip s is held
    # at a small positive value. Use omega_w = vl / r * (1 + s_target).
    s_target = 0.05
    omega_wf = vl_f_t / (WHEEL.r * (1.0 - s_target))
    omega_wr = vl_r   / (WHEEL.r * (1.0 - s_target))

    Fxf_t, Fyf_t, _, _ = _bekker_wheel_forces(
        Fz_required=N_f, vl=vl_f_t, vc=vc_f_t, omega=omega_wf, soil=soil)
    Fxr,   Fyr,   _, _ = _bekker_wheel_forces(
        Fz_required=N_r, vl=vl_r,   vc=vc_r,   omega=omega_wr, soil=soil)

    # Rotate the front-tyre force back into the body frame.
    Fxf = Fxf_t * math.cos(delta) - Fyf_t * math.sin(delta)
    Fyf = Fxf_t * math.sin(delta) + Fyf_t * math.cos(delta)

    # Translational dynamics (Eqs 1-3) with the pitch coupling cos θ
    # already approximated as ≈1 for the small pitch angles we see.
    udot     = (Fxf + Fxr + Fu) / veh.m + v * omega
    vdot     = (Fyf + Fyr)        / veh.m - u * omega
    omegadot = (veh.Lf * Fyf - veh.Lr * Fyr) / veh.Iz

    if half_car:
        # Half-car vertical / pitch dynamics (Eqs 6-7).
        Fz_susp_f = -veh.kf * (zf - zfg) - veh.cf * (zf_dot - zfg_dot)
        Fz_susp_r = -veh.kr * (zr - zrg) - veh.cr * (zr_dot - zrg_dot)
        zddot     = (Fz_susp_f + Fz_susp_r) / veh.m
        thetaddot = (Fz_susp_f * veh.Lf - Fz_susp_r * veh.Lr) / veh.Iy * costh
    else:
        zddot = 0.0
        thetaddot = 0.0

    # Kinematics
    Xdot   = u * cospsi - v * sinpsi
    Ydot   = u * sinpsi + v * cospsi
    psidot = omega

    dstate = np.array([Xdot, Ydot, psidot, udot, vdot, omegadot,
                       zdot, zddot, thetadot, thetaddot], dtype=float)
    diag = dict(ax_body=udot - v * omega,
                ay_body=vdot + u * omega,
                az_body=zddot,
                omega_y=thetadot,
                omega_z=omega)
    return dstate, diag


def integrate_rk4(state, inputs, soil, terr, dt, veh=Vehicle(), *, half_car=True):
    """One classical RK4 step of the 10-state vehicle model."""
    k1, _   = vehicle_derivatives(state,              inputs, soil, terr, veh, half_car=half_car)
    k2, _   = vehicle_derivatives(state + 0.5*dt*k1,  inputs, soil, terr, veh, half_car=half_car)
    k3, _   = vehicle_derivatives(state + 0.5*dt*k2,  inputs, soil, terr, veh, half_car=half_car)
    k4, dgs = vehicle_derivatives(state + dt*k3,      inputs, soil, terr, veh, half_car=half_car)
    return state + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4), dgs


# =============================================================================
# Parameter-only UKF (Wan & Van der Merwe; paper Eqs 22-31)
# =============================================================================


class ParameterUKF:
    """UKF over an L-dimensional parameter vector w."""

    def __init__(self,
                 w0: np.ndarray,
                 P0: np.ndarray,
                 Rn: np.ndarray,   # process / parameter random-walk cov
                 Re: np.ndarray,   # measurement noise cov
                 alpha: float = 0.5,
                 beta:  float = 2.0,
                 kappa: float = 0.0):
        self.w = w0.astype(float).copy()
        self.P = P0.astype(float).copy()
        self.Rn = Rn.astype(float).copy()
        self.Re = Re.astype(float).copy()
        self.L = len(self.w)
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.lmbda = alpha**2 * (self.L + kappa) - self.L
        scale = self.L + self.lmbda
        self.wm = np.full(2 * self.L + 1, 0.5 / scale)
        self.wc = self.wm.copy()
        self.wm[0] = self.lmbda / scale
        self.wc[0] = self.wm[0] + (1.0 - alpha**2 + beta)

    def _sigma_points(self) -> np.ndarray:
        P = 0.5 * (self.P + self.P.T)
        try:
            sqrtP = np.linalg.cholesky((self.L + self.lmbda) * P)
        except np.linalg.LinAlgError:
            vals, vecs = np.linalg.eigh(P)
            vals = np.maximum(vals, 1e-12)
            sqrtP = vecs @ np.diag(np.sqrt((self.L + self.lmbda) * vals))
        sigma = np.empty((2 * self.L + 1, self.L), dtype=float)
        sigma[0] = self.w
        for i in range(self.L):
            sigma[i + 1]            = self.w + sqrtP[:, i]
            sigma[self.L + i + 1]   = self.w - sqrtP[:, i]
        return sigma

    def step(self,
             d_k: np.ndarray,
             h_fn: Callable[[np.ndarray], np.ndarray],
             w_clip: Tuple[float, float] = (0.1, 1.4)) -> None:
        # Predict (random walk on parameters):
        self.P = self.P + self.Rn
        sigma = self._sigma_points()
        sigma = np.clip(sigma, w_clip[0], w_clip[1])
        # Propagate sigma points through the measurement function.
        D = np.asarray([h_fn(w_i) for w_i in sigma])     # (2L+1, m)
        d_hat = (self.wm[:, None] * D).sum(axis=0)
        m = D.shape[1]
        Pd  = self.Re.copy()
        Pwd = np.zeros((self.L, m), dtype=float)
        for i in range(2 * self.L + 1):
            dz = D[i] - d_hat
            dw = sigma[i] - self.w
            Pd  += self.wc[i] * np.outer(dz, dz)
            Pwd += self.wc[i] * np.outer(dw, dz)
        try:
            K = np.linalg.solve(Pd.T, Pwd.T).T
        except np.linalg.LinAlgError:
            K = Pwd @ np.linalg.pinv(Pd)
        self.w = self.w + K @ (d_k - d_hat)
        self.w = np.clip(self.w, w_clip[0], w_clip[1])
        self.P = self.P - K @ Pd @ K.T
        self.P = 0.5 * (self.P + self.P.T)


# =============================================================================
# State-augmented UKF (Dallas et al. 2019 / 2021) — Eqs 26-43 of the paper
# =============================================================================


class StateAugmentedUKF:
    """UKF over [bicycle_state ‖ n] with full-state observations.

    State (Dallas Eq 25, augmented per Eq 26):
        z = [x, y, ψ, u, v, ω, n]  (7-dim).
    Process: bicycle dynamics (Eq 24) with ṅ = 0.
    Measurement: identity on the first 6 states (Dallas Table V).
    """

    def __init__(self,
                 z0: np.ndarray,
                 P0: np.ndarray,
                 Q:  np.ndarray,
                 R:  np.ndarray,
                 alpha: float = 0.5,
                 beta:  float = 2.0,
                 kappa: float = 0.0,
                 n_clip: Tuple[float, float] = (0.2, 1.4)):
        self.z = z0.astype(float).copy()
        self.P = P0.astype(float).copy()
        self.Q = Q.astype(float).copy()
        self.R = R.astype(float).copy()
        self.L = len(self.z)
        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.n_clip = n_clip
        self.lmbda = alpha**2 * (self.L + kappa) - self.L
        s = self.L + self.lmbda
        self.wm = np.full(2 * self.L + 1, 0.5 / s)
        self.wc = self.wm.copy()
        self.wm[0] = self.lmbda / s
        self.wc[0] = self.wm[0] + (1.0 - alpha**2 + beta)

    @staticmethod
    def _wrap(a: float) -> float:
        return float((a + math.pi) % (2.0 * math.pi) - math.pi)

    def _sigma_points(self) -> np.ndarray:
        P = 0.5 * (self.P + self.P.T)
        for jitter in (0.0, 1e-9, 1e-7, 1e-5):
            try:
                sqrtP = np.linalg.cholesky((self.L + self.lmbda) *
                                            (P + jitter * np.eye(self.L)))
                break
            except np.linalg.LinAlgError:
                continue
        else:
            vals, vecs = np.linalg.eigh(P)
            vals = np.maximum(vals, 1e-12)
            sqrtP = vecs @ np.diag(np.sqrt((self.L + self.lmbda) * vals))
        sig = np.empty((2 * self.L + 1, self.L), dtype=float)
        sig[0] = self.z
        for i in range(self.L):
            sig[i + 1]            = self.z + sqrtP[:, i]
            sig[self.L + i + 1]   = self.z - sqrtP[:, i]
        sig[:, 2] = [self._wrap(a) for a in sig[:, 2]]
        sig[:, 6] = np.clip(sig[:, 6], self.n_clip[0], self.n_clip[1])
        return sig

    def step(self,
             y_k: np.ndarray,
             f_dynamics: Callable[[np.ndarray], np.ndarray],
             h_measure:  Callable[[np.ndarray], np.ndarray]) -> None:
        # Time update.
        sig = self._sigma_points()
        sig_pred = np.asarray([f_dynamics(s) for s in sig])
        sig_pred[:, 2] = [self._wrap(a) for a in sig_pred[:, 2]]
        sig_pred[:, 6] = np.clip(sig_pred[:, 6], self.n_clip[0], self.n_clip[1])
        z_pred = (self.wm[:, None] * sig_pred).sum(axis=0)
        z_pred[2] = math.atan2(float(np.sum(self.wm * np.sin(sig_pred[:, 2]))),
                                float(np.sum(self.wm * np.cos(sig_pred[:, 2]))))
        P_pred = self.Q.copy()
        for i in range(2 * self.L + 1):
            dz = sig_pred[i] - z_pred
            dz[2] = self._wrap(dz[2])
            P_pred += self.wc[i] * np.outer(dz, dz)
        P_pred = 0.5 * (P_pred + P_pred.T)

        # Measurement update.
        y_sig = np.asarray([h_measure(s) for s in sig_pred])
        y_hat = (self.wm[:, None] * y_sig).sum(axis=0)
        Pyy = self.R.copy()
        Pzy = np.zeros((self.L, y_sig.shape[1]), dtype=float)
        for i in range(2 * self.L + 1):
            dy = y_sig[i] - y_hat
            dz = sig_pred[i] - z_pred
            dz[2] = self._wrap(dz[2])
            Pyy += self.wc[i] * np.outer(dy, dy)
            Pzy += self.wc[i] * np.outer(dz, dy)
        K = np.linalg.solve(Pyy.T, Pzy.T).T

        self.z = z_pred + K @ (y_k - y_hat)
        self.z[2] = self._wrap(self.z[2])
        self.z[6] = float(np.clip(self.z[6], self.n_clip[0], self.n_clip[1]))
        self.P = P_pred - K @ Pyy @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        # Floor the variances so the filter never deadlocks numerically.
        floor = np.array([0.5, 0.5, 1e-4, 0.01, 0.01, 1e-4, 1e-3])
        if self.L > 7:
            # Extra states (e.g. NN α_F calibration): tight variance
            # floor so the filter can still update them after long runs.
            floor = np.concatenate([floor, np.full(self.L - 7, 1e-4)])
        self.P.flat[:: self.L + 1] = np.maximum(np.diag(self.P), floor)


# =============================================================================
# Helper: measurement function used by the UKF.
# =============================================================================


def make_measurement_function(state: np.ndarray,
                              inputs: Tuple[float, float],
                              soil_template: SoilParams,
                              terr: TerrainProfile,
                              veh: Vehicle = Vehicle(),
                              *,
                              half_car: bool = True) -> Callable[[np.ndarray], np.ndarray]:
    """Return f(w) → predicted observations for one tick.

    Half-car backend: [a_x, a_y, a_z, ω_y, ω_z]  (5 channels).
    Bicycle backend : [a_x, a_y, ω_z]              (3 channels — paper says
                       vertical accel + pitch rate are unmodelled and
                       therefore unobserved when running on the bicycle).
    """
    def _f(w: np.ndarray) -> np.ndarray:
        n_val = float(w[0])
        soil = SoilParams(kc=soil_template.kc, kphi=soil_template.kphi,
                          n=n_val, c=soil_template.c, phi=soil_template.phi,
                          kx=soil_template.kx, ky=soil_template.ky,
                          a0=soil_template.a0, a1=soil_template.a1,
                          b0=soil_template.b0, b1=soil_template.b1)
        _, diag = vehicle_derivatives(state, inputs, soil, terr, veh,
                                       half_car=half_car)
        if half_car:
            return np.array([diag["ax_body"], diag["ay_body"], diag["az_body"],
                             diag["omega_y"], diag["omega_z"]], dtype=float)
        return np.array([diag["ax_body"], diag["ay_body"], diag["omega_z"]],
                        dtype=float)
    return _f


# =============================================================================
# Scenario runner
# =============================================================================


@dataclass(frozen=True)
class Scenario:
    label:    str
    soil:     SoilParams
    n_true:   float
    H0:       float
    Fu_fn:    Callable[[float, float], float]   # (t, mass) → Fu (N)
    delta_fn: Callable[[float], float]          # (t) → steering (rad)
    T:        float = 100.0
    dt:       float = 0.01
    sub_steps: int = 2                          # estimator runs at dt*sub_steps


def _Fu_clay(t: float, m: float) -> float:
    return m * (0.8 + 0.5 * math.sin(0.8 * t))


def _Fu_sand(t: float, m: float) -> float:
    return m * (1.8 + 0.6 * math.sin(0.8 * t))


PAPER_SCENARIOS: List[Scenario] = [
    # (a) clay  H0=0.05  δ=0
    Scenario("(a) clay H0=0.05 δ=0",      SOIL_CLAY, 0.50, 0.05, _Fu_clay,
             lambda t: 0.0),
    # (b) sand  H0=0.05  δ=0
    Scenario("(b) sand H0=0.05 δ=0",      SOIL_SAND, 0.80, 0.05, _Fu_sand,
             lambda t: 0.0),
    # (c) clay  H0=0.05  δ=0.2 sin(t)
    Scenario("(c) clay H0=0.05 δ=0.2sin(t)", SOIL_CLAY, 0.50, 0.05, _Fu_clay,
             lambda t: 0.2 * math.sin(t)),
    # (d) sand  H0=0.05  δ=0.2 sin(t)
    Scenario("(d) sand H0=0.05 δ=0.2sin(t)", SOIL_SAND, 0.80, 0.05, _Fu_sand,
             lambda t: 0.2 * math.sin(t)),
    # (e) clay  H0=0.05  δ=0.5 sin(0.3 t)
    Scenario("(e) clay H0=0.05 δ=0.5sin(0.3t)", SOIL_CLAY, 0.50, 0.05, _Fu_clay,
             lambda t: 0.5 * math.sin(0.3 * t)),
    # (f) sand  H0=0.05  δ=0.5 sin(0.3 t)
    Scenario("(f) sand H0=0.05 δ=0.5sin(0.3t)", SOIL_SAND, 0.80, 0.05, _Fu_sand,
             lambda t: 0.5 * math.sin(0.3 * t)),
]


def run_scenario(sc: Scenario,
                 *,
                 backend: str = "half_car",         # "half_car" | "bicycle"
                 init_n: float = 1.0,
                 alpha: float = 0.25,
                 P0:    float = 0.05,
                 Rn:    float = 5e-7,
                 acc_sigma: float = 0.20,
                 gyro_sigma: float = 0.0175,
                 seed: int = 0) -> Tuple[np.ndarray, np.ndarray, float]:
    """Run one scenario; return (t_arr, n_est_arr, MSE).

    The vehicle simulator (ground truth) always runs the full half-car
    model — that's the "truth" the IMU samples. The UKF measurement
    function uses whichever backend is selected. The bicycle backend
    is structurally mismatched against the half-car truth, which is why
    the paper shows it failing to converge in many cases.
    """
    assert backend in ("half_car", "bicycle")
    half_car = backend == "half_car"
    rng = np.random.default_rng(seed)
    veh = Vehicle()
    terr = TerrainProfile(H0=sc.H0)
    soil_true = SoilParams(kc=sc.soil.kc, kphi=sc.soil.kphi, n=sc.n_true,
                           c=sc.soil.c, phi=sc.soil.phi,
                           kx=sc.soil.kx, ky=sc.soil.ky,
                           a0=sc.soil.a0, a1=sc.soil.a1,
                           b0=sc.soil.b0, b1=sc.soil.b1)

    state = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0], dtype=float)

    if half_car:
        Re = np.diag([acc_sigma**2] * 3 + [gyro_sigma**2] * 2)
    else:
        # bicycle backend: only ax, ay, ωz observed
        Re = np.diag([acc_sigma**2, acc_sigma**2, gyro_sigma**2])

    ukf = ParameterUKF(
        w0=np.array([init_n]),
        P0=np.array([[P0]]),
        Rn=np.array([[Rn]]),
        Re=Re,
        alpha=alpha,
    )

    n_steps = int(round(sc.T / sc.dt))
    t_log  = np.zeros(n_steps + 1)
    n_log  = np.zeros(n_steps + 1)
    n_log[0] = ukf.w[0]

    t = 0.0
    for k in range(1, n_steps + 1):
        delta = sc.delta_fn(t)
        Fu    = sc.Fu_fn(t, veh.m)
        # Ground-truth simulator always runs the full half-car model.
        state, diag = integrate_rk4(state, (delta, Fu), soil_true, terr,
                                    sc.dt, veh, half_car=True)
        t += sc.dt

        if k % sc.sub_steps == 0:
            # Synthetic IMU measurement with Gaussian noise. The bicycle
            # backend only reads three of the five channels.
            if half_car:
                d_true = np.array([diag["ax_body"], diag["ay_body"],
                                   diag["az_body"], diag["omega_y"],
                                   diag["omega_z"]], dtype=float)
                sigmas = np.array([acc_sigma]*3 + [gyro_sigma]*2)
            else:
                d_true = np.array([diag["ax_body"], diag["ay_body"],
                                   diag["omega_z"]], dtype=float)
                sigmas = np.array([acc_sigma, acc_sigma, gyro_sigma])
            d_noisy = d_true + rng.normal(0.0, sigmas)
            h_fn = make_measurement_function(state, (delta, Fu),
                                              sc.soil, terr, veh,
                                              half_car=half_car)
            ukf.step(d_noisy, h_fn)

        t_log[k] = t
        n_log[k] = ukf.w[0]

    # MSE in the tail (last 50 s) — what the paper's Table I reports.
    tail_mask = t_log >= sc.T * 0.5
    mse = float(np.mean((n_log[tail_mask] - sc.n_true) ** 2))
    return t_log, n_log, mse


# =============================================================================
# Driver
# =============================================================================


def main():
    out_dir = Path(__file__).resolve().parents[1] / "my_paper" / "paper_figures"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_png = out_dir / "ukf_validation.png"

    fig, axes = plt.subplots(2, 3, figsize=(13, 7.5), sharex=True, sharey=True)
    axes = axes.flatten()

    print(f"{'scenario':<36} {'MSE bicycle':>14} {'MSE half-car':>14}")
    for ax, sc in zip(axes, PAPER_SCENARIOS):
        t0 = time.time()
        t_h, n_h, mse_h = run_scenario(sc, backend="half_car")
        t_b, n_b, mse_b = run_scenario(sc, backend="bicycle")
        wall = time.time() - t0
        print(f"{sc.label:<36} {mse_b:>14.3e} {mse_h:>14.3e}  ({wall:5.1f}s)")
        ax.plot(t_b, n_b, color="#888888", lw=1.0, alpha=0.85,
                label=f"Bicycle  (MSE={mse_b:.2e})")
        ax.plot(t_h, n_h, color="#1f77b4", lw=1.3,
                label=f"Half-car (MSE={mse_h:.2e})")
        ax.axhline(sc.n_true, color="k", lw=1.0, ls="--",
                   label=f"True n = {sc.n_true:.2f}")
        ax.set_ylim(0.0, 1.25)
        ax.set_xlim(0.0, sc.T)
        ax.set_xlabel("t (s)")
        ax.set_ylabel("n")
        ax.set_title(sc.label, fontsize=10)
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=8, framealpha=0.92)

    fig.suptitle("UKF Bekker-n estimation — half-car vs bicycle backend "
                 "(reproducing Fig. 5 of Buzhardt & Tallapragada ACC 2022)",
                 fontsize=12, y=1.00)
    fig.tight_layout()
    fig.savefig(out_png, dpi=180, bbox_inches="tight")
    print(f"\nWrote {out_png}")


# =============================================================================
# Dallas (2019/2021) validation: state-augmented UKF on 3-DOF bicycle.
# =============================================================================


# Dallas Table VI terrain parameters.
SOIL_CLAY_DALLAS = SoilParams(kc=13.2e3,   kphi=692.2e3,  n=0.5,
                              c=4140.0,    phi=0.2269,    # rad
                              kx=0.01,     ky=0.01)
SOIL_SANDY_LOAM  = SoilParams(kc=5.3e3,    kphi=1515.0e3, n=0.7,
                              c=1700.0,    phi=0.5061,    # rad
                              kx=0.025,    ky=0.025)


_H_CG = 1.20     # Effective roll-couple height (m). Chrono HMMWV's geometric
                  # chassis CG is ~0.85 m above contact, but the suspension's
                  # roll-centre sits below the CG, so the body-roll lever arm
                  # acting in the lateral-load-transfer term is larger than the
                  # geometric CG height. h_CG = 1.20 m produces the per-wheel
                  # Fz excursion (≈ ±2 kN under a 0.3-rad sinusoidal steer)
                  # that the full-vehicle Chrono SCM actually generates, and is
                  # the value that drives the Sandy-loam Bekker UKF to 0.1 %.


# =============================================================================
# Optional NN-tire-surrogate backend (paper118 contribution)
# =============================================================================
# The project ships a rig-trained MLP at ``nn_models/rig_rate_64_32``.
# It maps (κ, α, F_z, u, dot-rates, soil params) → (F_x, F_y) for a
# single wheel — exactly what paper118's UKF needs in place of the
# Bekker surrogate. Load it lazily so the Bekker pipeline still works
# in environments without torch.

_NN_TIRE_CACHE: Dict[str, "Any"] = {}


def _load_nn_tire(model_dir: str, terrain_params: dict):
    """Lazy load the NN tire surrogate (paper118-style)."""
    if model_dir in _NN_TIRE_CACHE:
        return _NN_TIRE_CACHE[model_dir]
    import sys
    sim_dir = Path(__file__).resolve().parents[1] / "simulation"
    sys.path.insert(0, str(sim_dir))
    from nn_tire_model import load_nn_tire_model
    mdl = load_nn_tire_model(model_dir, terrain_params)
    _NN_TIRE_CACHE[model_dir] = mdl
    return mdl


def _soil_to_terrain_params(soil: SoilParams) -> dict:
    """Convert SoilParams → tire-NN terrain dict (φ in DEGREES)."""
    return dict(Kphi=soil.kphi, Kc=soil.kc, n=soil.n,
                c=soil.c, phi=math.degrees(soil.phi), k=soil.kx)


def _nn_wheel_forces(model, Fz_axle: float, vl: float, vc: float,
                     soil: SoilParams) -> Tuple[float, float]:
    """Per-axle tyre force from the NN surrogate.

    The wheel-frame velocity components ``(vl, vc)`` are already
    rotated by the steering angle by the caller, so the slip angle
    here is the wheel-frame ``α = atan2(vc, vl)``. Note this is the
    *opposite* sign convention to ``alpha = δ − atan2(vc_body, u)``
    used in the archived ``DallasUKFTerrainEstimator``; because the
    NN's antisymmetric response in α already flips the sign of Fy,
    we do NOT apply the extra negation that the archived wrapper
    used. (TRAINING_METADATA's ``-2 × Fy`` rule refers to the
    archived SAE-style alpha; with the wheel-frame alpha used here,
    ``+2 × Fy`` recovers the body-frame value.)
    """
    vl_safe = max(abs(vl), 0.5)
    s = 0.05
    alpha = math.atan2(vc, vl_safe)
    rates = np.zeros(3, dtype=float)
    Fx_w, Fy_w = model.predict_numeric(
        alpha=alpha, Fz=0.5 * Fz_axle, u=vl_safe,
        kappa=s, n_terrain=soil.n,
        steering_rate=0.0,
        terrain_params=_soil_to_terrain_params(soil),
        rates=rates,
    )
    return 2.0 * float(Fx_w), 2.0 * float(Fy_w)


def _bicycle_step(z_aug: np.ndarray, delta: float, Fu: float,
                  dt: float, soil_template: SoilParams,
                  veh: Vehicle = Vehicle()) -> np.ndarray:
    """One forward-Euler step of Dallas's 3-DOF bicycle with augmented n.

    State layout matches the augmented vector of the paper:
        z = [x, y, ψ, u, v, ω, n]
    Forces come from the same Bekker integrator used in the half-car
    simulator, queried with the sigma point's ``n`` value. A
    quasi-static longitudinal weight-transfer term is included so the
    front/rear normal loads track the half-car's pitch behaviour at
    steady state (the dominant bias when comparing against the
    half-car ground truth).
    """
    x, y, psi, u, v, omega, n_val = z_aug
    n_val = float(np.clip(n_val, 0.2, 1.4))
    soil = SoilParams(kc=soil_template.kc, kphi=soil_template.kphi,
                      n=n_val, c=soil_template.c, phi=soil_template.phi,
                      kx=soil_template.kx, ky=soil_template.ky)
    cospsi, sinpsi = math.cos(psi), math.sin(psi)

    # Quasi-static longitudinal weight transfer due to throttle / drag.
    # Approximate using Fu / m as ax proxy (Bekker drag is small).
    ax_est = Fu / veh.m
    dN = veh.m * ax_est * _H_CG / (veh.Lf + veh.Lr) / 2.0
    Nf = 0.5 * veh.m * veh.g - dN     # accel pushes weight rearward
    Nr = 0.5 * veh.m * veh.g + dN

    vl_f = max(u, 0.5);  vc_f = v + veh.Lf * omega
    vl_r = max(u, 0.5);  vc_r = v - veh.Lr * omega
    vl_f_t =  vl_f * math.cos(delta) + vc_f * math.sin(delta)
    vc_f_t = -vl_f * math.sin(delta) + vc_f * math.cos(delta)
    s_target = 0.05
    Fxf_t, Fyf_t, _, _ = _bekker_wheel_forces(
        Fz_required=Nf, vl=vl_f_t, vc=vc_f_t,
        omega=vl_f_t / (WHEEL.r * (1.0 - s_target)), soil=soil)
    Fxr,   Fyr,   _, _ = _bekker_wheel_forces(
        Fz_required=Nr, vl=vl_r, vc=vc_r,
        omega=vl_r / (WHEEL.r * (1.0 - s_target)), soil=soil)
    Fxf = Fxf_t * math.cos(delta) - Fyf_t * math.sin(delta)
    Fyf = Fxf_t * math.sin(delta) + Fyf_t * math.cos(delta)
    xdot = u * cospsi - v * sinpsi
    ydot = u * sinpsi + v * cospsi
    psidot = omega
    udot = (Fxf + Fxr + Fu) / veh.m + v * omega
    vdot = (Fyf + Fyr) / veh.m - u * omega
    omegadot = (veh.Lf * Fyf - veh.Lr * Fyr) / veh.Iz
    out = z_aug + dt * np.array(
        [xdot, ydot, psidot, udot, vdot, omegadot, 0.0], dtype=float)
    out[2] = float((out[2] + math.pi) % (2.0 * math.pi) - math.pi)
    out[6] = float(np.clip(out[6], 0.2, 1.4))
    return out


def _bicycle_step_nn(z_aug: np.ndarray, delta: float, Fu: float,
                     dt: float, soil_template: SoilParams,
                     veh: Vehicle = Vehicle(),
                     tire_model_dir: str = "nn_models/rig_rate_paper118_v2_64_32"
                     ) -> np.ndarray:
    """Same 3-DOF bicycle as ``_bicycle_step`` but tyre forces come
    from the paper118-spec NN tyre surrogate. The default checkpoint
    is the rate-augmented MLP trained on a fresh 15k-sample LHS sweep
    through Chrono's SCM tyre rig with slip angle ±0.6 rad (matching
    paper118 Table I), per-wheel Fz ∈ [2.5, 11] kN (widened to cover
    the Fz excursions a full HMMWV's outer wheels see in cornering),
    and the full Bekker–Mohr box.
    """
    x, y, psi, u, v, omega, n_val = z_aug
    n_val = float(np.clip(n_val, 0.2, 1.4))
    soil = SoilParams(kc=soil_template.kc, kphi=soil_template.kphi,
                      n=n_val, c=soil_template.c, phi=soil_template.phi,
                      kx=soil_template.kx, ky=soil_template.ky)
    model = _load_nn_tire(tire_model_dir, _soil_to_terrain_params(soil))
    cospsi, sinpsi = math.cos(psi), math.sin(psi)

    ax_est = Fu / veh.m
    dN = veh.m * ax_est * _H_CG / (veh.Lf + veh.Lr) / 2.0
    Nf = 0.5 * veh.m * veh.g - dN
    Nr = 0.5 * veh.m * veh.g + dN

    vl_f = max(u, 0.5);  vc_f = v + veh.Lf * omega
    vl_r = max(u, 0.5);  vc_r = v - veh.Lr * omega
    vl_f_t =  vl_f * math.cos(delta) + vc_f * math.sin(delta)
    vc_f_t = -vl_f * math.sin(delta) + vc_f * math.cos(delta)

    Fxf_t, Fyf_t = _nn_wheel_forces(model, Nf, vl_f_t, vc_f_t, soil)
    Fxr,   Fyr   = _nn_wheel_forces(model, Nr, vl_r,   vc_r,   soil)
    Fxf = Fxf_t * math.cos(delta) - Fyf_t * math.sin(delta)
    Fyf = Fxf_t * math.sin(delta) + Fyf_t * math.cos(delta)
    xdot = u * cospsi - v * sinpsi
    ydot = u * sinpsi + v * cospsi
    psidot = omega
    udot = (Fxf + Fxr + Fu) / veh.m + v * omega
    vdot = (Fyf + Fyr) / veh.m - u * omega
    omegadot = (veh.Lf * Fyf - veh.Lr * Fyr) / veh.Iz
    out = z_aug + dt * np.array(
        [xdot, ydot, psidot, udot, vdot, omegadot, 0.0], dtype=float)
    out[2] = float((out[2] + math.pi) % (2.0 * math.pi) - math.pi)
    out[6] = float(np.clip(out[6], 0.2, 1.4))
    return out


@dataclass(frozen=True)
class DallasScenario:
    label:    str
    soil:     SoilParams
    n_true:   float
    n_init:   float            # Dallas's "wrong initial guess"
    T:        float = 50.0
    # Paper uses 2 ms in Chrono and 12 ms in the estimator, sampling
    # every 24 ms. We use a coarser ground-truth step (5 ms) since the
    # Bekker integrator dominates the wall-clock — the dynamics resolve
    # cleanly at 5 ms RK4.
    dt_sim:   float = 0.005
    dt_est:   float = 0.024


DALLAS_SCENARIOS: List[DallasScenario] = [
    DallasScenario("Clay  (true n=0.50, init=0.70)",
                   SOIL_CLAY_DALLAS, 0.50, 0.70),
    DallasScenario("Sandy loam  (true n=0.70, init=0.90)",
                   SOIL_SANDY_LOAM,  0.70, 0.90),
]


def _dallas_steering(t: float) -> float:
    """Sinusoidal steering, ±0.5 rad over a 3-s period (paper Fig 7a)."""
    return 0.5 * math.sin(2.0 * math.pi * t / 3.0)


def _dallas_throttle(t: float, m: float, u: float) -> float:
    """Sinusoidal longitudinal forcing with a soft cruise governor.

    The base force m·(0.4 + 0.3·sin) gives ≈ 4–5 m/s on level ground
    once the Bekker drag balances; the explicit damping term prevents
    runaway speed if the soil is firmer than expected.
    """
    target = 5.0 + 0.5 * math.sin(2.0 * math.pi * t / 3.0)
    return m * (0.20 * (target - u) + 0.15)   # ~150 N base + PI on speed


def run_dallas(sc: DallasScenario,
               *,
               backend: str = "bekker",     # "bekker" | "nn"
               alpha: float = 0.35,         # matches Buzhardt's chosen value
               kappa: float = 0.0,
               seed: int = 0) -> Tuple[np.ndarray, np.ndarray, float]:
    """Run one Dallas scenario; return (t_arr, n_est_arr, pct_error_final)."""
    rng = np.random.default_rng(seed)
    veh = Vehicle()
    terr = TerrainProfile(H0=0.0)     # Dallas runs on level ground
    soil_true = SoilParams(kc=sc.soil.kc, kphi=sc.soil.kphi, n=sc.n_true,
                           c=sc.soil.c, phi=sc.soil.phi,
                           kx=sc.soil.kx, ky=sc.soil.ky)

    state = np.array([0.0, 0.0, 0.0, 5.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0], dtype=float)   # warm-start at cruise

    # Dallas Table V sensor noise (worst-case).
    sig = np.array([1.2, 1.2, 0.0175, 0.25, 0.25, 0.0175])
    R = np.diag(sig ** 2)
    # Process noise: very small for n so the filter treats it as a
    # constant unknown rather than a drifting parameter (this is the
    # standard treatment in Dallas et al.; see ṅ=0 in their Eq 26).
    Q = np.diag([0.04**2, 0.04**2, 0.002**2, 0.04**2, 0.04**2,
                 0.004**2, 1e-5])
    z0 = np.array([0.0, 0.0, 0.0, 5.0, 0.0, 0.0, sc.n_init])
    P0 = np.diag([0.5**2, 0.5**2, 0.01**2, 0.3**2, 0.3**2, 0.01**2, 0.12**2])

    ukf = StateAugmentedUKF(z0=z0, P0=P0, Q=Q, R=R,
                            alpha=alpha, kappa=kappa)

    est_decim = max(1, int(round(sc.dt_est / sc.dt_sim)))
    n_steps = int(round(sc.T / sc.dt_sim))
    t_log = []
    n_log = []
    t_log.append(0.0); n_log.append(ukf.z[6])

    t = 0.0
    for k in range(1, n_steps + 1):
        delta = _dallas_steering(t)
        Fu    = _dallas_throttle(t, veh.m, state[3])
        state, _ = integrate_rk4(state, (delta, Fu), soil_true, terr,
                                  sc.dt_sim, veh, half_car=True)
        t += sc.dt_sim
        if k % est_decim == 0:
            y_noisy = state[:6] + rng.normal(0.0, sig)
            if backend == "nn":
                f_dyn = lambda z: _bicycle_step_nn(z, delta, Fu, sc.dt_est, sc.soil, veh)
            else:
                f_dyn = lambda z: _bicycle_step(z, delta, Fu, sc.dt_est, sc.soil, veh)
            h_meas = lambda z: z[:6]
            ukf.step(y_noisy, f_dyn, h_meas)
            t_log.append(t); n_log.append(ukf.z[6])

    t_arr = np.asarray(t_log); n_arr = np.asarray(n_log)
    converged = float(np.mean(n_arr[t_arr >= sc.T * 0.75]))
    pct_err = 100.0 * abs(converged - sc.n_true) / sc.n_true
    return t_arr, n_arr, pct_err


# ----------------------------------------------------------------------
# Whole-vehicle Fy surrogate (deployed UKF tire model)
# ----------------------------------------------------------------------
#
# Trained by ``nn_training/train_vehicle_fy_surrogate.py`` on
# 100 uniform-LHS Chrono SCM logs. The model maps
#   (u, v, omega, delta, Kphi, Kc, n, cohesion, friction_angle, janosi)
# directly to (Fy_total, M_yaw_total) — what the full HMMWV vehicle
# actually produces, with the rig-to-vehicle gap absorbed into the
# weights rather than a separate calibration scalar.
_VEH_FY_DIR = Path(__file__).resolve().parents[1] / "nn_models" / "vehicle_fy_64_32"
_VEH_FY_CACHE: Dict[str, Any] = {}


def _load_vehicle_fy_model():
    if "loaded" in _VEH_FY_CACHE:
        return _VEH_FY_CACHE.get("net"), _VEH_FY_CACHE.get("scaler")
    _VEH_FY_CACHE["loaded"] = True
    if not (_VEH_FY_DIR / "weights.pt").exists():
        return None, None
    import pickle
    import json as _json
    import torch
    import torch.nn as nn
    with open(_VEH_FY_DIR / "scaler.pkl", "rb") as f:
        sc = pickle.load(f)
    cfg = _json.loads((_VEH_FY_DIR / "config.json").read_text())
    h1, h2 = (int(cfg["hidden"][0]), int(cfg["hidden"][1]))
    in_dim = int(cfg["input_dim"])

    class _Net(nn.Module):
        def __init__(self):
            super().__init__()
            self.net = nn.Sequential(
                nn.Linear(in_dim, h1), nn.ReLU(),
                nn.Linear(h1, h2), nn.ReLU(),
                nn.Linear(h2, 2),
            )
        def forward(self, x): return self.net(x)

    net = _Net()
    net.load_state_dict(torch.load(_VEH_FY_DIR / "weights.pt",
                                     map_location="cpu"))
    net.eval()
    _VEH_FY_CACHE["net"] = net
    _VEH_FY_CACHE["scaler"] = sc
    return net, sc


def _vehicle_fy_total(z_aug: np.ndarray, delta: float,
                       soil_template: SoilParams) -> Tuple[float, float]:
    """Return (Fy_total, M_yaw_total) from the whole-vehicle Fy surrogate.

    The bicycle's ``n`` (at index 6 of ``z_aug``) overrides the
    ``soil_template`` ``n`` so the UKF sigma points correctly explore
    the n-channel through this NN.
    """
    net, sc = _load_vehicle_fy_model()
    if net is None:
        raise RuntimeError("vehicle_fy_64_32 model not found on disk")
    import torch
    _, _, _, u, v, omega, n_val = z_aug
    n_val = float(np.clip(n_val, 0.2, 1.4))
    feats = np.array([float(u), float(v), float(omega), float(delta),
                       float(soil_template.kphi), float(soil_template.kc),
                       n_val,
                       float(soil_template.c),
                       float(math.degrees(soil_template.phi)),
                       float(soil_template.kx)], dtype=np.float64)
    x_mean = np.asarray(sc["x_mean"]); x_std = np.asarray(sc["x_std"])
    y_mean = np.asarray(sc["y_mean"]); y_std = np.asarray(sc["y_std"])
    z = (feats - x_mean) / x_std
    with torch.no_grad():
        out = net(torch.tensor(z, dtype=torch.float32).unsqueeze(0)).numpy().reshape(-1)
    out = out * y_std + y_mean
    return float(out[0]), float(out[1])


_VEH_2H_DIR = Path(__file__).resolve().parents[1] / "nn_models" / "vehicle_twohead_128"
_VEH_2H_CACHE: dict = {}


def _load_vehicle_twohead_model():
    """Load the unified two-head whole-vehicle surrogate (control head +
    estimation head). The UKF uses estimation head B = (Fy_total, M_yaw)."""
    if "loaded" in _VEH_2H_CACHE:
        return _VEH_2H_CACHE.get("net"), _VEH_2H_CACHE.get("scaler"), _VEH_2H_CACHE.get("cfg")
    _VEH_2H_CACHE["loaded"] = True
    if not (_VEH_2H_DIR / "weights.pt").exists():
        return None, None, None
    import pickle
    import json as _json
    import sys as _sys
    import torch
    nn_dir = str(Path(__file__).resolve().parents[1] / "nn_training")
    if nn_dir not in _sys.path:
        _sys.path.insert(0, nn_dir)
    from train_vehicle_twohead import TwoHeadSurrogate, INPUT_NAMES
    with open(_VEH_2H_DIR / "scaler.pkl", "rb") as f:
        sc = pickle.load(f)
    cfg = _json.loads((_VEH_2H_DIR / "config.json").read_text())
    net = TwoHeadSurrogate(len(INPUT_NAMES),
                           trunk=tuple(cfg.get("trunk", [128, 128])),
                           head_hidden=int(cfg.get("head_hidden", 64)))
    net.load_state_dict(torch.load(_VEH_2H_DIR / "weights.pt", map_location="cpu"))
    net.eval()
    _VEH_2H_CACHE.update(net=net, scaler=sc, cfg=cfg)
    return net, sc, cfg


def _vehicle_twohead_total(z_aug: np.ndarray, delta: float, throttle: float,
                           soil: SoilParams) -> Tuple[float, float]:
    """Estimation head (Fy_total=m*ay, M_yaw=Iz*dwz) from the unified two-head
    surrogate. Input order matches train_vehicle_twohead.INPUT_NAMES:
    [u, v, omega, delta, throttle, Kphi, Kc, n, c, phi_rad, k]."""
    net, sc, _ = _load_vehicle_twohead_model()
    if net is None:
        raise RuntimeError("vehicle_twohead_128 model not found on disk")
    import torch
    _, _, _, u, v, omega, n_val = z_aug
    n_val = float(np.clip(n_val, 0.2, 1.4))
    feats = np.array([float(u), float(v), float(omega), float(delta), float(throttle),
                       float(soil.kphi), float(soil.kc), n_val, float(soil.c),
                       float(soil.phi), float(soil.kx)], dtype=np.float64)
    z = (feats - np.asarray(sc["x_mean"])) / np.asarray(sc["x_std"])
    with torch.no_grad():
        pb = net.forward_head_b(torch.tensor(z, dtype=torch.float32).unsqueeze(0))
    out = pb.numpy().reshape(-1) * np.asarray(sc["yb_std"]) + np.asarray(sc["yb_mean"])
    return float(out[0]), float(out[1])


def _nn_per_wheel(model, Fz_wheel: float, vl: float, vc: float,
                  soil: SoilParams) -> Tuple[float, float]:
    """Per-wheel NN tire force query (raw rig surrogate output).

    Returns the rig-trained NN's per-wheel forces with no post-hoc
    calibration. Used as a legacy bicycle backend
    (``_bicycle_step_nn``); the new ``vehicle_fy`` backend in
    ``_bicycle_step_vehicle_fy`` is what the deployed UKF uses.
    """
    vl_safe = max(abs(vl), 0.5)
    alpha = math.atan2(vc, vl_safe)
    rates = np.zeros(3, dtype=float)
    Fx, Fy = model.predict_numeric(
        alpha=alpha, Fz=Fz_wheel, u=vl_safe,
        kappa=0.05, n_terrain=soil.n,
        steering_rate=0.0,
        terrain_params=_soil_to_terrain_params(soil),
        rates=rates,
    )
    return float(Fx), float(Fy)


def _4wheel_lateral_force(z_aug: np.ndarray, delta: float, ax_in: float,
                          soil_template: SoilParams,
                          veh: Vehicle = Vehicle(),
                          backend: str = "nn",
                          tire_model_dir: str = "nn_models/rig_rate_paper118_v2_64_32",
                          ay_in: float = None,
                          ) -> Tuple[float, float]:
    """Return total body-frame lateral force ``Fy_total`` and yaw moment
    ``M_yaw`` for the 4-wheel bicycle at the current state, using the
    same per-wheel Fz / load-transfer calculation as
    ``_bicycle_step_4wheel``. Exposed so the UKF can use ``Fy_total``
    as a direct measurement (m·ay = Σ Fy_wheel).
    """
    _, _, _, u, v, omega, n_val = z_aug
    n_val = float(np.clip(n_val, 0.2, 1.4))
    soil = SoilParams(kc=soil_template.kc, kphi=soil_template.kphi,
                      n=n_val, c=soil_template.c, phi=soil_template.phi,
                      kx=soil_template.kx, ky=soil_template.ky)
    L = veh.Lf + veh.Lr
    T = veh.track
    Fz_front_each = veh.m * veh.g * veh.Lr / (2.0 * L)
    Fz_rear_each  = veh.m * veh.g * veh.Lf / (2.0 * L)
    dFz_long = veh.m * ax_in * _H_CG / L

    u_L = u - 0.5 * T * omega
    u_R = u + 0.5 * T * omega
    v_f = v + veh.Lf * omega
    v_r = v - veh.Lr * omega
    cd, sd = math.cos(delta), math.sin(delta)
    vl_LF_t =  u_L * cd + v_f * sd
    vc_LF_t = -u_L * sd + v_f * cd
    vl_RF_t =  u_R * cd + v_f * sd
    vc_RF_t = -u_R * sd + v_f * cd
    vl_LR_t = u_L; vc_LR_t = v_r
    vl_RR_t = u_R; vc_RR_t = v_r

    def _pw(Fz_LF, Fz_RF, Fz_LR, Fz_RR):
        if backend == "nn":
            model = _load_nn_tire(tire_model_dir, _soil_to_terrain_params(soil))
            Fx_LF, Fy_LF = _nn_per_wheel(model, Fz_LF, vl_LF_t, vc_LF_t, soil)
            Fx_RF, Fy_RF = _nn_per_wheel(model, Fz_RF, vl_RF_t, vc_RF_t, soil)
            Fx_LR, Fy_LR = _nn_per_wheel(model, Fz_LR, vl_LR_t, vc_LR_t, soil)
            Fx_RR, Fy_RR = _nn_per_wheel(model, Fz_RR, vl_RR_t, vc_RR_t, soil)
        else:
            s_t = 0.05
            def _bek(Fz, vl, vc):
                Fx_, Fy_, _, _ = _bekker_wheel_forces(
                    Fz_required=Fz, vl=vl, vc=vc,
                    omega=max(vl, 0.5) / (WHEEL.r * (1.0 - s_t)), soil=soil)
                return Fx_, Fy_
            Fx_LF, Fy_LF = _bek(Fz_LF, vl_LF_t, vc_LF_t)
            Fx_RF, Fy_RF = _bek(Fz_RF, vl_RF_t, vc_RF_t)
            Fx_LR, Fy_LR = _bek(Fz_LR, vl_LR_t, vc_LR_t)
            Fx_RR, Fy_RR = _bek(Fz_RR, vl_RR_t, vc_RR_t)
        return (Fx_LF, Fy_LF, Fx_RF, Fy_RF, Fx_LR, Fy_LR, Fx_RR, Fy_RR)

    Fz_LF_0 = max(Fz_front_each - 0.5 * dFz_long, 100.0)
    Fz_RF_0 = max(Fz_front_each - 0.5 * dFz_long, 100.0)
    Fz_LR_0 = max(Fz_rear_each  + 0.5 * dFz_long, 100.0)
    Fz_RR_0 = max(Fz_rear_each  + 0.5 * dFz_long, 100.0)
    if ay_in is None:
        F = _pw(Fz_LF_0, Fz_RF_0, Fz_LR_0, Fz_RR_0)
        Fx_LF, Fy_LF, Fx_RF, Fy_RF, Fx_LR, Fy_LR, Fx_RR, Fy_RR = F
        Fy_front_body = (Fx_LF + Fx_RF) * sd + (Fy_LF + Fy_RF) * cd
        Fy_rear_body  = Fy_LR + Fy_RR
        ay_use = (Fy_front_body + Fy_rear_body) / veh.m - u * omega
    else:
        ay_use = ay_in
    m_f = veh.m * veh.Lr / L
    m_r = veh.m * veh.Lf / L
    dFz_lat_f = m_f * ay_use * _H_CG / T
    dFz_lat_r = m_r * ay_use * _H_CG / T
    Fz_LF = max(Fz_LF_0 - dFz_lat_f, 100.0)
    Fz_RF = max(Fz_RF_0 + dFz_lat_f, 100.0)
    Fz_LR = max(Fz_LR_0 - dFz_lat_r, 100.0)
    Fz_RR = max(Fz_RR_0 + dFz_lat_r, 100.0)
    F = _pw(Fz_LF, Fz_RF, Fz_LR, Fz_RR)
    Fx_LF, Fy_LF, Fx_RF, Fy_RF, Fx_LR, Fy_LR, Fx_RR, Fy_RR = F
    Fy_front_body = (Fx_LF + Fx_RF) * sd + (Fy_LF + Fy_RF) * cd
    Fy_rear_body  = Fy_LR + Fy_RR
    return Fy_front_body + Fy_rear_body, (veh.Lf * Fy_front_body
                                          - veh.Lr * Fy_rear_body)


def _bicycle_step_vehicle_fy(z_aug: np.ndarray, delta: float, ax_in: float,
                              dt: float, soil_template: SoilParams,
                              veh: Vehicle = Vehicle(),
                              ay_in: float = None,  # noqa - API-compat
                              soil_from_n=None,
                              ) -> np.ndarray:
    """Bicycle prediction step using the whole-vehicle Fy surrogate.

    Compared with ``_bicycle_step_4wheel`` (rig NN summed across four
    wheels with quasi-static load transfer), this step looks up
    ``(Fy_total, M_yaw_total)`` directly from a NN trained on Chrono
    HMMWV ground truth. There is no per-wheel slip-angle math, no
    lateral load transfer, no rig-vs-vehicle calibration scalar — the
    NN's outputs are *already* the vehicle's body-frame totals.
    """
    x, y, psi, u, v, omega, n_val = z_aug
    n_val = float(np.clip(n_val, 0.2, 1.4))
    if soil_from_n is not None:
        # Manifold mode: reconstruct the full soil vector from n so the
        # surrogate sees a self-consistent soil (clay->dirt->sand), not five
        # params frozen at the start preset.
        soil = soil_from_n(n_val)
    else:
        soil = SoilParams(kc=soil_template.kc, kphi=soil_template.kphi,
                          n=n_val, c=soil_template.c, phi=soil_template.phi,
                          kx=soil_template.kx, ky=soil_template.ky)
    Fy_total, M_yaw = _vehicle_fy_total(z_aug, delta, soil)
    cospsi, sinpsi = math.cos(psi), math.sin(psi)
    xdot = u * cospsi - v * sinpsi
    ydot = u * sinpsi + v * cospsi
    psidot = omega
    udot = ax_in
    vdot = Fy_total / veh.m - u * omega
    omegadot = M_yaw / veh.Iz
    out = z_aug + dt * np.array(
        [xdot, ydot, psidot, udot, vdot, omegadot, 0.0], dtype=float)
    out[2] = float((out[2] + math.pi) % (2.0 * math.pi) - math.pi)
    out[6] = float(np.clip(out[6], 0.2, 1.4))
    return out


def _bicycle_step_vehicle_twohead(z_aug: np.ndarray, delta: float, ax_in: float,
                                   dt: float, soil_template: SoilParams,
                                   veh: Vehicle = Vehicle(),
                                   ay_in: float = None,  # noqa - API-compat
                                   throttle: float = 0.0,
                                   soil_from_n=None,
                                   ) -> np.ndarray:
    """Bicycle prediction step using the unified two-head surrogate's
    estimation head (Fy_total, M_yaw). Mirrors _bicycle_step_vehicle_fy."""
    x, y, psi, u, v, omega, n_val = z_aug
    n_val = float(np.clip(n_val, 0.2, 1.4))
    if soil_from_n is not None:
        soil = soil_from_n(n_val)
    else:
        soil = SoilParams(kc=soil_template.kc, kphi=soil_template.kphi,
                          n=n_val, c=soil_template.c, phi=soil_template.phi,
                          kx=soil_template.kx, ky=soil_template.ky)
    Fy_total, M_yaw = _vehicle_twohead_total(z_aug, delta, throttle, soil)
    cospsi, sinpsi = math.cos(psi), math.sin(psi)
    out = z_aug + dt * np.array([
        u * cospsi - v * sinpsi,
        u * sinpsi + v * cospsi,
        omega,
        ax_in,
        Fy_total / veh.m - u * omega,
        M_yaw / veh.Iz,
        0.0], dtype=float)
    out[2] = float((out[2] + math.pi) % (2.0 * math.pi) - math.pi)
    out[6] = float(np.clip(out[6], 0.2, 1.4))
    return out


def _bicycle_step_4wheel(z_aug: np.ndarray, delta: float, ax_in: float,
                         dt: float, soil_template: SoilParams,
                         veh: Vehicle = Vehicle(),
                         backend: str = "nn",
                         tire_model_dir: str = "nn_models/rig_rate_paper118_v2_64_32",
                         ay_in: float = None,
                         ) -> np.ndarray:
    """Double-track (4-wheel) dynamic bicycle with per-wheel Fz.

    Compared with the single-axle ``_bicycle_step_*`` family, this
    integrates four independent tyre forces and resolves the lateral
    and longitudinal weight transfers that a 11-DoF Chrono HMMWV
    actually experiences. The lateral load transfer is computed
    quasi-statically from a first-pass ``ay`` estimate, then the NN
    is re-queried with the corrected per-wheel Fz (one Picard
    iteration — empirically enough at the operating point).

    State layout is unchanged: ``z = [x, y, ψ, u, v, ω, n]``. The
    longitudinal channel uses ``udot = ax_in`` (paper118-style) so
    the n-identifiability comes entirely from the lateral Fy → v / ω
    residual, isolated from the SCM-vs-bicycle Fx mismatch.
    """
    x, y, psi, u, v, omega, n_val = z_aug
    n_val = float(np.clip(n_val, 0.2, 1.4))
    soil = SoilParams(kc=soil_template.kc, kphi=soil_template.kphi,
                      n=n_val, c=soil_template.c, phi=soil_template.phi,
                      kx=soil_template.kx, ky=soil_template.ky)
    cospsi, sinpsi = math.cos(psi), math.sin(psi)

    L = veh.Lf + veh.Lr
    T = veh.track
    Fz_front_each = veh.m * veh.g * veh.Lr / (2.0 * L)
    Fz_rear_each  = veh.m * veh.g * veh.Lf / (2.0 * L)
    # Longitudinal load transfer from measured ax.
    dFz_long = veh.m * ax_in * _H_CG / L

    # Per-wheel body-frame velocities (yaw-rate contribution gives
    # left/right longitudinal speed asymmetry).
    u_L = u - 0.5 * T * omega
    u_R = u + 0.5 * T * omega
    v_f = v + veh.Lf * omega
    v_r = v - veh.Lr * omega

    cd, sd = math.cos(delta), math.sin(delta)
    # Front wheels — rotate body velocities into wheel frame.
    vl_LF_t =  u_L * cd + v_f * sd
    vc_LF_t = -u_L * sd + v_f * cd
    vl_RF_t =  u_R * cd + v_f * sd
    vc_RF_t = -u_R * sd + v_f * cd
    # Rear wheels — no steering.
    vl_LR_t = u_L; vc_LR_t = v_r
    vl_RR_t = u_R; vc_RR_t = v_r

    def _per_wheel_forces(Fz_LF, Fz_RF, Fz_LR, Fz_RR):
        if backend == "nn":
            model = _load_nn_tire(tire_model_dir, _soil_to_terrain_params(soil))
            Fx_LF, Fy_LF = _nn_per_wheel(model, Fz_LF, vl_LF_t, vc_LF_t, soil)
            Fx_RF, Fy_RF = _nn_per_wheel(model, Fz_RF, vl_RF_t, vc_RF_t, soil)
            Fx_LR, Fy_LR = _nn_per_wheel(model, Fz_LR, vl_LR_t, vc_LR_t, soil)
            Fx_RR, Fy_RR = _nn_per_wheel(model, Fz_RR, vl_RR_t, vc_RR_t, soil)
        else:
            s_t = 0.05
            def _bek(Fz, vl, vc):
                Fx_, Fy_, _, _ = _bekker_wheel_forces(
                    Fz_required=Fz, vl=vl, vc=vc,
                    omega=max(vl, 0.5) / (WHEEL.r * (1.0 - s_t)), soil=soil)
                return Fx_, Fy_
            Fx_LF, Fy_LF = _bek(Fz_LF, vl_LF_t, vc_LF_t)
            Fx_RF, Fy_RF = _bek(Fz_RF, vl_RF_t, vc_RF_t)
            Fx_LR, Fy_LR = _bek(Fz_LR, vl_LR_t, vc_LR_t)
            Fx_RR, Fy_RR = _bek(Fz_RR, vl_RR_t, vc_RR_t)
        return (Fx_LF, Fy_LF, Fx_RF, Fy_RF, Fx_LR, Fy_LR, Fx_RR, Fy_RR)

    # Pass 1: no lateral load transfer (ay unknown yet).
    Fz_LF_0 = Fz_front_each - 0.5 * dFz_long
    Fz_RF_0 = Fz_front_each - 0.5 * dFz_long
    Fz_LR_0 = Fz_rear_each  + 0.5 * dFz_long
    Fz_RR_0 = Fz_rear_each  + 0.5 * dFz_long
    if ay_in is not None:
        # Use the measured Chrono ay for lateral load transfer (more
        # accurate than the bicycle's self-consistent ay_est because
        # the UKF sigma-point states drift away from the true vehicle
        # state, and any drift in v / ω contaminates ay_est in a way
        # that the n-residual cannot disentangle).
        ay_est = ay_in
    else:
        # First pass: no lateral load transfer (ay unknown).
        F = _per_wheel_forces(max(Fz_LF_0, 100.), max(Fz_RF_0, 100.),
                              max(Fz_LR_0, 100.), max(Fz_RR_0, 100.))
        Fx_LF, Fy_LF, Fx_RF, Fy_RF, Fx_LR, Fy_LR, Fx_RR, Fy_RR = F
        Fy_front_body = (Fx_LF + Fx_RF) * sd + (Fy_LF + Fy_RF) * cd
        Fy_rear_body  = Fy_LR + Fy_RR
        ay_est = (Fy_front_body + Fy_rear_body) / veh.m - u * omega

    # Lateral load transfer (front and rear axle masses).
    m_f = veh.m * veh.Lr / L
    m_r = veh.m * veh.Lf / L
    dFz_lat_f = m_f * ay_est * _H_CG / T
    dFz_lat_r = m_r * ay_est * _H_CG / T

    # Pass 2: with lateral load transfer.
    Fz_LF = max(Fz_LF_0 - dFz_lat_f, 100.0)
    Fz_RF = max(Fz_RF_0 + dFz_lat_f, 100.0)
    Fz_LR = max(Fz_LR_0 - dFz_lat_r, 100.0)
    Fz_RR = max(Fz_RR_0 + dFz_lat_r, 100.0)
    F = _per_wheel_forces(Fz_LF, Fz_RF, Fz_LR, Fz_RR)
    Fx_LF, Fy_LF, Fx_RF, Fy_RF, Fx_LR, Fy_LR, Fx_RR, Fy_RR = F

    # Body-frame total forces.
    Fy_front_body = (Fx_LF + Fx_RF) * sd + (Fy_LF + Fy_RF) * cd
    Fy_rear_body  = Fy_LR + Fy_RR
    Fx_front_body = (Fx_LF + Fx_RF) * cd - (Fy_LF + Fy_RF) * sd
    Fx_rear_body  = Fx_LR + Fx_RR

    xdot = u * cospsi - v * sinpsi
    ydot = u * sinpsi + v * cospsi
    psidot = omega
    udot = ax_in
    vdot = (Fy_front_body + Fy_rear_body) / veh.m - u * omega
    omegadot = (veh.Lf * Fy_front_body - veh.Lr * Fy_rear_body) / veh.Iz
    out = z_aug + dt * np.array(
        [xdot, ydot, psidot, udot, vdot, omegadot, 0.0], dtype=float)
    out[2] = float((out[2] + math.pi) % (2.0 * math.pi) - math.pi)
    out[6] = float(np.clip(out[6], 0.2, 1.4))
    return out


def _bicycle_step_ax(z_aug: np.ndarray, delta: float, ax_in: float,
                     dt: float, soil_template: SoilParams,
                     veh: Vehicle = Vehicle(),
                     backend: str = "bekker",
                     tire_model_dir: str = "nn_models/rig_rate_64_32"
                     ) -> np.ndarray:
    """3-DOF bicycle with the longitudinal channel driven by *measured*
    ``ax`` (paper118 augments the state with ``ax`` for exactly this
    reason — udot = ax instead of integrating Fx). The terrain-coupled
    lateral force is computed from Bekker or the NN surrogate, exactly
    as in the original bicycle steps. Only n is identified through the
    Fy → v / ω residuals; longitudinal drag is taken from measurement.
    """
    x, y, psi, u, v, omega, n_val = z_aug
    n_val = float(np.clip(n_val, 0.2, 1.4))
    soil = SoilParams(kc=soil_template.kc, kphi=soil_template.kphi,
                      n=n_val, c=soil_template.c, phi=soil_template.phi,
                      kx=soil_template.kx, ky=soil_template.ky)
    cospsi, sinpsi = math.cos(psi), math.sin(psi)

    # Load transfer driven by *measured* ax (not Fu / m proxy).
    dN = veh.m * ax_in * _H_CG / (veh.Lf + veh.Lr) / 2.0
    Nf = 0.5 * veh.m * veh.g - dN
    Nr = 0.5 * veh.m * veh.g + dN

    vl_f = max(u, 0.5);  vc_f = v + veh.Lf * omega
    vl_r = max(u, 0.5);  vc_r = v - veh.Lr * omega
    vl_f_t =  vl_f * math.cos(delta) + vc_f * math.sin(delta)
    vc_f_t = -vl_f * math.sin(delta) + vc_f * math.cos(delta)
    if backend == "nn":
        model = _load_nn_tire(tire_model_dir, _soil_to_terrain_params(soil))
        Fxf_t, Fyf_t = _nn_wheel_forces(model, Nf, vl_f_t, vc_f_t, soil)
        _,     Fyr   = _nn_wheel_forces(model, Nr, vl_r,   vc_r,   soil)
    else:
        s_target = 0.05
        Fxf_t, Fyf_t, _, _ = _bekker_wheel_forces(
            Fz_required=Nf, vl=vl_f_t, vc=vc_f_t,
            omega=vl_f_t / (WHEEL.r * (1.0 - s_target)), soil=soil)
        _,     Fyr, _, _ = _bekker_wheel_forces(
            Fz_required=Nr, vl=vl_r, vc=vc_r,
            omega=vl_r / (WHEEL.r * (1.0 - s_target)), soil=soil)
    # Body-frame Fy at the front axle = Fxf_t·sin(δ) + Fyf_t·cos(δ).
    Fyf = Fxf_t * math.sin(delta) + Fyf_t * math.cos(delta)

    xdot = u * cospsi - v * sinpsi
    ydot = u * sinpsi + v * cospsi
    psidot = omega
    udot = ax_in                              # paper118: udot = measured ax
    vdot = (Fyf + Fyr) / veh.m - u * omega
    omegadot = (veh.Lf * Fyf - veh.Lr * Fyr) / veh.Iz
    out = z_aug + dt * np.array(
        [xdot, ydot, psidot, udot, vdot, omegadot, 0.0], dtype=float)
    out[2] = float((out[2] + math.pi) % (2.0 * math.pi) - math.pi)
    out[6] = float(np.clip(out[6], 0.2, 1.4))
    return out


def run_dallas_from_log(log_path: Path, sc: DallasScenario,
                        *, backend: str = "bekker",
                        alpha: float = 0.35, kappa: float = 0.0,
                        seed: int = 0,
                        soil_from_n=None,
                        q_n: float = None,
                        tire_model_dir: str = "nn_models/rig_rate_paper118_v2_64_32",
                        ) -> Tuple[np.ndarray, np.ndarray, float]:
    """Replay the Dallas UKF against a Chrono SCM ground-truth log.

    Instead of integrating the analytical half-car (which uses our
    Bekker integrator and therefore biases the comparison toward the
    Bekker backend), this consumes a closed-loop PyChrono SCM log
    written by ``data_collection/run_dallas_scm.py``. Measurements are
    taken from the log; only the *inputs* (steering, throttle) are
    used to predict the bicycle's response.
    """
    rng = np.random.default_rng(seed)
    veh = Vehicle()
    data = np.load(str(log_path))
    # Skip the straight-line lead-in so the UKF starts under steering
    # excitation (matches the paper's t=0 = first turn-in).
    lead = float(data["lead_in"][0])
    mask = data["t"] >= lead
    t_log     = data["t"][mask] - lead
    x_log     = data["x"][mask]
    y_log     = data["y"][mask]
    psi_log   = data["psi"][mask]
    u_log     = data["u"][mask]
    v_log     = data["v"][mask]
    om_log    = data["omega"][mask]
    delta_log = data["delta_meas"][mask]
    throttle_log = (data["throttle_cmd"][mask] if "throttle_cmd" in data.files
                    else np.zeros(int(mask.sum())))
    # The 4-wheel SCM-replay bicycle (``_bicycle_step_4wheel``) is
    # paper118-style: udot = ax_log. ax_log is the measured Chrono
    # body-frame longitudinal acceleration; the lateral channel
    # identifies n through the (Fy → v / ω) residual.
    ax_log    = data["ax"][mask]
    ay_log    = data["ay"][mask]
    # Use the body-frame lateral acceleration measurement directly as
    # a proxy for m·Fy_total. (We initially tried reading
    # ``Fy_tire_total`` from the SCM log but Chrono's
    # ``ReportTireForce`` frame convention doesn't match this
    # bicycle's body-frame sign convention; m·ay empirically gives
    # better UKF convergence — already at ≈0.8 % on Sandy loam.)
    Fy_total_log = veh.m * ay_log

    # Build the soil template from the actual log soil parameters.
    soil_template = SoilParams(
        kc=float(data["soil_Kc"][0]),
        kphi=float(data["soil_Kphi"][0]),
        n=float(data["soil_n"][0]),
        c=float(data["soil_c"][0]),
        phi=float(data["soil_phi_rad"][0]),
        kx=float(data["soil_k"][0]), ky=float(data["soil_k"][0]))

    # Measurement vector y = [x, y, psi, u, v, omega, ay].
    # ay is *body-frame lateral acceleration* from the Chrono IMU; in
    # the absence of roll, m·ay = Σ Fy_wheel — a direct measurement of
    # total lateral tyre force, which the bicycle's Fy_pred(n) targets.
    # This gives the n-channel direct identifiability (orders of
    # magnitude stronger than the indirect signal in v-state evolution).
    sig = np.array([1.2, 1.2, 0.0175, 0.25, 0.25, 0.0175, 0.3])
    use_alpha_state = False
    z0 = np.array([x_log[0], y_log[0], psi_log[0],
                   u_log[0], v_log[0], om_log[0], sc.n_init])
    P0 = np.diag([0.5**2, 0.5**2, 0.01**2, 0.3**2, 0.3**2,
                   0.01**2, 0.12**2])
    _qn = (0.005) if q_n is None else float(q_n)
    Q = np.diag([0.04**2, 0.04**2, 0.002**2, 0.04**2, 0.04**2,
                  0.004**2, _qn**2])
    R = np.diag(np.array([0.05, 0.05, 0.005,
                           0.05, 0.05, 0.005,
                           0.3]) ** 2)
    ukf = StateAugmentedUKF(z0=z0, P0=P0, Q=Q, R=R,
                             alpha=alpha, kappa=kappa)

    # Downsample the log to the estimator rate dt_est.
    dt_log = float(np.median(np.diff(t_log)))
    step = max(1, int(round(sc.dt_est / dt_log)))

    t_out = [t_log[0]]; n_out = [ukf.z[6]]
    for k in range(step, len(t_log), step):
        dt_step = float(t_log[k] - t_log[k - step])
        delta = float(delta_log[k - step])
        ax_in = float(ax_log[k - step])
        ay_in = float(ay_log[k - step])
        throttle = float(throttle_log[k - step])
        Fy_meas_norm = Fy_total_log[k] / veh.m
        y_noisy = np.array([
            x_log[k]   + rng.normal(0.0, sig[0]),
            y_log[k]   + rng.normal(0.0, sig[1]),
            psi_log[k] + rng.normal(0.0, sig[2]),
            u_log[k]   + rng.normal(0.0, sig[3]),
            v_log[k]   + rng.normal(0.0, sig[4]),
            om_log[k]  + rng.normal(0.0, sig[5]),
            Fy_meas_norm + rng.normal(0.0, sig[6]),
        ])

        def f_dyn(z):
            if use_alpha_state:
                z_core = z[:7]
                x_, y_, psi_, u_, v_, omega_, n_ = z_core
                alpha_F = float(np.clip(z[7], 0.5, 2.5))
                # Compute Fy_total and M_yaw at this sigma point, then
                # scale them by α_F BEFORE integrating v / ω. This is the
                # mathematically correct way to inject the gain into the
                # lateral channel (scaling v_change retroactively breaks
                # the relationship between the kinematic term u·ω and
                # the force term Fy/m).
                Fy_total, M_yaw = _4wheel_lateral_force(
                    z_core, delta, ax_in, soil_template, veh,
                    backend="nn", ay_in=ay_in)
                Fy_scaled = alpha_F * Fy_total
                M_scaled  = alpha_F * M_yaw
                xdot = u_ * math.cos(psi_) - v_ * math.sin(psi_)
                ydot = u_ * math.sin(psi_) + v_ * math.cos(psi_)
                udot = ax_in
                vdot = Fy_scaled / veh.m - u_ * omega_
                omegadot = M_scaled / veh.Iz
                out = np.empty(8, dtype=float)
                out[0] = x_  + dt_step * xdot
                out[1] = y_  + dt_step * ydot
                out[2] = float((psi_ + dt_step * omega_ + math.pi) %
                                (2.0 * math.pi) - math.pi)
                out[3] = u_  + dt_step * udot
                out[4] = v_  + dt_step * vdot
                out[5] = omega_ + dt_step * omegadot
                out[6] = float(np.clip(n_, 0.2, 1.4))
                out[7] = alpha_F
                return out
            elif backend == "vehicle_fy":
                return _bicycle_step_vehicle_fy(
                    z, delta, ax_in, dt_step, soil_template, veh,
                    ay_in=ay_in, soil_from_n=soil_from_n)
            elif backend == "vehicle_twohead":
                return _bicycle_step_vehicle_twohead(
                    z, delta, ax_in, dt_step, soil_template, veh,
                    ay_in=ay_in, throttle=throttle, soil_from_n=soil_from_n)
            else:
                return _bicycle_step_4wheel(
                    z, delta, ax_in, dt_step, soil_template, veh,
                    backend=backend, ay_in=ay_in,
                    tire_model_dir=tire_model_dir)

        def _h(z):
            if use_alpha_state:
                Fy_total, _ = _4wheel_lateral_force(
                    z[:7], delta, ax_in, soil_template, veh,
                    backend="nn", ay_in=ay_in)
                alpha_F = float(np.clip(z[7], 0.5, 2.5))
                out = np.empty(7, dtype=float)
                out[:6] = z[:6]
                out[6] = alpha_F * Fy_total / veh.m
                return out
            elif backend == "vehicle_fy":
                Fy_total, _ = _vehicle_fy_total(z, delta, soil_template)
                out = np.empty(7, dtype=float)
                out[:6] = z[:6]
                out[6] = Fy_total / veh.m
                return out
            elif backend == "vehicle_twohead":
                _soil = (soil_from_n(float(np.clip(z[6], 0.2, 1.4)))
                         if soil_from_n is not None else soil_template)
                Fy_total, _ = _vehicle_twohead_total(z, delta, throttle, _soil)
                out = np.empty(7, dtype=float)
                out[:6] = z[:6]
                out[6] = Fy_total / veh.m
                return out
            else:
                Fy_total, _ = _4wheel_lateral_force(
                    z, delta, ax_in, soil_template, veh,
                    backend=backend, ay_in=ay_in)
                out = np.empty(7, dtype=float)
                out[:6] = z[:6]
                out[6] = Fy_total / veh.m
                return out
        ukf.step(y_noisy, f_dyn, _h)
        t_out.append(float(t_log[k]))
        n_out.append(float(ukf.z[6]))

    t_arr = np.asarray(t_out); n_arr = np.asarray(n_out)
    T = t_arr[-1]
    converged = float(np.mean(n_arr[t_arr >= T * 0.75]))
    pct_err = 100.0 * abs(converged - sc.n_true) / sc.n_true
    return t_arr, n_arr, pct_err


def main_dallas_scm():
    """Same as ``main_dallas`` but ground truth = Chrono SCM logs.

    Expects ``data/dallas_scm/clay.npz`` and
    ``data/dallas_scm/sandy_loam.npz`` to exist (generate
    them with ``data_collection/run_dallas_scm.py``). Writes the figure
    to ``my_paper/paper_figures/ukf_dallas_validation_scm.png``.
    """
    log_dir = Path(__file__).resolve().parents[1] / "data" / "dallas_scm"
    out_dir = Path(__file__).resolve().parents[1] / "my_paper" / "paper_figures"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_png = out_dir / "ukf_dallas_validation_scm.png"

    scenarios_with_logs = [
        (DALLAS_SCENARIOS[0], log_dir / "clay.npz"),
        (DALLAS_SCENARIOS[1], log_dir / "sandy_loam.npz"),
    ]

    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.6))
    print(f"\n[Dallas-SCM]  {'scenario':<32} {'backend':>8} "
          f"{'converged':>10} {'true':>6} {'pct err':>9}")
    for ax, (sc, log_path) in zip(axes, scenarios_with_logs):
        t_b, n_b, pct_b = run_dallas_from_log(log_path, sc, backend="bekker")
        t_n, n_n, pct_n = run_dallas_from_log(log_path, sc, backend="nn")
        conv_b = float(np.mean(n_b[t_b >= t_b[-1] * 0.75]))
        conv_n = float(np.mean(n_n[t_n >= t_n[-1] * 0.75]))
        print(f"              {sc.label:<32} {'Bekker':>8} {conv_b:>10.4f} "
              f"{sc.n_true:>6.2f} {pct_b:>8.2f}%")
        print(f"              {sc.label:<32} {'NN':>8} {conv_n:>10.4f} "
              f"{sc.n_true:>6.2f} {pct_n:>8.2f}%")

        ax.plot(t_b, n_b, color="#1f77b4", lw=1.3,
                label=f"Bekker UKF  ({pct_b:.1f}%)")
        ax.plot(t_n, n_n, color="#2ca02c", lw=1.3,
                label=f"NN UKF  ({pct_n:.1f}%)")
        ax.axhline(sc.n_true, color="r", lw=1.0, label="True n")
        ax.axhline(sc.n_true * 1.10, color="r", lw=0.8, ls="--",
                   label="±10% band")
        ax.axhline(sc.n_true * 0.90, color="r", lw=0.8, ls="--")
        ax.set_xlim(0.0, t_b[-1])
        ax.set_ylim(min(sc.n_true, sc.n_init) * 0.5,
                    max(sc.n_true, sc.n_init) * 1.30)
        ax.set_xlabel("t (s)")
        ax.set_ylabel("n  (sinkage exponent)")
        ax.set_title(sc.label, fontsize=10.5)
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=9, framealpha=0.92)
    fig.suptitle("State-augmented UKF on 3-DOF bicycle "
                 "(Bekker vs NN tyre force, Chrono SCM ground truth)",
                 fontsize=12, y=1.02)
    fig.tight_layout()
    fig.savefig(out_png, dpi=180, bbox_inches="tight")
    print(f"Wrote {out_png}")


def main_dallas():
    out_dir = Path(__file__).resolve().parents[1] / "my_paper" / "paper_figures"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_png = out_dir / "ukf_dallas_validation.png"

    fig, axes = plt.subplots(1, 2, figsize=(11.5, 4.6))
    print(f"\n[Dallas]  {'scenario':<32} {'backend':>8} "
          f"{'converged':>10} {'true':>6} {'pct err':>9}")
    for ax, sc in zip(axes, DALLAS_SCENARIOS):
        t_b, n_b, pct_b = run_dallas(sc, backend="bekker")
        try:
            t_n, n_n, pct_n = run_dallas(sc, backend="nn")
            have_nn = True
        except Exception as exc:
            print(f"          [NN backend unavailable: {exc}]")
            have_nn = False

        conv_b = float(np.mean(n_b[t_b >= sc.T * 0.75]))
        print(f"          {sc.label:<32} {'Bekker':>8} {conv_b:>10.4f} "
              f"{sc.n_true:>6.2f} {pct_b:>8.2f}%")

        if have_nn:
            conv_n = float(np.mean(n_n[t_n >= sc.T * 0.75]))
            print(f"          {sc.label:<32} {'NN':>8} {conv_n:>10.4f} "
                  f"{sc.n_true:>6.2f} {pct_n:>8.2f}%")

        ax.plot(t_b, n_b, color="#1f77b4", lw=1.3,
                label=f"Bekker UKF  ({pct_b:.1f}%)")
        if have_nn:
            ax.plot(t_n, n_n, color="#2ca02c", lw=1.3, ls="-",
                    label=f"NN UKF  ({pct_n:.1f}%)")
        ax.axhline(sc.n_true, color="r", lw=1.0, ls="-", label="True n")
        ax.axhline(sc.n_true * 1.10, color="r", lw=0.8, ls="--",
                   label="±10% band")
        ax.axhline(sc.n_true * 0.90, color="r", lw=0.8, ls="--")
        ax.set_xlim(0.0, sc.T)
        ax.set_ylim(min(sc.n_true, sc.n_init) * 0.7,
                    max(sc.n_true, sc.n_init) * 1.15)
        ax.set_xlabel("t (s)")
        ax.set_ylabel("n  (sinkage exponent)")
        ax.set_title(sc.label, fontsize=10.5)
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=9, framealpha=0.92)
    fig.suptitle("State-augmented UKF on 3-DOF bicycle "
                 "(Bekker vs NN tyre force — reproducing Dallas 2021 Fig. 2)",
                 fontsize=12, y=1.02)
    fig.tight_layout()
    fig.savefig(out_png, dpi=180, bbox_inches="tight")
    print(f"Wrote {out_png}")


if __name__ == "__main__":
    main()
    main_dallas()
