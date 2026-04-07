#!/usr/bin/env python3
"""
Analytical Tire Models (CasADi symbolic)
=========================================

Shared CasADi-symbolic tire force functions used by both the CasADi+IPOPT
(DallasMPC) and ACADOS (AcadosDallasMPC) solvers.

Each function takes CasADi symbolic slip angles and normal forces and returns
``(Fyf, Fyr, Fx_traction)`` as CasADi expressions suitable for embedding in
an NLP or OCP.

Supported models:
  - Pacejka Magic Formula (simplified single-parameter set from HMMWV_Pac02Tire.tir)
  - TMeasy degressive model (smooth sin-based approximation)
  - Linear cornering stiffness (valid for small slip angles only)
"""

import casadi as ca

# ============================================================================
# Default parameters (from HMMWV_Pac02Tire.tir / Dallas paper calibration)
# ============================================================================

# Pacejka Magic Formula
PACEJKA_B = 8.77    # Stiffness factor (|PKY1|/(PCY1*PDY1))
PACEJKA_C = 1.5874  # Shape factor (PCY1)
PACEJKA_E = 0.376   # Curvature factor (PEY1)
PACEJKA_MU = 0.74   # Peak friction (PDY1)

# TMeasy
TMEASY_DFY0 = 40000.0       # Initial slope (N/rad per tire)
TMEASY_FYM = 4000.0         # Peak lateral force per tire (N)
TMEASY_ALPHA_M = 0.12       # Slip angle at peak (~7 deg)
TMEASY_ALPHA_SLIDE = 0.25   # Slip angle at full sliding (~14 deg)

# Linear
LINEAR_CF = 80000.0  # Front cornering stiffness (N/rad)
LINEAR_CR = 80000.0  # Rear cornering stiffness (N/rad)


# ============================================================================
# Combined slip reduction factor
# ============================================================================

def combined_slip_factor(kappa):
    """Lateral force reduction due to combined longitudinal+lateral slip.

    Returns a CasADi expression in [sqrt(0.1), 1.0].
    """
    return ca.sqrt(ca.fmax(1.0 - (kappa / 0.2) ** 2, 0.1))


# ============================================================================
# Pacejka Magic Formula
# ============================================================================

def pacejka_tire_forces(alpha_f, alpha_r, Fz_f_axle, Fz_r_axle, kappa,
                        B=PACEJKA_B, C=PACEJKA_C, E=PACEJKA_E, mu=PACEJKA_MU):
    """Pacejka Magic Formula lateral forces (simplified, per-axle).

    Fy = D * sin(C * atan(B*α − E*(B*α − atan(B*α))))
    where D = μ * Fz_axle * combined_slip_factor

    Returns:
        (Fyf, Fyr, Fx_traction) — CasADi symbolic expressions.
    """
    lat = combined_slip_factor(kappa)

    Df = mu * Fz_f_axle * lat
    Dr = mu * Fz_r_axle * lat

    Baf = B * alpha_f
    Bar = B * alpha_r

    Fyf = Df * ca.sin(C * ca.atan(Baf - E * (Baf - ca.atan(Baf))))
    Fyr = Dr * ca.sin(C * ca.atan(Bar - E * (Bar - ca.atan(Bar))))

    Fx_traction = mu * (Fz_f_axle + Fz_r_axle)
    return Fyf, Fyr, Fx_traction


# ============================================================================
# TMeasy degressive model
# ============================================================================

def tmeasy_tire_forces(alpha_f, alpha_r, Fz_f_axle, Fz_r_axle, kappa,
                       dFy0=TMEASY_DFY0, Fym=TMEASY_FYM,
                       alpha_m=TMEASY_ALPHA_M, alpha_slide=TMEASY_ALPHA_SLIDE,
                       mu=PACEJKA_MU):
    """TMeasy degressive lateral force model (smooth sin-based approx).

    Per-tire force is scaled by (Fz_axle / 2*Fz_nom) to account for load
    transfer, then doubled for the axle total.

    Returns:
        (Fyf, Fyr, Fx_traction) — CasADi symbolic expressions.
    """
    lat = combined_slip_factor(kappa)

    Fz_nom = (Fz_f_axle + Fz_r_axle) / 2.0
    Fz_f_ratio = Fz_f_axle / (2.0 * ca.fmax(Fz_nom, 1.0))
    Fz_r_ratio = Fz_r_axle / (2.0 * ca.fmax(Fz_nom, 1.0))

    half_pi = 1.5707963
    am_safe = ca.fmax(alpha_m, 1e-4)

    Fyf_per_tire = Fym * Fz_f_ratio * lat * ca.sin(
        ca.fmin(half_pi * alpha_f / am_safe, half_pi))
    Fyr_per_tire = Fym * Fz_r_ratio * lat * ca.sin(
        ca.fmin(half_pi * alpha_r / am_safe, half_pi))

    Fyf = 2.0 * Fyf_per_tire
    Fyr = 2.0 * Fyr_per_tire

    Fx_traction = mu * (Fz_f_axle + Fz_r_axle)
    return Fyf, Fyr, Fx_traction


# ============================================================================
# Linear cornering stiffness
# ============================================================================

def linear_tire_forces(alpha_f, alpha_r, Fz_f_axle, Fz_r_axle, kappa,
                       Cf=LINEAR_CF, Cr=LINEAR_CR, mu=PACEJKA_MU):
    """Simple linear cornering stiffness (no saturation).

    Fy = -C * α * combined_slip_factor

    Returns:
        (Fyf, Fyr, Fx_traction) — CasADi symbolic expressions.
    """
    lat = combined_slip_factor(kappa)

    Fyf = -Cf * alpha_f * lat
    Fyr = -Cr * alpha_r * lat

    Fx_traction = mu * (Fz_f_axle + Fz_r_axle)
    return Fyf, Fyr, Fx_traction


# ============================================================================
# Dispatch helper
# ============================================================================

def get_tire_forces(tire_model, alpha_f, alpha_r, Fz_f_axle, Fz_r_axle, kappa,
                    **params):
    """Dispatch to the appropriate tire model by name.

    Args:
        tire_model: One of 'pacejka', 'tmeasy', 'linear'.
        alpha_f, alpha_r: CasADi symbolic slip angles.
        Fz_f_axle, Fz_r_axle: CasADi symbolic axle normal forces.
        kappa: CasADi symbolic longitudinal slip ratio.
        **params: Model-specific overrides (e.g. B, C, E, mu for Pacejka).

    Returns:
        (Fyf, Fyr, Fx_traction)
    """
    if tire_model == 'pacejka':
        return pacejka_tire_forces(alpha_f, alpha_r, Fz_f_axle, Fz_r_axle,
                                   kappa, **params)
    elif tire_model == 'tmeasy':
        return tmeasy_tire_forces(alpha_f, alpha_r, Fz_f_axle, Fz_r_axle,
                                  kappa, **params)
    elif tire_model == 'linear':
        return linear_tire_forces(alpha_f, alpha_r, Fz_f_axle, Fz_r_axle,
                                  kappa, **params)
    else:
        raise ValueError(f"Unknown tire model: {tire_model!r}. "
                         f"Choose from: pacejka, tmeasy, linear")
