#!/usr/bin/env python3
"""
Analytical Tire Models (CasADi symbolic)
=========================================

Shared CasADi-symbolic tire force functions used by both the CasADi+IPOPT
the ACADOS MPC solver.

Each function takes CasADi symbolic slip angles and normal forces and returns
``(Fyf, Fyr, Fx_traction)`` as CasADi expressions suitable for embedding in
an NLP or OCP.

Supported models:
  - Pacejka Magic Formula (simplified single-parameter set from HMMWV_Pac02Tire.tir)
  - TMeasy degressive model (smooth sin-based approximation)

Note: The linear cornering stiffness model has been removed.  It is only
valid at small slip angles and provides no advantage over Pacejka or TMeasy
for the SCM deformable-terrain scenarios this codebase targets.  The
StatePredictor in mpc_helpers.py uses its own internal linear bicycle model
for delay compensation only (not exposed as a selectable tire model).
"""

import casadi as ca

# ============================================================================
# Default parameters: SINGLE GLOBAL, SCM-calibrated friction.
# The peak friction coefficient mu is the dominant terrain-dependent
# parameter and was the one egregiously wrong before: the old rigid-road
# default mu=0.74 over-predicts grip ~2x, so the analytical baselines planned
# over-aggressively and blew out on curvy references. We calibrate mu to the
# SCM value -- mu=0.42, the mean lateral force coefficient |Fy|/Fz across the
# whole soil box (data/tire_rig/scm_static_100k_v4.csv; SCM peak ranges
# ~0.18 clay-like to ~0.47 sand-like, see calibrate_analytical_tires.py) --
# and use a SINGLE global value for every terrain because the controller has
# no per-terrain knowledge (the fair, deployment-consistent baseline). The
# magic-formula / TMeasy shape factors are kept at their standard values so
# the baselines retain realistic cornering stiffness (a pure force-magnitude
# refit collapses the initial slope and cripples closed-loop tracking, which
# would unfairly sandbag them). A single global mu still cannot track
# terrain-to-terrain force variation -- that residual gap is what the NN
# surrogate closes. Per-terrain "oracle" params below are a separate
# upper-bound reference.
# ============================================================================

# Pacejka Magic Formula (standard shape; SCM-calibrated peak friction)
PACEJKA_B = 8.77    # Stiffness factor (|PKY1|/(PCY1*PDY1)) -- standard
PACEJKA_C = 1.5874  # Shape factor (PCY1) -- standard
PACEJKA_E = 0.376   # Curvature factor (PEY1) -- standard
PACEJKA_MU = 0.42   # Peak friction calibrated to SCM (was 0.74 rigid-road)

# TMeasy (standard lateral shape; mu drives traction, calibrated to SCM)
TMEASY_DFY0 = 40000.0       # Initial slope (N/rad per tire)
TMEASY_FYM = 4000.0         # Peak lateral force per tire (N) ~ 0.40*Fz, matches SCM
TMEASY_ALPHA_M = 0.12       # Slip angle at peak (~7 deg)
TMEASY_ALPHA_SLIDE = 0.25   # Slip angle at full sliding (~14 deg)


# ============================================================================
# Terrain-specific ("oracle") Pacejka parameters
# ============================================================================
# These are physically motivated fits for SCM terrain types.  They represent
# the performance ceiling for a perfectly-calibrated analytical model on each
# terrain — the comparison upper bound used in paper evaluations.
#
# Compared with the rigid-terrain defaults above:
#   mu   → terrain friction coefficient: tan(mohr_friction_angle).
#           This is the dominant effect: soft soils have much lower peak Fy.
#   B    → cornering stiffness factor: lower on soft soils because deformation
#           absorbs lateral load more gradually before reaching peak.
#
# Note: Even with these "oracle" params, Pacejka cannot capture SCM-specific
# effects (Bekker pressure-sinkage saturation, cohesion-enhanced traction,
# non-linear slip stiffness), which is why the NN surrogate outperforms it
# on clay in particular.
#
# Reference: tan(phi) for clay=13°, sand=30°, dirt=29° (see param_consistency.py)
PACEJKA_ORACLE = {
    'clay': {
        'B': 5.5,     # Lower stiffness — SCM clay deforms under lateral load
        'C': 1.5874,
        'E': 0.376,
        'mu': 0.231,  # tan(13°) — very low traction
    },
    'sand': {
        'B': 7.5,     # Moderate stiffness — granular material, still deformable
        'C': 1.5874,
        'E': 0.376,
        'mu': 0.577,  # tan(30°)
    },
    'dirt': {
        'B': 7.5,     # Similar to sand
        'C': 1.5874,
        'E': 0.376,
        'mu': 0.554,  # tan(29°)
    },
}


def get_oracle_pacejka_params(terrain_name: str) -> dict:
    """Return oracle Pacejka parameter dict for *terrain_name*.

    These parameters use the terrain's Mohr–Coulomb friction angle as the
    peak friction coefficient, representing the best achievable Pacejka fit
    without full data-driven calibration.

    Args:
        terrain_name: One of 'clay', 'sand', 'dirt'.

    Returns:
        Dict with keys 'B', 'C', 'E', 'mu' suitable for
        ``pacejka_tire_forces(**params)``.
    """
    if terrain_name not in PACEJKA_ORACLE:
        raise ValueError(
            f"No oracle params for terrain {terrain_name!r}. "
            f"Available: {list(PACEJKA_ORACLE.keys())}"
        )
    return dict(PACEJKA_ORACLE[terrain_name])


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
# Dispatch helper
# ============================================================================

def get_tire_forces(tire_model, alpha_f, alpha_r, Fz_f_axle, Fz_r_axle, kappa,
                    **params):
    """Dispatch to the appropriate tire model by name.

    Args:
        tire_model: One of 'pacejka', 'tmeasy'.
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
    else:
        raise ValueError(f"Unknown tire model: {tire_model!r}. "
                         f"Choose from: pacejka, tmeasy")
