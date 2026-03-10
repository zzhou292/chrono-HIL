#!/usr/bin/env python3
"""
Parameter consistency for SCM_Teleop: NN training, data collection, and MPC/demo.
================================================================================
Single source of truth for:
- NN training data ranges (must match collect_scm_data*.cpp LHS ranges)
- Vehicle parameters (must match MPC and Chrono HMMWV usage)
- Terrain config validation (ensure demo soil is within training range)

Use this module to avoid mismatches that cause poor NN-MPC performance.
"""

from __future__ import annotations

from typing import Dict, Any, List, Tuple

# =============================================================================
# NN training data ranges (must match cpp_collect/collect_scm_data.cpp and
# collect_scm_data_fast.cpp ScaleSample / ParameterRanges)
# EXPANDED to cover clay/sand/dirt terrains for UKF terrain estimation
# =============================================================================
TRAINING_RANGES = {
    "vertical_load": (2500.0, 7500.0),       # N (per wheel)
    "slip_angle": (-0.6, 0.6),               # rad (~ -34° to 34°, Dallas et al.)
    "longitudinal_slip": (-0.12, 0.12),      # slip ratio
    "camber_angle": (-0.087, 0.087),         # rad (~ -5° to 5°)
    "velocity": (0.5, 10.5),                 # m/s
    "bekker_Kphi": (0.5e6, 4.0e6),           # Pa - expanded for clay (692k)
    "bekker_Kc": (0.0, 20000.0),             # Pa - expanded for clay (13.2k)
    "bekker_n": (0.3, 1.5),                  # expanded for sand (1.38)
    "mohr_cohesion": (0.0, 10000.0),         # Pa - expanded for clay (4140)
    "mohr_friction": (10.0, 45.0),           # degrees - expanded for clay (13°)
    "janosi_shear": (0.005, 0.06),           # m - slightly expanded
}

# NN input column order (training and NNCasADi must match)
NN_INPUT_ORDER = [
    "Fz_or_vertical_load",
    "slip_angle",
    "longitudinal_slip",
    "camber_angle",
    "velocity",
    "bekker_Kphi",
    "bekker_Kc",
    "bekker_n",
    "mohr_cohesion",
    "mohr_friction",
    "janosi_shear",
]

# Terrain keys as used in YAML/config vs internal (Kphi, Kc, n, c, phi, k)
# setup_scm_terrain returns c, phi; YAML uses cohesion, friction_angle
TERRAIN_CONFIG_TO_NN = {
    "Kphi": "bekker_Kphi",
    "Kc": "bekker_Kc",
    "n": "bekker_n",
    "cohesion": "mohr_cohesion",
    "c": "mohr_cohesion",
    "friction_angle": "mohr_friction",
    "phi": "mohr_friction",
    "janosi_shear": "janosi_shear",
    "k": "janosi_shear",
}

# =============================================================================
# Vehicle parameters: match Chrono HMMWV_Full and MPC bicycle model
# HMMWV curb weight ~2700 kg; Lf/Lr from typical HMMWV wheelbase ~2.95 m
# =============================================================================
HMMWV_VEHICLE_PARAMS = {
    "M": 2573.0,       # kg — GetVehicle().GetMass()
    "Izz": 3570.0,     # kg*m^2 — GetChassisBody().GetInertiaXX().z
    "Lf": 1.593,       # m — front spindle x (1.6486) minus CG x (0.056)
    "Lr": 1.709,       # m — CG x (0.056) minus rear spindle x (-1.6534)
    "L": 3.302,        # Lf + Lr
}

# Per-wheel static load range check: 2700 * 9.81 / 4 ≈ 6622 N (within 2500–7500)
def get_static_fz_per_wheel(vehicle_params: Dict[str, float] | None = None) -> Tuple[float, float]:
    """Return (Fz_front_per_wheel, Fz_rear_per_wheel) in N for static weight distribution."""
    p = vehicle_params or HMMWV_VEHICLE_PARAMS
    M, Lf, Lr = p["M"], p["Lf"], p["Lr"]
    L = Lf + Lr
    Fz_f_axle = M * 9.81 * Lr / L
    Fz_r_axle = M * 9.81 * Lf / L
    return Fz_f_axle / 2.0, Fz_r_axle / 2.0


def get_vehicle_params_for_demo() -> Dict[str, float]:
    """Vehicle params to pass to DallasMPC when running Chrono HMMWV demo."""
    return dict(HMMWV_VEHICLE_PARAMS)


# =============================================================================
# Terrain validation: is a config within NN training range?
# =============================================================================
def check_terrain_in_training_range(
    terrain: Dict[str, Any],
    *,
    keys: Dict[str, str] | None = None,
) -> Tuple[bool, List[str]]:
    """
    Check whether terrain parameters lie within the NN training data ranges.
    
    Args:
        terrain: Dict with keys Kphi, Kc, n, cohesion, friction_angle, janosi_shear
                 (or bekker_Kphi, mohr_cohesion, mohr_friction, etc.)
        keys: Optional mapping from terrain key to TRAINING_RANGES key.
              Default uses TERRAIN_CONFIG_TO_NN.
    
    Returns:
        (all_ok, list of warning/error messages)
    """
    key_map = keys or TERRAIN_CONFIG_TO_NN
    msgs: List[str] = []
    all_ok = True
    
    for config_key, range_key in key_map.items():
        if config_key not in terrain:
            continue
        val = float(terrain[config_key])
        if range_key not in TRAINING_RANGES:
            continue
        lo, hi = TRAINING_RANGES[range_key]
        if val < lo or val > hi:
            all_ok = False
            msgs.append(
                f"{config_key}={val} is outside training range [{lo}, {hi}]"
            )
    
    if all_ok and msgs:
        msgs.clear()
    return all_ok, msgs


def assert_terrain_in_training_range(terrain: Dict[str, Any]) -> None:
    """Raise ValueError if any terrain parameter is outside training range."""
    ok, msgs = check_terrain_in_training_range(terrain)
    if not ok:
        raise ValueError(
            "Terrain config outside NN training range:\n  " + "\n  ".join(msgs)
        )


# =============================================================================
# Tire / data collection constants (for reference)
# =============================================================================
# HMMWV tire radius used in C++ collectors for slip ratio: omega = v/r * (1 + slip)
HMMWV_TIRE_RADIUS_M = 0.47

# Data collection CSV: slip_angle and camber_angle in RADIANS; mohr_friction in DEGREES
# NNCasADi and MPC must pass phi (friction angle) in DEGREES to match training.


if __name__ == "__main__":
    # Quick validation of presets
    soft = {"Kphi": 2.1e6, "Kc": 500, "n": 1.38, "cohesion": 300,
            "friction_angle": 26, "janosi_shear": 0.048}
    hard = {"Kphi": 5.0e6, "Kc": 3000, "n": 1.1, "cohesion": 1000,
            "friction_angle": 35, "janosi_shear": 0.01}
    mean = {"Kphi": 3.0e6, "Kc": 5000, "n": 1.2, "cohesion": 2500,
            "friction_angle": 35, "janosi_shear": 0.03}
    
    for name, t in [("soft_soil", soft), ("hard_soil", hard), ("training_mean", mean)]:
        ok, msgs = check_terrain_in_training_range(t)
        print(f"{name}: {'OK' if ok else 'OUT OF RANGE'}")
        for m in msgs:
            print(f"  - {m}")
    
    Fz_f, Fz_r = get_static_fz_per_wheel()
    print(f"\nStatic Fz per wheel (HMMWV): front={Fz_f:.0f} N, rear={Fz_r:.0f} N")
    print(f"Training load range: {TRAINING_RANGES['vertical_load']} N")
    in_range = TRAINING_RANGES["vertical_load"][0] <= Fz_f <= TRAINING_RANGES["vertical_load"][1]
    print(f"Within training range: {in_range}")
