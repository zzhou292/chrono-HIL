#!/usr/bin/env python3
"""
Parameter consistency for SCM_Teleop: NN training, data collection, and MPC/demo.
================================================================================
**THE** single source of truth for:
- NN training data ranges (must match data_collection/collect_scm_data_fast.cpp ParameterRanges)
- Vehicle parameters (must match MPC and Chrono HMMWV usage)
- Terrain presets (all hardcoded terrain configs live here)
- Terrain config validation (ensure demo soil is within training range)
- Steering excitation defaults for terrain estimation

Import from here instead of hardcoding values in individual files.
"""

from __future__ import annotations

import math
import random
from typing import Dict, Any, List, Sequence, Tuple

# =============================================================================
# NN training data ranges — v6 format
# Must match data_collection/collect_scm_data_fast.cpp ParameterRanges struct EXACTLY.
# These are the ranges used during Latin Hypercube sampling for data collection.
# =============================================================================
TRAINING_RANGES_V6 = {
    # Operating conditions
    "slip_ratio":     (-1.0, 1.0),            # dimensionless
    "slip_angle":     (-0.6, 0.6),            # rad (~-34.4° to 34.4°)
    "velocity":       (2.0, 10.0),            # m/s
    "vertical_load":  (1500.0, 7500.0),       # N per wheel (HMMWV-adjusted from reference 500-5500)
    "steering_rate":  (-0.56, 0.56),          # rad/s — critical transient input

    # Bekker pressure-sinkage
    "bekker_Kphi":    (0.5e6, 4.0e6),         # Pa — covers clay (692k) through stiff soils
    "bekker_Kc":      (0.0, 20000.0),         # Pa — covers clay (13.2k)
    "bekker_n":       (0.3, 1.3),             # dimensionless

    # Mohr-Coulomb
    "mohr_cohesion":  (650.0, 20700.0),       # Pa
    "mohr_friction":  (0.105, 0.66),          # RADIANS in v6 CSV (6° to 37.8°)

    # Janosi (upper 0.025 m matches sand/dirt presets; reference Table I lists 0.024)
    "janosi_shear":   (0.01, 0.025),          # m
}

# Legacy v3 ranges (kept for backward compatibility with older models)
TRAINING_RANGES_V3 = {
    "vertical_load":      (2500.0, 7500.0),   # N
    "slip_angle":         (-0.6, 0.6),        # rad
    "longitudinal_slip":  (-0.12, 0.12),      # slip ratio (narrower in v3)
    "camber_angle":       (-0.087, 0.087),    # rad (~-5° to 5°)
    "velocity":           (0.5, 10.5),        # m/s
    "bekker_Kphi":        (2.0e6, 4.0e6),     # Pa (narrower in v3)
    "bekker_Kc":          (0.0, 10000.0),     # Pa
    "bekker_n":           (1.0, 1.4),         # dimensionless (narrower in v3)
    "mohr_cohesion":      (0.0, 5000.0),      # Pa
    "mohr_friction":      (25.0, 45.0),       # DEGREES in v3 CSV
    "janosi_shear":       (0.01, 0.05),       # m
}

# Default to v6 — all new code should use v6 format
TRAINING_RANGES = TRAINING_RANGES_V6

# NN input column order per format (training CSV and NNCasADi must match)
NN_INPUT_ORDER_V6 = [
    "slip_ratio", "slip_angle", "velocity", "vertical_load", "steering_rate",
    "bekker_Kphi", "bekker_Kc", "bekker_n",
    "mohr_cohesion", "mohr_friction", "janosi_shear",
]

NN_INPUT_ORDER_V3 = [
    "vertical_load", "slip_angle", "longitudinal_slip", "camber_angle", "velocity",
    "bekker_Kphi", "bekker_Kc", "bekker_n",
    "mohr_cohesion", "mohr_friction", "janosi_shear",
]

# Default to v6
NN_INPUT_ORDER = NN_INPUT_ORDER_V6

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
# Queried from HMMWV_Full::GetVehicle() at runtime:
#   Mass:  GetVehicle().GetMass() = 2573 kg
#   Izz:   GetChassisBody().GetInertiaXX().z = 3570 kg*m^2
#   Lf:    front spindle x (1.6486) - CG x (0.056) = 1.593 m
#   Lr:    CG x (0.056) - rear spindle x (-1.6534) = 1.709 m
# =============================================================================
HMMWV_VEHICLE_PARAMS = {
    "M": 2573.0,       # kg
    "Izz": 3570.0,     # kg*m^2
    "Lf": 1.593,       # m — front axle to CG
    "Lr": 1.709,       # m — CG to rear axle
    "L": 3.302,        # m — Lf + Lr (wheelbase)
    "h_cg": 0.65,      # m — CG height above ground (for longitudinal load transfer)
    "T": 1.8194,        # m — track width (spindle-to-spindle, from Chrono HMMWV_Full)
}

# Tire radius (used in C++ collector for slip ratio computation)
HMMWV_TIRE_RADIUS_M = 0.47

# Max steering angle (from GetVehicle().GetMaxSteeringAngle())
HMMWV_MAX_STEER_ANGLE_RAD = 0.528  # ~30.25°

# =============================================================================
# Terrain presets — THE canonical definitions.
# All friction_angle values are in DEGREES (matches SCM SetSoilParameters API).
# Literature values from Wong/Bekker terramechanics references.
# =============================================================================
TERRAIN_PRESETS = {
    "clay": {
        "Kphi": 692200,           # Pa/m^n — 692.2 kPa/m^n
        "Kc": 13200,              # Pa/m^(n-1) — 13.2 kPa/m^(n-1)
        "n": 0.5,
        "cohesion": 4140,         # Pa — 4.14 kPa
        "friction_angle": 13.0,   # deg
        "janosi_shear": 0.01,     # m
        "elastic_stiffness": 2e8,
        "damping": 3e4,
        "description": "Clayey soil (literature)",
    },
    "sand": {
        "Kphi": 1523400,          # Pa/m^n — 1523.4 kPa/m^n
        "Kc": 900,                # Pa/m^(n-1) — 0.9 kPa/m^(n-1)
        "n": 1.1,
        "cohesion": 1000,         # Pa — 1.0 kPa
        "friction_angle": 30.0,
        "janosi_shear": 0.025,    # m
        "elastic_stiffness": 2e8,
        "damping": 3e4,
        "description": "Dry sand (literature)",
    },
    "dirt": {
        "Kphi": 1515000,          # Pa/m^n — 1515.0 kPa/m^n
        "Kc": 5300,               # Pa/m^(n-1) — 5.3 kPa/m^(n-1)
        "n": 0.7,
        "cohesion": 1700,         # Pa — 1.7 kPa
        "friction_angle": 29.0,
        "janosi_shear": 0.025,    # m
        "elastic_stiffness": 2e8,
        "damping": 3e4,
        "description": "Sandy loam (literature)",
    },
}

# Latin hypercube axis order: same six soil parameters as TRAINING_RANGES_V6 / rig LHS.
# The box encloses clay, sand, and dirt presets (see __main__ checks).
TERRAIN_LHS_V6_ORDER: Tuple[str, ...] = (
    "bekker_Kphi",
    "bekker_Kc",
    "bekker_n",
    "mohr_cohesion",
    "mohr_friction",
    "janosi_shear",
)

# =============================================================================
# =============================================================================
# Terrain topology presets — bumpiness levels 1–10.
# These define ONLY the Perlin-noise heightmap parameters (independent of soil type).
# Select via --bumpiness N on the CLI, combine with --terrain for soil type.
# =============================================================================
TOPOLOGY_LEVELS = {
    1:  {"description": "Nearly flat",    "bump_amplitude": 0.03, "bump_wavelength": 25.0, "bump_octaves": 1, "bump_max_slope": 0.08},
    2:  {"description": "Gentle",         "bump_amplitude": 0.08, "bump_wavelength": 22.0, "bump_octaves": 2, "bump_max_slope": 0.12},
    3:  {"description": "Mild",           "bump_amplitude": 0.15, "bump_wavelength": 18.0, "bump_octaves": 3, "bump_max_slope": 0.18},
    4:  {"description": "Moderate",       "bump_amplitude": 0.25, "bump_wavelength": 14.0, "bump_octaves": 3, "bump_max_slope": 0.25},
    5:  {"description": "Bumpy",          "bump_amplitude": 0.35, "bump_wavelength": 11.0, "bump_octaves": 4, "bump_max_slope": 0.30},
    6:  {"description": "Rough",          "bump_amplitude": 0.45, "bump_wavelength":  9.0, "bump_octaves": 4, "bump_max_slope": 0.40},
    7:  {"description": "Very rough",     "bump_amplitude": 0.55, "bump_wavelength":  7.0, "bump_octaves": 5, "bump_max_slope": 0.50},
    8:  {"description": "Rocky",          "bump_amplitude": 0.70, "bump_wavelength":  6.0, "bump_octaves": 5, "bump_max_slope": 0.60},
    9:  {"description": "Severe",         "bump_amplitude": 0.85, "bump_wavelength":  5.0, "bump_octaves": 6, "bump_max_slope": 0.70},
    10: {"description": "Extreme",        "bump_amplitude": 1.00, "bump_wavelength":  4.0, "bump_octaves": 6, "bump_max_slope": 0.80},
}


def get_bumpiness_params(level: int, seed: int = 12345) -> dict:
    """Return Perlin-noise bump parameters for a given bumpiness level (0-10).

    Level 0 = flat (no heightmap).  Levels 1-10 map to TOPOLOGY_LEVELS.

    Returns dict with keys: bump_amplitude, bump_wavelength, bump_octaves,
                            bump_max_slope, bump_seed, description.
    """
    if level <= 0:
        return {"bump_amplitude": 0.0, "bump_wavelength": 20.0,
                "bump_octaves": 1, "bump_max_slope": 0.05,
                "bump_seed": seed, "description": "Flat"}
    level = min(level, 10)
    params = dict(TOPOLOGY_LEVELS[level])  # copy
    params["bump_seed"] = seed
    return params


# Steering excitation defaults for terrain estimation
# Reference paper: "sinusoidal steering commands, steering fully in both directions"
# =============================================================================
EXCITATION_DEFAULTS = {
    "steer_amp_rad": 0.35,       # ~20° — aggressive but won't spin out
    "steer_freq_hz": 0.15,       # slow sinusoid (6.7s period) — smooth transitions
    "steer_ramp_s": 3.0,         # half-cosine ramp-up over 3 seconds
    "throttle": 0.5,             # base throttle (PI controller adjusts around this)
}

# =============================================================================
# Helper functions
# =============================================================================

def get_static_fz_per_wheel(vehicle_params: Dict[str, float] | None = None) -> Tuple[float, float]:
    """Return (Fz_front_per_wheel, Fz_rear_per_wheel) in N for static weight distribution."""
    p = vehicle_params or HMMWV_VEHICLE_PARAMS
    M, Lf, Lr = p["M"], p["Lf"], p["Lr"]
    L = Lf + Lr
    Fz_f_axle = M * 9.81 * Lr / L
    Fz_r_axle = M * 9.81 * Lf / L
    return Fz_f_axle / 2.0, Fz_r_axle / 2.0


def get_vehicle_params_for_demo() -> Dict[str, float]:
    """Vehicle params for the MPC when running Chrono HMMWV demo."""
    return dict(HMMWV_VEHICLE_PARAMS)


def get_terrain_preset(name: str) -> Dict[str, Any]:
    """Get a terrain preset dict by name. Raises KeyError if not found."""
    if name not in TERRAIN_PRESETS:
        raise KeyError(f"Unknown terrain preset '{name}'. "
                       f"Available: {list(TERRAIN_PRESETS.keys())}")
    return dict(TERRAIN_PRESETS[name])


def latin_hypercube_unit(n: int, d: int, rng: random.Random) -> List[List[float]]:
    """Classic LHS on the unit hypercube: n points in dimension d, one per stratum per axis."""
    if n < 1 or d < 1:
        raise ValueError("n and d must be positive")
    u = [[rng.random() for _ in range(d)] for _ in range(n)]
    per_dim = [list(range(n)) for _ in range(d)]
    for j in range(d):
        rng.shuffle(per_dim[j])
    out: List[List[float]] = []
    for i in range(n):
        row = []
        for j in range(d):
            a = per_dim[j][i]
            row.append((a + u[i][j]) / n)
        out.append(row)
    return out


def terrain_yaml_dict_from_lhs_unit_row(
    unit_row: Sequence[float],
    *,
    elastic_stiffness: float = 2e8,
    damping: float = 3e4,
    description: str = "",
) -> Dict[str, Any]:
    """
    Map one LHS sample in [0,1]^6 (order TERRAIN_LHS_V6_ORDER) to keys expected by
    ``chrono_setup.load_terrain_config`` / SCM YAML (friction in degrees).
    """
    if len(unit_row) != len(TERRAIN_LHS_V6_ORDER):
        raise ValueError(
            f"unit_row must have length {len(TERRAIN_LHS_V6_ORDER)}, got {len(unit_row)}"
        )
    out: Dict[str, Any] = {
        "elastic_stiffness": elastic_stiffness,
        "damping": damping,
    }
    if description:
        out["description"] = description
    for u, key in zip(unit_row, TERRAIN_LHS_V6_ORDER):
        lo, hi = TRAINING_RANGES_V6[key]
        v = lo + float(u) * (hi - lo)
        if key == "bekker_Kphi":
            out["Kphi"] = v
        elif key == "bekker_Kc":
            out["Kc"] = v
        elif key == "bekker_n":
            out["n"] = v
        elif key == "mohr_cohesion":
            out["cohesion"] = v
        elif key == "mohr_friction":
            out["friction_angle"] = math.degrees(v)
        elif key == "janosi_shear":
            out["janosi_shear"] = v
    return out


def generate_lhs_terrain_yaml_dicts(
    n_samples: int,
    *,
    seed: int = 0,
    elastic_stiffness: float = 2e8,
    damping: float = 3e4,
) -> List[Dict[str, Any]]:
    """
    ``n_samples`` Latin-hypercube draws over TRAINING_RANGES_V6 soil parameters.

    Same hull as ``data_collection`` / v6 rig sampling; includes the region around
    clay, sand, and dirt presets in ``TERRAIN_PRESETS``.
    """
    rng = random.Random(seed)
    d = len(TERRAIN_LHS_V6_ORDER)
    unit = latin_hypercube_unit(n_samples, d, rng)
    return [
        terrain_yaml_dict_from_lhs_unit_row(
            row,
            elastic_stiffness=elastic_stiffness,
            damping=damping,
            description=f"LHS v6 soil sample {i + 1}/{n_samples} (seed={seed})",
        )
        for i, row in enumerate(unit)
    ]


def terrain_preset_to_internal(preset: Dict[str, Any]) -> Dict[str, Any]:
    """Convert preset keys (cohesion, friction_angle, janosi_shear) to internal keys (c, phi, k).

    The returned dict uses the short keys expected by NNCasADi and terrain estimators:
    Kphi, Kc, n, c, phi (degrees), k.
    """
    return {
        "Kphi": preset["Kphi"],
        "Kc": preset["Kc"],
        "n": preset["n"],
        "c": preset["cohesion"],
        "phi": preset["friction_angle"],        # degrees
        "k": preset["janosi_shear"],
    }


# =============================================================================
# Terrain validation: is a config within NN training range?
# =============================================================================
def check_terrain_in_training_range(
    terrain: Dict[str, Any],
    *,
    model_format: str = "v6",
    keys: Dict[str, str] | None = None,
) -> Tuple[bool, List[str]]:
    """
    Check whether terrain parameters lie within the NN training data ranges.

    Args:
        terrain: Dict with terrain keys (Kphi/Kc/n + cohesion/friction_angle/janosi_shear
                 OR c/phi/k).
        model_format: 'v6' (default) or 'v3'. Selects which TRAINING_RANGES to use.
        keys: Optional mapping override.

    Returns:
        (all_ok, list of warning/error messages)
    """
    ranges = TRAINING_RANGES_V6 if model_format == "v6" else TRAINING_RANGES_V3
    key_map = keys or TERRAIN_CONFIG_TO_NN
    msgs: List[str] = []
    all_ok = True

    for config_key, range_key in key_map.items():
        if config_key not in terrain:
            continue
        val = float(terrain[config_key])
        if range_key not in ranges:
            continue

        # v6 stores mohr_friction in radians; terrain configs use degrees.
        # Convert for comparison when checking v6 ranges.
        if model_format == "v6" and range_key == "mohr_friction":
            val = math.radians(val)

        lo, hi = ranges[range_key]
        if val < lo or val > hi:
            all_ok = False
            if model_format == "v6" and range_key == "mohr_friction":
                msgs.append(
                    f"{config_key}={math.degrees(val):.1f}° ({val:.4f} rad) "
                    f"is outside training range [{math.degrees(lo):.1f}°, {math.degrees(hi):.1f}°]"
                )
            else:
                msgs.append(
                    f"{config_key}={terrain[config_key]} is outside training range [{lo}, {hi}]"
                )

    return all_ok, msgs


def assert_terrain_in_training_range(terrain: Dict[str, Any], model_format: str = "v6") -> None:
    """Raise ValueError if any terrain parameter is outside training range."""
    ok, msgs = check_terrain_in_training_range(terrain, model_format=model_format)
    if not ok:
        raise ValueError(
            "Terrain config outside NN training range:\n  " + "\n  ".join(msgs)
        )


# =============================================================================
# Self-test
# =============================================================================
if __name__ == "__main__":
    print("=" * 60)
    print("Parameter Consistency Validation")
    print("=" * 60)

    # Check all presets against v6 training ranges
    for name, preset in TERRAIN_PRESETS.items():
        ok, msgs = check_terrain_in_training_range(preset, model_format="v6")
        status = "OK" if ok else "OUT OF RANGE"
        print(f"\n  {name}: {status}")
        for m in msgs:
            print(f"    - {m}")

    # Vehicle static load check
    Fz_f, Fz_r = get_static_fz_per_wheel()
    print(f"\nHMMWV static Fz per wheel: front={Fz_f:.0f} N, rear={Fz_r:.0f} N")
    lo, hi = TRAINING_RANGES_V6["vertical_load"]
    print(f"v6 training load range: [{lo:.0f}, {hi:.0f}] N")
    in_range = lo <= Fz_f <= hi and lo <= Fz_r <= hi
    print(f"Both within range: {in_range}")

    # Quick NN input order check
    print(f"\nv6 NN inputs ({len(NN_INPUT_ORDER_V6)}): {NN_INPUT_ORDER_V6}")
    print(f"v3 NN inputs ({len(NN_INPUT_ORDER_V3)}): {NN_INPUT_ORDER_V3}")
