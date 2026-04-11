#!/usr/bin/env python3
"""
Sensor-realistic tire NN inputs (train = inference contract).

All quantities here are computable on a real vehicle from:
  IMU (ax, ω), wheel-speed / fused speed (u, optionally v), steering encoder (δ),
  and fixed vehicle geometry — i.e. the same estimators MPC already uses.

Use this module from:
  - acados_mpc_controller_node (logging + NN feature consistency)
  - Offline dataset builders / validators
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, List, Literal, Tuple

import numpy as np

from param_consistency import HMMWV_VEHICLE_PARAMS

KappaMode = Literal["zero", "approx"]


@dataclass(frozen=True)
class VehicleGeometry:
    """Bicycle + load-transfer geometry (matches AcadosMPC / HMMWV defaults)."""

    Lf: float
    Lr: float
    M: float
    h_cg: float
    T: float

    @classmethod
    def from_hmmwv_defaults(cls) -> "VehicleGeometry":
        vp = HMMWV_VEHICLE_PARAMS
        return cls(
            Lf=float(vp["Lf"]),
            Lr=float(vp["Lr"]),
            M=float(vp["M"]),
            h_cg=float(vp["h_cg"]),
            T=float(vp["T"]),
        )


@dataclass
class BicycleOperatingPoint:
    """One timestep of MPC-style tire inputs (per axle mean wheel, front row for logging)."""

    kappa: float
    alpha_f: float
    alpha_r: float
    u_safe: float
    Fz_f: float  # mean front wheel vertical load (N), model-based
    Fz_r: float
    steering_rate_cmd: float  # δ̇ used as NN steering_rate feature (rad/s)


def compute_bicycle_operating_point(
    steering_angle_rad: float,
    u_body: float,
    v_body: float,
    omega: float,
    ax_body: float,
    *,
    geom: VehicleGeometry,
    kappa_mode: KappaMode = "zero",
    g: float = 9.81,
) -> Tuple[float, float, float, float, float, float]:
    """
    Returns (kappa, alpha_f, alpha_r, u_safe, Fz_f, Fz_r).

    Matches acados_mpc_controller_node pre-solve convention (no delay-comp shift here;
    pass already-compensated state if you want exact parity with z0 after predictor).
    """
    u_safe = float(max(abs(u_body), 0.5))
    alpha_f = float(
        steering_angle_rad - math.atan2(v_body + geom.Lf * omega, u_safe)
    )
    alpha_r = float(-math.atan2(v_body - geom.Lr * omega, u_safe))
    L = geom.Lf + geom.Lr
    Fz_f = float((geom.M * g * geom.Lr - geom.M * ax_body * geom.h_cg) / L / 2.0)
    Fz_r = float((geom.M * g * geom.Lf + geom.M * ax_body * geom.h_cg) / L / 2.0)
    if kappa_mode == "approx":
        kappa = float(np.clip(ax_body / (0.4 * 9.81), -0.3, 0.3))
    else:
        kappa = 0.0
    return kappa, alpha_f, alpha_r, u_safe, Fz_f, Fz_r


def lateral_load_transfer_dFz(
    u_body: float,
    omega: float,
    *,
    geom: VehicleGeometry,
) -> float:
    """Lateral load transfer half-amplitude per side (N), ay ≈ u*omega."""
    ay = u_body * omega
    return float(geom.M * ay * geom.h_cg / geom.T / 2.0)


def fz_with_lateral_transfer(
    Fz_f_mean: float,
    Fz_r_mean: float,
    dFz: float,
) -> Tuple[float, float, float, float]:
    """Outer/inner Fz clamps matching acados_mpc_solver / controller diagnostics."""
    Fz_fo = min(Fz_f_mean + dFz, 1.9 * Fz_f_mean)
    Fz_fi = max(Fz_f_mean - dFz, 0.1 * Fz_f_mean)
    Fz_ro = min(Fz_r_mean + dFz, 1.9 * Fz_r_mean)
    Fz_ri = max(Fz_r_mean - dFz, 0.1 * Fz_r_mean)
    return Fz_fo, Fz_fi, Fz_ro, Fz_ri


def terrain_internal_to_bekker_columns(
    terrain_params: Dict[str, float],
) -> List[float]:
    """Six terrain scalars (bekker_*, mohr_*, janosi_*) in training CSV order."""
    return [
        float(terrain_params["Kphi"]),
        float(terrain_params["Kc"]),
        float(terrain_params["n"]),
        float(terrain_params["c"]),
        float(terrain_params["phi"]),
        float(terrain_params["k"]),
    ]


def write_vehicle_tire_csv_header() -> List[str]:
    """Header for vehicle-collected training CSV (matches train_temporal_nn / train_rate_nn)."""
    return [
        "scenario_id",
        "timestep",
        "slip_ratio",
        "slip_angle",
        "velocity",
        "vertical_load",
        "steering_rate",
        "bekker_Kphi",
        "bekker_Kc",
        "bekker_n",
        "mohr_cohesion",
        "mohr_friction",
        "janosi_shear",
        "mesh_spacing",
        "Fx",
        "Fy",
    ]


def pack_vehicle_tire_csv_row(
    scenario_id: int,
    timestep: float,
    kappa: float,
    alpha_f: float,
    u_safe: float,
    Fz_f: float,
    steering_rate: float,
    terrain_params: Dict[str, float],
    fx_label: float,
    fy_label: float,
    *,
    mesh_spacing: float = 0.04,
) -> List[float]:
    """One row: NN inputs (MPC-style estimates) + labels (sim tire forces, not inputs)."""
    ter6 = terrain_internal_to_bekker_columns(terrain_params)
    return [
        int(scenario_id),
        float(timestep),
        float(kappa),
        float(alpha_f),
        float(u_safe),
        float(Fz_f),
        float(steering_rate),
        *ter6,
        float(mesh_spacing),
        float(fx_label),
        float(fy_label),
    ]
