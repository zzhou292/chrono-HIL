#!/usr/bin/env python3
"""
Collect per-wheel tire force data from the full HMMWV vehicle on SCM terrain.

Unlike the rig-based ``collect_static_data.cpp`` (single-wheel, steady-state),
this script captures dynamic wheel forces under realistic vehicle-level effects:
weight transfer, suspension compliance, transient tire-terrain coupling, etc.

Output CSV columns match the rig training format so the dataset can be mixed
with or replace rig data for NN tire surrogate training:

    slip_ratio, slip_angle, velocity, vertical_load, steering_rate,
    bekker_Kphi, bekker_Kc, bekker_n, mohr_cohesion, mohr_friction,
    janosi_shear, Fx, Fy

Forces (Fx, Fy) are in the **tire local frame** — same convention as the rig.

Usage:
    conda activate sim
    cd simulation
    python ../data_collection/collect_vehicle_data.py --terrain clay --time 60
    python ../data_collection/collect_vehicle_data.py --terrain dirt --time 60
    python ../data_collection/collect_vehicle_data.py --terrain sand --time 60

    # LHS terrains for broader coverage:
    python ../data_collection/collect_vehicle_data.py --lhs 50 --time 30

    # All three presets in one invocation:
    python ../data_collection/collect_vehicle_data.py --terrain all --time 60
"""

import argparse
import csv
import math
import os
import sys
from pathlib import Path

import numpy as np

# Ensure simulation/ is on path for imports
_sim_dir = str(Path(__file__).resolve().parent.parent / "simulation")
if _sim_dir not in sys.path:
    sys.path.insert(0, _sim_dir)

import pychrono as chrono
import pychrono.vehicle as veh

from chrono_setup import setup_chrono_vehicle, setup_scm_terrain
from param_consistency import (
    TERRAIN_PRESETS,
    TRAINING_RANGES_V6,
    get_terrain_preset,
    generate_lhs_terrain_yaml_dicts,
)


# ─── steering / throttle trajectory generators ──────────────────────────

def _sinusoidal_steering(t, amp, period):
    """Sinusoidal steering command in [-1, 1]."""
    return amp * math.sin(2 * math.pi * t / period)


def _chirp_steering(t, amp, period_start, period_end, duration):
    """Linear chirp: frequency ramps from 1/period_start to 1/period_end."""
    f0 = 1.0 / period_start
    f1 = 1.0 / period_end
    f = f0 + (f1 - f0) * t / duration
    return amp * math.sin(2 * math.pi * f * t)


def _step_steering(t, amp, switch_every):
    """Alternating step inputs."""
    idx = int(t / switch_every)
    return amp if idx % 2 == 0 else -amp


class TrajectorySet:
    """Runs a batch of different steering excitations sequentially."""

    def __init__(self, total_time: float):
        self._total = total_time
        # Split time equally among patterns
        self._seg = total_time / 4.0

    def steering(self, t: float) -> float:
        seg = self._seg
        if t < seg:
            # Slow sinusoid — low freq excitation
            return _sinusoidal_steering(t, 0.6, 6.0)
        elif t < 2 * seg:
            # Fast sinusoid — high freq excitation
            t2 = t - seg
            return _sinusoidal_steering(t2, 0.4, 2.0)
        elif t < 3 * seg:
            # Chirp — sweeps frequency range
            t3 = t - 2 * seg
            return _chirp_steering(t3, 0.5, 8.0, 1.5, seg)
        else:
            # Step inputs — extreme transients
            t4 = t - 3 * seg
            return _step_steering(t4, 0.5, 3.0)

    def throttle(self, t: float) -> float:
        # Vary throttle slightly to cover different speeds
        base = 0.5
        if t > self._total * 0.75:
            base = 0.3  # slow down for step section
        return base + 0.1 * math.sin(2 * math.pi * t / 20.0)


# ─── data extraction ────────────────────────────────────────────────────

WHEEL_MAP = [
    (0, veh.LEFT, "front_left"),
    (0, veh.RIGHT, "front_right"),
    (1, veh.LEFT, "rear_left"),
    (1, veh.RIGHT, "rear_right"),
]


def _extract_wheel_row(vehicle, terrain, axle_idx, side_idx, label,
                        terrain_params, u_body, steering_rate):
    """Extract one training row for a single wheel."""
    veh_obj = vehicle.GetVehicle()
    tire = veh_obj.GetTire(axle_idx, side_idx)
    chassis = vehicle.GetChassisBody()

    # Tire-frame forces: rotate global force into spindle (wheel) frame
    force_global = tire.ReportTireForce(terrain)
    spindle_rot = veh_obj.GetSpindleRot(axle_idx, side_idx)
    f_tire = spindle_rot.RotateBack(force_global.force)

    # Operating conditions
    slip_angle = tire.GetSlipAngle()
    long_slip = tire.GetLongitudinalSlip()
    Fz = abs(f_tire.z)

    return {
        "slip_ratio": float(long_slip),
        "slip_angle": float(slip_angle),
        "velocity": float(max(abs(u_body), 0.5)),
        "vertical_load": float(Fz),
        "steering_rate": float(steering_rate),
        "bekker_Kphi": terrain_params["Kphi"],
        "bekker_Kc": terrain_params["Kc"],
        "bekker_n": terrain_params["n"],
        "mohr_cohesion": terrain_params["c"],
        "mohr_friction": math.radians(terrain_params["phi"]),  # store in radians (rig convention)
        "janosi_shear": terrain_params["k"],
        "Fx": float(f_tire.x),
        "Fy": float(f_tire.y),
    }


# ─── main sim loop ──────────────────────────────────────────────────────

def collect_one_terrain(terrain_config, total_time, output_path, step_size=5e-4,
                         log_interval=0.02):
    """Run one sim on a given terrain config and collect per-wheel data."""

    print(f"\n{'='*60}")
    print(f"Collecting vehicle data: {terrain_config.get('description', 'custom')}")
    print(f"  n={terrain_config['n']:.2f}, time={total_time}s → {output_path}")
    print(f"{'='*60}")

    # ── Setup Chrono ────────────────────────────────────────────
    system, vehicle = setup_chrono_vehicle(visualize=False)
    terrain, tp = setup_scm_terrain(
        system, vehicle=vehicle, visualize=False,
        terrain_config=terrain_config,
    )

    # Simple direct driver
    driver = veh.ChDriver(vehicle.GetVehicle())

    traj = TrajectorySet(total_time)

    # CSV setup
    fieldnames = [
        "slip_ratio", "slip_angle", "velocity", "vertical_load", "steering_rate",
        "bekker_Kphi", "bekker_Kc", "bekker_n", "mohr_cohesion", "mohr_friction",
        "janosi_shear", "Fx", "Fy",
    ]

    rows_written = 0
    last_log_time = -1.0
    prev_steer = 0.0
    prev_t = 0.0

    with open(output_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()

        t = 0.0
        settle_time = 2.0  # let vehicle settle on terrain before recording
        while t < total_time + settle_time:
            steer_cmd = traj.steering(max(t - settle_time, 0.0)) if t > settle_time else 0.0
            throttle_cmd = traj.throttle(max(t - settle_time, 0.0)) if t > settle_time else 0.3

            driver_inputs = veh.DriverInputs()
            driver_inputs.m_steering = float(np.clip(steer_cmd, -1.0, 1.0))
            driver_inputs.m_throttle = float(np.clip(throttle_cmd, 0.0, 1.0))
            driver_inputs.m_braking = 0.0

            # Synchronize
            driver.Synchronize(t)
            terrain.Synchronize(t)
            vehicle.Synchronize(t, driver_inputs, terrain)

            # Advance
            driver.Advance(step_size)
            terrain.Advance(step_size)
            vehicle.Advance(step_size)

            # Body-frame velocity
            chassis = vehicle.GetChassisBody()
            vel_loc = chassis.GetRot().RotateBack(chassis.GetPosDt())
            u_body = vel_loc.x

            # Steering rate
            dt = t - prev_t if t > prev_t else step_size
            steering_rate = (steer_cmd - prev_steer) / max(dt, 1e-6)
            prev_steer = steer_cmd
            prev_t = t

            # Log at reduced rate (every log_interval seconds) after settling
            if t >= settle_time and (t - last_log_time) >= log_interval:
                # Skip if vehicle is barely moving (boring data)
                if abs(u_body) > 1.0:
                    for axle_idx, side_idx, label in WHEEL_MAP:
                        row = _extract_wheel_row(
                            vehicle, terrain, axle_idx, side_idx, label,
                            tp, u_body, steering_rate,
                        )
                        writer.writerow(row)
                        rows_written += 1
                last_log_time = t

            t += step_size

            # Progress
            if int(t) % 10 == 0 and abs(t - round(t)) < step_size:
                n_sec = int(t)
                print(f"  t={n_sec:4d}s  u={u_body:+5.2f}m/s  rows={rows_written}")

    print(f"  Done: {rows_written} rows → {output_path}")
    return rows_written


# ─── CLI ────────────────────────────────────────────────────────────────

def main():
    p = argparse.ArgumentParser(
        description="Collect per-wheel tire force data from full HMMWV vehicle on SCM terrain",
    )
    p.add_argument("--terrain", default="dirt",
                   help="Preset name (clay/dirt/sand/all) or 'none' for LHS-only")
    p.add_argument("--time", type=float, default=60.0,
                   help="Sim duration per terrain (s)")
    p.add_argument("--lhs", type=int, default=0,
                   help="Number of LHS terrain samples (0=disabled)")
    p.add_argument("--lhs-seed", type=int, default=42,
                   help="LHS random seed")
    p.add_argument("--lhs-time", type=float, default=30.0,
                   help="Sim duration per LHS sample (s)")
    p.add_argument("--output-dir", default=None,
                   help="Output directory (default: data/vehicle_forces/)")
    p.add_argument("--step-size", type=float, default=5e-4,
                   help="Chrono step size (s)")
    p.add_argument("--log-interval", type=float, default=0.02,
                   help="Data logging interval (s). 0.02 = 50 Hz per wheel.")
    args = p.parse_args()

    out_dir = Path(args.output_dir) if args.output_dir else (
        Path(__file__).resolve().parent.parent / "data" / "vehicle_forces"
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    total_rows = 0

    # ── Preset terrains ─────────────────────────────────────────
    if args.terrain.lower() == "all":
        presets = ["clay", "dirt", "sand"]
    elif args.terrain.lower() != "none":
        presets = [args.terrain]
    else:
        presets = []

    for name in presets:
        cfg = get_terrain_preset(name)
        out_file = out_dir / f"vehicle_{name}.csv"
        total_rows += collect_one_terrain(
            cfg, args.time, str(out_file),
            step_size=args.step_size,
            log_interval=args.log_interval,
        )

    # ── LHS terrains ────────────────────────────────────────────
    if args.lhs > 0:
        lhs_configs = generate_lhs_terrain_yaml_dicts(
            args.lhs, seed=args.lhs_seed,
        )
        for i, cfg in enumerate(lhs_configs):
            out_file = out_dir / f"vehicle_lhs_{i:04d}.csv"
            total_rows += collect_one_terrain(
                cfg, args.lhs_time, str(out_file),
                step_size=args.step_size,
                log_interval=args.log_interval,
            )

    print(f"\n{'='*60}")
    print(f"Collection complete: {total_rows} total rows across {len(presets) + args.lhs} terrains")
    print(f"Output: {out_dir}/")
    print(f"{'='*60}")


if __name__ == "__main__":
    main()
