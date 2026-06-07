#!/usr/bin/env python3
"""Generate Chrono SCM ground-truth logs for the Dallas UKF validation.

This script runs a *closed-loop* PyChrono HMMWV on the SCM deformable
terrain (clay or sandy loam = "dirt" preset), driven by a scripted
sinusoidal steering and a simple cruise-speed throttle. The full body
state ``[t, x, y, psi, u, v, omega, ax, ay, delta_meas, throttle]`` is
logged every ``--log-dt`` seconds (default 24 ms — matches the Dallas
estimator rate) and written to an NPZ.

The companion ``run_dallas_from_log`` mode in ``ukf_paper_validation.py``
then replays the UKF off these logs, replacing the analytical-Bekker
half-car ground truth. This brings the UKF's measurements onto the same
physics as the NN tyre surrogate's training data so the paper118 claim
("NN beats Bekker") is testable.

Usage::

    conda activate sim
    python data_collection/run_dallas_scm.py --terrain clay --time 50 \\
        --output data/dallas_scm/clay.npz
    python data_collection/run_dallas_scm.py --terrain dirt --time 50 \\
        --output data/dallas_scm/sandy_loam.npz
"""
from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
SIM = REPO / "simulation"
sys.path.insert(0, str(SIM))

import pychrono as chrono                       # noqa: E402
import pychrono.vehicle as veh                  # noqa: E402

from chrono_setup import (setup_chrono_vehicle, setup_scm_terrain,   # noqa: E402
                            load_terrain_config)


class _ScriptedDriver(veh.ChDriver):
    """Tiny driver that lets us push (steer, throttle, brake) every tick."""

    def __init__(self, vehicle):
        super().__init__(vehicle.GetVehicle())
        self.m_steering = 0.0
        self.m_throttle = 0.0
        self.m_braking = 0.0

    def set(self, steer: float, throttle: float, brake: float = 0.0):
        self.m_steering = float(np.clip(steer, -1.0, 1.0))
        self.m_throttle = float(np.clip(throttle, 0.0, 1.0))
        self.m_braking = float(np.clip(brake,    0.0, 1.0))

    def Synchronize(self, time): pass
    def Advance(self, step): pass

    def GetSteering(self): return self.m_steering
    def GetThrottle(self): return self.m_throttle
    def GetBraking(self):  return self.m_braking


def _dallas_steer_target_rad(t: float, amp_rad: float, period_s: float) -> float:
    """Dallas Fig 7 sinusoidal steering: ±amp at given period."""
    return amp_rad * math.sin(2.0 * math.pi * t / period_s)


def main():
    p = argparse.ArgumentParser(__doc__)
    p.add_argument("--terrain", choices=["clay", "dirt", "sand"], default="clay",
                   help='SCM terrain preset. "dirt" matches Dallas\'s '
                        '"sandy loam" (n=0.7, Kphi=1515 kPa). Ignored if '
                        '--terrain-config is given.')
    p.add_argument("--terrain-config", type=Path, default=None,
                   help="YAML/JSON soil parameters (Kphi, Kc, n, cohesion, "
                        "friction_angle, janosi_shear). Overrides --terrain.")
    p.add_argument("--time", type=float, default=50.0,
                   help="Sim duration after lead-in (s).")
    p.add_argument("--lead-in", type=float, default=3.0,
                   help="Straight-line lead-in before steering excitation (s).")
    p.add_argument("--target-speed", type=float, default=5.0,
                   help="Cruise speed setpoint (m/s). Used only when "
                        "--open-loop-throttle is not set.")
    p.add_argument("--open-loop-throttle", type=float, default=None,
                   help="If set to a value >= 0, drive throttle at this "
                        "constant fraction (Buzhardt-style open-loop "
                        "excitation; matches the deployed window-MLP's "
                        "training profile of constant 0.75 throttle) and "
                        "disable the PI cruise loop. A negative value (e.g. "
                        "-1) explicitly selects PI cruise to --target-speed.")
    p.add_argument("--steer-amp-rad", type=float, default=0.50,
                   help="Steering amplitude (rad). Default 0.50 rad matches "
                        "the Dallas paper118 Fig. 7a spec. Larger amplitude "
                        "produces stronger lateral excitation and tighter "
                        "n-identifiability for all three estimators.")
    p.add_argument("--steer-period", type=float, default=3.0,
                   help="Steering sinusoid period (s).")
    p.add_argument("--max-steer-rad", type=float, default=0.5,
                   help="HMMWV max road-wheel steer (rad). Used to scale "
                        "the [-1,1] steering command.")
    p.add_argument("--step-size", type=float, default=3e-3,
                   help="Chrono physics step (s).")
    p.add_argument("--log-dt", type=float, default=0.024,
                   help="Logging interval (s) — default 24 ms matches Dallas "
                        "estimator rate.")
    p.add_argument("--output", type=Path, required=True,
                   help="NPZ to write.")
    p.add_argument("--seed", type=int, default=42)
    # Spatial soil transition (per-location SCM callback): soil changes type
    # along +x, so a UKF / MLP replay can be evaluated across a regime shift.
    p.add_argument("--terrain-transition", action="store_true",
                   help="Enable a spatial soil transition along +x.")
    p.add_argument("--terrain-start", choices=["clay", "dirt", "sand"], default=None,
                   help="Soil before the transition (defaults to --terrain).")
    p.add_argument("--terrain-end", choices=["clay", "dirt", "sand"], default=None,
                   help="Soil after the transition.")
    p.add_argument("--transition-x", type=float, default=45.0,
                   help="Center of the soil transition in terrain x (m).")
    p.add_argument("--transition-width", type=float, default=2.0,
                   help="Full width of the linear soil blend (m); 0 = hard step.")
    args = p.parse_args()

    np.random.seed(args.seed)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    print(f"Output : {args.output}")
    print(f"Terrain: {args.terrain}")

    # --- Chrono setup ------------------------------------------------------
    system, vehicle = setup_chrono_vehicle(visualize=False)
    terrain_config = None
    if args.terrain_config is not None:
        terrain_config = load_terrain_config(str(args.terrain_config))

    spatial_spec = None
    base_preset = args.terrain
    if args.terrain_transition:
        from spatial_terrain import SpatialTransitionSpec
        start_preset = args.terrain_start or args.terrain
        if args.terrain_end is None:
            raise SystemExit("--terrain-transition requires --terrain-end")
        spatial_spec = SpatialTransitionSpec(
            start_preset=start_preset, end_preset=args.terrain_end,
            transition_x=args.transition_x, transition_width=args.transition_width,
        )
        base_preset = start_preset
        print(f"Transition: {start_preset} -> {args.terrain_end} at "
              f"x={args.transition_x:.1f}m (blend {args.transition_width:.1f}m)")

    terrain, terrain_params = setup_scm_terrain(
        system, vehicle=vehicle, visualize=False,
        terrain_preset=base_preset, terrain_config=terrain_config,
        mesh_resolution=0.10, spatial_spec=spatial_spec,
    )
    driver = _ScriptedDriver(vehicle)

    # Sanity print of the realised mass.
    veh_obj = vehicle.GetVehicle()
    m_chassis = veh_obj.GetChassisBody().GetMass()
    m_total = m_chassis
    for ax in veh_obj.GetAxles():
        for w in ax.GetWheels():
            m_total += w.GetSpindle().GetMass()
    print(f"  HMMWV chassis = {m_chassis:.0f} kg, total ≈ {m_total:.0f} kg")
    print(f"  Soil: Kphi={terrain_params['Kphi']:.0f}, Kc={terrain_params['Kc']:.0f}, "
          f"n={terrain_params['n']:.2f}")

    # --- Logging buffers ---------------------------------------------------
    T_total = args.lead_in + args.time
    n_log = int(T_total / args.log_dt) + 2
    log = {k: np.zeros(n_log, dtype=np.float64) for k in
           ["t", "x", "y", "psi", "u", "v", "omega",
            "ax", "ay", "az",
            "roll", "pitch",
            "roll_rate", "pitch_rate",
            "Fy_tire_total",
            "w_fl", "w_fr", "w_rl", "w_rr",
            "delta_meas", "throttle_cmd", "steer_cmd"]}

    # Simple PI throttle to hit target_speed.
    throttle_int = 0.0
    KP, KI = 0.30, 0.10

    step_size = args.step_size
    log_steps = max(1, int(round(args.log_dt / step_size)))
    n_steps = int(T_total / step_size) + 1
    print(f"  step_size = {step_size*1000:.1f} ms, log every {log_steps} steps "
          f"({args.log_dt*1000:.0f} ms), total {n_steps} steps over {T_total:.0f} s")

    t_wall = time.time()
    log_idx = 0
    for k in range(n_steps):
        t = k * step_size
        chassis = veh_obj.GetChassisBody()
        rot = chassis.GetRot()
        vel = chassis.GetPosDt()
        vel_body = rot.RotateBack(vel)
        u_meas = vel_body.x

        if args.open_loop_throttle is not None and args.open_loop_throttle >= 0:
            # Buzhardt-style scripted open-loop throttle — matches the
            # deployed window-MLP's training excitation. No PI cruise.
            throttle_cmd = float(np.clip(args.open_loop_throttle, 0.0, 1.0))
        else:
            # PI speed governor on u.
            e = args.target_speed - u_meas
            throttle_int = float(np.clip(throttle_int + KI * e * step_size,
                                          -0.2, 1.0))
            throttle_cmd = float(np.clip(KP * e + throttle_int, 0.0, 1.0))

        # Steering excitation (after lead-in straight section).
        if t < args.lead_in:
            steer_target_rad = 0.0
        else:
            steer_target_rad = _dallas_steer_target_rad(
                t - args.lead_in, args.steer_amp_rad, args.steer_period)
        steer_cmd = float(np.clip(steer_target_rad / args.max_steer_rad,
                                  -1.0, 1.0))
        driver.set(steer_cmd, throttle_cmd)

        if k % log_steps == 0 and log_idx < n_log:
            pos = chassis.GetPos()
            ang = rot.GetCardanAnglesXYZ()
            yaw, pitch, roll = ang.z, ang.y, ang.x
            acc_body = rot.RotateBack(chassis.GetPosDt2())
            ang_vel_body = chassis.GetAngVelLocal()
            omega = ang_vel_body.z
            roll_rate = ang_vel_body.x
            pitch_rate = ang_vel_body.y
            delta_meas = 0.5 * (veh_obj.GetSteeringAngle(0, veh.LEFT) +
                                veh_obj.GetSteeringAngle(0, veh.RIGHT))
            w_fl = veh_obj.GetSpindleOmega(0, veh.LEFT)
            w_fr = veh_obj.GetSpindleOmega(0, veh.RIGHT)
            w_rl = veh_obj.GetSpindleOmega(1, veh.LEFT)
            w_rr = veh_obj.GetSpindleOmega(1, veh.RIGHT)

            # Sum body-frame tyre forces over all 4 wheels. ChWheel
            # exposes ``GetTire().ReportTireForce(terrain)`` which
            # returns the tyre force at the contact patch in the
            # tire's reference frame. Rotate to chassis body frame
            # and accumulate.
            Fy_tire = 0.0
            for ax_obj in veh_obj.GetAxles():
                for w in ax_obj.GetWheels():
                    tire = w.GetTire()
                    if tire is None:
                        continue
                    tf = tire.ReportTireForce(terrain)
                    # tf.force is in WORLD frame for RIGID tires in
                    # current Chrono — rotate to chassis body.
                    F_body = rot.RotateBack(tf.force)
                    Fy_tire += F_body.y

            log["t"][log_idx]            = t
            log["x"][log_idx]            = pos.x
            log["y"][log_idx]            = pos.y
            log["psi"][log_idx]          = yaw
            log["u"][log_idx]            = vel_body.x
            log["v"][log_idx]            = vel_body.y
            log["omega"][log_idx]        = omega
            log["ax"][log_idx]           = acc_body.x
            log["ay"][log_idx]           = acc_body.y
            log["az"][log_idx]           = acc_body.z
            log["roll"][log_idx]         = roll
            log["pitch"][log_idx]        = pitch
            log["roll_rate"][log_idx]    = roll_rate
            log["pitch_rate"][log_idx]   = pitch_rate
            log["Fy_tire_total"][log_idx] = Fy_tire
            log["w_fl"][log_idx]         = w_fl
            log["w_fr"][log_idx]         = w_fr
            log["w_rl"][log_idx]         = w_rl
            log["w_rr"][log_idx]         = w_rr
            log["delta_meas"][log_idx]   = delta_meas
            log["throttle_cmd"][log_idx] = throttle_cmd
            log["steer_cmd"][log_idx]    = steer_cmd
            log_idx += 1

        # Chrono step.
        driver_inputs = veh.DriverInputs()
        driver_inputs.m_steering = driver.GetSteering()
        driver_inputs.m_throttle = driver.GetThrottle()
        driver_inputs.m_braking  = driver.GetBraking()

        driver.Synchronize(t)
        terrain.Synchronize(t)
        vehicle.Synchronize(t, driver_inputs, terrain)
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)

    wall_s = time.time() - t_wall
    print(f"  Done. wall={wall_s:.0f}s ({wall_s/T_total:.2f}× real-time), "
          f"log rows={log_idx}")

    # Trim and persist.
    out = {k: v[:log_idx] for k, v in log.items()}
    out["terrain"] = np.array([args.terrain])
    out["soil_Kphi"] = np.array([terrain_params["Kphi"]])
    out["soil_Kc"]   = np.array([terrain_params["Kc"]])
    out["soil_n"]    = np.array([terrain_params["n"]])
    out["soil_c"]    = np.array([terrain_params["c"]])
    out["soil_phi_rad"] = np.array([math.radians(terrain_params["phi"])])
    out["soil_k"]    = np.array([terrain_params["k"]])
    out["lead_in"]   = np.array([args.lead_in])
    if spatial_spec is not None:
        out["transition"] = np.array([1])
        out["transition_start"] = np.array([spatial_spec.start_preset])
        out["transition_end"] = np.array([spatial_spec.end_preset])
        out["transition_x"] = np.array([spatial_spec.transition_x])
        out["transition_width"] = np.array([spatial_spec.transition_width])
    np.savez(args.output, **out)
    print(f"  Wrote {args.output}")


if __name__ == "__main__":
    main()
