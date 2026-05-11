#!/usr/bin/env python3
"""
Quick HMMWV-on-bumpy-sand Irrlicht demo.

This is intentionally standalone: no ZMQ, no MPC, no sensor pipeline.  It uses
the same HMMWV and SCM setup helpers as chrono_sim_node.py, then drives forward
with a simple open-loop/speed-hold throttle command.
"""

import argparse
import sys
from pathlib import Path

import pychrono as chrono
import pychrono.vehicle as veh

sys.path.insert(0, str(Path(__file__).parent))
from chrono_setup import setup_chrono_vehicle, setup_scm_terrain
from param_consistency import HMMWV_VEHICLE_PARAMS


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def driver_inputs_for_forward_motion(vehicle, time_s, target_speed, max_throttle):
    """Return straight-ahead driver inputs with a small speed-hold loop."""
    speed = vehicle.GetVehicle().GetSpeed()
    ramp = clamp(time_s / 2.0, 0.0, 1.0)

    throttle = ramp * (0.25 + 0.10 * (target_speed - speed))
    throttle = clamp(throttle, 0.0, max_throttle)

    braking = 0.0
    if speed > target_speed + 1.0:
        throttle = 0.0
        braking = clamp(0.15 * (speed - target_speed - 1.0), 0.0, 0.35)

    inputs = veh.DriverInputs()
    inputs.m_steering = 0.0
    inputs.m_throttle = throttle
    inputs.m_braking = braking
    return inputs


def vehicle_point_to_world(vehicle, local_point):
    chassis = vehicle.GetChassisBody()
    return chassis.GetPos() + chassis.GetRot().Rotate(local_point)


def set_visual_color(item, color):
    """Set color on all visual shapes owned by a Chrono item, if exposed."""
    def color_shape(shape):
        shape.SetColor(color)
        try:
            for i in range(shape.GetNumMaterials()):
                material = shape.GetMaterial(i)
                material.SetAmbientColor(color)
                material.SetDiffuseColor(color)
        except Exception:
            pass

    try:
        model = item.GetVisualModel()
    except Exception:
        model = None

    if model:
        try:
            for i in range(model.GetNumShapes()):
                color_shape(model.GetShape(i))
            return
        except Exception:
            pass

    try:
        shape_count = item.GetNumVisualShapes()
    except Exception:
        shape_count = 0

    for i in range(shape_count):
        try:
            color_shape(item.GetVisualShape(i))
        except Exception:
            pass


def color_hmmwv(vehicle):
    """Apply explicit colors for Irrlicht builds that do not load HMMWV materials."""
    body_color = chrono.ChColor(0.88, 0.82, 0.66)       # light tan
    set_visual_color(vehicle.GetChassisBody(), body_color)


def update_front_right_camera(vis, vehicle, args):
    """Aim an Irrlicht camera at the HMMWV front-right corner.

    The camera pose is updated explicitly here instead of using Chrono's vehicle
    chase-camera wrapper.  Vehicle frame convention: +X forward, +Y left, +Z up.
    """
    half_track = 0.5 * HMMWV_VEHICLE_PARAMS["T"]
    front_right_local = chrono.ChVector3d(args.corner_x, -half_track - args.corner_y_out, args.corner_z)
    target = vehicle_point_to_world(vehicle, front_right_local)

    if args.fixed_camera:
        camera_pos = chrono.ChVector3d(args.fixed_camera_x, args.fixed_camera_y, args.fixed_camera_z)
    else:
        camera_offset_local = chrono.ChVector3d(args.camera_x, args.camera_y, args.camera_z)
        camera_pos = vehicle_point_to_world(vehicle, camera_offset_local)

    if hasattr(vis, "SetChaseCameraPosition"):
        vis.SetChaseCameraPosition(camera_pos, target)
    vis.SetCameraPosition(camera_pos)
    vis.SetCameraTarget(target)


def set_z_up_if_available(vis):
    if hasattr(chrono, "CameraVerticalDir_Z"):
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)


def build_visual_system(vehicle, args):
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle("Quick HMMWV: bumpy sand SCM")
    vis.SetWindowSize(args.window_width, args.window_height)
    set_z_up_if_available(vis)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile("logo_chrono_alpha.png"))
    vis.AddLightDirectional(
        45.0,
        120.0,
        chrono.ChColor(0.28, 0.28, 0.28),
        chrono.ChColor(0.08, 0.08, 0.08),
        chrono.ChColor(0.68, 0.68, 0.68),
    )
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle.GetVehicle())
    update_front_right_camera(vis, vehicle, args)
    return vis


def run(args):
    system, vehicle = setup_chrono_vehicle(visualize=True)
    terrain, _ = setup_scm_terrain(
        system,
        vehicle=vehicle,
        visualize=True,
        terrain_preset="sand",
        bumpiness=args.bumpiness,
        bump_seed=args.bump_seed,
        mesh_resolution=args.mesh_resolution,
        texture=False,
    )
    # terrain.SetColor(chrono.ChColor(1.0, 1.0, 1.0))
    color_hmmwv(vehicle)

    vis = build_visual_system(vehicle, args)
    realtime_timer = chrono.ChRealtimeStepTimer()

    print("Quick sim: HMMWV driving straight on bumpy sand")
    print(f"  duration={args.time:.1f}s, target_speed={args.speed:.1f}m/s, bumpiness={args.bumpiness}")
    print("  camera: Irrlicht, manually aimed at the front-right corner")

    step = args.step_size
    render_period = 1.0 / args.render_fps
    last_render = -render_period

    while vis.Run():
        time_s = system.GetChTime()
        if time_s >= args.time:
            break

        driver_inputs = driver_inputs_for_forward_motion(
            vehicle,
            time_s,
            target_speed=args.speed,
            max_throttle=args.max_throttle,
        )

        terrain.Synchronize(time_s)
        vehicle.Synchronize(time_s, driver_inputs, terrain)
        vis.Synchronize(time_s, driver_inputs)

        if time_s - last_render >= render_period:
            update_front_right_camera(vis, vehicle, args)
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            last_render = time_s

        terrain.Advance(step)
        vehicle.Advance(step)
        vis.Advance(step)
        realtime_timer.Spin(step)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Standalone bumpy-sand HMMWV sim with a non-chase Irrlicht camera."
    )
    parser.add_argument("--time", type=float, default=30.0, help="Simulation duration [s]")
    parser.add_argument("--speed", type=float, default=5.0, help="Forward target speed [m/s]")
    parser.add_argument("--max-throttle", type=float, default=0.65, help="Throttle limit [0, 1]")
    parser.add_argument("--bumpiness", type=int, default=7, help="Terrain bumpiness level 0-10")
    parser.add_argument("--bump-seed", type=int, default=12345, help="Procedural terrain seed")
    parser.add_argument("--mesh-resolution", type=float, default=0.1, help="SCM mesh spacing [m]")
    parser.add_argument("--step-size", type=float, default=0.003, help="Physics step size [s]")
    parser.add_argument("--render-fps", type=float, default=35.0, help="Irrlicht render rate [Hz]")
    parser.add_argument("--window-width", type=int, default=1280)
    parser.add_argument("--window-height", type=int, default=720)

    parser.add_argument("--corner-x", type=float, default=2.55, help="Front corner target x in chassis frame [m]")
    parser.add_argument("--corner-y-out", type=float, default=0.35, help="Outboard offset beyond half-track [m]")
    parser.add_argument("--corner-z", type=float, default=0.75, help="Front corner target z in chassis frame [m]")

    parser.add_argument("--camera-x", type=float, default=5.5, help="Vehicle-relative camera x [m]")
    parser.add_argument("--camera-y", type=float, default=-5.0, help="Vehicle-relative camera y [m]")
    parser.add_argument("--camera-z", type=float, default=2.6, help="Vehicle-relative camera z [m]")
    parser.add_argument("--fixed-camera", action="store_true",
                        help="Use a world-fixed camera that tracks the front-right corner")
    parser.add_argument("--fixed-camera-x", type=float, default=10.0)
    parser.add_argument("--fixed-camera-y", type=float, default=-8.0)
    parser.add_argument("--fixed-camera-z", type=float, default=3.0)
    return parser.parse_args()


if __name__ == "__main__":
    run(parse_args())
