#!/usr/bin/env python3
"""
Chrono Simulation Node (Decoupled)
===================================

Runs the PyChrono HMMWV + SCM terrain simulation and communicates with an
external MPC controller via ZMQ.

Published: VehicleState at configurable rate (default: 100 Hz decimated from 333 Hz physics)
Subscribed: ControlCommand from MPC controller

The simulation applies the latest received ControlCommand each physics step.
If no command has arrived yet, it holds zero throttle / zero steering (safe default).

Usage:
    python chrono_sim_node.py --terrain sand --time 30 --path sinusoidal
"""

import argparse
import math
import sys
import time as wall_time
from pathlib import Path

import numpy as np

# Chrono imports (must be available in environment)
import pychrono as chrono
import pychrono.vehicle as veh

# Sensor imports (optional — only needed for sensor visualization mode)
try:
    import pychrono.sensor as sens
    HAS_SENSOR = True
except ImportError:
    HAS_SENSOR = False

# Local imports
sys.path.insert(0, str(Path(__file__).parent))
from hil_messages import (
    VehicleState, ControlCommand, SimStatus,
    ZMQPublisher, ZMQSubscriber,
    sim_pub_endpoint, ctrl_sub_endpoint,
)

from param_consistency import (
    TERRAIN_PRESETS, get_vehicle_params_for_demo,
    get_terrain_preset, terrain_preset_to_internal,
)

# Re-use terrain/vehicle setup helpers from scm_hmmwv_demo
from scm_hmmwv_demo import (
    setup_chrono_vehicle,
    setup_scm_terrain,
    add_trajectory_markers,
)


# =============================================================================
# Simple driver that applies external commands
# =============================================================================

class ExternalDriver(veh.ChDriver):
    """Minimal Chrono driver that applies commands received over the network."""

    def __init__(self, vehicle):
        super().__init__(vehicle.GetVehicle())
        self.m_steering = 0.0
        self.m_throttle = 0.0
        self.m_braking = 0.0

    def apply(self, cmd: ControlCommand):
        self.m_steering = np.clip(cmd.steering, -1.0, 1.0)
        self.m_throttle = np.clip(cmd.throttle, 0.0, 1.0)
        self.m_braking = np.clip(cmd.braking, 0.0, 1.0)

    def Synchronize(self, time):
        pass  # Nothing to do — commands are applied externally

    def Advance(self, step):
        pass

    def GetSteering(self):
        return self.m_steering

    def GetThrottle(self):
        return self.m_throttle

    def GetBraking(self):
        return self.m_braking


# =============================================================================
# Vehicle state extraction (matches scm_hmmwv_demo._read_vehicle_state)
# =============================================================================

# Default measurement noise standard deviations (sensor-fusion realistic)
DEFAULT_MEAS_NOISE = {
    'x':     0.05,    # Differential GPS position (m)
    'y':     0.05,    # Differential GPS position (m)
    'psi':   0.005,   # ~0.3° heading (rad)
    'u':     0.05,    # Speed (m/s)
    'v':     0.05,    # Lateral speed (m/s)
    'omega': 0.005,   # Yaw rate (rad/s)
}


def extract_tire_forces(vehicle, terrain) -> dict:
    """Extract per-wheel tire forces and slips from Chrono."""
    tf = {}
    veh_obj = vehicle.GetVehicle()
    for axle_idx, axle_name in enumerate(['front', 'rear']):
        for side_idx, side_name in [(veh.LEFT, 'left'), (veh.RIGHT, 'right')]:
            tire = veh_obj.GetTire(axle_idx, side_idx)
            force = tire.ReportTireForce(terrain)
            key = f'{axle_name}_{side_name}'
            tf[f'{key}_Fx'] = force.force.x
            tf[f'{key}_Fy'] = force.force.y
            tf[f'{key}_Fz'] = force.force.z
            tf[f'{key}_slip_angle'] = tire.GetSlipAngle()
            tf[f'{key}_long_slip'] = tire.GetLongitudinalSlip()
    return tf


def extract_vehicle_state(vehicle, sim_time: float, terrain=None,
                          noise: dict = None) -> VehicleState:
    """Read Chrono vehicle and pack into a VehicleState message.

    Args:
        terrain: If provided, tire forces are included.
        noise: If provided, dict of std-devs to add Gaussian noise to sensors.
    """
    chassis = vehicle.GetChassisBody()
    pos = chassis.GetPos()
    rot = chassis.GetRot()
    vel = chassis.GetPosDt()
    omega_vec = chassis.GetAngVelLocal()

    # Velocity in body frame
    vel_loc = rot.RotateBack(vel)

    x_cg = pos.x
    y_cg = pos.y
    u = vel_loc.x
    v = vel_loc.y
    omega = omega_vec.z

    # Compute yaw from quaternion for noise injection
    psi = math.atan2(2 * (rot.e0 * rot.e3 + rot.e1 * rot.e2),
                     1 - 2 * (rot.e2 * rot.e2 + rot.e3 * rot.e3))

    # Sensor noise injection
    if noise:
        x_cg  += np.random.normal(0, noise['x'])
        y_cg  += np.random.normal(0, noise['y'])
        psi   += np.random.normal(0, noise['psi'])
        u     += np.random.normal(0, noise['u'])
        v     += np.random.normal(0, noise['v'])
        omega += np.random.normal(0, noise['omega'])
        # Reconstruct quaternion from noisy yaw (keep pitch/roll from Chrono)
        half = psi / 2.0
        qe0, qe1, qe2, qe3 = math.cos(half), 0.0, 0.0, math.sin(half)
    else:
        qe0, qe1, qe2, qe3 = rot.e0, rot.e1, rot.e2, rot.e3

    # Tire forces (optional)
    tf = extract_tire_forces(vehicle, terrain) if terrain is not None else None

    # Embed ground truth for analytics (when noise is applied, plots need true path)
    if noise and tf is not None:
        tf['true_x_cg'] = pos.x
        tf['true_y_cg'] = pos.y
        tf['true_psi'] = math.atan2(
            2 * (rot.e0 * rot.e3 + rot.e1 * rot.e2),
            1 - 2 * (rot.e2 * rot.e2 + rot.e3 * rot.e3))
        tf['true_u'] = vel_loc.x
    elif noise and tf is None:
        tf = {
            'true_x_cg': pos.x,
            'true_y_cg': pos.y,
            'true_psi': math.atan2(
                2 * (rot.e0 * rot.e3 + rot.e1 * rot.e2),
                1 - 2 * (rot.e2 * rot.e2 + rot.e3 * rot.e3)),
            'true_u': vel_loc.x,
        }

    return VehicleState(
        time=sim_time,
        wall_time=wall_time.time(),
        x_cg=x_cg,
        y_cg=y_cg,
        z_cg=pos.z,
        quat_e0=qe0,
        quat_e1=qe1,
        quat_e2=qe2,
        quat_e3=qe3,
        u=u,
        v=v,
        omega=omega,
        tire_forces=tf,
    )


# =============================================================================
# Main simulation loop
# =============================================================================

def run_sim_node(args):
    print("=" * 60)
    print("Chrono Simulation Node (Decoupled)")
    print("=" * 60)

    # Determine visualization flags
    use_irrlicht = args.vis_mode in ('irrlicht', 'both')
    use_sensor = args.vis_mode in ('sensor', 'both')
    any_vis = use_irrlicht or use_sensor

    if use_sensor and not HAS_SENSOR:
        print("WARNING: pychrono.sensor not available, falling back to irrlicht")
        use_sensor = False
        use_irrlicht = True
        any_vis = True

    # ------------------------------------------------------------------
    # Setup vehicle
    # ------------------------------------------------------------------
    system, vehicle = setup_chrono_vehicle(any_vis)

    # ------------------------------------------------------------------
    # Setup terrain
    # ------------------------------------------------------------------
    terrain_config = None
    if args.terrain_config:
        from scm_hmmwv_demo import load_terrain_config
        terrain_config = load_terrain_config(args.terrain_config)

    terrain, terrain_params = setup_scm_terrain(
        system, vehicle=vehicle, visualize=any_vis,
        terrain_preset=args.terrain, terrain_config=terrain_config,
        bump_amplitude=args.bump, bump_wavelength=args.bump_wavelength,
        bump_octaves=args.bump_octaves, bump_seed=args.bump_seed,
        bump_max_slope=args.bump_max_slope,
    )

    # ------------------------------------------------------------------
    # Trajectory markers (visual only)
    # ------------------------------------------------------------------
    if any_vis:
        marker_z = args.bump + 0.5 if args.bump > 0 else 0.15
        add_trajectory_markers(
            system, args.path, args.time, v_target=args.speed,
            marker_z=marker_z,
            sine_amplitude=args.sine_amplitude,
            sine_wavelength=args.sine_wavelength,
            lead_in=args.lead_in,
        )

    # ------------------------------------------------------------------
    # Driver (external commands)
    # ------------------------------------------------------------------
    driver = ExternalDriver(vehicle)

    # ------------------------------------------------------------------
    # Visualization — Irrlicht
    # ------------------------------------------------------------------
    vis = None
    if use_irrlicht:
        try:
            vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
            vis.SetWindowTitle("Chrono Sim Node (decoupled)")
            vis.SetWindowSize(1920, 1080)
            vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.5), 6.0, 0.5)
            vis.Initialize()
            vis.AddLightDirectional()
            vis.AddSkyBox()
            vis.AttachVehicle(vehicle.GetVehicle())
        except Exception as e:
            print(f"Warning: Irrlicht visualization failed: {e}")
            vis = None

    # ------------------------------------------------------------------
    # Visualization — Chrono Sensor (driver POV camera)
    # ------------------------------------------------------------------
    sensor_manager = None
    driver_cam = None
    if use_sensor:
        try:
            sensor_manager = sens.ChSensorManager(system)
            # Scene lighting and environment
            sensor_manager.scene.AddPointLight(
                chrono.ChVector3f(0, 0, 100),
                chrono.ChColor(1.5, 1.5, 1.5),
                500.0,
            )
            sensor_manager.scene.SetAmbientLight(chrono.ChVector3f(0.1, 0.1, 0.1))
            sensor_manager.scene.SetSceneEpsilon(1e-3)
            sensor_manager.scene.EnableDynamicOrigin(True)
            sensor_manager.scene.SetOriginOffsetThreshold(500.0)

            # Driver POV camera attached to chassis
            # Eye-point matches HMMWV left-hand-drive seat position
            cam_offset = chrono.ChFramed(
                chrono.ChVector3d(0.4, 0.7, 1.0),
                chrono.ChQuaterniond(1, 0, 0, 0),
            )
            driver_cam = sens.ChCameraSensor(
                vehicle.GetChassisBody(),  # attached body
                30,                        # update rate (Hz)
                cam_offset,                # offset pose
                3440,                      # image width
                1440,                      # image height
                1.92,                      # horizontal FOV (~110°, natural for ultrawide)
            )
            driver_cam.SetName("DriverPOV")
            driver_cam.SetLag(0.0)
            driver_cam.PushFilter(sens.ChFilterVisualize(
                3440, 1440, "Driver POV", False
            ))
            driver_cam.PushFilter(sens.ChFilterRGBA8Access())
            sensor_manager.AddSensor(driver_cam)
            print("  Chrono Sensor: driver POV camera active")
        except Exception as e:
            print(f"Warning: Sensor visualization failed: {e}")
            sensor_manager = None
            driver_cam = None

    # ------------------------------------------------------------------
    # ZMQ transport
    # ------------------------------------------------------------------
    state_pub = ZMQPublisher(sim_pub_endpoint(args.sim_port))
    ctrl_sub = ZMQSubscriber(ctrl_sub_endpoint(args.ctrl_host, args.ctrl_port))
    print(f"  Publishing state on port {args.sim_port}")
    print(f"  Subscribing to controls from {args.ctrl_host}:{args.ctrl_port}")

    # Give ZMQ sockets time to connect
    wall_time.sleep(0.3)

    # Publish initial config so controller knows terrain / vehicle params
    vehicle_params = get_vehicle_params_for_demo()
    internal_terrain = terrain_preset_to_internal(
        terrain_config if terrain_config else get_terrain_preset(args.terrain)
    )
    config_msg = SimStatus(
        event="config",
        time=0.0,
        wall_time=wall_time.time(),
        config={
            "vehicle_params": vehicle_params,
            "terrain_params": internal_terrain,
            "terrain_preset": args.terrain,
            "path_type": args.path,
            "v_target": args.speed,
            "sim_time": args.time,
            "step_size": args.step_size,
            "sine_amplitude": args.sine_amplitude,
            "sine_wavelength": args.sine_wavelength,
            "lead_in": args.lead_in,
        },
    )
    state_pub.send(config_msg)

    # ------------------------------------------------------------------
    # Simulation loop
    # ------------------------------------------------------------------
    step_size = args.step_size
    state_pub_interval = 1.0 / args.state_rate  # Decimated publishing rate
    last_state_pub_time = -state_pub_interval

    render_interval = 1.0 / 35.0
    last_render_time = -render_interval
    last_report_time = 0.0
    start_wall = wall_time.time()
    cmd_count = 0

    noise_cfg = None if args.no_noise else DEFAULT_MEAS_NOISE
    print(f"  Sensor noise: {'OFF' if noise_cfg is None else 'ON'}")
    print(f"  Physics step: {step_size * 1000:.0f}ms, state rate: {args.state_rate} Hz")
    print(f"  Running {args.time}s simulation...")

    while True:
        time_chrono = vehicle.GetSystem().GetChTime()

        if time_chrono >= args.time:
            break
        if vis is not None and not vis.Run():
            break

        # --- Render Irrlicht (frame-skipped) ---
        if vis is not None and (time_chrono - last_render_time >= render_interval):
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            last_render_time = time_chrono

        # --- Update Chrono Sensor manager ---
        if sensor_manager is not None:
            sensor_manager.Update()

        # --- Receive latest control command (non-blocking) ---
        result = ctrl_sub.recv(timeout_ms=0)
        if result is not None:
            topic, msg = result
            if isinstance(msg, ControlCommand):
                driver.apply(msg)
                cmd_count += 1

        # --- Synchronize ---
        driver.Synchronize(time_chrono)

        driver_inputs = veh.DriverInputs()
        driver_inputs.m_steering = driver.GetSteering()
        driver_inputs.m_throttle = driver.GetThrottle()
        driver_inputs.m_braking = driver.GetBraking()

        terrain.Synchronize(time_chrono)
        vehicle.Synchronize(time_chrono, driver_inputs, terrain)
        if vis is not None:
            vis.Synchronize(time_chrono, driver_inputs)

        # --- Advance ---
        driver.Advance(step_size)
        terrain.Advance(step_size)
        vehicle.Advance(step_size)
        if vis is not None:
            vis.Advance(step_size)

        # --- Publish vehicle state at decimated rate ---
        if time_chrono - last_state_pub_time >= state_pub_interval:
            state_msg = extract_vehicle_state(
                vehicle, time_chrono, terrain=terrain,
                noise=noise_cfg,
            )
            state_pub.send(state_msg)
            last_state_pub_time = time_chrono

        # --- Real-time pacing (always on unless --no-rt) ---
        # Without this, the headless sim runs 4-5x real-time and the
        # decoupled MPC controller can only process ~10% of state messages.
        if not args.no_rt:
            target_wall = start_wall + time_chrono
            remaining = target_wall - wall_time.time()
            if remaining > 0:
                wall_time.sleep(remaining)

        # --- Progress report ---
        if time_chrono - last_report_time >= 2.0:
            last_report_time = time_chrono
            elapsed = wall_time.time() - start_wall
            rt = time_chrono / elapsed if elapsed > 0 else 0
            pos = vehicle.GetChassisBody().GetPos()
            print(f"  t={time_chrono:.1f}s  pos=({pos.x:.1f},{pos.y:.1f})  "
                  f"RT={rt:.2f}x  cmds_recv={cmd_count}")

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    stop_msg = SimStatus(event="stop", time=time_chrono, wall_time=wall_time.time())
    state_pub.send(stop_msg)

    elapsed = wall_time.time() - start_wall
    print(f"\n  Simulation complete: {time_chrono:.1f}s in {elapsed:.1f}s "
          f"(RT factor {time_chrono / elapsed:.2f}x)")
    print(f"  Total control commands received: {cmd_count}")

    state_pub.close()
    ctrl_sub.close()
    if vis is not None:
        vis.GetDevice().closeDevice()
    if sensor_manager is not None:
        del sensor_manager


# =============================================================================
# Entry point
# =============================================================================

def main():
    p = argparse.ArgumentParser(description="Chrono Simulation Node (decoupled)")

    # Simulation
    p.add_argument("--time", type=float, default=15.0, help="Simulation duration (s)")
    p.add_argument("--step-size", type=float, default=3e-3, help="Physics step (s)")
    p.add_argument("--vis-mode", default="irrlicht",
                   choices=["irrlicht", "sensor", "both", "none"],
                   help="Visualization mode: irrlicht, sensor (driver POV), both, or none")
    p.add_argument("--no-rt",  action="store_true",
                   help="Disable real-time pacing (fast-forward; breaks decoupled MPC)")
    p.add_argument("--speed", type=float, default=5.0, help="Target speed for markers (m/s)")

    # Terrain
    p.add_argument("--terrain", default="sand", choices=["sand", "clay", "dirt"])
    p.add_argument("--terrain-config", type=str, default=None, help="YAML terrain config")
    p.add_argument("--bump", type=float, default=0.0, help="Bump amplitude (m)")
    p.add_argument("--bump-wavelength", type=float, default=20.0)
    p.add_argument("--bump-octaves", type=int, default=4)
    p.add_argument("--bump-seed", type=int, default=12345)
    p.add_argument("--bump-max-slope", type=float, default=0.3)

    # Path (for visual markers only; the controller handles actual path generation)
    p.add_argument("--path", default="lane_change",
                   choices=["lane_change", "double_lane_change", "sinusoidal"])
    p.add_argument("--sine-amplitude", type=float, default=2.0)
    p.add_argument("--sine-wavelength", type=float, default=30.0)
    p.add_argument("--lead-in", type=float, default=0.0,
                   help="Straight lead-in distance (m) before path starts")

    # Network
    p.add_argument("--sim-port", type=int, default=5555, help="Port to publish state")
    p.add_argument("--ctrl-host", default="localhost", help="Controller host")
    p.add_argument("--ctrl-port", type=int, default=5556, help="Controller command port")
    p.add_argument("--state-rate", type=int, default=100,
                   help="Vehicle state publish rate (Hz)")
    p.add_argument("--no-noise", action="store_true",
                   help="Disable sensor noise (noise ON by default)")

    args = p.parse_args()
    run_sim_node(args)


if __name__ == "__main__":
    main()
