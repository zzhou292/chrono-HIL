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

# Re-use terrain/vehicle setup helpers (extracted modules)
from chrono_setup import (
    setup_chrono_vehicle,
    setup_scm_terrain,
    add_trajectory_markers,
    load_terrain_config,
)
from g29_controller import ManualDriver

# Safety filter + obstacles (optional)
from sensors.obstacles import add_rock_obstacles, get_rock_positions, get_rock_radii
from safety import CBFSafetyFilter

# NN tire model for terrain-aware CBF traction limits
try:
    from nn_tire_model import load_nn_tire_model
except ImportError:
    load_nn_tire_model = None


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
# Vehicle state extraction
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
        terrain_config = load_terrain_config(args.terrain_config)

    terrain, terrain_params = setup_scm_terrain(
        system, vehicle=vehicle, visualize=any_vis,
        terrain_preset=args.terrain, terrain_config=terrain_config,
        bumpiness=args.bumpiness,
    )

    # ------------------------------------------------------------------
    # Rock obstacles
    # ------------------------------------------------------------------
    rocks = []
    if args.rocks > 0:
        exclusion_zones = [(-25.0, 0.0, 5.0)]  # Vehicle spawn at x=-25
        rocks = add_rock_obstacles(
            system, num_rocks=args.rocks,
            zone_x=tuple(args.rock_zone_x), zone_y=tuple(args.rock_zone_y),
            size_range=tuple(args.rock_size), seed=args.rock_seed,
            exclusion_zones=exclusion_zones,
        )
        print(f"  Placed {len(rocks)} rock obstacles")

    # ------------------------------------------------------------------
    # CBF safety filter
    # ------------------------------------------------------------------
    safety_filter = None
    if args.safety_filter:
        vehicle_params = get_vehicle_params_for_demo()
        nn_for_cbf = None
        if load_nn_tire_model is not None:
            base_path = Path(__file__).parent.parent
            model_version = args.nn_model if hasattr(args, 'nn_model') else "v6"
            cbf_model_dir = base_path / "nn_models" / model_version
            if (cbf_model_dir / "best_terrain_nn.pt").exists():
                try:
                    nn_for_cbf = load_nn_tire_model(cbf_model_dir, terrain_params)
                    print(f"  [SAFETY] NN tire model loaded for CBF: {model_version}")
                except Exception as e:
                    print(f"  [SAFETY] NN load failed ({e}), using kinematic fallback")
        safety_filter = CBFSafetyFilter(
            vehicle_params=vehicle_params,
            nn_casadi=nn_for_cbf,
            cbf_alpha=args.cbf_alpha,
            obstacle_buffer=args.safety_buffer,
            delay_steps=args.delay_steps,
            control_dt=0.1,  # Match MPC rate (10 Hz)
            w_long=args.cbf_w_long,
            w_lat=args.cbf_w_lat,
            forward_bias=args.cbf_forward_bias,
            dob_bandwidth=args.dob_bandwidth,
            cbf_flavor=args.cbf_flavor,
            teleop_delay=args.teleop_delay,
            stale_cmd_timeout=args.stale_cmd_timeout,
        )
        delay_msg = f", teleop_delay={args.teleop_delay*1000:.0f}ms" if args.teleop_delay > 0 else ""
        print(f"  [SAFETY] DOB-CBF filter enabled: alpha={args.cbf_alpha}, "
              f"buffer={args.safety_buffer}m, flavor={args.cbf_flavor}{delay_msg}")

    # ------------------------------------------------------------------
    # Trajectory markers (visual only)
    # ------------------------------------------------------------------
    if any_vis:
        marker_z = 0.5 if args.bumpiness > 0 else 0.15
        add_trajectory_markers(
            system, args.path, args.time, v_target=args.speed,
            marker_z=marker_z,
            sine_amplitude=args.sine_amplitude,
            sine_wavelength=args.sine_wavelength,
            lead_in=args.lead_in,
        )

    # ------------------------------------------------------------------
    # Driver (external commands or manual G29)
    # ------------------------------------------------------------------
    if args.manual:
        print("  Manual mode: using G29 steering wheel")
        driver = ManualDriver(vehicle)
    else:
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
    # ZMQ transport (skipped in manual mode)
    # ------------------------------------------------------------------
    state_pub = None
    ctrl_sub = None
    if not args.manual:
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

        # --------------------------------------------------------------
        # Gate: wait for controller ready (neutral ControlCommand) after
        # ACADOS build + warmup.  acados_mpc_controller_node sends these
        # pings before/with VehicleState so we do not deadlock.  Prevents
        # codegen time from consuming --time once the loop runs.
        # --------------------------------------------------------------
        wait_s = 0.0 if args.no_wait_for_controller else float(args.wait_for_controller)
        if wait_s > 0:
            print(f"  Waiting for controller ready signal (timeout {wait_s:.0f}s)...")
            t0_wait = wall_time.time()
            last_cfg_send = t0_wait
            got_ready = False
            while wall_time.time() - t0_wait < wait_s:
                # Re-publish config while waiting so late-starting controllers
                # (e.g. during ACADOS codegen/compile) can still receive it.
                now_wait = wall_time.time()
                if now_wait - last_cfg_send >= 0.5:
                    config_msg.wall_time = now_wait
                    state_pub.send(config_msg)
                    last_cfg_send = now_wait
                result = ctrl_sub.recv(timeout_ms=100)
                if result is None:
                    continue
                _, msg = result
                if isinstance(msg, ControlCommand):
                    driver.apply(msg)
                    print("  Controller ready — starting simulation.")
                    got_ready = True
                    break
            if not got_ready:
                print("  WARNING: No controller handshake before timeout — "
                      "starting simulation anyway. Chrono time may run ahead of MPC.")

    # ------------------------------------------------------------------
    # Simulation loop
    # ------------------------------------------------------------------
    step_size = args.step_size
    state_pub_interval = 1.0 / args.state_rate  # Decimated publishing rate
    last_state_pub_time = -state_pub_interval
    last_config_resend = 0.0  # Re-publish config during first 2s so controller catches it

    render_interval = 1.0 / 35.0
    last_render_time = -render_interval
    last_report_time = 0.0
    start_wall = wall_time.time()
    cmd_count = 0
    step_count = 0

    noise_cfg = None if args.no_noise else DEFAULT_MEAS_NOISE
    print(f"  Sensor noise: {'OFF' if noise_cfg is None else 'ON'}")
    print(f"  Physics step: {step_size * 1000:.0f}ms, state rate: {args.state_rate} Hz")
    if args.manual:
        print(f"  Manual mode: close window to exit")
    else:
        print(f"  Running {args.time}s simulation...")

    while True:
        time_chrono = vehicle.GetSystem().GetChTime()

        if not args.manual and time_chrono >= args.time:
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
        if ctrl_sub is not None:
            result = ctrl_sub.recv(timeout_ms=0)
            if result is not None:
                topic, msg = result
                if isinstance(msg, ControlCommand):
                    driver.apply(msg)
                    cmd_count += 1
                    # Feed command age to safety filter for teleop delay est.
                    if safety_filter is not None and msg.wall_time > 0:
                        safety_filter.update_command_age(msg.wall_time)

        # --- Synchronize ---
        driver.Synchronize(time_chrono)

        driver_inputs = veh.DriverInputs()
        driver_inputs.m_steering = driver.GetSteering()
        driver_inputs.m_throttle = driver.GetThrottle()
        driver_inputs.m_braking = driver.GetBraking()

        # --- Safety Filter ---
        # Safety filter at ~10Hz
        sf_interval = max(1, int(1.0 / (10.0 * step_size)))  # 10 Hz, matching MPC rate
        if safety_filter is not None and step_count % sf_interval == 0:
            chassis_body = vehicle.GetChassisBody()
            veh_pos = chassis_body.GetPos()
            veh_rot = chassis_body.GetRot()
            veh_psi = np.arctan2(
                2 * (veh_rot.e0 * veh_rot.e3 + veh_rot.e1 * veh_rot.e2),
                1 - 2 * (veh_rot.e2**2 + veh_rot.e3**2))
            vel_world = chassis_body.GetPosDt()
            vel_loc = veh_rot.RotateBack(vel_world)

            all_obstacles = []
            if args.rocks > 0:
                rock_pos = get_rock_positions(rocks)
                rock_rad = get_rock_radii(rocks)
                for i in range(len(rock_pos)):
                    dist = np.sqrt((rock_pos[i, 0] - veh_pos.x)**2 +
                                   (rock_pos[i, 1] - veh_pos.y)**2)
                    if dist < 30.0:
                        all_obstacles.append((rock_pos[i, 0], rock_pos[i, 1], rock_rad[i]))

            veh_state = {
                'x': veh_pos.x, 'y': veh_pos.y, 'psi': veh_psi,
                'u': vehicle.GetVehicle().GetSpeed(),
                'v': vel_loc.y, 'omega': chassis_body.GetAngVelLocal().z,
                'delta': driver_inputs.m_steering * 0.49,
            }
            sf_result = safety_filter.filter(
                desired_steering=driver_inputs.m_steering,
                desired_throttle=driver_inputs.m_throttle,
                desired_brake=driver_inputs.m_braking,
                vehicle_state=veh_state,
                obstacles=all_obstacles,
            )
            driver_inputs.m_steering = sf_result.steering
            driver_inputs.m_throttle = sf_result.throttle
            driver_inputs.m_braking = sf_result.braking
        elif safety_filter is not None and safety_filter._last_result is not None:
            cached = safety_filter._last_result
            if cached.was_modified:
                driver_inputs.m_steering = cached.steering
                driver_inputs.m_throttle = cached.throttle
                driver_inputs.m_braking = cached.braking

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

        step_count += 1

        # --- Re-publish config during first 2s (CONFLATE can drop it) ---
        if state_pub is not None and time_chrono < 2.0 and time_chrono - last_config_resend >= 0.2:
            config_msg.wall_time = wall_time.time()
            state_pub.send(config_msg)
            last_config_resend = time_chrono

        # --- Publish vehicle state at decimated rate ---
        if state_pub is not None and time_chrono - last_state_pub_time >= state_pub_interval:
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
    if state_pub is not None:
        stop_msg = SimStatus(event="stop", time=time_chrono, wall_time=wall_time.time())
        state_pub.send(stop_msg)

    elapsed = wall_time.time() - start_wall
    print(f"\n  Simulation complete: {time_chrono:.1f}s in {elapsed:.1f}s "
          f"(RT factor {time_chrono / elapsed:.2f}x)")
    if not args.manual:
        print(f"  Total control commands received: {cmd_count}")

    # Safety filter summary
    if safety_filter is not None:
        diag = safety_filter.get_diagnostics()
        print(f"  [SAFETY] Calls: {diag['filter_calls']}, "
              f"Interventions: {diag['interventions']} ({diag['intervention_rate']*100:.1f}%)")

    if state_pub is not None:
        state_pub.close()
    if ctrl_sub is not None:
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
    p.add_argument("--bumpiness", type=int, default=0, choices=range(0, 11),
                    help="Terrain bumpiness level 0 (flat) to 10 (extreme)")

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
    p.add_argument("--wait-for-controller", type=float, default=300.0,
                   help="Wait up to this many seconds for the controller's first control message (ready ping after "
                        "ACADOS init) before advancing Chrono. Default 300. Start the sim first, then the "
                        "controller, or use launch_decoupled.py.")
    p.add_argument("--no-wait-for-controller", action="store_true",
                   help="Enter the sim loop immediately (no MPC handshake). Use for sim-only / debugging without a "
                        "controller node.")

    # Manual control
    p.add_argument("--manual", action="store_true",
                   help="Manual control with G29 steering wheel (no MPC controller)")

    # Rock obstacles
    p.add_argument("--rocks", type=int, default=0,
                   help="Number of rock obstacles (0 = none)")
    p.add_argument("--rock-zone-x", type=float, nargs=2, default=[-15.0, 50.0])
    p.add_argument("--rock-zone-y", type=float, nargs=2, default=[-10.0, 10.0])
    p.add_argument("--rock-size", type=float, nargs=2, default=[0.5, 3.0])
    p.add_argument("--rock-seed", type=int, default=42)

    # Safety filter
    p.add_argument("--safety-filter", action="store_true",
                   help="Enable DOB-CBF safety filter")
    p.add_argument("--cbf-alpha", type=float, default=5.0)
    p.add_argument("--safety-buffer", type=float, default=0.25)
    p.add_argument("--delay-steps", type=int, default=5)
    p.add_argument("--cbf-w-long", type=float, default=0.06)
    p.add_argument("--cbf-w-lat", type=float, default=0.50)
    p.add_argument("--cbf-forward-bias", type=float, default=3.0)
    p.add_argument("--dob-bandwidth", type=float, default=10.0)
    p.add_argument("--cbf-flavor", type=str, default="balance",
                   choices=["balance", "steer_priority", "throttle_priority"])
    p.add_argument("--nn-model", type=str, default="v6",
                   help="NN model version for CBF traction limits")
    p.add_argument("--teleop-delay", type=float, default=0.0,
                   help="Initial one-way teleop delay estimate in seconds "
                        "(0 = local, auto-measured from cmd timestamps)")
    p.add_argument("--stale-cmd-timeout", type=float, default=2.0,
                   help="Auto-brake if no command received for this many seconds")

    args = p.parse_args()
    run_sim_node(args)


if __name__ == "__main__":
    main()
