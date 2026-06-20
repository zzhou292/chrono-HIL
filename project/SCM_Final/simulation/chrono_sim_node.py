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
import csv
import math
import os
import sys
import time as wall_time
from pathlib import Path

import numpy as np

# Chrono imports (must be available in environment)
import pychrono as chrono
import pychrono.vehicle as veh

# Driver-view camera pose shared by Irrlicht and Chrono Sensor visualization.
# Chrono sensor cameras look forward along the local +X axis.
DRIVER_CAM_POS_LOCAL = chrono.ChVector3d(0.53, 0.7, 1.0)
DRIVER_CAM_ROT_LOCAL = chrono.ChQuaterniond(1, 0, 0, 0)
DRIVER_CAM_LOOKAHEAD_DISTANCE = 12.0

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
from safety import make_safety_filter
from collision_detector import CollisionLogger
from traffic import TrafficManager
from latency_profile import LatencyProfile

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


class ReplayDriver(veh.ChDriver):
    """Replays a recorded operator command trace for counterfactual replay.

    Reads a trace CSV (e.g. a prior run's sim_diag.csv) and, at each sim time,
    provides the recorded *operator raw* command (steering_op/throttle_op/
    braking_op, falling back to the applied columns). The sim's safety-filter
    path then screens this identical intent, so the same operator trace can be
    replayed filter-off vs each filter and every outcome difference is the
    filter's effect (the sim is bit-for-bit deterministic given the inputs).
    """

    def __init__(self, vehicle, csv_path):
        super().__init__(vehicle.GetVehicle())
        import pandas as pd
        d = pd.read_csv(csv_path)
        sc = "steering_op" if "steering_op" in d.columns else "steering"
        tc = "throttle_op" if "throttle_op" in d.columns else "throttle"
        bc = "braking_op" if "braking_op" in d.columns else "braking"
        self._t = pd.to_numeric(d["time"], errors="coerce").to_numpy()
        self._s = pd.to_numeric(d[sc], errors="coerce").to_numpy()
        self._th = pd.to_numeric(d[tc], errors="coerce").to_numpy()
        self._b = pd.to_numeric(d[bc], errors="coerce").to_numpy()
        self.m_steering = 0.0
        self.m_throttle = 0.0
        self.m_braking = 0.0
        print(f"  Replay driver: {len(self._t)} command samples "
              f"({self._t[0]:.2f}-{self._t[-1]:.2f}s) from {os.path.basename(csv_path)} [{sc}]")

    def Synchronize(self, time):
        self.m_steering = float(np.clip(np.interp(time, self._t, self._s), -1.0, 1.0))
        self.m_throttle = float(np.clip(np.interp(time, self._t, self._th), 0.0, 1.0))
        self.m_braking = float(np.clip(np.interp(time, self._t, self._b), 0.0, 1.0))

    def Advance(self, step):
        pass

    def GetSteering(self):
        return self.m_steering

    def GetThrottle(self):
        return self.m_throttle

    def GetBraking(self):
        return self.m_braking


def get_driver_camera_view(vehicle):
    """Return Irrlicht eye and look-at points matching the sensor driver POV."""
    chassis = vehicle.GetChassisBody()
    chassis_pos = chassis.GetPos()
    chassis_rot = chassis.GetRot()
    eye = chassis_pos + chassis_rot.Rotate(DRIVER_CAM_POS_LOCAL)

    # Sensor camera orientation is defined by DRIVER_CAM_ROT_LOCAL.  Irrlicht
    # uses a look-at target, so convert the same local +X camera direction into
    # a point in front of the camera.
    camera_forward_local = DRIVER_CAM_ROT_LOCAL.Rotate(chrono.ChVector3d(1, 0, 0))
    lookahead_local = chrono.ChVector3d(
        camera_forward_local.x * DRIVER_CAM_LOOKAHEAD_DISTANCE,
        camera_forward_local.y * DRIVER_CAM_LOOKAHEAD_DISTANCE,
        camera_forward_local.z * DRIVER_CAM_LOOKAHEAD_DISTANCE,
    )
    target = eye + chassis_rot.Rotate(lookahead_local)
    return eye, target


def update_irrlicht_driver_camera(vis, vehicle):
    """Keep the Irrlicht camera at the same chassis-fixed pose as DriverPOV."""
    eye, target = get_driver_camera_view(vehicle)
    # Match the standalone Irrlicht demo: drive the active camera explicitly.
    # The vehicle visual system's chase-camera wrapper can otherwise keep
    # restoring chase behavior on Synchronize/Advance in some PyChrono builds.
    if hasattr(vis, "SetChaseCameraPosition"):
        vis.SetChaseCameraPosition(eye, target)
    vis.SetCameraPosition(eye)
    vis.SetCameraTarget(target)


def set_z_up_if_available(vis):
    """Use Chrono's Z-up camera convention when exposed by the local bindings."""
    if hasattr(chrono, "CameraVerticalDir_Z"):
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)


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
    body_color = chrono.ChColor(0.88, 0.82, 0.66)
    set_visual_color(vehicle.GetChassisBody(), body_color)


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
    """Extract per-wheel tire forces and slips from Chrono.

    Forces are rotated from the global frame into the vehicle body frame
    so that Fx/Fy/Fz align with the bicycle-model convention used by MPC.
    """
    tf = {}
    veh_obj = vehicle.GetVehicle()
    chassis = vehicle.GetChassisBody()
    rot = chassis.GetRot()
    for axle_idx, axle_name in enumerate(['front', 'rear']):
        for side_idx, side_name in [(veh.LEFT, 'left'), (veh.RIGHT, 'right')]:
            tire = veh_obj.GetTire(axle_idx, side_idx)
            force_global = tire.ReportTireForce(terrain)
            # Rotate global-frame force into body frame
            f_body = rot.RotateBack(force_global.force)
            key = f'{axle_name}_{side_name}'
            tf[f'{key}_Fx'] = f_body.x
            tf[f'{key}_Fy'] = f_body.y
            tf[f'{key}_Fz'] = f_body.z
            tf[f'{key}_slip_angle'] = tire.GetSlipAngle()
            tf[f'{key}_long_slip'] = tire.GetLongitudinalSlip()
    return tf


def extract_vehicle_state(vehicle, sim_time: float, terrain=None,
                          noise: dict = None,
                          imu_acc_sensor=None,
                          imu_gyro_sensor=None,
                          obstacles_flat: list = None,
                          driver_io: tuple = None) -> VehicleState:
    """Read Chrono vehicle and pack into a VehicleState message.

    Args:
        terrain: If provided, tire forces are included.
        noise: If provided, dict of std-devs to add Gaussian noise to sensors.
        imu_acc_sensor: ChAccelerometerSensor (if available, replaces GetPosDt2).
        imu_gyro_sensor: ChGyroscopeSensor (if available, replaces GetAngVelLocal).
    """
    chassis = vehicle.GetChassisBody()
    pos = chassis.GetPos()
    rot = chassis.GetRot()
    vel = chassis.GetPosDt()

    # Velocity in body frame
    vel_loc = rot.RotateBack(vel)

    x_cg = pos.x
    y_cg = pos.y
    u = vel_loc.x
    v = vel_loc.y

    # --- IMU accelerometer (body-frame acceleration from sensor module) ---
    # Chrono's ChAccelerometerSensor outputs: a_global - gravity_global (in global frame).
    # Rotating to body frame gives the same result as rot.RotateBack(GetPosDt2()).
    _imu_acc_ok = False
    if imu_acc_sensor is not None:
        buf = imu_acc_sensor.GetMostRecentAccelBuffer()
        if buf.HasData():
            data = buf.GetAccelData()  # numpy (3,): global-frame, gravity subtracted
            acc_global = chrono.ChVector3d(float(data[0]), float(data[1]), float(data[2]))
            acc_body = rot.RotateBack(acc_global)
            ax = acc_body.x
            ay = acc_body.y
            az = acc_body.z
            _imu_acc_ok = True
    if not _imu_acc_ok:
        # Fallback: analytical rigid-body acceleration (ground truth)
        acc = chassis.GetPosDt2()
        acc_loc = rot.RotateBack(acc)
        ax = acc_loc.x
        ay = acc_loc.y
        az = acc_loc.z

    # --- IMU gyroscope (body-frame, includes noise from sensor module) ---
    _imu_gyro_ok = False
    if imu_gyro_sensor is not None:
        buf = imu_gyro_sensor.GetMostRecentGyroBuffer()
        if buf.HasData():
            data = buf.GetGyroData()  # numpy (3,): [Roll, Pitch, Yaw]
            omega_x = float(data[0])  # Roll rate
            omega_y = float(data[1])  # Pitch rate
            omega   = float(data[2])  # Yaw rate
            _imu_gyro_ok = True
    if not _imu_gyro_ok:
        omega_vec = chassis.GetAngVelLocal()
        omega_x = omega_vec.x
        omega_y = omega_vec.y
        omega   = omega_vec.z

    # Wheel angular velocities (wheel-encoder equivalent)
    veh_obj = vehicle.GetVehicle()
    wheel_omega_fl = veh_obj.GetSpindleOmega(0, veh.LEFT)
    wheel_omega_fr = veh_obj.GetSpindleOmega(0, veh.RIGHT)
    wheel_omega_rl = veh_obj.GetSpindleOmega(1, veh.LEFT)
    wheel_omega_rr = veh_obj.GetSpindleOmega(1, veh.RIGHT)

    # Road-wheel steering angle (steering-angle sensor equivalent, avg L/R)
    steer_angle = 0.5 * (veh_obj.GetSteeringAngle(0, veh.LEFT)
                         + veh_obj.GetSteeringAngle(0, veh.RIGHT))

    # Compute yaw from quaternion for noise injection
    psi = math.atan2(2 * (rot.e0 * rot.e3 + rot.e1 * rot.e2),
                     1 - 2 * (rot.e2 * rot.e2 + rot.e3 * rot.e3))

    # Sensor noise injection
    # NOTE: ax, ay, omega are already noisy when using Chrono sensor-module IMU.
    # Manual noise is only added to non-IMU channels (GPS, speed, etc.).
    if noise:
        x_cg  += np.random.normal(0, noise['x'])
        y_cg  += np.random.normal(0, noise['y'])
        psi   += np.random.normal(0, noise['psi'])
        u     += np.random.normal(0, noise['u'])
        v     += np.random.normal(0, noise['v'])
        # Only add manual noise to IMU channels if sensor module not active
        if not _imu_gyro_ok:
            omega += np.random.normal(0, noise['omega'])
        if not _imu_acc_ok:
            ax    += np.random.normal(0, noise.get('ax', 0.05))
            ay    += np.random.normal(0, noise.get('ay', 0.05))
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
        ax=ax,
        ay=ay,
        az=az,
        omega_x=omega_x,
        omega_y=omega_y,
        wheel_omega_fl=wheel_omega_fl,
        wheel_omega_fr=wheel_omega_fr,
        wheel_omega_rl=wheel_omega_rl,
        wheel_omega_rr=wheel_omega_rr,
        steering_angle=steer_angle,
        steering_op=float(driver_io[0]) if driver_io else 0.0,
        throttle_op=float(driver_io[1]) if driver_io else 0.0,
        braking_op=float(driver_io[2]) if driver_io else 0.0,
        steering_app=float(driver_io[3]) if driver_io else 0.0,
        throttle_app=float(driver_io[4]) if driver_io else 0.0,
        braking_app=float(driver_io[5]) if driver_io else 0.0,
        tire_forces=tf,
        obstacles=obstacles_flat,
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

    latency_profile = None
    if args.latency_profile_json:
        latency_profile = LatencyProfile.from_json(args.latency_profile_json)
        print(f"  Latency profile: {latency_profile.describe()}")
    initial_control_delay = (
        latency_profile.delay(0.0, "control") if latency_profile is not None
        else float(args.teleop_delay)
    )

    # ------------------------------------------------------------------
    # Setup vehicle
    # ------------------------------------------------------------------
    system, vehicle = setup_chrono_vehicle(
        any_vis, payload_mass=getattr(args, "payload_mass", 0.0),
        simple_powertrain=getattr(args, "simple_powertrain", False))

    # ------------------------------------------------------------------
    # Setup terrain
    # ------------------------------------------------------------------
    terrain_config = None
    if args.terrain_config:
        terrain_config = load_terrain_config(args.terrain_config)

    # Optional spatial soil transition (one preset blends into another along +x).
    spatial_spec = None
    base_preset = args.terrain
    if args.terrain_transition:
        from spatial_terrain import SpatialTransitionSpec
        start_preset = args.terrain_start or args.terrain
        if args.terrain_end is None:
            raise ValueError("--terrain-transition requires --terrain-end")
        spatial_spec = SpatialTransitionSpec(
            start_preset=start_preset,
            end_preset=args.terrain_end,
            transition_x=args.transition_x,
            transition_width=args.transition_width,
        )
        # Base soil must match the start of the patch so the fallback agrees
        # with the callback before the transition.
        base_preset = start_preset

    terrain, terrain_params = setup_scm_terrain(
        system, vehicle=vehicle, visualize=any_vis,
        terrain_preset=base_preset, terrain_config=terrain_config,
        bumpiness=args.bumpiness, spatial_spec=spatial_spec,
        mesh_resolution=args.mesh_resolution,
    )

    if any_vis:
        color_hmmwv(vehicle)

    # ------------------------------------------------------------------
    # Rock obstacles
    # ------------------------------------------------------------------
    rocks = []
    collision_logger = None
    if args.rocks > 0:
        # Only clear the immediate spawn so you're in the field quickly; also
        # keep the goal gate clear if one is set.
        exclusion_zones = [(0.0, 0.0, args.rock_spawn_clear)]
        if args.goal_distance > 0:
            exclusion_zones.append((float(args.goal_distance), 0.0, 6.0))
        rocks = add_rock_obstacles(
            system, num_rocks=args.rocks,
            zone_x=tuple(args.rock_zone_x), zone_y=tuple(args.rock_zone_y),
            size_range=tuple(args.rock_size), seed=args.rock_seed,
            min_spacing=args.rock_min_spacing,
            centerline_clear=args.rock_centerline_clear,
            exclusion_zones=exclusion_zones,
        )
        print(f"  Placed {len(rocks)} rock obstacles")

    # --- Convoy traffic vehicles (PID-driven, shared system) ---
    traffic_mgr = None
    if args.convoy:
        traffic_mgr = TrafficManager.from_preset(args.convoy, ego_lane_y=0.0)
        _detail = args.traffic_detail if any_vis else "none"
        traffic_mgr.build(system, terrain, detail=_detail)
        print(f"  Convoy '{args.convoy}': {len(traffic_mgr.vehicles)} traffic vehicles")

    # --- Goal gate (visible finish line; round ends early on reaching it) ---
    if args.goal_distance > 0 and any_vis:
        _gx = float(args.goal_distance)
        for _gy in (-3.5, 3.5):                       # two bright posts
            post = chrono.ChBodyEasyBox(0.4, 0.4, 3.5, 100.0, True, False)
            post.SetPos(chrono.ChVector3d(_gx, _gy, 1.75)); post.SetFixed(True)
            try:
                post.GetVisualShape(0).SetColor(chrono.ChColor(0.1, 0.9, 0.2))
            except Exception:
                pass
            system.Add(post)
        banner = chrono.ChBodyEasyBox(0.4, 7.4, 0.5, 100.0, True, False)
        banner.SetPos(chrono.ChVector3d(_gx, 0.0, 3.3)); banner.SetFixed(True)
        try:
            banner.GetVisualShape(0).SetColor(chrono.ChColor(0.1, 0.9, 0.2))
        except Exception:
            pass
        system.Add(banner)
        print(f"  Goal gate at x={_gx:.0f} m (round ends on reaching it)")

    # --- Collision detector (active when rocks OR traffic present) ---
    # Parallel sweeps set HIL_RUN_LOG_DIR to a unique per-run directory so the
    # collision / shield / warning logs are NOT shared across concurrent
    # workers (a shared global ``logs/`` races on truncation and cross-
    # contaminates collision counts between runs).  Live/manual runs leave the
    # env unset and keep the historical global ``logs/`` location.
    _log_dir = os.environ.get('HIL_RUN_LOG_DIR') or os.path.join(
        os.path.dirname(__file__), '..', 'logs')
    collision_logger = CollisionLogger(rocks, run_dir=_log_dir) if (rocks or traffic_mgr) else None

    # ------------------------------------------------------------------
    # CBF safety filter
    # ------------------------------------------------------------------
    safety_filter = None
    if args.safety_filter:
        vehicle_params = get_vehicle_params_for_demo()

        # Load NN tire model for terrain-aware CBF traction limits.
        # Uses the same model as the MPC controller for consistency.
        # Falls back to kinematic/linear if NN unavailable (import failed or model missing).
        _nn_cbf = None
        if args.no_safety_nn:
            print("  [CBF] NN tire model disabled by --no-safety-nn; using kinematic fallback")
        elif load_nn_tire_model is not None:
            try:
                _preset = terrain_config if terrain_config else get_terrain_preset(args.terrain)
                _tp = terrain_preset_to_internal(_preset)
                _root = Path(__file__).resolve().parent.parent
                _requested = Path(args.nn_model).expanduser()
                if _requested.is_absolute() or len(_requested.parts) > 1:
                    _model_dir = _requested if _requested.is_absolute() else _root / _requested
                else:
                    _model_dir = _root / "nn_models" / args.nn_model
                _nn_cbf = load_nn_tire_model(str(_model_dir), _tp)
                print(f"  [CBF] NN tire model loaded: {args.nn_model} on {args.terrain}")
            except Exception as _e:
                print(f"  [CBF] NN load failed ({_e}), using kinematic fallback")

        flavor = (args.safety_flavor or 'dob_cbf').lower()
        if flavor in ('mppi', 'nmpc'):
            # Predictive shield needs the live terrain preset dict
            _preset_internal = (terrain_preset_to_internal(terrain_config)
                                if terrain_config is not None
                                else terrain_preset_to_internal(get_terrain_preset(args.terrain)))
            common_kwargs = dict(
                max_speed=15.0,
                # Keep the predictive shield's collision geometry aligned
                # with CollisionLogger.  The previous 1.0 m radius
                # under-estimated the HMMWV footprint by 0.5 m, so MPPI/NMPC
                # selected rollouts that looked safe internally but still
                # clipped rocks in the benchmark.
                vehicle_radius=1.5,
                obstacle_buffer=args.safety_buffer,
                teleop_delay=initial_control_delay,
                stale_cmd_timeout=args.stale_cmd_timeout,
                control_dt=0.1,
            )
            if flavor == 'mppi':
                # --shield-no-sigma-gate is a back-compat alias for
                # --shield-sigma-mode off; if both are set, off wins.
                sigma_mode = ("off" if getattr(args, "shield_no_sigma_gate", False)
                              else getattr(args, "shield_sigma_mode", "inflate"))
                safety_filter = make_safety_filter(
                    'mppi', vehicle_params=vehicle_params,
                    nn_model=_nn_cbf, terrain_params=_preset_internal,
                    horizon=args.shield_horizon,
                    n_samples=args.mppi_samples,
                    sigma_steer=args.mppi_sigma_steer,
                    sigma_alpha=args.mppi_sigma_alpha,
                    temperature=args.mppi_temperature,
                    disable_seeds=args.mppi_no_seeds,
                    sigma_mode=sigma_mode,
                    sigma_buffer_gain=args.shield_sigma_buffer_gain,
                    **common_kwargs,
                )
                delay_msg = (f", teleop_delay={initial_control_delay*1000:.0f}ms"
                             if initial_control_delay > 0 else "")
                print(f"  [SAFETY] MPPI shield enabled: K={args.mppi_samples}, "
                      f"H={args.shield_horizon}, sigma=({args.mppi_sigma_steer},"
                      f"{args.mppi_sigma_alpha}){delay_msg}")
            else:  # nmpc
                safety_filter = make_safety_filter(
                    'nmpc', vehicle_params=vehicle_params,
                    nn_model=_nn_cbf, terrain_params=_preset_internal,
                    horizon=args.shield_horizon,
                    n_iter=args.nmpc_iter,
                    **common_kwargs,
                )
                delay_msg = (f", teleop_delay={initial_control_delay*1000:.0f}ms"
                             if initial_control_delay > 0 else "")
                print(f"  [SAFETY] NMPC shield enabled: H={args.shield_horizon}, "
                      f"maxiter={args.nmpc_iter}{delay_msg}")
        else:
            safety_filter = make_safety_filter(
                'dob_cbf', vehicle_params=vehicle_params,
                nn_model=_nn_cbf,
                cbf_alpha=args.cbf_alpha,
                obstacle_buffer=args.safety_buffer,
                delay_steps=args.delay_steps,
                control_dt=0.1,
                w_long=args.cbf_w_long,
                w_lat=args.cbf_w_lat,
                forward_bias=args.cbf_forward_bias,
                dob_bandwidth=args.dob_bandwidth,
                cbf_flavor=args.cbf_flavor,
                teleop_delay=initial_control_delay,
                stale_cmd_timeout=args.stale_cmd_timeout,
            )
            delay_msg = (f", teleop_delay={initial_control_delay*1000:.0f}ms"
                         if initial_control_delay > 0 else "")
            print(f"  [SAFETY] DOB-CBF filter enabled: alpha={args.cbf_alpha}, "
                  f"buffer={args.safety_buffer}m, flavor={args.cbf_flavor}{delay_msg}")

    # ------------------------------------------------------------------
    # Collision warning system (modular, runs in parallel with any safety filter)
    # ------------------------------------------------------------------
    warning_system = None
    warning_csv_file = None
    warning_csv_writer = None
    last_warning_severity = 0
    if args.collision_warning:
        from safety.collision_warning import make_collision_warning_system
        # Default initial n from the configured terrain preset; will be
        # overridden by live ControlCommand.terrain_n if the controller
        # supplies one.
        _preset_for_n = (terrain_config if terrain_config
                          else get_terrain_preset(args.terrain))
        _initial_n = float(_preset_for_n.get("n", 0.7))
        warning_system = make_collision_warning_system(
            flavor="ttc", verbose=False,
            tire_model_dir=args.cw_tire_model,
            reaction_time_s=args.cw_reaction_time,
        )
        warning_system.set_teleop_delay(float(args.teleop_delay or 0.0))
        # CSV log for post-hoc analysis
        _wpath = (Path(args.collision_warning_csv)
                  if args.collision_warning_csv
                  else Path(_log_dir) / "collision_warning_log.csv")
        _wpath.parent.mkdir(parents=True, exist_ok=True)
        warning_csv_file = _wpath.open("w", newline="")
        warning_csv_writer = csv.writer(warning_csv_file)
        warning_csv_writer.writerow([
            "sim_time", "u", "severity", "ttc", "clearance",
            "stopping_distance", "margin", "latency_inflation_m",
            "terrain_n_used",
        ])
        print(f"  [WARN] Collision warning ENABLED "
              f"(tire_model={Path(args.cw_tire_model).name}, "
              f"initial_n={_initial_n:.2f}, log={_wpath})")
        # Pre-built brake-decel table summary (helpful for sanity)
        if warning_system._brake_table:
            head = warning_system._brake_table[0]
            tail = warning_system._brake_table[-1]
            print(f"  [WARN] brake-decel table: "
                  f"a({head[0]:.2f})={head[1]:.2f} → "
                  f"a({tail[0]:.2f})={tail[1]:.2f} m/s²")
    warning_n_live = None    # set by ControlCommand.terrain_n if present

    # ------------------------------------------------------------------
    # Trajectory markers (visual only)
    # ------------------------------------------------------------------
    if any_vis:
        marker_z = 0.5 if args.bumpiness > 0 else 0.15
        add_trajectory_markers(
            system, args.path,
            marker_z=marker_z,
            lead_in=args.lead_in,
        )

    # ------------------------------------------------------------------
    # Driver (external commands or manual G29)
    # ------------------------------------------------------------------
    if args.replay_cmds:
        print(f"  Replay mode: re-driving from command trace {args.replay_cmds}")
        driver = ReplayDriver(vehicle, args.replay_cmds)
    elif args.wasd:
        print("  Manual mode: using WASD keyboard (via Irrlicht window)")
        driver = veh.ChInteractiveDriver(vehicle.GetVehicle())
        driver.SetSteeringDelta(1.0 / 50)
        driver.SetThrottleDelta(1.0 / 50)
        driver.SetBrakingDelta(1.0 / 50)
        driver.SetGains(4.0, 4.0, 4.0, 4.0)
        driver.Initialize()
    elif args.manual:
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
            vis.SetWindowSize(args.cam_width, args.cam_height)
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
            update_irrlicht_driver_camera(vis, vehicle)
            if args.wasd:
                vis.AttachDriver(driver)
            print("  Irrlicht: driver POV camera active")
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
                DRIVER_CAM_POS_LOCAL,
                DRIVER_CAM_ROT_LOCAL,
            )
            driver_cam = sens.ChCameraSensor(
                vehicle.GetChassisBody(),  # attached body
                args.cam_rate,             # render rate (Hz) — real-time lever
                cam_offset,                # offset pose
                args.cam_width,            # image width
                args.cam_height,           # image height
                args.cam_fov,              # horizontal FOV (rad)
            )
            driver_cam.SetName("DriverPOV")
            driver_cam.SetLag(latency_profile.delay(0.0, "camera") if latency_profile is not None else 0.0)
            driver_cam.PushFilter(sens.ChFilterVisualize(
                args.cam_width, args.cam_height, "Driver POV", args.cam_fullscreen
            ))
            sensor_manager.AddSensor(driver_cam)
            print("  Chrono Sensor: driver POV camera active")
        except Exception as e:
            print(f"Warning: Sensor visualization failed: {e}")
            sensor_manager = None
            driver_cam = None

    # ------------------------------------------------------------------
    # IMU Sensors (Chrono Sensor module — accelerometer + gyroscope)
    # ------------------------------------------------------------------
    imu_acc_sensor = None
    imu_gyro_sensor = None
    if HAS_SENSOR and not args.no_imu:
        try:
            # Create a sensor manager if camera mode didn't already
            if sensor_manager is None:
                sensor_manager = sens.ChSensorManager(system)

            imu_rate = args.imu_rate  # Hz
            imu_offset = chrono.ChFramed(
                chrono.ChVector3d(0, 0, 0),
                chrono.ChQuaterniond(1, 0, 0, 0),
            )

            # --- Noise models ---
            if args.no_noise:
                acc_noise = sens.ChNoiseNone()
                gyro_noise = sens.ChNoiseNone()
            else:
                # ChNoiseNormalDrift: Gaussian + slow-varying bias drift
                #   (updateRate, mean, stdev, bias_drift, tau_drift)
                # Typical automotive-grade MEMS accelerometer:
                #   noise density ~150 µg/√Hz → stdev ≈ 0.015 m/s² at 100 Hz
                #   bias stability ~10 µg → drift ~ 1e-4 m/s²
                acc_noise = sens.ChNoiseNormalDrift(
                    float(imu_rate),
                    chrono.ChVector3d(0, 0, 0),                                      # mean
                    chrono.ChVector3d(args.imu_acc_stdev, args.imu_acc_stdev, args.imu_acc_stdev),  # stdev
                    args.imu_acc_bias_drift,                                          # bias drift rate
                    args.imu_acc_tau_drift,                                           # tau drift (s)
                )
                # Typical automotive-grade MEMS gyroscope:
                #   noise density ~0.005 °/s/√Hz → stdev ≈ 0.001 rad/s at 100 Hz
                #   bias stability ~1 °/hr → drift ~ 5e-6 rad/s
                gyro_noise = sens.ChNoiseNormalDrift(
                    float(imu_rate),
                    chrono.ChVector3d(0, 0, 0),                                          # mean
                    chrono.ChVector3d(args.imu_gyro_stdev, args.imu_gyro_stdev, args.imu_gyro_stdev),  # stdev
                    args.imu_gyro_bias_drift,                                            # bias drift rate
                    args.imu_gyro_tau_drift,                                             # tau drift (s)
                )

            # --- Accelerometer ---
            imu_acc_sensor = sens.ChAccelerometerSensor(
                vehicle.GetChassisBody(),
                float(imu_rate),
                imu_offset,
                acc_noise,
            )
            imu_acc_sensor.SetName("IMU_Accelerometer")
            imu_acc_sensor.SetLag(args.imu_lag)
            imu_acc_sensor.SetCollectionWindow(0.0)
            imu_acc_sensor.PushFilter(sens.ChFilterAccelAccess())
            sensor_manager.AddSensor(imu_acc_sensor)

            # --- Gyroscope ---
            imu_gyro_sensor = sens.ChGyroscopeSensor(
                vehicle.GetChassisBody(),
                float(imu_rate),
                imu_offset,
                gyro_noise,
            )
            imu_gyro_sensor.SetName("IMU_Gyroscope")
            imu_gyro_sensor.SetLag(args.imu_lag)
            imu_gyro_sensor.SetCollectionWindow(0.0)
            imu_gyro_sensor.PushFilter(sens.ChFilterGyroAccess())
            sensor_manager.AddSensor(imu_gyro_sensor)

            noise_label = "OFF" if args.no_noise else (
                f"acc_σ={args.imu_acc_stdev}, gyro_σ={args.imu_gyro_stdev}"
            )
            print(f"  IMU sensors: {imu_rate} Hz, lag={args.imu_lag}s, noise={noise_label}")
        except Exception as e:
            print(f"Warning: IMU sensor setup failed: {e}")
            imu_acc_sensor = None
            imu_gyro_sensor = None
    elif not HAS_SENSOR and not args.no_imu:
        print("  WARNING: pychrono.sensor not available — using analytical accel/gyro (ground truth)")

    # ------------------------------------------------------------------
    # ZMQ transport (skipped in manual mode)
    # ------------------------------------------------------------------
    _manual_mode = args.manual or args.wasd or bool(args.replay_cmds)
    state_pub = None
    ctrl_sub = None
    # Always publish vehicle_state: the live HUD, terrain classifier, and
    # telemetry all subscribe to it (including in g29/wasd manual rounds). Only
    # the autonomous controller link needs the inbound ctrl_sub.
    if True:
        state_pub = ZMQPublisher(sim_pub_endpoint(args.sim_port))
        print(f"  Publishing state on port {args.sim_port}")
        if not _manual_mode:
            ctrl_sub = ZMQSubscriber(ctrl_sub_endpoint(args.ctrl_host, args.ctrl_port))
            print(f"  Subscribing to controls from {args.ctrl_host}:{args.ctrl_port}")

        # Give ZMQ sockets time to connect
        wall_time.sleep(0.3)

        # Publish initial config so controller knows terrain / vehicle params
        vehicle_params = get_vehicle_params_for_demo()
        internal_terrain = terrain_preset_to_internal(
            terrain_config if terrain_config else get_terrain_preset(args.terrain)
        )
        # Named preset string is for logging/telemetry; YAML soil overrides physics.
        terrain_label = args.terrain
        if terrain_config is not None:
            terrain_label = "custom"

        config_msg = SimStatus(
            event="config",
            time=0.0,
            wall_time=wall_time.time(),
            config={
                "vehicle_params": vehicle_params,
                "terrain_params": internal_terrain,
                "terrain_preset": terrain_label,
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
        if ctrl_sub is not None and wait_s > 0:
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
    # Gate sensor manager updates:
    # - When IMU sensors are active, Update() must be called EVERY physics step
    #   (the sensor internally handles its own update rate scheduling).
    # - When only camera is active, gate to camera FPS to avoid overhead.
    _imu_active = (imu_acc_sensor is not None or imu_gyro_sensor is not None)
    sensor_interval = 0.0 if _imu_active else (1.0 / 30.0)
    last_sensor_time = -1.0
    last_report_time = 0.0
    start_wall = wall_time.time()
    cmd_count = 0
    cmd_buffer = []
    cmd_seq = 0
    terrain_update_count = 0
    last_terrain_seq = -1
    manual_cmd_buffer = []
    delayed_manual_inputs = [0.0, 0.0, 0.0]
    cam_lag_ema = None   # EMA-smoothed camera lag (anti-stutter, see SetLag below)
    steer_diverge_t = 0.0   # accumulated time the actual steer angle defies the command
    steer_broken = False    # latched once the front steering/suspension breaks
    step_count = 0
    sim_diag_file = None
    sim_diag_writer = None
    sim_diag_interval = 0.1
    last_sim_diag_time = -sim_diag_interval
    if args.sim_diag_csv:
        diag_path = Path(args.sim_diag_csv)
        diag_path.parent.mkdir(parents=True, exist_ok=True)
        sim_diag_file = diag_path.open("w", newline="")
        sim_diag_writer = csv.writer(sim_diag_file)
        sim_diag_writer.writerow([
            "time", "x", "y", "z", "speed", "vx_local", "vy_local", "omega_z",
            "steering", "throttle", "braking", "collisions", "near_misses",
            "nearest_clearance_m", "latency_control_s", "latency_manual_s", "latency_camera_s",
            "steering_op", "throttle_op", "braking_op",
        ])
        print(f"  Sim diagnostic CSV: {diag_path}")

    latency_log_file = None
    latency_log_writer = None
    latency_log_interval = 0.05
    last_latency_log_time = -latency_log_interval
    if args.latency_profile_log:
        latency_log_path = Path(args.latency_profile_log)
        latency_log_path.parent.mkdir(parents=True, exist_ok=True)
        latency_log_file = latency_log_path.open("w", newline="")
        latency_log_writer = csv.writer(latency_log_file)
        latency_log_writer.writerow(["time", "control_delay_s", "manual_delay_s", "camera_delay_s"])
        print(f"  Latency profile log: {latency_log_path}")

    noise_cfg = None if args.no_noise else DEFAULT_MEAS_NOISE
    print(f"  Sensor noise: {'OFF' if noise_cfg is None else 'ON'}")
    print(f"  Physics step: {step_size * 1000:.0f}ms, state rate: {args.state_rate} Hz")
    if _manual_mode:
        if args.manual_honor_time:
            print(f"  Manual mode: automatic stop after {args.time}s")
        else:
            print(f"  Manual mode: close window to exit")
        if args.manual_input_delay > 0:
            print(f"  Manual input actuation delay: {args.manual_input_delay:.3f}s")
        elif latency_profile is not None:
            print(f"  Manual input actuation delay: profile-driven")
    else:
        print(f"  Running {args.time}s simulation...")

    # --- Timing accumulators (debug) ---
    _t_irr = 0.0; _t_sensor = 0.0; _t_terrain_sync = 0.0; _t_terrain_adv = 0.0
    _t_veh_sync = 0.0; _t_veh_adv = 0.0; _t_driver = 0.0; _t_safety = 0.0
    _t_zmq = 0.0; _t_vis_sync = 0.0; _t_vis_adv = 0.0
    _t_loop_total = 0.0; _t_state_extract = 0.0; _t_rt_sleep = 0.0
    _t_report_steps = 0; _sensor_calls = 0

    while True:
        _t_loop_start = wall_time.time()
        time_chrono = vehicle.GetSystem().GetChTime()
        if latency_profile is not None:
            control_delay_s = latency_profile.delay(time_chrono, "control")
            manual_delay_s = latency_profile.delay(time_chrono, "manual")
            camera_delay_s = latency_profile.delay(time_chrono, "camera")
        else:
            control_delay_s = float(args.teleop_delay)
            manual_delay_s = float(args.manual_input_delay)
            camera_delay_s = float(args.camera_input_delay)
        if safety_filter is not None:
            safety_filter.set_teleop_delay(control_delay_s)

        if (not _manual_mode or args.manual_honor_time or args.replay_cmds) and time_chrono >= args.time:
            break
        if args.goal_distance > 0 and vehicle.GetVehicle().GetPos().x >= args.goal_distance:
            print(f"  GOAL REACHED: ego x={vehicle.GetVehicle().GetPos().x:.1f} m "
                  f">= goal {args.goal_distance:.0f} m at t={time_chrono:.1f} s -- ending round early")
            break
        if vis is not None and not vis.Run():
            break

        # --- Render Irrlicht (frame-skipped) ---
        if vis is not None and (time_chrono - last_render_time >= render_interval):
            _tw = wall_time.time()
            update_irrlicht_driver_camera(vis, vehicle)
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            _t_irr += wall_time.time() - _tw
            last_render_time = time_chrono

        # --- Receive latest control command (non-blocking) ---
        if ctrl_sub is not None:
            result = ctrl_sub.recv(timeout_ms=0)
            if result is not None:
                topic, msg = result
                if isinstance(msg, ControlCommand):
                    if control_delay_s > 0:
                        cmd_seq += 1
                        cmd_buffer.append((wall_time.time() + control_delay_s, cmd_seq, msg))
                        cmd_buffer.sort(key=lambda item: (item[0], item[1]))
                    else:
                        driver.apply(msg)
                        cmd_count += 1
                        # Feed command age to safety filter for teleop delay est.
                        if safety_filter is not None and msg.wall_time > 0:
                            safety_filter.update_command_age(msg.wall_time)
                        # Same feed to the collision warning system.
                        if warning_system is not None and msg.wall_time > 0:
                            warning_system.update_command_age(msg.wall_time)
                        # Live terrain n from controller (if available)
                        if (warning_system is not None
                                and getattr(msg, "terrain_n", None) is not None):
                            warning_n_live = float(msg.terrain_n)
                    # Live terrain estimate piggybacks on ControlCommand.
                    # Dispatch to the shield whenever the controller's
                    # terrain_update_seq advances; --shield-no-sigma-gate
                    # zeroes sigma so the shield ignores estimator
                    # disagreement (the abstract's ablation).
                    if (safety_filter is not None
                            and getattr(msg, "terrain_n", None) is not None
                            and msg.terrain_update_seq > last_terrain_seq):
                        last_terrain_seq = int(msg.terrain_update_seq)
                        sigma_deg = (0.0 if getattr(args, "shield_no_sigma_gate", False)
                                     else float(msg.terrain_phi_sigma_deg or 0.0))
                        tp = {
                            "Kphi": float(msg.terrain_Kphi or 0.0),
                            "Kc":   float(msg.terrain_Kc or 0.0),
                            "n":    float(msg.terrain_n),
                            "c":    float(msg.terrain_c or 0.0),
                            "phi":  float(msg.terrain_phi_deg),
                            "k":    float(msg.terrain_k or 0.0),
                        }
                        try:
                            safety_filter.update_terrain(tp, phi_uncertainty_deg=sigma_deg)
                        except TypeError:
                            safety_filter.update_terrain(tp)
                        terrain_update_count += 1
                        if terrain_update_count == 1 or terrain_update_count % 50 == 0:
                            print(f"  [SHIELD-TERRAIN] update #{terrain_update_count}: "
                                  f"n={msg.terrain_n:.3f} phi={msg.terrain_phi_deg:.2f}° "
                                  f"sigma_phi={sigma_deg:.2f}° "
                                  f"class={msg.terrain_class}", flush=True)

            now = wall_time.time()
            while cmd_buffer and cmd_buffer[0][0] <= now:
                _, _, msg = cmd_buffer.pop(0)
                driver.apply(msg)
                cmd_count += 1
                if safety_filter is not None and msg.wall_time > 0:
                    safety_filter.update_command_age(msg.wall_time)

        # --- Synchronize ---
        _tw = wall_time.time()
        driver.Synchronize(time_chrono)

        driver_inputs = veh.DriverInputs()
        driver_inputs.m_steering = driver.GetSteering()
        driver_inputs.m_throttle = driver.GetThrottle()
        driver_inputs.m_braking = driver.GetBraking()
        # Operator's raw command (pre-delay, pre-safety-filter) for the HMI ghost.
        op_io = (driver_inputs.m_steering, driver_inputs.m_throttle,
                 driver_inputs.m_braking)
        if _manual_mode and manual_delay_s > 0:
            manual_cmd_buffer.append((
                wall_time.time() + manual_delay_s,
                driver_inputs.m_steering,
                driver_inputs.m_throttle,
                driver_inputs.m_braking,
            ))
            # Sort by due time: under jittery latency a command with a large
            # delay must not block newer low-delay commands behind it (FIFO
            # would freeze the controls when the delay spikes then drops).
            manual_cmd_buffer.sort(key=lambda c: c[0])
            now_manual = wall_time.time()
            while manual_cmd_buffer and manual_cmd_buffer[0][0] <= now_manual:
                _, delayed_manual_inputs[0], delayed_manual_inputs[1], delayed_manual_inputs[2] = manual_cmd_buffer.pop(0)
            driver_inputs.m_steering = delayed_manual_inputs[0]
            driver_inputs.m_throttle = delayed_manual_inputs[1]
            driver_inputs.m_braking = delayed_manual_inputs[2]
        _t_driver += wall_time.time() - _tw

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
                        # 4th element False => static rock (CBF prefers steering)
                        all_obstacles.append((rock_pos[i, 0], rock_pos[i, 1], rock_rad[i], False))
            if traffic_mgr is not None:
                for ox, oy, orad in traffic_mgr.obstacles():
                    if (ox - veh_pos.x) ** 2 + (oy - veh_pos.y) ** 2 < 30.0 ** 2:
                        # 4th element True => vehicle (CBF weights braking equally)
                        all_obstacles.append((ox, oy, orad, True))

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

        # Applied command (post delay + safety filter) for the HMI solid trace.
        app_io = (driver_inputs.m_steering, driver_inputs.m_throttle,
                  driver_inputs.m_braking)
        driver_io = op_io + app_io

        # --- Front steering/suspension break detection ---
        # Compare the ACTUAL front road-wheel angle (from the vehicle) to what we
        # commanded. A real break makes the wheels stop responding, splay apart,
        # or snap to an impossible angle. (The earlier in-filter check compared
        # the command to itself -- it was fed the commanded steering as 'delta',
        # not the measured angle -- so it could never fire. This runs every step,
        # filter or not.) A sustained-divergence timer keeps normal steering lag
        # from false-tripping; the insane-angle checks fire immediately.
        if not steer_broken:
            try:
                _vo = vehicle.GetVehicle()
                _sa_l = _vo.GetSteeringAngle(0, veh.LEFT)
                _sa_r = _vo.GetSteeringAngle(0, veh.RIGHT)
                _steer_act = 0.5 * (_sa_l + _sa_r)
                _cmd_ang = driver_inputs.m_steering * 0.49
                _insane = ((not math.isfinite(_steer_act)) or abs(_steer_act) > 0.9
                           or abs(_sa_l - _sa_r) > 0.6)   # max physical ~0.49; Ackermann split is small
                if abs(_cmd_ang) > 0.12 and abs(_steer_act - _cmd_ang) > 0.28:
                    steer_diverge_t += step_size
                else:
                    steer_diverge_t = 0.0
                if _insane or steer_diverge_t > 1.0:
                    steer_broken = True
                    _msg = (f"FRONT STEERING/SUSPENSION LIKELY BROKEN at t={time_chrono:.1f}s: "
                            f"commanded {_cmd_ang:+.2f} rad, actual L/R "
                            f"{_sa_l:+.2f}/{_sa_r:+.2f} rad -- vehicle is unresponsive to "
                            f"steering. DISCARD THIS ROUND.")
                    print(f"\n  ** {_msg} **\n", flush=True)
                    try:
                        _ld = os.environ.get('HIL_RUN_LOG_DIR') or os.path.join(
                            os.path.dirname(__file__), 'logs')
                        os.makedirs(_ld, exist_ok=True)
                        with open(os.path.join(_ld, 'steering_break.txt'), 'w') as _fh:
                            _fh.write(_msg + "\n")
                    except Exception:
                        pass
            except Exception:
                pass

        _tw = wall_time.time()
        terrain.Synchronize(time_chrono)
        _t_terrain_sync += wall_time.time() - _tw

        _tw = wall_time.time()
        vehicle.Synchronize(time_chrono, driver_inputs, terrain)
        if traffic_mgr is not None:
            _rock_obs = None
            if args.rocks > 0 and rocks:
                _rp = get_rock_positions(rocks); _rr = get_rock_radii(rocks)
                _rock_obs = [(float(_rp[i, 0]), float(_rp[i, 1]), float(_rr[i]))
                             for i in range(len(_rp))]
            traffic_mgr.synchronize(time_chrono, terrain,
                                    ego_speed=vehicle.GetVehicle().GetSpeed(),
                                    avoid_obstacles=_rock_obs)
        _t_veh_sync += wall_time.time() - _tw

        if vis is not None:
            _tw = wall_time.time()
            vis.Synchronize(time_chrono, driver_inputs)
            _t_vis_sync += wall_time.time() - _tw

        # --- Advance ---
        driver.Advance(step_size)

        _tw = wall_time.time()
        terrain.Advance(step_size)
        _t_terrain_adv += wall_time.time() - _tw

        _tw = wall_time.time()
        vehicle.Advance(step_size)        # ego owns the system -> steps it once
        if traffic_mgr is not None:
            traffic_mgr.advance(step_size)
        _t_veh_adv += wall_time.time() - _tw

        if vis is not None:
            _tw = wall_time.time()
            vis.Advance(step_size)
            _t_vis_adv += wall_time.time() - _tw

        # --- Update Chrono Sensor manager (gated to camera FPS) ---
        if sensor_manager is not None and (time_chrono - last_sensor_time >= sensor_interval):
            # Apply camera lag whenever a non-zero delay is active, whether it
            # comes from the time-varying latency profile or the fixed
            # --camera-input-delay flag used by the HIL delay sweep.
            #
            # Smooth the lag with an EMA before applying it. The profile's
            # per-frame camera delay is jittery; pushing a different lag into the
            # sensor every frame makes buffered frames release at uneven
            # intervals, which shows up as visible stutter once the scene is
            # moving (i.e. while you're driving) and looks fine when stopped.
            # The EMA keeps the slow good/poor-regime variation but kills the
            # frame-to-frame jump, so the view is smooth at a realistic delay.
            if driver_cam is not None and camera_delay_s > 0.0:
                if cam_lag_ema is None:
                    cam_lag_ema = camera_delay_s
                else:
                    cam_lag_ema += 0.1 * (camera_delay_s - cam_lag_ema)
                try:
                    driver_cam.SetLag(cam_lag_ema)
                except Exception:
                    pass
            _tw = wall_time.time()
            sensor_manager.Update()
            _dt_s = wall_time.time() - _tw
            _t_sensor += _dt_s
            _sensor_calls += 1
            last_sensor_time = time_chrono

        step_count += 1
        _t_report_steps += 1

        # --- Re-publish config during first 2s (CONFLATE can drop it) ---
        if state_pub is not None and time_chrono < 2.0 and time_chrono - last_config_resend >= 0.2:
            config_msg.wall_time = wall_time.time()
            state_pub.send(config_msg)
            last_config_resend = time_chrono

        # --- Publish vehicle state at decimated rate ---
        if state_pub is not None and time_chrono - last_state_pub_time >= state_pub_interval:
            _tw = wall_time.time()
            # Nearest 3 obstacles (rocks + traffic) within 40m → flat list for
            # MPC horizon planning. Traffic poses are dynamic, refreshed here.
            _obs_flat_msg = []
            _vpos_now = vehicle.GetChassisBody().GetPos()
            _vx, _vy = _vpos_now.x, _vpos_now.y
            _cands = []  # (dist, x, y, r)
            if args.rocks > 0:
                _rpos = get_rock_positions(rocks)
                _rrad = get_rock_radii(rocks)
                for _i in range(len(_rpos)):
                    _cands.append((math.hypot(_rpos[_i, 0] - _vx, _rpos[_i, 1] - _vy),
                                   float(_rpos[_i, 0]), float(_rpos[_i, 1]), float(_rrad[_i])))
            if traffic_mgr is not None:
                for ox, oy, orad in traffic_mgr.obstacles():
                    _cands.append((math.hypot(ox - _vx, oy - _vy), ox, oy, orad))
            _cands = sorted((c for c in _cands if c[0] < 40.0), key=lambda c: c[0])
            for _d, ox, oy, orad in _cands[:3]:
                _obs_flat_msg += [ox, oy, orad]

            state_msg = extract_vehicle_state(
                vehicle, time_chrono,
                terrain=None if args.no_tire_forces else terrain,
                noise=noise_cfg,
                imu_acc_sensor=imu_acc_sensor,
                imu_gyro_sensor=imu_gyro_sensor,
                obstacles_flat=_obs_flat_msg if _obs_flat_msg else None,
                driver_io=driver_io,
            )
            state_pub.send(state_msg)
            _t_state_extract += wall_time.time() - _tw
            last_state_pub_time = time_chrono

        # --- Real-time pacing (always on unless --no-rt) ---
        # Without this, the headless sim runs 4-5x real-time and the
        # decoupled MPC controller can only process ~10% of state messages.
        if not args.no_rt:
            target_wall = start_wall + time_chrono
            remaining = target_wall - wall_time.time()
            if remaining > 0:
                _t_rt_sleep += remaining
                wall_time.sleep(remaining)

        _t_loop_total += wall_time.time() - _t_loop_start

        # --- Collision detection (every physics step when rocks present) ---
        if collision_logger is not None:
            _veh_cg = vehicle.GetChassisBody().GetPos()
            _veh_spd = vehicle.GetVehicle().GetSpeed()
            _traffic_obs = traffic_mgr.obstacles() if traffic_mgr is not None else None
            collision_logger.check(time_chrono, _veh_cg.x, _veh_cg.y, _veh_spd,
                                   extra_obstacles=_traffic_obs)

        # ----- Collision warning evaluation -----
        if warning_system is not None and args.rocks > 0 and rocks:
            chassis_w = vehicle.GetChassisBody()
            pos_w = chassis_w.GetPos()
            rot_w = chassis_w.GetRot()
            vel_loc_w = rot_w.RotateBack(chassis_w.GetPosDt())
            # Yaw from quaternion (same formula used in extract_vehicle_state).
            yaw_w = math.atan2(
                2 * (rot_w.e0 * rot_w.e3 + rot_w.e1 * rot_w.e2),
                1 - 2 * (rot_w.e2 * rot_w.e2 + rot_w.e3 * rot_w.e3),
            )
            _rpos_w = get_rock_positions(rocks)
            _rrad_w = get_rock_radii(rocks)
            obstacles_w = [(float(_rpos_w[i, 0]), float(_rpos_w[i, 1]),
                             float(_rrad_w[i]))
                            for i in range(len(_rpos_w))]
            vehicle_state_w = {
                "x": float(pos_w.x), "y": float(pos_w.y),
                "psi": yaw_w, "u": float(vel_loc_w.x),
            }
            terrain_n_for_warn = (warning_n_live
                                  if warning_n_live is not None
                                  else _initial_n)
            warn = warning_system.evaluate(
                vehicle_state_w, obstacles_w,
                terrain_n=terrain_n_for_warn,
            )
            # Console banner on every severity transition
            if warn.severity != last_warning_severity:
                _names = {0: "GREEN", 1: "YELLOW", 2: "ORANGE", 3: "RED"}
                print(f"  [WARN t={time_chrono:.2f}] "
                      f"{_names[last_warning_severity]} → {_names[warn.severity]}  "
                      f"{warn.message}",
                      flush=True)
                last_warning_severity = warn.severity
            if warning_csv_writer is not None:
                warning_csv_writer.writerow([
                    f"{time_chrono:.4f}",
                    f"{vehicle_state_w['u']:.3f}",
                    int(warn.severity),
                    f"{warn.ttc:.4f}" if math.isfinite(warn.ttc) else "inf",
                    f"{warn.clearance:.4f}" if math.isfinite(warn.clearance) else "inf",
                    f"{warn.stopping_distance:.4f}",
                    f"{warn.margin:.4f}" if math.isfinite(warn.margin) else "inf",
                    f"{warn.latency_inflation_m:.4f}",
                    f"{warn.terrain_n_used:.3f}",
                ])

        if sim_diag_writer is not None and time_chrono - last_sim_diag_time >= sim_diag_interval:
            chassis = vehicle.GetChassisBody()
            pos = chassis.GetPos()
            rot = chassis.GetRot()
            vel_loc = rot.RotateBack(chassis.GetPosDt())
            _clear = []
            if args.rocks > 0 and rocks:
                _rpos = get_rock_positions(rocks)
                _rrad = get_rock_radii(rocks)
                if len(_rpos):
                    _d = np.sqrt((_rpos[:, 0] - pos.x) ** 2 + (_rpos[:, 1] - pos.y) ** 2)
                    _clear.append(float(np.min(_d - _rrad - 1.5)))
            if traffic_mgr is not None:
                for ox, oy, orad in traffic_mgr.obstacles():
                    _clear.append(math.hypot(ox - pos.x, oy - pos.y) - orad - 1.5)
            nearest_clearance = min(_clear) if _clear else math.nan
            sim_diag_writer.writerow([
                f"{time_chrono:.6f}",
                f"{pos.x:.6f}", f"{pos.y:.6f}", f"{pos.z:.6f}",
                f"{vehicle.GetVehicle().GetSpeed():.6f}",
                f"{vel_loc.x:.6f}", f"{vel_loc.y:.6f}",
                f"{chassis.GetAngVelLocal().z:.6f}",
                f"{driver_inputs.m_steering:.6f}",
                f"{driver_inputs.m_throttle:.6f}",
                f"{driver_inputs.m_braking:.6f}",
                collision_logger.total_collisions if collision_logger is not None else 0,
                collision_logger.total_near_misses if collision_logger is not None else 0,
                f"{nearest_clearance:.6f}" if math.isfinite(nearest_clearance) else "",
                f"{control_delay_s:.6f}",
                f"{manual_delay_s:.6f}",
                f"{camera_delay_s:.6f}",
                f"{op_io[0]:.6f}", f"{op_io[1]:.6f}", f"{op_io[2]:.6f}",
            ])
            last_sim_diag_time = time_chrono

        if latency_log_writer is not None and time_chrono - last_latency_log_time >= latency_log_interval:
            latency_log_writer.writerow([
                f"{time_chrono:.6f}",
                f"{control_delay_s:.6f}",
                f"{manual_delay_s:.6f}",
                f"{camera_delay_s:.6f}",
            ])
            last_latency_log_time = time_chrono

        # --- Progress report ---
        if time_chrono - last_report_time >= 2.0:
            last_report_time = time_chrono
            elapsed = wall_time.time() - start_wall
            rt = time_chrono / elapsed if elapsed > 0 else 0
            pos = vehicle.GetChassisBody().GetPos()
            _col_str = ""
            if collision_logger is not None:
                _col_str = (f"  collisions={collision_logger.total_collisions}"
                            f"  near_misses={collision_logger.total_near_misses}")
            print(f"  t={time_chrono:.1f}s  pos=({pos.x:.1f},{pos.y:.1f})  "
                  f"RT={rt:.2f}x  cmds_recv={cmd_count}{_col_str}")
            if traffic_mgr is not None:
                _tz = [round(s['x'], 0) for s in traffic_mgr.states()]
                _zz = [round(s.get('z', 0.0), 2) for s in traffic_mgr.states()]
                print(f"    [TRAFFIC] x={_tz}  z={_zz}")
            # --- Timing breakdown (per 2s window) ---
            n = max(_t_report_steps, 1)
            accounted = (_t_terrain_sync + _t_terrain_adv + _t_veh_sync + _t_veh_adv +
                         _t_irr + _t_sensor + _t_driver + _t_safety +
                         _t_vis_sync + _t_vis_adv + _t_state_extract + _t_rt_sleep)
            unaccounted = _t_loop_total - accounted
            sensor_avg_ms = (_t_sensor / max(_sensor_calls, 1)) * 1000
            print(f"    [TIMING] steps={n}  loop_total={_t_loop_total:.3f}s  "
                  f"rt_sleep={_t_rt_sleep:.3f}s  unaccounted={unaccounted:.3f}s")
            print(f"    [TIMING] terrain_sync={_t_terrain_sync:.3f}s  "
                  f"terrain_adv={_t_terrain_adv:.3f}s  veh_sync={_t_veh_sync:.3f}s  "
                  f"veh_adv={_t_veh_adv:.3f}s")
            print(f"    [TIMING] irrlicht={_t_irr:.3f}s  sensor={_t_sensor:.3f}s "
                  f"({_sensor_calls} calls, avg={sensor_avg_ms:.1f}ms)  "
                  f"state_extract={_t_state_extract:.3f}s")
            print(f"    [TIMING] driver={_t_driver:.3f}s  safety={_t_safety:.3f}s  "
                  f"vis_sync={_t_vis_sync:.3f}s  vis_adv={_t_vis_adv:.3f}s")
            _t_irr = 0.0; _t_sensor = 0.0; _t_terrain_sync = 0.0; _t_terrain_adv = 0.0
            _t_veh_sync = 0.0; _t_veh_adv = 0.0; _t_driver = 0.0; _t_safety = 0.0
            _t_zmq = 0.0; _t_vis_sync = 0.0; _t_vis_adv = 0.0
            _t_loop_total = 0.0; _t_state_extract = 0.0; _t_rt_sleep = 0.0
            _t_report_steps = 0; _sensor_calls = 0

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    if collision_logger is not None:
        collision_logger.close()
    if sim_diag_file is not None:
        sim_diag_file.close()
    if latency_log_file is not None:
        latency_log_file.close()
    if warning_csv_file is not None:
        warning_csv_file.close()

    if state_pub is not None:
        stop_msg = SimStatus(event="stop", time=time_chrono, wall_time=wall_time.time())
        state_pub.send(stop_msg)

    elapsed = wall_time.time() - start_wall
    print(f"\n  Simulation complete: {time_chrono:.1f}s in {elapsed:.1f}s "
          f"(RT factor {time_chrono / elapsed:.2f}x)")
    if not _manual_mode:
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
    # Visualization sizing (driver POV camera + Irrlicht window). Defaults are a
    # single 1080p screen; the legacy 5760x1080 was a triple-monitor rig and is
    # far too many ray-traced pixels for real-time. Use 1200 height for 16:10.
    p.add_argument("--cam-width", type=int, default=1920,
                   help="Driver POV camera / window width (px).")
    p.add_argument("--cam-height", type=int, default=1080,
                   help="Driver POV camera / window height (px). Use 1200 for 16:10.")
    p.add_argument("--cam-fov", type=float, default=1.05,
                   help="Driver POV camera horizontal FOV (rad). ~1.05 = 60 deg "
                        "for a single screen (the ultrawide rig used 1.92).")
    p.add_argument("--cam-rate", type=float, default=30.0,
                   help="Driver POV camera render rate (Hz). Each render ray-traces "
                        "the deformable terrain; combined with --mesh-resolution "
                        "this sets the real-time budget (1080p@30Hz is real-time "
                        "at mesh 0.12, but only ~0.55x at the fine 0.08 mesh).")
    p.add_argument("--cam-fullscreen", action="store_true",
                   help="Display the driver POV fullscreen (renders at "
                        "--cam-width x --cam-height, scaled to the screen).")
    p.add_argument("--convoy", type=str, default="",
                   help="Spawn PID-driven traffic vehicles for a convoy safety "
                        "scenario (lead_brake/cut_in/stalled/swerver/convoy/platoon/"
                        "oncoming/double_cut/stop_and_go/jam/overtake/gauntlet). The "
                        "ego must avoid them; they appear as dynamic obstacles.")
    p.add_argument("--goal-distance", type=float, default=0.0,
                   help="If >0, place a visible goal gate this far ahead (m) and "
                        "end the round early once the ego reaches it.")
    p.add_argument("--replay-cmds", type=str, default="",
                   help="Counterfactual replay: re-drive the ego from a recorded "
                        "operator command trace CSV (steering_op/throttle_op/"
                        "braking_op, e.g. a prior run's sim_diag.csv) instead of a "
                        "live driver. The safety filter still screens the replayed "
                        "intent, so the same trace can be run filter-off vs each "
                        "filter for a causal harm-prevented comparison.")
    p.add_argument("--traffic-detail", choices=["auto", "mesh", "primitives"],
                   default="mesh",
                   help="Traffic vehicle render detail. 'mesh' (default, full HMMWV "
                        "mesh) is real-time for ~3 vehicles on a large terrain; "
                        "'auto' downgrades big scenes to primitive boxes; 'primitives' "
                        "forces boxes. Ignored when headless (no visual assets).")
    p.add_argument("--mesh-resolution", type=float, default=None,
                   help="SCM mesh spacing (m). Default 0.08 (paper fidelity); "
                        "0.12 is the real-time value for interactive/HIL runs.")
    p.add_argument("--vis-mode", default="irrlicht",
                   choices=["irrlicht", "sensor", "both", "none"],
                   help="Visualization mode: irrlicht, sensor (driver POV), both, or none")
    p.add_argument("--irrlicht-window-size", type=int, nargs=2,
                   metavar=("WIDTH", "HEIGHT"), default=[4320, 720],
                   help="Irrlicht window size in pixels")
    p.add_argument("--no-rt",  action="store_true",
                   help="Disable real-time pacing (fast-forward; breaks decoupled MPC)")
    p.add_argument("--no-tire-forces", action="store_true",
                   help="Disable per-wheel tire force extraction in state messages")
    p.add_argument("--speed", type=float, default=5.0, help="Target speed for markers (m/s)")

    # Terrain
    p.add_argument("--terrain", default="sand", choices=["sand", "clay", "dirt"])
    p.add_argument("--terrain-config", type=str, default=None, help="YAML terrain config")
    p.add_argument("--bumpiness", type=int, default=0, choices=range(0, 11),
                    help="Terrain bumpiness level 0 (flat) to 10 (extreme)")

    # Spatial soil transition: soil changes type partway across the patch via a
    # per-location SCM callback (vehicle drives +x through the boundary).
    p.add_argument("--terrain-transition", action="store_true",
                   help="Enable a spatial soil transition along +x "
                        "(--terrain-start blends into --terrain-end).")
    p.add_argument("--terrain-start", default=None, choices=["sand", "clay", "dirt"],
                   help="Soil preset before the transition (defaults to --terrain).")
    p.add_argument("--terrain-end", default=None, choices=["sand", "clay", "dirt"],
                   help="Soil preset after the transition.")
    p.add_argument("--transition-x", type=float, default=60.0,
                   help="Center of the soil transition, in terrain x (m).")
    p.add_argument("--transition-width", type=float, default=2.0,
                   help="Full width of the linear soil blend (m); 0 = hard step.")

    # Path (for visual markers only; the controller handles actual path generation)
    p.add_argument("--path", default="lane_change",
                   choices=["lane_change", "double_lane_change", "right_left", "sinusoidal", "straight"])
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
    p.add_argument("--sim-diag-csv", default="",
                   help="Write sim-side state/control diagnostics to this CSV.")

    # IMU sensor (Chrono sensor module)
    p.add_argument("--no-imu", action="store_true",
                   help="Disable Chrono sensor-module IMU (use analytical ground-truth accel/gyro)")
    p.add_argument("--imu-rate", type=int, default=100,
                   help="IMU update rate in Hz (default 100)")
    p.add_argument("--imu-lag", type=float, default=0.0,
                   help="IMU sensor lag in seconds (default 0)")
    p.add_argument("--imu-acc-stdev", type=float, default=0.015,
                   help="Accelerometer noise stdev in m/s² (default 0.015, ~150µg/√Hz MEMS)")
    p.add_argument("--imu-acc-bias-drift", type=float, default=1e-4,
                   help="Accelerometer bias drift rate (default 1e-4)")
    p.add_argument("--imu-acc-tau-drift", type=float, default=100.0,
                   help="Accelerometer drift time constant in s (default 100)")
    p.add_argument("--imu-gyro-stdev", type=float, default=0.001,
                   help="Gyroscope noise stdev in rad/s (default 0.001, ~0.005°/s/√Hz MEMS)")
    p.add_argument("--imu-gyro-bias-drift", type=float, default=5e-6,
                   help="Gyroscope bias drift rate (default 5e-6)")
    p.add_argument("--imu-gyro-tau-drift", type=float, default=500.0,
                   help="Gyroscope drift time constant in s (default 500)")

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
    p.add_argument("--wasd", action="store_true",
                   help="Manual control with WASD keyboard (no MPC controller)")
    p.add_argument("--manual-honor-time", action="store_true",
                   help="In manual mode, stop automatically after --time seconds.")
    p.add_argument("--manual-input-delay", type=float, default=0.0,
                   help="Fixed actuation delay applied to manual steering/throttle/brake inputs.")
    p.add_argument("--camera-input-delay", type=float, default=0.0,
                   help="Fixed lag applied to the driver POV camera feed. Models "
                        "downlink video latency to the operator. Overridden by the "
                        "camera channel of --latency-profile-json when supplied.")
    p.add_argument("--latency-profile-json", default="",
                   help="JSON profile for time-varying 5G-like one-way latency. "
                        "Overrides fixed --teleop-delay/--manual-input-delay per channel.")
    p.add_argument("--latency-profile-log", default="",
                   help="Optional CSV path for logging active control/manual/camera latency samples.")

    # Rock obstacles
    p.add_argument("--payload-mass", type=float, default=0.0,
                   help="Unmodelled cargo mass (kg) added to the chassis. The "
                        "controller keeps the nominal empty-vehicle mass, so a "
                        "non-zero value creates a persistent plant/model "
                        "mismatch for the online-learning experiments.")
    p.add_argument("--simple-powertrain", action="store_true",
                   help="Near-direct drive: linear EngineSimple + CVT (no engine "
                        "RPM map, no gear shifts) so throttle->wheel-torque is "
                        "~linear/soil-independent -- the clean actuation map the "
                        "force-balance NMPC needs.")
    p.add_argument("--rocks", type=int, default=0,
                   help="Number of rock obstacles (0 = none)")
    p.add_argument("--rock-zone-x", type=float, nargs=2, default=[-15.0, 50.0])
    p.add_argument("--rock-zone-y", type=float, nargs=2, default=[-10.0, 10.0])
    p.add_argument("--rock-size", type=float, nargs=2, default=[0.5, 3.0])
    p.add_argument("--rock-seed", type=int, default=42)
    p.add_argument("--rock-min-spacing", type=float, default=0.0,
                   help="Min center-to-center spacing (m) between rocks. >0 makes "
                        "a threadable blue-noise boulder field (no free bypass).")
    p.add_argument("--rock-centerline-clear", type=float, default=0.0,
                   help="Lateral half-width (m) around y=0 where rock density is "
                        "thinned so the convoy lead can pick a line (not a clear lane).")
    p.add_argument("--rock-spawn-clear", type=float, default=12.0,
                   help="Radius (m) of the rock-free circle around the spawn.")

    # Safety filter
    p.add_argument("--safety-filter", action="store_true",
                   help="Enable the safety filter (flavor controlled by --safety-flavor)")
    p.add_argument("--safety-flavor", type=str, default="mppi",
                   choices=["mppi", "nmpc", "dob_cbf"],
                   help="Filter flavor: mppi (primary predictive shield), "
                        "nmpc (gradient ablation), dob_cbf (legacy DOB-CBF-QP).")
    # MPPI shield parameters
    p.add_argument("--mppi-samples", type=int, default=384,
                   help="MPPI rollouts per step (K).")
    p.add_argument("--mppi-sigma-steer", type=float, default=0.35,
                   help="Stddev of steering-norm noise per MPPI sample.")
    p.add_argument("--mppi-sigma-alpha", type=float, default=0.35,
                   help="Stddev of throttle-norm noise per MPPI sample.")
    p.add_argument("--mppi-temperature", type=float, default=1.0,
                   help="MPPI temperature lambda (smaller = sharper weighting).")
    p.add_argument("--mppi-no-seeds", action="store_true",
                   help="Ablation: disable the hand-crafted MPPI seed trajectories "
                        "(passthrough/brake/evade). Rollouts then rely solely on "
                        "Gaussian sampling around the operator command.")
    p.add_argument("--shield-no-sigma-gate", action="store_true",
                   help="Ablation: zero out the controller's phi_sigma_deg before "
                        "the shield sees it (equivalent to --shield-sigma-mode off).")
    p.add_argument("--shield-sigma-mode", type=str, default="off",
                   choices=["tighten", "inflate", "both", "off"],
                   help="How estimator phi-uncertainty acts on the shield. "
                        "Default off: the shield runs on its initial terrain "
                        "(paper Sec. IX-B ablation showed every live-terrain "
                        "gate underperforms). tighten/inflate/both retained "
                        "only for the sigma_gate_ablation experiment.")
    p.add_argument("--shield-sigma-buffer-gain", type=float, default=0.05,
                   help="Metres of extra obstacle buffer per degree of phi_sigma.")
    # NMPC shield parameters
    p.add_argument("--nmpc-iter", type=int, default=6,
                   help="L-BFGS-B iteration cap for the NMPC shield ablation.")
    # Common shield params
    p.add_argument("--shield-horizon", type=int, default=12,
                   help="Prediction horizon steps (latency adds more dynamically).")
    p.add_argument("--cbf-alpha", type=float, default=5.0)
    p.add_argument("--safety-buffer", type=float, default=0.25)
    p.add_argument("--delay-steps", type=int, default=5)
    p.add_argument("--cbf-w-long", type=float, default=0.06)
    p.add_argument("--cbf-w-lat", type=float, default=0.50)
    p.add_argument("--cbf-forward-bias", type=float, default=3.0)
    p.add_argument("--dob-bandwidth", type=float, default=10.0)
    p.add_argument("--cbf-flavor", type=str, default="balance",
                   choices=["balance", "steer_priority", "throttle_priority"])
    p.add_argument("--nn-model", type=str, default="vehicle_rate_64_32_lhs",
                   help="NN model version directory for CBF traction limits")
    p.add_argument("--no-safety-nn", action="store_true",
                   help="Disable the NN tire model inside the safety filter. "
                        "DOB-CBF then uses its kinematic fallback; predictive "
                        "MPPI/NMPC shields still require NN dynamics and will fail fast.")
    p.add_argument("--teleop-delay", type=float, default=0.0,
                   help="Initial one-way teleop delay estimate in seconds "
                        "(0 = local, auto-measured from cmd timestamps)")
    p.add_argument("--stale-cmd-timeout", type=float, default=2.0,
                   help="Auto-brake if no command received for this many seconds")

    # Collision warning (modular HMI signal — runs in parallel with any
    # safety filter and does not modify commands).
    p.add_argument("--collision-warning", action="store_true",
                   help="Enable the modular forward collision warning. "
                        "Logs per-tick severity and emits a console banner "
                        "on every severity transition for operator visibility.")
    p.add_argument("--cw-tire-model", type=str,
                   default="nn_models/rig_rate_64_32",
                   help="Path to the tire NN surrogate the warning system "
                        "queries at init to build its analytical brake-decel "
                        "table. Default is the rig rate model (the same one "
                        "the controller uses for traction-budget queries).")
    p.add_argument("--cw-reaction-time", type=float, default=0.20,
                   help="Operator baseline reaction time (s) added to "
                        "RTT+jitter when computing required stopping distance.")
    p.add_argument("--collision-warning-csv", type=str, default="",
                   help="Optional CSV path for the per-tick warning log. "
                        "Defaults to <log_dir>/collision_warning_log.csv.")

    args = p.parse_args()
    run_sim_node(args)


if __name__ == "__main__":
    main()
