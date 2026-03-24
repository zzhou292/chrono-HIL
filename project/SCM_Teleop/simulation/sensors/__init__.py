"""
Lidar Sensor Module for PyChrono HMMWV
=======================================

Provides lidar sensor setup for the HMMWV vehicle in PyChrono simulations.
Ported from the C++ implementation in cpp_hil/proj_HIL_scm_teleop.cpp.

The lidar produces a 360° point cloud at configurable resolution and range.
Point cloud data is accessible via the XYZI buffer for downstream perception.

Usage:
    from sensors.lidar_sensor import LidarManager
    
    lidar_mgr = LidarManager(system, vehicle)
    lidar_mgr.initialize()
    
    # In simulation loop:
    lidar_mgr.update()
    points = lidar_mgr.get_point_cloud()  # Nx4 array [x,y,z,intensity]
"""

import numpy as np
from typing import Optional, Tuple

try:
    import pychrono as chrono
    import pychrono.vehicle as veh
    import pychrono.sensor as sens
    SENSOR_AVAILABLE = True
except ImportError:
    SENSOR_AVAILABLE = False
    print("WARNING: pychrono.sensor not available. Lidar will be disabled.")


class LidarManager:
    """
    Manages a lidar sensor attached to the HMMWV chassis.
    
    Wraps ChSensorManager and ChLidarSensor with a filter pipeline:
        ChFilterPCfromDepth -> ChFilterXYZIAccess -> (optional) ChFilterVisualizePointCloud
    
    The lidar is mounted on the vehicle chassis at a configurable offset.
    Default position is slightly forward and above the roof, matching the 
    C++ reference implementation.
    
    Attributes:
        sensor_manager: ChSensorManager instance
        lidar: ChLidarSensor instance
        latest_cloud: Most recent Nx4 point cloud [x, y, z, intensity]
    """
    
    # Default configuration (matches C++ proj_HIL_scm_teleop.cpp)
    DEFAULT_CONFIG = {
        'update_rate': 20.0,              # Hz — lidar spin rate
        'offset': (1.5, 0.0, 2.0),       # Mount position relative to chassis CoG (m)
        'horizontal_samples': 900,         # Points per revolution
        'vertical_channels': 32,           # Vertical beam count
        'horizontal_fov': 2 * np.pi,      # 360 degree horizontal field of view (rad)
        'vertical_fov_up': np.pi / 12,    # +15 degrees up (rad)
        'vertical_fov_down': -np.pi / 6,  # -30 degrees down (rad)
        'max_range': 100.0,               # Maximum detection range (m)
        'beam_shape': 'rectangular',       # Beam shape: 'rectangular' or 'round'
        'sample_radius': 2,               # Multi-sample radius for noise reduction
        'divergence_v': 0.003,            # Vertical beam divergence (rad)
        'divergence_h': 0.003,            # Horizontal beam divergence (rad)
        'return_mode': 'strongest',       # Return mode: 'strongest', 'first', 'last'
        'lag': 0.01,                      # Sensor processing lag (s)
        'collection_window': 0.04,        # Collection window duration (s)
        'visualize': False,               # Show point cloud visualization window
        'vis_width': 1280,                # Visualization window width
        'vis_height': 720,                # Visualization window height
    }
    
    def __init__(self, system: 'chrono.ChSystem', vehicle: 'veh.HMMWV_Full',
                 config: dict = None):
        """
        Initialize the LidarManager.
        
        Args:
            system: PyChrono system instance
            vehicle: HMMWV_Full vehicle instance
            config: Optional dict overriding DEFAULT_CONFIG keys.
                    Only specified keys are overridden; others use defaults.
        """
        if not SENSOR_AVAILABLE:
            raise RuntimeError("pychrono.sensor module not available. "
                             "Rebuild PyChrono with sensor support enabled.")
        
        self.system = system
        self.vehicle = vehicle
        
        # Merge user config with defaults
        self.config = dict(self.DEFAULT_CONFIG)
        if config:
            self.config.update(config)
        
        self.sensor_manager = None
        self.lidar = None
        self.latest_cloud = None  # Nx4 numpy array [x, y, z, intensity]
        self._xyzi_buffer = None
        self._initialized = False
    
    def initialize(self):
        """
        Create the sensor manager, lidar sensor, and filter pipeline.
        Must be called after the vehicle and system are fully set up,
        but before the simulation loop begins.
        """
        # Create sensor manager
        self.sensor_manager = sens.ChSensorManager(self.system)
        
        # Configure scene lighting
        self.sensor_manager.scene.AddPointLight(
            chrono.ChVector3f(0, 0, 10000),
            chrono.ChColor(1.5, 1.5, 1.5),
            100000
        )
        self.sensor_manager.scene.SetAmbientLight(chrono.ChVector3f(0.2, 0.2, 0.2))
        self.sensor_manager.scene.EnableDynamicOrigin(True)
        self.sensor_manager.scene.SetOriginOffsetThreshold(500.0)
        
        # Get chassis body for sensor attachment
        chassis_body = self.vehicle.GetChassisBody()
        
        # Build lidar sensor
        cfg = self.config
        offset = chrono.ChVector3d(cfg['offset'][0], cfg['offset'][1], cfg['offset'][2])
        lidar_frame = chrono.ChFramed(offset, chrono.ChQuaterniond(1, 0, 0, 0))
        
        # Map return mode string to enum
        return_mode_map = {
            'strongest': sens.LidarReturnMode_STRONGEST_RETURN,
            'first': sens.LidarReturnMode_FIRST_RETURN,
            'last': sens.LidarReturnMode_LAST_RETURN,
        }
        return_mode = return_mode_map.get(cfg['return_mode'], 
                                           sens.LidarReturnMode_STRONGEST_RETURN)
        
        # Map beam shape string to enum
        beam_shape_map = {
            'rectangular': sens.LidarBeamShape_RECTANGULAR,
            'round': sens.LidarBeamShape_ELLIPTICAL,
            'elliptical': sens.LidarBeamShape_ELLIPTICAL,
        }
        beam_shape = beam_shape_map.get(cfg['beam_shape'],
                                         sens.LidarBeamShape_RECTANGULAR)
        
        self.lidar = sens.ChLidarSensor(
            chassis_body,
            cfg['update_rate'],
            lidar_frame,
            cfg['horizontal_samples'],
            cfg['vertical_channels'],
            float(cfg['horizontal_fov']),
            float(cfg['vertical_fov_up']),
            float(cfg['vertical_fov_down']),
            float(cfg['max_range']),
            beam_shape,
            cfg['sample_radius'],
            float(cfg['divergence_v']),
            float(cfg['divergence_h']),
            return_mode
        )
        self.lidar.SetName("HMMWVLidar")
        self.lidar.SetLag(float(cfg['lag']))
        self.lidar.SetCollectionWindow(float(cfg['collection_window']))
        
        # Filter pipeline: depth → point cloud → XYZI buffer access
        self.lidar.PushFilter(sens.ChFilterPCfromDepth())
        self.lidar.PushFilter(sens.ChFilterXYZIAccess())
        
        # Optional visualization
        if cfg['visualize']:
            self.lidar.PushFilter(sens.ChFilterVisualizePointCloud(
                cfg['vis_width'], cfg['vis_height'],
                1.0, "Lidar Point Cloud"
            ))
        
        self.sensor_manager.AddSensor(self.lidar)
        
        # Reconstruct scenes so all bodies (including rocks) are registered
        self.sensor_manager.ReconstructScenes()
        
        self._initialized = True
        print(f"  [LIDAR] Initialized: {cfg['horizontal_samples']}h x {cfg['vertical_channels']}v, "
              f"range={cfg['max_range']}m, rate={cfg['update_rate']}Hz, "
              f"offset=({cfg['offset'][0]}, {cfg['offset'][1]}, {cfg['offset'][2]})")
    
    def update(self):
        """
        Update the sensor manager (call once per physics step).
        
        This triggers the sensor pipeline to process new data if the
        sensor's update interval has elapsed.
        """
        if not self._initialized:
            return
        self.sensor_manager.Update()
    
    def get_point_cloud(self) -> Optional[np.ndarray]:
        """
        Retrieve the latest lidar point cloud.
        
        Returns:
            Nx4 numpy array with columns [x, y, z, intensity] in the 
            sensor (vehicle) frame, or None if no data is available yet.
            Points at max_range or with zero intensity are typically
            invalid (no return).
        """
        if not self._initialized:
            return None
        
        buffer = self.lidar.GetMostRecentXYZIBuffer()
        if buffer is None or not buffer.HasData():
            return self.latest_cloud
        
        try:
            raw = buffer.GetXYZIData()
            if raw is None or raw.size == 0:
                return self.latest_cloud
            
            # GetXYZIData() returns shape (H, W, 4); flatten to Nx4
            raw = np.asarray(raw, dtype=np.float32).reshape(-1, 4)
            
            self.latest_cloud = raw
            return self.latest_cloud
            
        except Exception:
            return self.latest_cloud
    
    def get_valid_points(self, min_range: float = 0.5, 
                         max_range: float = None) -> Optional[np.ndarray]:
        """
        Get point cloud filtered to valid (non-zero) returns within range.
        
        Args:
            min_range: Minimum distance to include (m). Filters out 
                      self-reflections and near-field noise.
            max_range: Maximum distance to include (m). Defaults to
                      sensor max_range - 1m to exclude max-range returns.
        
        Returns:
            Mx4 numpy array of valid points [x, y, z, intensity], 
            or None if no data available.
        """
        cloud = self.get_point_cloud()
        if cloud is None or len(cloud) == 0:
            return None
        
        if max_range is None:
            max_range = self.config['max_range'] - 1.0
        
        # Compute range from x, y, z columns
        ranges = np.sqrt(cloud[:, 0]**2 + cloud[:, 1]**2 + cloud[:, 2]**2)
        
        # Filter: valid range and non-zero intensity
        valid = (ranges > min_range) & (ranges < max_range) & (cloud[:, 3] > 0)
        
        return cloud[valid] if np.any(valid) else None
    
    def get_point_cloud_world(self) -> Optional[np.ndarray]:
        """
        Get point cloud transformed to world coordinates.
        
        Returns:
            Nx4 numpy array [x_world, y_world, z_world, intensity],
            or None if no data available.
        """
        cloud = self.get_valid_points()
        if cloud is None:
            return None
        
        # Get chassis pose
        chassis = self.vehicle.GetChassisBody()
        pos = chassis.GetPos()
        rot = chassis.GetRot()
        
        # Apply sensor offset in body frame, then transform to world
        cfg = self.config
        sensor_offset = chrono.ChVector3d(cfg['offset'][0], cfg['offset'][1], cfg['offset'][2])
        
        # Build world points
        world_cloud = np.zeros_like(cloud)
        world_cloud[:, 3] = cloud[:, 3]  # Preserve intensity
        
        for i in range(len(cloud)):
            # Point in sensor frame
            pt_sensor = chrono.ChVector3d(float(cloud[i, 0]), float(cloud[i, 1]), float(cloud[i, 2]))
            # Sensor frame → body frame (add offset)
            pt_body = pt_sensor + sensor_offset
            # Body frame → world frame
            pt_world = pos + rot.Rotate(pt_body)
            world_cloud[i, 0] = pt_world.x
            world_cloud[i, 1] = pt_world.y
            world_cloud[i, 2] = pt_world.z
        
        return world_cloud
    
    def shutdown(self):
        """Clean up sensor resources."""
        self._initialized = False
        self.sensor_manager = None
        self.lidar = None
