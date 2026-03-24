# Sensors Module

Provides sensor integration for PyChrono vehicle simulations.

## Components

### `sensors/__init__.py` — LidarManager
Wraps PyChrono's `ChLidarSensor` with a simple interface for the HMMWV simulation.

**Features:**
- 360° lidar mounted on the vehicle chassis
- Configurable resolution (default: 900 horizontal × 32 vertical channels)
- Configurable range (default: 100m), update rate (default: 20Hz)
- Automatic point cloud retrieval via XYZI buffer
- World-frame coordinate transformation
- Optional point cloud visualization window

**Usage:**
```python
from sensors import LidarManager

lidar = LidarManager(system, vehicle, config={'visualize': True})
lidar.initialize()

# In sim loop:
lidar.update()
cloud = lidar.get_point_cloud()        # Nx4 [x,y,z,intensity] sensor frame
world_cloud = lidar.get_point_cloud_world()  # Nx4 in world frame
valid = lidar.get_valid_points()        # Filtered (no max-range returns)
```

**Requirements:** PyChrono must be built with the sensor module (`pychrono.sensor`).

### `sensors/obstacles.py` — Rock Obstacle Placement
Creates randomized rock obstacles with collision shapes.

**Features:**
- Configurable number, size range, and placement zone
- Ellipsoid collision shapes (efficient for SCM terrain interaction)
- Partially buried rocks (configurable burial fraction)
- Exclusion zones to protect vehicle spawn and path corridor
- Deterministic placement via random seed

**Usage:**
```python
from sensors.obstacles import add_rock_obstacles

rocks = add_rock_obstacles(system, num_rocks=20,
                           zone_x=(-15, 50), zone_y=(-10, 10),
                           size_range=(0.5, 3.0), seed=42)
```

## CLI Integration

These features are available in `scm_hmmwv_demo.py` via command-line flags:

```bash
# Enable lidar with visualization
python scm_hmmwv_demo.py --nn --lidar --lidar-vis

# Add 20 rocks
python scm_hmmwv_demo.py --nn --rocks 20

# Combined: manual driving with lidar, rocks, and safety filter
python scm_hmmwv_demo.py --nn --manual --lidar --rocks 15 --safety-filter
```
