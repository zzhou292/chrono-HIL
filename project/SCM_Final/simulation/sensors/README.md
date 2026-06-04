# Sensors Module

Provides obstacle placement utilities for PyChrono vehicle simulations.

## Components

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

These features are available in `launch_decoupled.py` via command-line flags:

```bash
# Add 20 rocks
python launch_decoupled.py --model nn --rocks 20

# Manual driving with rocks and safety filter (ground truth obstacles)
python launch_decoupled.py --manual --rocks 15 --safety-filter
```
