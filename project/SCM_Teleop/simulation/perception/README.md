# Perception Module

Processes lidar point cloud data to produce obstacle and terrain maps for 
the safety filter.

## Pipeline

1. **Range Filter**: Only analyze points within `perception_range` of the vehicle
2. **Ground Segmentation**: Height-based with local grid statistics (25th percentile)
3. **Obstacle Clustering**: Grid-based connected component analysis (BFS flood-fill)
4. **Terrain Analysis**: Per-cell ground height, slope, and roughness statistics
5. **Feature Classification**: Identifies ditches (depressions) and hills

## Key Classes

### `LidarPerception`
Main perception processor. Call `process()` with a world-frame point cloud.

### `Obstacle`
Detected obstacle with position, radius, height, and confidence score.

### `TerrainCell`
Terrain info for one grid cell: ground height, slope, roughness, ditch/hill flags.

### `PerceptionResult`
Complete output: obstacle list + terrain grid + separated ground/obstacle points.

## Usage

```python
from perception import LidarPerception

perception = LidarPerception(grid_resolution=1.0, perception_range=30.0)

# Process lidar data
result = perception.process(cloud_world, vehicle_x=x, vehicle_y=y)

# Get obstacles formatted for safety filter
obstacles = perception.get_obstacles_for_safety(result, x, y)

# Get terrain roughness ahead for speed limiting
roughness = perception.get_terrain_roughness_ahead(result, x, y, psi)

# Check for ditches
has_ditch, dist = perception.has_ditch_ahead(result, x, y, psi)
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `grid_resolution` | 1.0 m | Analysis grid cell size |
| `perception_range` | 30.0 m | Max distance from vehicle to analyze |
| `ground_height_threshold` | 0.4 m | Height above local ground to classify as obstacle |
| `min_obstacle_points` | 5 | Minimum points to form an obstacle cluster |
| `ditch_threshold` | 0.3 m | Height difference to classify ditch/hill |
