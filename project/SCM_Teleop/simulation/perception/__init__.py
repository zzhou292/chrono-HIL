"""
Lidar Perception Module
========================

Processes lidar point cloud data to produce:
1. **Obstacle map**: Detected obstacles (rocks, vehicles) with position and size
2. **Terrain roughness map**: Local terrain slope/roughness estimation from ground points

The module uses a simple but effective pipeline:
- Ground plane segmentation (height-based with local statistics)
- Obstacle clustering (grid-based connected components)
- Terrain slope estimation (local plane fitting in cells)

This feeds directly into the safety filter (safety/cbf_filter.py) to enforce
obstacle avoidance and terrain-aware speed limits.

Usage:
    from perception.lidar_perception import LidarPerception
    
    perception = LidarPerception(grid_resolution=1.0)
    
    # In loop:
    result = perception.process(point_cloud_world)  # Nx4 [x,y,z,i]
    obstacles = result['obstacles']       # List of (x, y, radius)
    terrain_map = result['terrain_grid']  # 2D grid of terrain info
"""

import numpy as np
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass, field


@dataclass
class Obstacle:
    """Detected obstacle with position, size, and confidence."""
    x: float              # World X position of obstacle center (m)
    y: float              # World Y position of obstacle center (m)
    z: float              # World Z position (m)
    radius: float         # Effective collision radius (m)
    num_points: int       # Number of lidar points in this cluster
    height: float = 0.0   # Obstacle height above ground (m)
    
    @property
    def confidence(self) -> float:
        """Confidence score based on number of supporting points (0-1)."""
        # 10+ points = high confidence, fewer = lower
        return min(1.0, self.num_points / 10.0)


@dataclass
class TerrainCell:
    """Terrain information for one grid cell."""
    x_center: float        # Cell center X (m)
    y_center: float        # Cell center Y (m)
    ground_height: float   # Estimated ground height (m)
    slope: float           # Local terrain slope magnitude (0-1, fraction)
    roughness: float       # RMS height variation within cell (m)
    num_points: int        # Number of ground points in cell
    is_ditch: bool = False # True if ground is significantly below neighbors
    is_hill: bool = False  # True if ground is significantly above neighbors


@dataclass
class PerceptionResult:
    """Complete perception output for one lidar scan."""
    obstacles: List[Obstacle]                      # Detected obstacles
    terrain_cells: Dict[Tuple[int, int], TerrainCell]  # Grid indexed terrain info
    ground_points: Optional[np.ndarray] = None     # Nx3 ground point subset
    obstacle_points: Optional[np.ndarray] = None   # Mx3 obstacle point subset
    timestamp: float = 0.0                         # Simulation time of scan


class LidarPerception:
    """
    Lidar-based obstacle detection and terrain mapping.
    
    Processing pipeline:
    1. Ground segmentation: Points below (local_mean_z + height_threshold) 
       are classified as ground. Points above are obstacle candidates.
    2. Obstacle clustering: Non-ground points are binned into a 2D grid.
       Adjacent occupied cells are merged into obstacle clusters.
    3. Terrain analysis: Ground points are binned into cells. Per-cell 
       statistics (mean height, slope, roughness) characterize the terrain.
    
    All coordinates are in the world frame. The vehicle position is used
    to define the analysis region (perception_range around the vehicle).
    
    Args:
        grid_resolution: Size of each analysis grid cell (m). Smaller = 
                        more detail but slower.
        perception_range: Max distance from vehicle to analyze (m)
        ground_height_threshold: Points this far above local ground mean
                                are classified as obstacles (m)
        min_obstacle_points: Minimum points in a cluster to count as obstacle
        ditch_threshold: Height difference to classify as ditch/hill (m)
    """
    
    def __init__(self,
                 grid_resolution: float = 1.0,
                 perception_range: float = 30.0,
                 ground_height_threshold: float = 0.4,
                 min_obstacle_points: int = 5,
                 ditch_threshold: float = 0.3):
        
        self.grid_res = grid_resolution
        self.perception_range = perception_range
        self.ground_height_thresh = ground_height_threshold
        self.min_obstacle_points = min_obstacle_points
        self.ditch_threshold = ditch_threshold
        
        # Persistent state for temporal filtering
        self._prev_obstacles: List[Obstacle] = []
        self._obstacle_tracks: Dict[int, List[Obstacle]] = {}
        self._track_id_counter = 0
    
    def process(self, cloud_world: np.ndarray, 
                vehicle_x: float = 0.0, vehicle_y: float = 0.0,
                timestamp: float = 0.0) -> PerceptionResult:
        """
        Process a world-frame point cloud into obstacles and terrain info.
        
        Args:
            cloud_world: Nx4 array [x, y, z, intensity] in world frame.
                        Output of LidarManager.get_point_cloud_world().
            vehicle_x: Vehicle X position for range filtering (m)
            vehicle_y: Vehicle Y position for range filtering (m)
            timestamp: Current simulation time (s)
        
        Returns:
            PerceptionResult with obstacles and terrain grid.
        """
        if cloud_world is None or len(cloud_world) == 0:
            return PerceptionResult(obstacles=[], terrain_cells={},
                                    timestamp=timestamp)
        
        # Step 1: Range filter — only analyze points near the vehicle
        pts = cloud_world[:, :3]  # xyz only
        dx = pts[:, 0] - vehicle_x
        dy = pts[:, 1] - vehicle_y
        dist_sq = dx**2 + dy**2
        in_range = dist_sq < self.perception_range**2
        pts_filtered = pts[in_range]
        
        if len(pts_filtered) == 0:
            return PerceptionResult(obstacles=[], terrain_cells={},
                                    timestamp=timestamp)
        
        # Step 2: Ground segmentation (height-based with grid statistics)
        ground_mask, obstacle_mask = self._segment_ground(pts_filtered)
        ground_pts = pts_filtered[ground_mask]
        obstacle_pts = pts_filtered[obstacle_mask]
        
        # Step 3: Obstacle clustering from non-ground points
        obstacles = self._cluster_obstacles(obstacle_pts)
        
        # Step 4: Terrain analysis from ground points
        terrain_cells = self._analyze_terrain(ground_pts)
        
        # Step 5: Mark ditches and hills based on neighbor comparison
        self._classify_terrain_features(terrain_cells)
        
        return PerceptionResult(
            obstacles=obstacles,
            terrain_cells=terrain_cells,
            ground_points=ground_pts if len(ground_pts) > 0 else None,
            obstacle_points=obstacle_pts if len(obstacle_pts) > 0 else None,
            timestamp=timestamp,
        )
    
    def _segment_ground(self, pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Separate ground points from obstacle points using height-based 
        segmentation with local grid statistics.
        
        Method: The terrain is divided into a coarse 2D grid. Within each 
        cell, the median Z is computed as the local ground estimate. Points
        more than ground_height_threshold above this median are obstacles.
        
        This handles sloped terrain naturally since the threshold is 
        relative to the local ground, not a global plane.
        
        Returns:
            (ground_mask, obstacle_mask) boolean arrays of shape (N,)
        """
        n = len(pts)
        ground_mask = np.ones(n, dtype=bool)
        
        # Use a coarse grid (2x cell resolution) for ground estimation
        coarse_res = self.grid_res * 2.0
        
        # Compute grid indices
        gx = np.floor(pts[:, 0] / coarse_res).astype(int)
        gy = np.floor(pts[:, 1] / coarse_res).astype(int)
        
        # Group points by grid cell and compute local ground height
        cell_ground = {}
        cells = np.column_stack([gx, gy])
        unique_cells = np.unique(cells, axis=0)
        
        for cx, cy in unique_cells:
            mask = (gx == cx) & (gy == cy)
            cell_z = pts[mask, 2]
            # Use 25th percentile as ground estimate (robust to obstacles)
            ground_z = np.percentile(cell_z, 25)
            cell_ground[(cx, cy)] = ground_z
        
        # Classify each point
        for cx, cy in unique_cells:
            mask = (gx == cx) & (gy == cy)
            ground_z = cell_ground[(cx, cy)]
            above_ground = pts[mask, 2] - ground_z
            
            # Points well above local ground are obstacles
            obstacle_in_cell = above_ground > self.ground_height_thresh
            cell_indices = np.where(mask)[0]
            ground_mask[cell_indices[obstacle_in_cell]] = False
        
        obstacle_mask = ~ground_mask
        return ground_mask, obstacle_mask
    
    def _cluster_obstacles(self, obstacle_pts: np.ndarray) -> List[Obstacle]:
        """
        Cluster obstacle points into individual obstacles using grid-based
        connected component analysis.
        
        Method: Points are binned into a fine 2D grid. Adjacent occupied 
        cells are merged using flood-fill. Each connected component 
        becomes one obstacle with position at the centroid and radius 
        enclosing all points.
        
        Returns:
            List of Obstacle dataclass instances.
        """
        if len(obstacle_pts) == 0:
            return []
        
        # Bin points into fine grid
        gx = np.floor(obstacle_pts[:, 0] / self.grid_res).astype(int)
        gy = np.floor(obstacle_pts[:, 1] / self.grid_res).astype(int)
        
        # Build cell → point indices map
        cell_points: Dict[Tuple[int, int], List[int]] = {}
        for i in range(len(obstacle_pts)):
            key = (gx[i], gy[i])
            if key not in cell_points:
                cell_points[key] = []
            cell_points[key].append(i)
        
        # Connected component labeling via flood fill
        visited = set()
        clusters = []
        
        for start_cell in cell_points:
            if start_cell in visited:
                continue
            
            # BFS to find connected cells
            component_indices = []
            queue = [start_cell]
            visited.add(start_cell)
            
            while queue:
                cell = queue.pop(0)
                component_indices.extend(cell_points[cell])
                
                # Check 8-connected neighbors
                cx, cy = cell
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        if dx == 0 and dy == 0:
                            continue
                        neighbor = (cx + dx, cy + dy)
                        if neighbor in cell_points and neighbor not in visited:
                            visited.add(neighbor)
                            queue.append(neighbor)
            
            if len(component_indices) >= self.min_obstacle_points:
                cluster_pts = obstacle_pts[component_indices]
                centroid = cluster_pts.mean(axis=0)
                
                # Radius: max distance from centroid in XY plane
                dists = np.sqrt((cluster_pts[:, 0] - centroid[0])**2 + 
                               (cluster_pts[:, 1] - centroid[1])**2)
                radius = max(dists.max(), self.grid_res * 0.5)  # At least half a cell
                
                height = cluster_pts[:, 2].max() - cluster_pts[:, 2].min()
                
                clusters.append(Obstacle(
                    x=centroid[0],
                    y=centroid[1],
                    z=centroid[2],
                    radius=radius,
                    num_points=len(component_indices),
                    height=height,
                ))
        
        return clusters
    
    def _analyze_terrain(self, ground_pts: np.ndarray) -> Dict[Tuple[int, int], TerrainCell]:
        """
        Analyze terrain properties from ground points.
        
        For each grid cell, computes:
        - Ground height (median z)
        - Roughness (RMS height deviation)
        - Local slope (from height gradient between adjacent cells)
        
        Returns:
            Dict mapping (grid_x, grid_y) → TerrainCell
        """
        if len(ground_pts) == 0:
            return {}
        
        # Bin ground points
        gx = np.floor(ground_pts[:, 0] / self.grid_res).astype(int)
        gy = np.floor(ground_pts[:, 1] / self.grid_res).astype(int)
        
        # Compute per-cell statistics
        cells: Dict[Tuple[int, int], TerrainCell] = {}
        
        unique_keys = np.unique(np.column_stack([gx, gy]), axis=0)
        for cx, cy in unique_keys:
            mask = (gx == cx) & (gy == cy)
            cell_z = ground_pts[mask, 2]
            
            if len(cell_z) < 2:
                continue
            
            mean_z = np.median(cell_z)
            roughness = np.sqrt(np.mean((cell_z - mean_z)**2))
            
            cells[(cx, cy)] = TerrainCell(
                x_center=(cx + 0.5) * self.grid_res,
                y_center=(cy + 0.5) * self.grid_res,
                ground_height=mean_z,
                slope=0.0,  # Computed in next pass
                roughness=roughness,
                num_points=int(mask.sum()),
            )
        
        # Compute slope from height gradients between adjacent cells
        for (cx, cy), cell in cells.items():
            slopes = []
            for dx, dy in [(1, 0), (0, 1), (-1, 0), (0, -1)]:
                neighbor_key = (cx + dx, cy + dy)
                if neighbor_key in cells:
                    dz = abs(cells[neighbor_key].ground_height - cell.ground_height)
                    slope = dz / self.grid_res  # Rise over run
                    slopes.append(slope)
            
            if slopes:
                cell.slope = max(slopes)  # Steepest neighbor direction
        
        return cells
    
    def _classify_terrain_features(self, cells: Dict[Tuple[int, int], TerrainCell]):
        """
        Classify terrain cells as ditches or hills based on comparison 
        with neighboring cells.
        
        A cell is a ditch if it's significantly lower than all neighbors.
        A cell is a hill if it's significantly higher than all neighbors.
        """
        for (cx, cy), cell in cells.items():
            neighbor_heights = []
            for dx, dy in [(1, 0), (0, 1), (-1, 0), (0, -1)]:
                neighbor_key = (cx + dx, cy + dy)
                if neighbor_key in cells:
                    neighbor_heights.append(cells[neighbor_key].ground_height)
            
            if not neighbor_heights:
                continue
            
            mean_neighbor = np.mean(neighbor_heights)
            diff = cell.ground_height - mean_neighbor
            
            cell.is_ditch = diff < -self.ditch_threshold
            cell.is_hill = diff > self.ditch_threshold
    
    def get_obstacles_for_safety(self, result: PerceptionResult,
                                  vehicle_x: float, vehicle_y: float,
                                  max_dist: float = 20.0) -> List[Tuple[float, float, float]]:
        """
        Extract obstacle list formatted for the CBF safety filter.
        
        Returns obstacles as (x, y, radius) tuples, sorted by distance 
        to the vehicle (closest first). Only includes obstacles within
        max_dist of the vehicle.
        
        Args:
            result: PerceptionResult from process()
            vehicle_x: Current vehicle X (m)
            vehicle_y: Current vehicle Y (m)
            max_dist: Maximum distance to include (m)
        
        Returns:
            List of (x, y, effective_radius) tuples
        """
        filtered = []
        for obs in result.obstacles:
            dist = np.sqrt((obs.x - vehicle_x)**2 + (obs.y - vehicle_y)**2)
            if dist < max_dist and obs.confidence > 0.3:
                # Add safety margin to radius
                safe_radius = obs.radius + 0.5  # 0.5m safety buffer
                filtered.append((obs.x, obs.y, safe_radius, dist))
        
        # Sort by distance (closest first)
        filtered.sort(key=lambda x: x[3])
        
        return [(x, y, r) for x, y, r, _ in filtered]
    
    def get_terrain_roughness_ahead(self, result: PerceptionResult,
                                     vehicle_x: float, vehicle_y: float,
                                     vehicle_psi: float,
                                     look_ahead: float = 10.0,
                                     corridor_width: float = 4.0) -> float:
        """
        Estimate terrain roughness in a corridor ahead of the vehicle.
        
        Used by the safety filter to determine safe speed limits.
        Higher roughness → lower safe speed.
        
        Args:
            result: PerceptionResult from process()
            vehicle_x: Vehicle X position (m)
            vehicle_y: Vehicle Y position (m)
            vehicle_psi: Vehicle heading angle (rad)
            look_ahead: Distance ahead to analyze (m)
            corridor_width: Width of analysis corridor (m)
        
        Returns:
            Mean terrain roughness (m). 0 = smooth, >0.1 = rough.
        """
        if not result.terrain_cells:
            return 0.0
        
        cos_psi = np.cos(vehicle_psi)
        sin_psi = np.sin(vehicle_psi)
        half_width = corridor_width / 2.0
        
        roughness_values = []
        slope_values = []
        
        for (cx, cy), cell in result.terrain_cells.items():
            # Transform cell center to vehicle-relative frame
            dx = cell.x_center - vehicle_x
            dy = cell.y_center - vehicle_y
            
            # Rotate to vehicle heading frame
            forward = dx * cos_psi + dy * sin_psi
            lateral = -dx * sin_psi + dy * cos_psi
            
            # Check if cell is in the look-ahead corridor
            if 0 < forward < look_ahead and abs(lateral) < half_width:
                roughness_values.append(cell.roughness)
                slope_values.append(cell.slope)
        
        if not roughness_values:
            return 0.0
        
        # Combined metric: roughness + slope contribution
        mean_roughness = np.mean(roughness_values)
        max_slope = max(slope_values) if slope_values else 0.0
        
        return mean_roughness + 0.5 * max_slope
    
    def has_ditch_ahead(self, result: PerceptionResult,
                        vehicle_x: float, vehicle_y: float,
                        vehicle_psi: float,
                        look_ahead: float = 8.0,
                        corridor_width: float = 3.0) -> Tuple[bool, float]:
        """
        Check if there's a ditch or steep drop in the driving corridor.
        
        Returns:
            (has_ditch, distance_to_ditch) — distance is float('inf') if none.
        """
        if not result.terrain_cells:
            return False, float('inf')
        
        cos_psi = np.cos(vehicle_psi)
        sin_psi = np.sin(vehicle_psi)
        half_width = corridor_width / 2.0
        
        min_ditch_dist = float('inf')
        
        for (cx, cy), cell in result.terrain_cells.items():
            if not (cell.is_ditch or cell.slope > 0.35):
                continue
            
            dx = cell.x_center - vehicle_x
            dy = cell.y_center - vehicle_y
            forward = dx * cos_psi + dy * sin_psi
            lateral = -dx * sin_psi + dy * cos_psi
            
            if 0 < forward < look_ahead and abs(lateral) < half_width:
                min_ditch_dist = min(min_ditch_dist, forward)
        
        has_ditch = min_ditch_dist < float('inf')
        return has_ditch, min_ditch_dist
