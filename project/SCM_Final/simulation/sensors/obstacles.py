"""
Rock Obstacle Module for PyChrono Simulations
================================================

Creates randomized rock obstacles in the simulation environment.
Ported from the C++ implementation in cpp_hil/proj_HIL_scm_teleop.cpp.

Rocks are placed as fixed rigid bodies with:
- Spherical collision shapes (efficient for SCM interaction)
- Simple visual geometry (spheres with rock-like appearance)
- Randomized position, size, and orientation within a configurable zone

Usage:
    from sensors.obstacles import add_rock_obstacles
    
    rocks = add_rock_obstacles(system, num_rocks=20,
                               zone_x=(-15, 50), zone_y=(-10, 10),
                               size_range=(0.5, 3.0))
"""

from typing import List, Tuple, Optional

import numpy as np

try:
    import pychrono as chrono
    CHRONO_AVAILABLE = True
except ImportError:
    CHRONO_AVAILABLE = False


def add_rock_obstacles(system: 'chrono.ChSystem',
                       num_rocks: int = 20,
                       zone_x: Tuple[float, float] = (-15.0, 50.0),
                       zone_y: Tuple[float, float] = (-10.0, 10.0),
                       size_range: Tuple[float, float] = (0.5, 3.0),
                       seed: int = 42,
                       bury_fraction: float = 0.3,
                       min_spacing: float = 0.0,
                       centerline_clear: float = 0.0,
                       centerline_keep_prob: float = 0.35,
                       exclusion_zones: List[Tuple[float, float, float]] = None) -> List[dict]:
    """
    Add randomized rock obstacles to the Chrono system.
    
    Creates fixed rigid bodies with sphere collision shapes distributed
    randomly within the specified zone. Rocks are partially buried in 
    the ground (controlled by bury_fraction).
    
    An exclusion zone can be specified to keep rocks away from the 
    vehicle's starting position or reference path.
    
    Args:
        system: PyChrono ChSystem instance
        num_rocks: Number of rocks to place
        zone_x: (min_x, max_x) placement zone in world X (m)
        zone_y: (min_y, max_y) placement zone in world Y (m)
        size_range: (min_diameter, max_diameter) of rocks (m)
        seed: Random seed for reproducibility
        bury_fraction: Fraction of rock below ground level (0-1).
                      0.3 means 30% buried, matching C++ reference.
        min_spacing: If >0, reject any candidate whose center is closer than
                      this (m) to an already-placed rock. Turns a random
                      scatter into a blue-noise "boulder field": always a
                      threadable gap between rocks, never an impassable clump,
                      and no free bypass (the field is uniformly dense).
        centerline_clear: If >0, rocks landing within this lateral half-width
                      (m) of y=0 are kept only with probability
                      centerline_keep_prob. This thins (not clears) the line
                      the convoy lead takes through the field, so the lead can
                      pick a route without a painted "lane" -- rocks still
                      intrude on the centerline.
        centerline_keep_prob: Keep-probability for rocks inside centerline_clear.
        exclusion_zones: List of (center_x, center_y, radius) tuples.
                        No rocks will be placed within these circles.
                        Use to protect vehicle spawn and path corridor.
    
    Returns:
        List of dicts with rock metadata:
            {'body': ChBody, 'x': float, 'y': float, 'z': float, 
             'size': float, 'yaw': float}
    """
    if not CHRONO_AVAILABLE:
        raise RuntimeError("pychrono not available")
    
    rng = np.random.RandomState(seed)
    rocks = []
    
    # Contact material for rocks. Young's modulus 1e9 Pa is "soft rock"
    # range and produces enough SMC penalty force to block a 2500 kg
    # chassis at ~7 m/s without visible penetration; the prior 1e8 Pa
    # let the chassis push through buried rocks.
    rock_material = chrono.ChContactMaterialSMC()
    rock_material.SetFriction(0.9)
    rock_material.SetYoungModulus(1e9)
    rock_material.SetRestitution(0.1)
    
    attempts = 0
    # Blue-noise rejection (min_spacing) and centerline thinning reject many
    # candidates, so allow more attempts before giving up on the requested count.
    max_attempts = num_rocks * (40 if (min_spacing > 0.0 or centerline_clear > 0.0) else 10)
    
    while len(rocks) < num_rocks and attempts < max_attempts:
        attempts += 1
        
        # Random position within zone
        x = rng.uniform(zone_x[0], zone_x[1])
        y = rng.uniform(zone_y[0], zone_y[1])
        
        # Check exclusion zones
        if exclusion_zones:
            excluded = False
            for ex, ey, er in exclusion_zones:
                if (x - ex)**2 + (y - ey)**2 < er**2:
                    excluded = True
                    break
            if excluded:
                continue

        # Soft centerline thinning: keep only some rocks near the route the
        # convoy lead takes (no painted lane -- the line is just less dense).
        if centerline_clear > 0.0 and abs(y) < centerline_clear:
            if rng.random() > centerline_keep_prob:
                continue

        # Blue-noise spacing: reject candidates too close to an existing rock so
        # the field is always threadable (a gap >= min_spacing always exists)
        # but never leaves a clear lateral bypass.
        if min_spacing > 0.0 and rocks:
            too_close = False
            for r in rocks:
                if (x - r['x'])**2 + (y - r['y'])**2 < min_spacing**2:
                    too_close = True
                    break
            if too_close:
                continue

        # Random size and orientation
        size = rng.uniform(size_range[0], size_range[1])
        yaw = rng.uniform(0, 2 * np.pi)
        
        # Vertical position: partially buried
        z = size * bury_fraction
        
        # Create rock body (fixed, no dynamics)
        rock_body = chrono.ChBody()
        rock_body.SetPos(chrono.ChVector3d(x, y, z))
        rock_body.SetRot(chrono.QuatFromAngleZ(yaw))
        rock_body.SetFixed(True)
        rock_body.SetMass(2500 * size**3)  # Approximate rock density * volume
        
        # Collision shape: ellipsoid approximation (slightly flattened vertically)
        # Matches C++ reference: (0.5*size, 0.5*size, 0.4*size)
        collision_size = chrono.ChVector3d(size * 0.5, size * 0.5, size * 0.4)
        coll_shape = chrono.ChCollisionShapeEllipsoid(rock_material, collision_size)
        rock_body.AddCollisionShape(coll_shape)
        rock_body.EnableCollision(True)
        
        # Visual shape: sphere with rock-like color
        vis_sphere = chrono.ChVisualShapeSphere(size * 0.45)
        vis_sphere.SetColor(chrono.ChColor(0.45, 0.38, 0.30))  # Brown-grey rock color
        rock_body.AddVisualShape(vis_sphere)
        
        system.Add(rock_body)
        
        rocks.append({
            'body': rock_body,
            'x': x, 'y': y, 'z': z,
            'size': size, 'yaw': yaw,
        })
    
    if len(rocks) < num_rocks:
        print(f"  [ROCKS] WARNING: Only placed {len(rocks)}/{num_rocks} rocks "
              f"(exclusion zones too restrictive)")
    
    print(f"  [ROCKS] Placed {len(rocks)} rocks in zone "
          f"x=[{zone_x[0]:.0f}, {zone_x[1]:.0f}], y=[{zone_y[0]:.0f}, {zone_y[1]:.0f}], "
          f"size=[{size_range[0]:.1f}, {size_range[1]:.1f}]m")
    
    return rocks


def get_rock_positions(rocks: List[dict]) -> np.ndarray:
    """
    Extract rock center positions as an Nx3 array.
    
    Args:
        rocks: List returned by add_rock_obstacles()
    
    Returns:
        Nx3 numpy array of [x, y, z] world positions
    """
    if not rocks:
        return np.zeros((0, 3))
    return np.array([[r['x'], r['y'], r['z']] for r in rocks])


def get_rock_radii(rocks: List[dict]) -> np.ndarray:
    """
    Extract effective collision radii as an N-length array.
    
    Args:
        rocks: List returned by add_rock_obstacles()
    
    Returns:
        N-length numpy array of effective radii (m)
    """
    if not rocks:
        return np.zeros(0)
    return np.array([r['size'] * 0.5 for r in rocks])
