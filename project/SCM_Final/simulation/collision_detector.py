#!/usr/bin/env python3
"""
Collision Detector for SCM_Teleop Simulations
===============================================

Provides lightweight, proximity-based collision detection between the HMMWV
and rock obstacles.  Uses Euclidean distance in the ground plane rather than
Chrono's contact system, which is sufficient for large rocks and the HMMWV's
footprint.

Two thresholds are reported:
  - **Hard collision** (``is_collision=1``): vehicle CG within
    ``rock_r + VEHICLE_COLLISION_RADIUS`` of the rock centre — physical overlap.
  - **Near miss** (``is_near_miss=1``): within ``rock_r + VEHICLE_COLLISION_RADIUS
    + NEAR_MISS_MARGIN`` — useful for evaluating how close the safety filter
    came to a collision.

CSV output format (one row per step, only when obstacles are present):
    time, veh_x, veh_y, rock_id, rock_x, rock_y, rock_r,
    dist_2d, is_collision, is_near_miss, v_veh

Usage::

    from collision_detector import CollisionLogger

    logger = CollisionLogger(rocks, run_dir="logs/")
    # In simulation step loop:
    report = logger.check(sim_time, veh_x, veh_y, veh_speed)
    if report['any_collision']:
        print(f"COLLISION at t={sim_time:.2f}s!")
"""

import csv
import os
from typing import List, Optional, Tuple

import numpy as np

# -------------------------------------------------------------------------
# Physical parameters
# -------------------------------------------------------------------------

# Effective collision radius of the HMMWV chassis in the ground plane.
# The vehicle is ~4.7 m long × 2.2 m wide; we use half the width + margin.
VEHICLE_COLLISION_RADIUS = 1.5  # metres

# Extra margin beyond the hard collision zone that counts as a near-miss.
NEAR_MISS_MARGIN = 1.0  # metres


class CollisionLogger:
    """Per-step collision checker and CSV logger.

    Args:
        rocks: List of dicts returned by ``sensors.obstacles.add_rock_obstacles``.
               Each dict must contain ``'x'``, ``'y'``, and ``'size'`` keys.
        run_dir: Directory for the CSV log file.  Created if it doesn't exist.
        vehicle_radius: Effective collision radius of the vehicle (m).
        near_miss_margin: Extra margin for near-miss detection (m).
        log_all: If True, write a CSV row every step (not just on events).
                 Default False keeps the log small; set True for full traces.
    """

    def __init__(self,
                 rocks: list,
                 run_dir: str = "logs/",
                 vehicle_radius: float = VEHICLE_COLLISION_RADIUS,
                 near_miss_margin: float = NEAR_MISS_MARGIN,
                 log_all: bool = False):
        self.vehicle_radius = vehicle_radius
        self.near_miss_margin = near_miss_margin
        self.log_all = log_all

        # Extract rock positions and effective radii (half of 'size')
        if rocks:
            self._rock_x = np.array([r['x'] for r in rocks], dtype=float)
            self._rock_y = np.array([r['y'] for r in rocks], dtype=float)
            self._rock_r = np.array([r['size'] * 0.5 for r in rocks], dtype=float)
        else:
            self._rock_x = np.zeros(0)
            self._rock_y = np.zeros(0)
            self._rock_r = np.zeros(0)

        self.n_rocks = len(self._rock_r)

        # Counters
        self.total_collisions = 0
        self.total_near_misses = 0
        self.first_collision_time: Optional[float] = None
        self._steps = 0

        # CSV setup
        os.makedirs(run_dir, exist_ok=True)
        csv_path = os.path.join(run_dir, 'collision_log.csv')
        self._csv_file = open(csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'time', 'veh_x', 'veh_y', 'v_veh',
            'rock_id', 'rock_x', 'rock_y', 'rock_r',
            'dist_2d', 'hard_margin', 'near_margin',
            'is_collision', 'is_near_miss',
        ])
        print(f"  [COLLISION] Logger active: {self.n_rocks} rocks, "
              f"vehicle_r={vehicle_radius:.1f}m, near_miss_margin={near_miss_margin:.1f}m")
        print(f"  [COLLISION] Log: {csv_path}")

    def check(self, sim_time: float, veh_x: float, veh_y: float,
              veh_speed: float = 0.0) -> dict:
        """Check proximity to all rocks and update logs.

        Args:
            sim_time: Current simulation time (s).
            veh_x, veh_y: Vehicle chassis CG position (m).
            veh_speed: Vehicle longitudinal speed for context (m/s).

        Returns:
            Dict with summary:
                ``any_collision``: bool — at least one hard collision this step.
                ``any_near_miss``: bool — at least one near miss this step.
                ``min_dist``: float — closest rock distance (m), inf if no rocks.
                ``n_collisions``: int — hard collision count this step.
                ``n_near_misses``: int — near-miss count this step.
                ``closest_rock_id``: int or None.
        """
        self._steps += 1
        result = {
            'any_collision': False,
            'any_near_miss': False,
            'min_dist': float('inf'),
            'n_collisions': 0,
            'n_near_misses': 0,
            'closest_rock_id': None,
        }

        if self.n_rocks == 0:
            return result

        dx = self._rock_x - veh_x
        dy = self._rock_y - veh_y
        dists = np.sqrt(dx ** 2 + dy ** 2)

        result['min_dist'] = float(dists.min())
        result['closest_rock_id'] = int(dists.argmin())

        for i in range(self.n_rocks):
            d = float(dists[i])
            r = float(self._rock_r[i])
            hard_margin = r + self.vehicle_radius
            near_margin = hard_margin + self.near_miss_margin

            is_collision = d < hard_margin
            is_near_miss = d < near_margin and not is_collision

            if is_collision:
                result['any_collision'] = True
                result['n_collisions'] += 1
                self.total_collisions += 1
                if self.first_collision_time is None:
                    self.first_collision_time = sim_time
                    print(f"\n  !!! COLLISION DETECTED !!!")
                    print(f"  !!! t={sim_time:.3f}s  rock_id={i}  "
                          f"rock=({self._rock_x[i]:.1f},{self._rock_y[i]:.1f})  "
                          f"dist={d:.2f}m < margin={hard_margin:.2f}m  "
                          f"v={veh_speed:.1f} m/s !!!")
                elif self.total_collisions % 50 == 0:
                    print(f"  [COLLISION] t={sim_time:.1f}s  rock={i}  "
                          f"d={d:.2f}m  total_events={self.total_collisions}")

            if is_near_miss:
                result['any_near_miss'] = True
                result['n_near_misses'] += 1
                self.total_near_misses += 1

            if is_collision or is_near_miss or self.log_all:
                self._csv_writer.writerow([
                    f'{sim_time:.4f}',
                    f'{veh_x:.4f}', f'{veh_y:.4f}', f'{veh_speed:.3f}',
                    i,
                    f'{self._rock_x[i]:.3f}', f'{self._rock_y[i]:.3f}',
                    f'{r:.3f}', f'{d:.4f}',
                    f'{hard_margin:.3f}', f'{near_margin:.3f}',
                    int(is_collision), int(is_near_miss),
                ])

        # Flush periodically (every ~5 s at 200 Hz = 1000 steps)
        if self._steps % 1000 == 0:
            self._csv_file.flush()

        return result

    def close(self) -> dict:
        """Flush CSV and return summary statistics.

        Returns dict with:
            ``total_collisions``: total hard-collision events logged.
            ``total_near_misses``: total near-miss events logged.
            ``first_collision_time``: time of first collision or None.
            ``collision_free``: True if no hard collisions occurred.
        """
        if self._csv_file and not self._csv_file.closed:
            self._csv_file.flush()
            self._csv_file.close()

        summary = {
            'total_collisions': self.total_collisions,
            'total_near_misses': self.total_near_misses,
            'first_collision_time': self.first_collision_time,
            'collision_free': self.total_collisions == 0,
        }

        status = "COLLISION-FREE" if summary['collision_free'] else "*** COLLISIONS DETECTED ***"
        print(f"\n  [COLLISION] Final summary: {status}")
        print(f"  [COLLISION] Hard collisions: {self.total_collisions}  "
              f"Near misses: {self.total_near_misses}")
        if self.first_collision_time is not None:
            print(f"  [COLLISION] First collision at t={self.first_collision_time:.3f}s")

        return summary
