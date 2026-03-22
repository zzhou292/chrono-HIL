"""
Fixed reference path with CSV storage and cubic-spline trajectory generation.

Workflow
--------
1.  At startup, generate dense waypoints for the chosen path type (or load
    from a previously-saved CSV).
2.  Build an arc-length–parameterised cubic spline through the waypoints.
3.  At each MPC step:
      a. Find the closest waypoint to the vehicle (x, y).
      b. Sample N+1 forward spline points spaced by dt·u along the arc.
      c. Compute heading from the spline tangent (first derivative).
      d. Return (x_ref, y_ref, psi_ref, v_ref, x_goal, y_goal, psi_goal).
"""

import csv
import os

import numpy as np
from scipy.interpolate import CubicSpline


# ============================================================================
# Path waypoint generation
# ============================================================================

def generate_path_waypoints(path_type, lead_in=0.0, lane_offset=3.0,
                            sine_amplitude=2.0, sine_wavelength=30.0,
                            total_length=None, ds=0.25):
    """
    Generate dense (x, y) waypoints for a reference path.

    Parameters
    ----------
    path_type : str
        'lane_change', 'double_lane_change', or 'sinusoidal'.
    lead_in : float
        Straight lead-in distance (m) before path geometry starts.
    lane_offset : float
        Lateral offset for lane-change maneuvers (m).
    sine_amplitude, sine_wavelength : float
        Sinusoidal path parameters.
    total_length : float or None
        Total path length in x (m).  Auto-computed if *None*.
    ds : float
        Spacing between waypoints (m).

    Returns
    -------
    x, y : ndarray
        Waypoint coordinates.
    """
    if total_length is None:
        if path_type == 'lane_change':
            total_length = 25.0 + lead_in + 40.0
        elif path_type == 'double_lane_change':
            total_length = 38.0 + lead_in + 40.0
        elif path_type == 'sinusoidal':
            total_length = lead_in + 5 * sine_wavelength
        else:
            total_length = 200.0

    n_pts = int(total_length / ds) + 1
    x = np.linspace(0, total_length, n_pts)
    y = np.zeros(n_pts)

    if path_type == 'lane_change':
        ts, te = 10.0 + lead_in, 25.0 + lead_in
        mask = (x > ts) & (x < te)
        b = (x[mask] - ts) / (te - ts)
        b = b * b * (3 - 2 * b)  # Hermite smooth-step
        y[mask] = b * lane_offset
        y[x >= te] = lane_offset

    elif path_type == 'double_lane_change':
        z1s, z1e = 8.0 + lead_in, 18.0 + lead_in
        z2s, z2e = 28.0 + lead_in, 38.0 + lead_in

        m1 = (x >= z1s) & (x < z1e)
        b1 = (x[m1] - z1s) / (z1e - z1s)
        b1 = b1 * b1 * (3 - 2 * b1)
        y[m1] = b1 * lane_offset

        y[(x >= z1e) & (x < z2s)] = lane_offset

        m2 = (x >= z2s) & (x < z2e)
        b2 = (x[m2] - z2s) / (z2e - z2s)
        b2 = b2 * b2 * (3 - 2 * b2)
        y[m2] = lane_offset * (1 - b2)

    elif path_type == 'sinusoidal':
        mask = x >= lead_in
        y[mask] = sine_amplitude * np.sin(
            2 * np.pi * (x[mask] - lead_in) / sine_wavelength)

    return x, y


# ============================================================================
# ReferencePath — arc-length spline with closest-point lookup
# ============================================================================

class ReferencePath:
    """Arc-length–parameterised cubic-spline reference path.

    Stores dense waypoints and pre-fits ``x(s)``, ``y(s)`` cubic splines.
    At runtime :meth:`get_reference` generates a per-solve reference
    trajectory that starts at the vehicle's current position and smoothly
    rejoins the desired path via Hermite blending.
    """

    def __init__(self, x_pts, y_pts, v_target, blend_fraction=0.5):
        """
        Parameters
        ----------
        x_pts, y_pts : array-like
            Dense waypoint coordinates (same length).
        v_target : float
            Target longitudinal speed (m/s).
        blend_fraction : float
            Fraction of the MPC horizon (0–1) over which the reference
            blends from the vehicle's current state to the desired path.
            0 = entire reference is on the desired path (old behaviour).
            0.5 = blend over half the horizon (default).
        """
        x_pts = np.asarray(x_pts, dtype=float)
        y_pts = np.asarray(y_pts, dtype=float)
        assert len(x_pts) == len(y_pts), "x and y must have same length"

        self.v_target = v_target
        self.blend_fraction = blend_fraction
        self.n_pts = len(x_pts)
        self.x_pts = x_pts
        self.y_pts = y_pts

        # Cumulative arc length
        dx = np.diff(x_pts)
        dy = np.diff(y_pts)
        ds = np.sqrt(dx ** 2 + dy ** 2)
        self.s = np.concatenate([[0.0], np.cumsum(ds)])
        self.s_max = self.s[-1]

        # Cubic splines x(s) and y(s)
        self.cs_x = CubicSpline(self.s, x_pts)
        self.cs_y = CubicSpline(self.s, y_pts)

        # Tangent direction at the end of the path (for extrapolation past s_max)
        self._end_dx = float(self.cs_x(self.s_max, 1))
        self._end_dy = float(self.cs_y(self.s_max, 1))
        _end_norm = np.hypot(self._end_dx, self._end_dy)
        if _end_norm > 1e-9:
            self._end_dx /= _end_norm
            self._end_dy /= _end_norm

        # Progress tracking: constrain closest-point search to forward window
        self._last_idx: int = 0

    # ------------------------------------------------------------------
    # MPC reference (drop-in replacement for old path_func)
    # ------------------------------------------------------------------

    def get_reference(self, time, z0, N, dt):
        """Generate a per-solve reference trajectory.

        When the vehicle is close to the desired path the reference is
        sampled directly from the spline (fast, accurate headings).
        When the vehicle has drifted off-path the reference is blended
        from a straight-line extrapolation of the current state to the
        desired path over a fraction of the horizon proportional to the
        cross-track error, up to ``blend_fraction``.

        Returns
        -------
        x_ref, y_ref, psi_ref, v_ref : ndarray (N+1,)
        x_goal, y_goal, psi_goal : float
        """
        x_veh, y_veh, psi_veh = z0[0], z0[1], z0[2]
        u_proj = max(z0[3], 1.0)
        step = dt * u_proj

        # 1. Forward-constrained closest-waypoint search.
        #    Only search from _last_idx onward (with a small backward window for
        #    robustness against noise) so the progress index can never jump
        #    backward to an earlier section of a symmetric/sinusoidal path.
        search_back = 10   # waypoints we allow backward look (handles noise)
        search_fwd  = 200  # waypoints ahead to consider
        lo = max(0, self._last_idx - search_back)
        hi = min(self.n_pts, self._last_idx + search_fwd)
        sub_x = self.x_pts[lo:hi]
        sub_y = self.y_pts[lo:hi]
        dists_sq = (sub_x - x_veh) ** 2 + (sub_y - y_veh) ** 2
        local_idx = int(np.argmin(dists_sq))
        idx = lo + local_idx
        self._last_idx = idx  # advance progress marker
        s0 = self.s[idx]
        cross_track = float(np.sqrt(dists_sq[local_idx]))

        # 2. Sample N+1 forward arc-length positions.
        #    Points beyond s_max are extrapolated linearly in the terminal
        #    tangent direction instead of being clamped — this prevents all
        #    reference points from piling up on the last waypoint.
        s_raw = s0 + np.arange(N + 1) * step
        on_path = s_raw <= self.s_max
        s_clamped = np.where(on_path, s_raw, self.s_max)
        overshoot = np.where(on_path, 0.0, s_raw - self.s_max)

        x_path = self.cs_x(s_clamped) + overshoot * self._end_dx
        y_path = self.cs_y(s_clamped) + overshoot * self._end_dy

        # Heading: use spline tangent on-path, terminal tangent beyond end
        dx_ds = np.where(on_path, self.cs_x(s_clamped, 1), self._end_dx)
        dy_ds = np.where(on_path, self.cs_y(s_clamped, 1), self._end_dy)
        psi_path = np.arctan2(dy_ds, dx_ds)

        # 3. Adaptive blending: proportional to cross-track error.
        #    Small CT (<0.3m) → no blend (use spline heading directly).
        #    Large CT → blend over up to blend_fraction of the horizon.
        ct_threshold = 0.3  # metres: below this, skip blending
        if cross_track < ct_threshold or self.blend_fraction <= 0.0:
            x_ref, y_ref, psi_ref = x_path, y_path, psi_path
        else:
            # Off-path: blend from vehicle state to desired path
            eff_frac = min(self.blend_fraction, cross_track / 5.0)
            n_blend = max(2, int(N * eff_frac))
            t_b = np.linspace(0.0, 1.0, n_blend + 1)
            alpha = t_b * t_b * (3.0 - 2.0 * t_b)  # Hermite smooth-step

            # Straight-line extrapolation from vehicle state
            d_fwd = np.arange(n_blend + 1) * step
            x_line = x_veh + d_fwd * np.cos(psi_veh)
            y_line = y_veh + d_fwd * np.sin(psi_veh)

            x_ref = x_path.copy()
            y_ref = y_path.copy()
            x_ref[:n_blend + 1] = (1.0 - alpha) * x_line + alpha * x_path[:n_blend + 1]
            y_ref[:n_blend + 1] = (1.0 - alpha) * y_line + alpha * y_path[:n_blend + 1]

            # Heading: finite differences over the blended section, then
            # hand off to the spline tangent for the on-path portion.
            # np.diff of (n_blend+2) points → (n_blend+1) headings, matching
            # the (n_blend+1) slots psi_ref[0..n_blend].
            dx = np.diff(x_ref[:n_blend + 2])
            dy = np.diff(y_ref[:n_blend + 2])
            psi_blend = np.arctan2(dy, dx)  # shape (n_blend+1,)
            psi_ref = psi_path.copy()
            psi_ref[:n_blend + 1] = psi_blend

        v_ref = self.v_target * np.ones(N + 1)

        return x_ref, y_ref, psi_ref, v_ref, x_ref[-1], y_ref[-1], psi_ref[-1]

    # ------------------------------------------------------------------
    # Point evaluation (for analytics / error computation)
    # ------------------------------------------------------------------

    def evaluate_at_x(self, x_query):
        """Return (y, psi) on the reference path nearest to a given *x* position.

        Uses the same forward-constrained window as :meth:`get_reference`
        (read-only — does NOT update ``_last_idx``) so that analytics and
        the MPC reference are always consistent, including when the vehicle
        is laterally displaced.
        """
        search_back = 10
        search_fwd  = 200
        lo = max(0, self._last_idx - search_back)
        hi = min(self.n_pts, self._last_idx + search_fwd)
        sub_x = self.x_pts[lo:hi]
        # Use x-only distance: x is monotone for all path types, so this is
        # insensitive to lateral drift direction.
        local_idx = int(np.argmin(np.abs(sub_x - x_query)))
        idx = lo + local_idx
        s_q = self.s[idx]

        y = float(self.cs_y(s_q))
        dx_ds = float(self.cs_x(s_q, 1))
        dy_ds = float(self.cs_y(s_q, 1))
        psi = float(np.arctan2(dy_ds, dx_ds))
        return y, psi

    # ------------------------------------------------------------------
    # CSV persistence
    # ------------------------------------------------------------------

    def save_csv(self, filepath):
        """Write waypoints to a CSV file (s, x, y, psi)."""
        os.makedirs(os.path.dirname(filepath) or '.', exist_ok=True)
        dx_ds = self.cs_x(self.s, 1)
        dy_ds = self.cs_y(self.s, 1)
        psi = np.arctan2(dy_ds, dx_ds)

        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['s', 'x', 'y', 'psi'])
            for i in range(self.n_pts):
                writer.writerow([
                    f'{self.s[i]:.4f}',
                    f'{self.x_pts[i]:.4f}',
                    f'{self.y_pts[i]:.4f}',
                    f'{psi[i]:.6f}',
                ])
        print(f"  Path CSV: {filepath} ({self.n_pts} pts, {self.s_max:.1f}m)")

    @classmethod
    def from_csv(cls, filepath, v_target):
        """Load a previously-saved path CSV."""
        data = np.loadtxt(filepath, delimiter=',', skiprows=1)
        return cls(data[:, 1], data[:, 2], v_target)

    def __repr__(self):
        return (f"ReferencePath({self.n_pts} pts, "
                f"{self.s_max:.1f}m, v={self.v_target:.1f} m/s)")
