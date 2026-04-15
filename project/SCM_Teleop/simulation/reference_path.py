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
        elif path_type == 'right_left':
            total_length = 48.0 + lead_in + 40.0
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

    elif path_type == 'right_left':
        # Right turn (negative offset), hold, then left turn (positive offset)
        z1s, z1e = 8.0 + lead_in, 18.0 + lead_in
        z2s, z2e = 28.0 + lead_in, 38.0 + lead_in
        z3s, z3e = 38.0 + lead_in, 48.0 + lead_in

        # Phase 1: steer right (0 -> -offset)
        m1 = (x >= z1s) & (x < z1e)
        b1 = (x[m1] - z1s) / (z1e - z1s)
        b1 = b1 * b1 * (3 - 2 * b1)
        y[m1] = -b1 * lane_offset

        # Hold right
        y[(x >= z1e) & (x < z2s)] = -lane_offset

        # Phase 2: steer left through centre to +offset
        m2 = (x >= z2s) & (x < z3e)
        b2 = (x[m2] - z2s) / (z3e - z2s)
        b2 = b2 * b2 * (3 - 2 * b2)
        y[m2] = -lane_offset + b2 * 2 * lane_offset

        # Hold left
        y[x >= z3e] = lane_offset

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

    def __init__(self, x_pts, y_pts, v_target, blend_fraction=0.5,
                 friction_angle_deg=None):
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
        friction_angle_deg : float or None
            Terrain internal friction angle in degrees.  When provided,
            ay_comfort in the speed profiler is clamped to the physical
            lateral acceleration limit μ·g·safety_factor.
        """
        self._friction_angle_deg = friction_angle_deg
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

        # Direct x→y spline for x-based reference generation (avoids arc-length
        # compression during lane changes that shortens the effective horizon).
        # Only valid when x is strictly monotonic.
        self._x_monotonic = bool(np.all(np.diff(x_pts) > 1e-9))
        self.cs_y_of_x = CubicSpline(x_pts, y_pts) if self._x_monotonic else None

        # Tangent direction at the end of the path (for extrapolation past s_max)
        self._end_dx = float(self.cs_x(self.s_max, 1))
        self._end_dy = float(self.cs_y(self.s_max, 1))
        _end_norm = np.hypot(self._end_dx, self._end_dy)
        if _end_norm > 1e-9:
            self._end_dx /= _end_norm
            self._end_dy /= _end_norm

        # Progress tracking: constrain closest-point search to forward window
        self._last_idx: int = 0

        # Pre-compute curvature-aware speed profile for the entire path.
        # This lets the vehicle brake for turns that are far beyond the
        # MPC horizon — critical at high speeds where the 2-second lookahead
        # doesn't cover the braking distance.
        self._build_speed_profile()

    def _build_speed_profile(self):
        """Pre-compute a curvature-aware max speed at every waypoint.

        Steps:
        1. Evaluate curvature κ(s) at each waypoint from the spline.
        2. Build a forward preview curvature envelope so upcoming turns
           start reducing speed before the curvature peak is reached.
        3. Compute cornering-limited speed: v = sqrt(ay_comfort / κ_preview).
        4. Run a backward pass: propagate braking constraint so the
           vehicle starts decelerating well before each turn.
        5. Run a forward pass: limit acceleration out of turns.
        6. Store as a 1-D interpolant v_max(s) for fast lookup.
        """
        # Proactive but not over-conservative defaults for SCM terrain.
        ay_comfort = 3.2   # m/s² — lateral acceleration budget in turns
        a_brake = 1.8      # m/s² — reduced decel authority -> earlier slowdown
        a_accel = 2.5      # m/s² — acceleration out of turns (was 1.5 — too slow to reach target)

        # Terrain-aware ay_comfort: on low-friction surfaces (e.g. clay φ=13°,
        # μ≈0.23) the physical lateral acceleration limit is μ·g ≈ 2.26 m/s².
        # Using a default of 3.2 leads to reference speeds that are physically
        # impossible to maintain through turns, causing overshoot and CTE blow-up.
        # On SCM deformable terrain, cohesion adds to traction beyond the
        # Coulomb friction μ=tan(φ), so we use a moderate safety factor.
        if self._friction_angle_deg is not None:
            mu = np.tan(np.radians(self._friction_angle_deg))
            ay_physical = mu * 9.81
            safety = 0.50  # SCM terrain has cohesion-based traction above μ·g
            ay_terrain = ay_physical * safety
            ay_comfort = min(ay_comfort, ay_terrain)
        preview_dist = 3.0  # m — anticipatory slowdown lookahead
        kappa_margin = 1.03  # mild inflation of previewed curvature

        s_pts = self.s  # shape (n_pts,)
        ds = np.diff(s_pts)

        # Curvature at each waypoint (arc-length parameterised)
        dx_ds = self.cs_x(s_pts, 1)
        dy_ds = self.cs_y(s_pts, 1)
        d2x_ds2 = self.cs_x(s_pts, 2)
        d2y_ds2 = self.cs_y(s_pts, 2)
        kappa = np.abs(dx_ds * d2y_ds2 - dy_ds * d2x_ds2)

        # Preview envelope: each point sees peak curvature over the next
        # preview_dist metres, so slowdown starts before entering the turn.
        n = len(s_pts)
        kappa_preview = np.empty_like(kappa)
        j = 0
        for i in range(n):
            s_hi = s_pts[i] + preview_dist
            if j < i:
                j = i
            while j + 1 < n and s_pts[j + 1] <= s_hi:
                j += 1
            kappa_preview[i] = np.max(kappa[i:j + 1])
        kappa_preview *= kappa_margin

        # Cornering speed limit
        v_max = np.full(len(s_pts), self.v_target)
        v_curv = np.sqrt(ay_comfort / np.maximum(kappa_preview, 1e-6))
        v_max = np.minimum(v_max, v_curv)

        # Backward pass: v[i] ≤ sqrt(v[i+1]² + 2·a_brake·ds)
        for i in range(len(v_max) - 2, -1, -1):
            v_max[i] = min(v_max[i],
                           np.sqrt(v_max[i + 1]**2 + 2.0 * a_brake * ds[i]))

        # Forward pass: v[i+1] ≤ sqrt(v[i]² + 2·a_accel·ds)
        for i in range(len(v_max) - 1):
            v_max[i + 1] = min(v_max[i + 1],
                               np.sqrt(v_max[i]**2 + 2.0 * a_accel * ds[i]))

        # End-of-path ramp: brake to zero over last stop_dist metres
        stop_dist = 5.0
        dist_to_end = self.s_max - s_pts
        end_scale = np.clip(dist_to_end / stop_dist, 0.0, 1.0)
        v_max *= end_scale

        self._v_profile_s = s_pts
        self._v_profile = v_max
        # Spline for smooth interpolation at arbitrary s values
        self._cs_v_profile = CubicSpline(s_pts, v_max)

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

        # 2. Forward projection along the path.
        if self._x_monotonic:
            # X-based projection: efficient for paths where x always increases.
            x_raw = x_veh + np.arange(N + 1) * step
            x_max = self.x_pts[-1]
            x_clamped = np.minimum(x_raw, x_max)
            on_path = x_raw <= x_max

            x_path = x_raw
            y_path = np.where(on_path,
                              self.cs_y_of_x(x_clamped),
                              self.cs_y_of_x(x_max))
            dy_dx = self.cs_y_of_x(x_clamped, 1)
            psi_path = np.where(on_path, np.arctan(dy_dx), 0.0)
        else:
            # Arc-length projection: works for any path (turns, loops, etc.).
            s_ref = s0 + np.arange(N + 1) * step
            s_clamped = np.minimum(s_ref, self.s_max)
            x_path = np.asarray(self.cs_x(s_clamped), dtype=float)
            y_path = np.asarray(self.cs_y(s_clamped), dtype=float)
            dx_ds = self.cs_x(s_clamped, 1)
            dy_ds = self.cs_y(s_clamped, 1)
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

        # Sample the pre-computed speed profile (curvature + brake + accel
        # + end-of-path ramp already baked in over the full path).
        if not self._x_monotonic:
            s_query = s0 + np.arange(N + 1) * step
            s_query = np.clip(s_query, 0.0, self.s_max)
        else:
            # Map x-positions back to arc-length for profile lookup.
            x_query = np.clip(x_veh + np.arange(N + 1) * step,
                              self.x_pts[0], self.x_pts[-1])
            s_query = np.interp(x_query, self.x_pts, self.s)

        v_profile = np.asarray(self._cs_v_profile(s_query), dtype=float)
        v_profile = np.clip(v_profile, 0.0, self.v_target)
        v_ref = np.minimum(v_ref, v_profile)

        return x_ref, y_ref, psi_ref, v_ref, x_ref[-1], y_ref[-1], psi_ref[-1]

    # ------------------------------------------------------------------
    # Point evaluation (for analytics / error computation)
    # ------------------------------------------------------------------

    def closest_point_on_path(self, x_q: float, y_q: float) -> dict:
        """Closest point on the arc-length spline to ``(x_q, y_q)`` in the plane.

        Searches the same ``s`` samples used to build the spline (dense spacing).
        Returns geometric distance to the path and Frenet-style signed errors
        at the projection (tangent from spline derivative).

        This is appropriate for *all* path shapes.  ``evaluate_at_x`` + ``y - y_ref``
        is only a lateral proxy when ``x`` is a good progress variable; it is
        misleading for general curves or when the vehicle barely moves.
        """
        xc = self.cs_x(self.s)
        yc = self.cs_y(self.s)
        dxw = xc - x_q
        dyw = yc - y_q
        d2 = dxw * dxw + dyw * dyw
        i = int(np.argmin(d2))
        s_star = float(self.s[i])
        xr = float(xc[i])
        yr = float(yc[i])
        pos_err = float(np.sqrt(float(d2[i])))

        tx = float(self.cs_x(s_star, 1))
        ty = float(self.cs_y(s_star, 1))
        tn = float(np.hypot(tx, ty))
        if tn < 1e-12:
            psi_r = 0.0
            tx, ty = 1.0, 0.0
        else:
            tx /= tn
            ty /= tn
            psi_r = float(np.arctan2(ty, tx))

        rx = x_q - xr
        ry = y_q - yr
        e_lon = float(rx * tx + ry * ty)
        e_lat = float(-rx * ty + ry * tx)

        return {
            "s": s_star,
            "x_ref": xr,
            "y_ref": yr,
            "psi_ref": psi_r,
            "pos_err": pos_err,
            "e_lat": e_lat,
            "e_lon": e_lon,
        }

    def evaluate_at_x(self, x_query, y_query=None):
        """Return (y, psi) on the reference path nearest to a given position.

        For x-monotonic paths, finds the closest point by x distance.
        For general paths, uses 2D distance (requires *y_query* for best
        results; falls back to x-only if not provided).
        """
        search_back = 10
        search_fwd  = 200
        lo = max(0, self._last_idx - search_back)
        hi = min(self.n_pts, self._last_idx + search_fwd)
        sub_x = self.x_pts[lo:hi]

        if self._x_monotonic or y_query is None:
            local_idx = int(np.argmin(np.abs(sub_x - x_query)))
        else:
            sub_y = self.y_pts[lo:hi]
            d2 = (sub_x - x_query) ** 2 + (sub_y - y_query) ** 2
            local_idx = int(np.argmin(d2))

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
    def from_csv(cls, filepath, v_target, friction_angle_deg=None):
        """Load a path CSV.  Accepts (x, y) or (s, x, y, psi) columns."""
        data = np.loadtxt(filepath, delimiter=',', skiprows=1)
        if data.shape[1] == 2:
            return cls(data[:, 0], data[:, 1], v_target,
                       friction_angle_deg=friction_angle_deg)
        return cls(data[:, 1], data[:, 2], v_target,
                   friction_angle_deg=friction_angle_deg)

    def __repr__(self):
        return (f"ReferencePath({self.n_pts} pts, "
                f"{self.s_max:.1f}m, v={self.v_target:.1f} m/s)")
