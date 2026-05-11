"""
Live matplotlib debug window for MPC controller.

Opens a non-blocking figure that updates every N control steps showing:
  - Full reference path (spline)
  - MPC horizon reference points
  - MPC predicted trajectory (Z_opt)
  - Actual vehicle trail + current pose arrow
  - Speed profile (actual vs reference)
  - Steering angle and MPC cost
  - Cross-track error history

Usage:
    plotter = LiveDebugPlotter(ref_path)
    # inside control loop:
    plotter.update(z0, Z_opt, x_ref, y_ref, v_ref, msg, mpc, integrator, analytics)
"""

from __future__ import annotations

import numpy as np


class LiveDebugPlotter:
    """Non-blocking matplotlib live debug window."""

    def __init__(self, ref_path, update_every: int = 5):
        """
        Args:
            ref_path: ReferencePath object (has .x_pts, .y_pts, ._v_profile, .s).
            update_every: Redraw every N calls to update().
        """
        import matplotlib
        matplotlib.use('TkAgg')  # non-blocking backend
        import matplotlib.pyplot as plt

        self._plt = plt
        self._update_every = update_every
        self._call_count = 0

        # History buffers
        self._x_hist = []
        self._y_hist = []
        self._t_hist = []
        self._u_hist = []
        self._vref_hist = []
        self._steer_hist = []
        self._cost_hist = []
        self._cte_hist = []
        self._ax_hist = []

        # Create figure with subplots
        self._fig, axes = plt.subplots(2, 3, figsize=(18, 10))
        self._fig.suptitle('MPC Live Debug', fontsize=14, fontweight='bold')
        self._fig.canvas.manager.set_window_title('MPC Debug')

        # --- Subplot 0,0: XY trajectory (main) ---
        ax = axes[0, 0]
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('XY Trajectory')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        # Full reference path
        ax.plot(ref_path.x_pts, ref_path.y_pts, 'k--', alpha=0.4, lw=1, label='Path')
        self._ln_trail, = ax.plot([], [], 'b-', lw=1.5, alpha=0.7, label='Actual')
        self._ln_mpc_ref, = ax.plot([], [], 'g.', ms=3, alpha=0.5, label='MPC ref')
        self._ln_mpc_pred, = ax.plot([], [], 'r-', lw=1.5, alpha=0.8, label='MPC pred')
        self._arrow = ax.annotate('', xy=(0, 0), xytext=(0, 0),
                                  arrowprops=dict(arrowstyle='->', color='blue', lw=2))
        self._veh_dot, = ax.plot([], [], 'bo', ms=8, zorder=5)
        ax.legend(loc='upper left', fontsize=8)
        self._ax_xy = ax
        self._obs_patches_xy = []   # obstacle circle artists (main view)

        # --- Subplot 0,1: Zoomed XY around vehicle ---
        ax = axes[0, 1]
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Zoomed View')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.plot(ref_path.x_pts, ref_path.y_pts, 'k--', alpha=0.4, lw=1, label='Path')
        self._ln_trail_z, = ax.plot([], [], 'b-', lw=2, alpha=0.7, label='Actual')
        self._ln_mpc_ref_z, = ax.plot([], [], 'g.', ms=5, alpha=0.5, label='MPC ref pts')
        self._ln_mpc_pred_z, = ax.plot([], [], 'r-', lw=2, alpha=0.8, label='MPC predicted')
        self._veh_dot_z, = ax.plot([], [], 'bo', ms=10, zorder=5)
        ax.legend(loc='upper left', fontsize=7)
        self._arrow_z = ax.annotate('', xy=(0, 0), xytext=(0, 0),
                                    arrowprops=dict(arrowstyle='->', color='blue', lw=2.5))
        self._ax_zoom = ax
        self._obs_patches_zoom = []  # obstacle circle artists (zoomed view)

        # --- Subplot 0,2: Speed profile ---
        ax = axes[0, 2]
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Speed (m/s)')
        ax.set_title('Speed')
        ax.grid(True, alpha=0.3)
        self._ln_u, = ax.plot([], [], 'b-', lw=1.5, label='Actual u')
        self._ln_vref, = ax.plot([], [], 'r--', lw=1, alpha=0.7, label='v_ref')
        ax.legend(loc='upper right', fontsize=8)
        self._ax_speed = ax

        # --- Subplot 1,0: Steering + ax ---
        ax = axes[1, 0]
        ax.set_xlabel('Time (s)')
        ax.set_title('Steering & Accel')
        ax.grid(True, alpha=0.3)
        self._ln_steer, = ax.plot([], [], 'b-', lw=1.5, label='δ (rad)')
        self._ln_ax, = ax.plot([], [], 'r-', lw=1, alpha=0.7, label='ax (m/s²)')
        ax.legend(loc='upper right', fontsize=8)
        ax.set_ylim(-3, 3)
        self._ax_steer = ax

        # --- Subplot 1,1: Cross-track error ---
        ax = axes[1, 1]
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('CTE (m)')
        ax.set_title('Cross-Track Error')
        ax.grid(True, alpha=0.3)
        self._ln_cte, = ax.plot([], [], 'b-', lw=1.5)
        ax.axhline(0, color='k', lw=0.5, alpha=0.3)
        self._ax_cte = ax

        # --- Subplot 1,2: MPC cost ---
        ax = axes[1, 2]
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Correction')
        ax.set_title('GP Corrections')
        ax.grid(True, alpha=0.3)
        self._ln_cost, = ax.plot([], [], 'b-', lw=1.0, alpha=0.5, label='cost/1k')
        self._ln_gp_fyf, = ax.plot([], [], 'r-', lw=1.5, label='ΔFy_f (N)')
        self._ln_gp_fyr, = ax.plot([], [], 'b-', lw=1.5, label='ΔFy_r (N)')
        self._ln_dyn_du, = ax.plot([], [], 'g--', lw=1.2, label='Δu̇ (m/s²)')
        self._ln_dyn_dv, = ax.plot([], [], 'm--', lw=1.2, label='Δv̇ (m/s²)')
        self._ln_dyn_dw, = ax.plot([], [], 'c--', lw=1.2, label='Δω̇ (rad/s²)')
        ax.axhline(0, color='k', lw=0.5, alpha=0.3)
        ax.legend(loc='upper right', fontsize=6, ncol=2)
        self._ax_cost = ax

        # GP correction history buffers
        self._gp_fyf_hist = []
        self._gp_fyr_hist = []
        self._dyn_du_hist = []
        self._dyn_dv_hist = []
        self._dyn_dw_hist = []

        self._fig.tight_layout()
        plt.ion()
        plt.show(block=False)
        self._fig.canvas.draw()
        self._fig.canvas.flush_events()

    def update(self, z0, Z_opt, x_ref, y_ref, v_ref,
               sim_time: float, u_meas: float,
               steering_angle: float, ax_state: float,
               mpc_cost: float, crosstrack_err: float,
               obstacles=None,
               gp_force_corr=None, dyn_gp_corr=None):
        """Feed new data. Redraws every ``update_every`` calls.

        Args:
            obstacles: list of (x, y, r_inflated) tuples, or None.
            gp_force_corr: (2,) array [ΔFy_f, ΔFy_r] at current stage, or None.
            dyn_gp_corr: (3,) array [Δu̇, Δv̇, Δω̇] at current stage, or None.
        """
        self._call_count += 1

        # Append history
        self._x_hist.append(float(z0[0]))
        self._y_hist.append(float(z0[1]))
        self._t_hist.append(float(sim_time))
        self._u_hist.append(float(u_meas))
        self._vref_hist.append(float(v_ref[0]))
        self._steer_hist.append(float(steering_angle))
        self._ax_hist.append(float(ax_state))
        self._cost_hist.append(float(min(mpc_cost, 50000)))  # cap for display
        self._cte_hist.append(float(crosstrack_err))

        # GP corrections (stage-0 values for visualization)
        if gp_force_corr is not None:
            self._gp_fyf_hist.append(float(gp_force_corr[0]))
            self._gp_fyr_hist.append(float(gp_force_corr[1]))
        else:
            self._gp_fyf_hist.append(0.0)
            self._gp_fyr_hist.append(0.0)
        if dyn_gp_corr is not None:
            self._dyn_du_hist.append(float(dyn_gp_corr[0]))
            self._dyn_dv_hist.append(float(dyn_gp_corr[1]))
            self._dyn_dw_hist.append(float(dyn_gp_corr[2]))
        else:
            self._dyn_du_hist.append(0.0)
            self._dyn_dv_hist.append(0.0)
            self._dyn_dw_hist.append(0.0)

        if self._call_count % self._update_every != 0:
            return

        t = np.array(self._t_hist)
        xh = np.array(self._x_hist)
        yh = np.array(self._y_hist)

        vx = float(z0[0])
        vy = float(z0[1])
        psi = float(z0[2])

        # Vehicle trail
        self._ln_trail.set_data(xh, yh)
        self._ln_trail_z.set_data(xh, yh)
        self._veh_dot.set_data([vx], [vy])
        self._veh_dot_z.set_data([vx], [vy])

        # MPC reference horizon
        self._ln_mpc_ref.set_data(x_ref, y_ref)
        self._ln_mpc_ref_z.set_data(x_ref, y_ref)

        # MPC predicted trajectory
        if Z_opt is not None:
            self._ln_mpc_pred.set_data(Z_opt[0, :], Z_opt[1, :])
            self._ln_mpc_pred_z.set_data(Z_opt[0, :], Z_opt[1, :])
        else:
            self._ln_mpc_pred.set_data([], [])
            self._ln_mpc_pred_z.set_data([], [])

        # Pose arrow
        arrow_len = 2.0
        dx = arrow_len * np.cos(psi)
        dy = arrow_len * np.sin(psi)
        self._arrow.remove()
        self._arrow = self._ax_xy.annotate(
            '', xy=(vx + dx, vy + dy), xytext=(vx, vy),
            arrowprops=dict(arrowstyle='->', color='blue', lw=2))
        self._arrow_z.remove()
        self._arrow_z = self._ax_zoom.annotate(
            '', xy=(vx + dx, vy + dy), xytext=(vx, vy),
            arrowprops=dict(arrowstyle='->', color='blue', lw=2.5))

        # Obstacle circles
        self._draw_obstacles(obstacles)

        # Auto-scale XY
        self._ax_xy.relim()
        self._ax_xy.autoscale_view()

        # Zoom view: center on vehicle
        r = 20.0
        self._ax_zoom.set_xlim(vx - r, vx + r)
        self._ax_zoom.set_ylim(vy - r, vy + r)

        # Speed
        self._ln_u.set_data(t, np.array(self._u_hist))
        self._ln_vref.set_data(t, np.array(self._vref_hist))
        self._ax_speed.relim()
        self._ax_speed.autoscale_view()

        # Steering + ax
        self._ln_steer.set_data(t, np.array(self._steer_hist))
        self._ln_ax.set_data(t, np.array(self._ax_hist))
        self._ax_steer.set_xlim(t[0], t[-1] + 0.1)

        # CTE
        self._ln_cte.set_data(t, np.array(self._cte_hist))
        self._ax_cte.relim()
        self._ax_cte.autoscale_view()

        # GP Corrections (replaces old cost-only plot)
        self._ln_cost.set_data(t, np.array(self._cost_hist) / 1000.0)
        self._ln_gp_fyf.set_data(t, np.array(self._gp_fyf_hist))
        self._ln_gp_fyr.set_data(t, np.array(self._gp_fyr_hist))
        self._ln_dyn_du.set_data(t, np.array(self._dyn_du_hist))
        self._ln_dyn_dv.set_data(t, np.array(self._dyn_dv_hist))
        self._ln_dyn_dw.set_data(t, np.array(self._dyn_dw_hist))
        self._ax_cost.relim()
        self._ax_cost.autoscale_view()

        try:
            self._fig.canvas.draw_idle()
            self._fig.canvas.flush_events()
        except Exception:
            pass  # window closed

    def _draw_obstacles(self, obstacles):
        """Update obstacle circles on both XY views."""
        import matplotlib.patches as mpatches

        # Remove previous obstacle patches
        for p in self._obs_patches_xy:
            p.remove()
        self._obs_patches_xy.clear()
        for p in self._obs_patches_zoom:
            p.remove()
        self._obs_patches_zoom.clear()

        if not obstacles:
            return

        for ox, oy, r_inflated in obstacles:
            # Physical rock radius (inflated radius minus the safety margin)
            r_rock = max(r_inflated - 2.0, 0.3)

            # Rock body (solid, dark)
            c1 = mpatches.Circle((ox, oy), r_rock, fc='sienna', ec='black',
                                 lw=1.0, alpha=0.7, zorder=4)
            c1z = mpatches.Circle((ox, oy), r_rock, fc='sienna', ec='black',
                                  lw=1.0, alpha=0.7, zorder=4)
            self._ax_xy.add_patch(c1)
            self._ax_zoom.add_patch(c1z)
            self._obs_patches_xy.append(c1)
            self._obs_patches_zoom.append(c1z)

            # MPC inflated zone (dashed outline)
            c2 = mpatches.Circle((ox, oy), r_inflated, fc='none', ec='red',
                                 ls='--', lw=1.0, alpha=0.5, zorder=3)
            c2z = mpatches.Circle((ox, oy), r_inflated, fc='none', ec='red',
                                  ls='--', lw=1.2, alpha=0.6, zorder=3)
            self._ax_xy.add_patch(c2)
            self._ax_zoom.add_patch(c2z)
            self._obs_patches_xy.append(c2)
            self._obs_patches_zoom.append(c2z)

    def close(self):
        """Close the figure."""
        try:
            self._plt.close(self._fig)
        except Exception:
            pass
