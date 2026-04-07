#!/usr/bin/env python3
"""
MPC Helper Classes
==================

Shared helper classes used by both the CasADi and ACADOS MPC controller nodes.

Extracted common helpers shared across controller nodes and scripts.
a single implementation without cyclic imports.
"""

import collections
import math

import numpy as np


# =============================================================================
# Transport delay estimator
# =============================================================================

class DelayEstimator:
    """Exponential-moving-average estimator for one-way transport delay.

    Measures the wall-clock difference between the state's wall_time stamp
    and the local receive time.  This gives approximately the one-way
    sim→controller latency.  The full round-trip (state arrives, MPC solves,
    command sent back, sim applies it) is estimated as:

        τ_roundtrip ≈ τ_one_way + t_solve + τ_one_way ≈ 2·τ_one_way + t_solve

    For delay compensation in the MPC we need the time between when the state
    was measured and when our command will actually be applied:

        τ_compensation = τ_one_way + t_solve + τ_one_way
                       ≈ 2 · τ_one_way + t_solve_avg

    We track t_solve separately and expose τ_compensation.
    """

    def __init__(self, alpha: float = 0.15, initial_delay: float = 0.02):
        self.alpha = alpha
        self.one_way_delay = initial_delay  # seconds
        self.solve_time = 0.01  # seconds (initial guess)

    def update_transport(self, state_wall_time: float, recv_wall_time: float):
        """Update one-way delay estimate from a received state message."""
        measured = max(recv_wall_time - state_wall_time, 0.0)
        self.one_way_delay += self.alpha * (measured - self.one_way_delay)

    def update_solve(self, solve_seconds: float):
        self.solve_time += self.alpha * (solve_seconds - self.solve_time)

    @property
    def compensation_delay(self) -> float:
        """Total delay to compensate for in MPC (seconds)."""
        return 2.0 * self.one_way_delay + self.solve_time


# =============================================================================
# State predictor (propagates state forward through delay using dynamics)
# =============================================================================

class StatePredictor:
    """Propagates the vehicle state forward using the bicycle model dynamics
    and a buffer of recently-sent control commands.

    This implements the delay compensation: given state z(t_meas) and controls
    u(t_meas), u(t_meas+dt), ..., u(t_meas + τ_d), predict z(t_meas + τ_d).
    """

    def __init__(self, vehicle_params: dict, dt_prop: float = 0.01):
        p = vehicle_params
        self.M = p["M"]
        self.Izz = p["Izz"]
        self.Lf = p["Lf"]
        self.Lr = p["Lr"]
        self.Cf = 80000.0  # Cornering stiffness (used only for forward prediction)
        self.Cr = 80000.0
        self.dt = dt_prop

    def propagate(self, z0: np.ndarray, control_buffer, delay_s: float) -> np.ndarray:
        """Propagate state z0 forward by delay_s seconds.

        Args:
            z0: 8-state vector [x, y, psi, u, v, omega, delta, ax]
            control_buffer: deque of (sim_time, delta_dot, Jx) sorted by time.
            delay_s: Total delay to compensate (seconds).

        Returns:
            z_pred: Predicted 8-state vector at t + delay_s.
        """
        if delay_s <= 0:
            return z0.copy()

        z = z0.copy()
        n_steps = max(1, int(round(delay_s / self.dt)))
        dt = delay_s / n_steps

        buf_list = list(control_buffer)

        for step_i in range(n_steps):
            delta_dot_cmd = 0.0
            jx_cmd = 0.0
            if buf_list:
                delta_dot_cmd = buf_list[-1][1]
                jx_cmd = buf_list[-1][2]

            z = self._rk4_step(z, delta_dot_cmd, jx_cmd, dt)

        return z

    def _rk4_step(self, z, delta_dot, Jx, dt):
        k1 = self._dynamics(z, delta_dot, Jx)
        k2 = self._dynamics(z + 0.5 * dt * k1, delta_dot, Jx)
        k3 = self._dynamics(z + 0.5 * dt * k2, delta_dot, Jx)
        k4 = self._dynamics(z + dt * k3, delta_dot, Jx)
        return z + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    def _dynamics(self, z, delta_dot, Jx):
        _, _, psi, u, v, omega, delta, ax = z
        u_safe = max(abs(u), 0.5)
        Lf, Lr = self.Lf, self.Lr
        M, Izz = self.M, self.Izz

        alpha_f = delta - np.arctan2(v + Lf * omega, u_safe)
        alpha_r = -np.arctan2(v - Lr * omega, u_safe)
        Fyf = self.Cf * alpha_f
        Fyr = self.Cr * alpha_r

        dz = np.zeros(8)
        dz[0] = u * np.cos(psi) - (v + Lf * omega) * np.sin(psi)
        dz[1] = u * np.sin(psi) + (v + Lf * omega) * np.cos(psi)
        dz[2] = omega
        dz[3] = ax
        dz[4] = (Fyf + Fyr) / M - u * omega
        dz[5] = (Fyf * Lf - Fyr * Lr) / Izz
        dz[6] = delta_dot
        dz[7] = Jx
        return dz


# =============================================================================
# Quaternion helper
# =============================================================================

def quat_to_yaw(e0, e1, e2, e3):
    """Extract yaw angle from quaternion (Chrono convention: e0 is scalar)."""
    return np.arctan2(2.0 * (e0 * e3 + e1 * e2), 1.0 - 2.0 * (e2**2 + e3**2))


# =============================================================================
# Control integrator (mirrors DallasMPCDriver logic)
# =============================================================================

class ControlIntegrator:
    """Integrates MPC rate commands (delta_dot, Jx) into steering/throttle/brake.

    Works with any MPC object that exposes ``delta_max``, ``ax_min``, and
    ``ax_max`` attributes (both DallasMPC and AcadosDallasMPC satisfy this).
    """

    def __init__(self, mpc, v_target: float = 5.0):
        self.v_target = v_target
        self.steering_angle = 0.0   # δ
        self.acceleration = 0.0     # ax
        self.delta_max = mpc.delta_max
        self.ax_min = mpc.ax_min
        self.ax_max = mpc.ax_max
        self.steering_gain = 1.0 / self.delta_max
        self.throttle_gain = 1.0
        self.brake_gain = 0.6
        self.speed_err_integral = 0.0

    def update(self, delta_dot: float, Jx: float, dt: float, u: float):
        """Integrate rate commands and produce vehicle inputs.

        Returns:
            (steering, throttle, braking) — normalised Chrono inputs.
        """
        self.steering_angle += delta_dot * dt
        self.steering_angle = np.clip(self.steering_angle,
                                      -self.delta_max, self.delta_max)
        self.acceleration += Jx * dt
        self.acceleration = np.clip(self.acceleration,
                                    self.ax_min, self.ax_max)

        # Steering → normalised
        steering = np.clip(self.steering_angle * self.steering_gain, -1.0, 1.0)

        dead_band = 0.1  # m/s²
        speed_err = self.v_target - u
        if speed_err > 0:
            self.speed_err_integral += speed_err * dt
            self.speed_err_integral = min(self.speed_err_integral, 3.0)
            speed_boost = 0.15 * speed_err + 0.05 * self.speed_err_integral
        else:
            self.speed_err_integral = max(self.speed_err_integral - 0.5 * dt, 0.0)
            speed_boost = 0.0

        if self.acceleration > dead_band:
            base = self.acceleration / self.ax_max * self.throttle_gain
            throttle = min(base + speed_boost, 1.0)
            braking = 0.0
        elif self.acceleration < -dead_band:
            throttle = 0.0
            braking = min(-self.acceleration / abs(self.ax_min) * self.brake_gain, 1.0)
        else:
            throttle = min(speed_boost, 1.0)
            braking = 0.0

        return steering, throttle, braking


# =============================================================================
# Tracking analytics
# =============================================================================

class TrackingAnalytics:
    """Accumulates path-tracking metrics over the simulation.

    Tracks:
        - Cross-track error (lateral deviation from reference path)
        - Heading error (yaw deviation from reference heading)
        - Speed error (actual vs target)
        - Position (x, y) for post-run analysis
    """

    def __init__(self, ref_path, v_target: float,
                 rms_time_start: float = 0.0,
                 path_type: str = ''):
        self.ref_path = ref_path
        self.v_target = v_target
        self.rms_time_start = rms_time_start
        self.path_type = path_type

        self.times: list[float] = []
        self.crosstrack_errors: list[float] = []
        self.heading_errors: list[float] = []
        self.speed_errors: list[float] = []
        self.xs: list[float] = []
        self.ys: list[float] = []
        self.us: list[float] = []

        self.steerings: list[float] = []
        self.throttles: list[float] = []
        self.brakings: list[float] = []
        self.deltas: list[float] = []
        self.accelerations: list[float] = []
        self.solve_times_ms: list[float] = []
        self.tau_comp_ms: list[float] = []
        self.ctrl_times: list[float] = []

        self.fy_times: list[float] = []
        self.actual_Fy_front: list[float] = []
        self.actual_Fy_rear: list[float] = []
        self.pred_Fy_front: list[float] = []
        self.pred_Fy_rear: list[float] = []
        self.actual_Fx_front: list[float] = []
        self.actual_Fx_rear: list[float] = []
        self.pred_Fx_front: list[float] = []
        self.pred_Fx_rear: list[float] = []

        self.y_refs: list[float] = []
        self.psi_refs: list[float] = []

        self._window: list[float] = []

    def record(self, t: float, x: float, y: float, psi: float, u: float):
        y_ref, psi_ref = self.ref_path.evaluate_at_x(x)

        ct_err = y - y_ref
        hd_err = psi - psi_ref
        hd_err = (hd_err + np.pi) % (2 * np.pi) - np.pi
        sp_err = u - self.v_target

        self.times.append(t)
        self.crosstrack_errors.append(ct_err)
        self.heading_errors.append(hd_err)
        self.speed_errors.append(sp_err)
        self.xs.append(x)
        self.ys.append(y)
        self.us.append(u)
        self.y_refs.append(y_ref)
        self.psi_refs.append(psi_ref)
        self._window.append(abs(ct_err))

    def record_control(self, t: float, steering: float, throttle: float,
                       braking: float, delta: float, acceleration: float,
                       solve_ms: float, tau_comp_ms: float):
        self.ctrl_times.append(t)
        self.steerings.append(steering)
        self.throttles.append(throttle)
        self.brakings.append(braking)
        self.deltas.append(delta)
        self.accelerations.append(acceleration)
        self.solve_times_ms.append(solve_ms)
        self.tau_comp_ms.append(tau_comp_ms)

    def record_tire_forces(self, t: float,
                           actual_Fy_f: float, actual_Fy_r: float,
                           pred_Fy_f: float, pred_Fy_r: float,
                           actual_Fx_f: float = 0.0, actual_Fx_r: float = 0.0,
                           pred_Fx_f: float = 0.0, pred_Fx_r: float = 0.0):
        self.fy_times.append(t)
        self.actual_Fy_front.append(actual_Fy_f)
        self.actual_Fy_rear.append(actual_Fy_r)
        self.pred_Fy_front.append(pred_Fy_f)
        self.pred_Fy_rear.append(pred_Fy_r)
        self.actual_Fx_front.append(actual_Fx_f)
        self.actual_Fx_rear.append(actual_Fx_r)
        self.pred_Fx_front.append(pred_Fx_f)
        self.pred_Fx_rear.append(pred_Fx_r)

    def periodic_summary(self, last_n: int = 20) -> str:
        if not self.crosstrack_errors:
            return ""
        recent_ct = self.crosstrack_errors[-last_n:]
        recent_hd = self.heading_errors[-last_n:]
        recent_sp = self.speed_errors[-last_n:]
        rms_ct = np.sqrt(np.mean(np.square(recent_ct)))
        rms_hd = np.degrees(np.sqrt(np.mean(np.square(recent_hd))))
        mean_sp = np.mean(recent_sp)
        return (f"ct={rms_ct:.3f}m  hd={rms_hd:.1f}°  "
                f"Δv={mean_sp:+.2f}m/s")

    def final_summary(self) -> str:
        if not self.crosstrack_errors:
            return "  No tracking data collected."

        ct = np.array(self.crosstrack_errors)
        hd = np.array(self.heading_errors)
        sp = np.array(self.speed_errors)
        ts = np.array(self.times)

        mask = ts >= self.rms_time_start
        ct_m = ct[mask] if mask.any() else ct
        hd_m = hd[mask] if mask.any() else hd
        sp_m = sp[mask] if mask.any() else sp

        mean_ct  = np.mean(np.abs(ct_m))
        rms_ct   = np.sqrt(np.mean(ct_m ** 2))
        max_ct   = np.max(np.abs(ct_m))
        rms_hd   = np.degrees(np.sqrt(np.mean(hd_m ** 2)))
        mean_sp  = np.mean(sp_m)

        lines = [
            f"\n  Tracking Summary (t≥{self.rms_time_start:.1f}s):",
            f"    Avg |CTE|:  {mean_ct:.4f} m",
            f"    RMS CTE:    {rms_ct:.4f} m",
            f"    Max |CTE|:  {max_ct:.4f} m",
            f"    RMS heading:{rms_hd:.2f}°",
            f"    Mean Δspeed:{mean_sp:+.3f} m/s",
        ]
        return "\n".join(lines)

    def plot_results(self, plot_dir: str, terrain_name: str = '', model_label: str = ''):
        import os
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        os.makedirs(plot_dir, exist_ok=True)

        times = np.array(self.times)
        crosstrack = np.array(self.crosstrack_errors)
        heading = np.degrees(np.array(self.heading_errors))
        speed = np.array(self.us)
        speed_err = np.array(self.speed_errors)
        xs = np.array(self.xs)
        ys = np.array(self.ys)

        plt.figure()
        plt.plot(times, crosstrack, label='Cross-track error (m)')
        plt.xlabel('Time (s)')
        plt.ylabel('Cross-track error (m)')
        plt.title(f'Cross-track Error\n{terrain_name} {model_label}')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(plot_dir, 'crosstrack_error.png'))
        plt.close()

        plt.figure()
        plt.plot(times, heading, label='Heading error (deg)')
        plt.xlabel('Time (s)')
        plt.ylabel('Heading error (deg)')
        plt.title(f'Heading Error\n{terrain_name} {model_label}')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(plot_dir, 'heading_error.png'))
        plt.close()

        plt.figure()
        plt.plot(times, speed_err, label='Speed error (m/s)')
        plt.xlabel('Time (s)')
        plt.ylabel('Speed error (m/s)')
        plt.title(f'Speed Error\n{terrain_name} {model_label}')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(plot_dir, 'speed_error.png'))
        plt.close()

        plt.figure()
        plt.plot(xs, ys, label='Actual trajectory')
        if hasattr(self.ref_path, 'x_pts') and hasattr(self.ref_path, 'y_pts'):
            plt.plot(self.ref_path.x_pts, self.ref_path.y_pts, '--', label='Reference path')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title(f'XY Trajectory\n{terrain_name} {model_label}')
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(os.path.join(plot_dir, 'xy_trajectory.png'))
        plt.close()

        # --- Tire force comparison plots (Fy and Fx, front & rear) ---
        if self.fy_times and self.pred_Fy_front:
            ft = np.array(self.fy_times)

            fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
            axes[0].plot(ft, self.actual_Fy_front, alpha=0.6, label='Chrono (actual)')
            axes[0].plot(ft, self.pred_Fy_front, alpha=0.8, label='Model (predicted)')
            axes[0].set_ylabel('Fy (N)')
            axes[0].set_title(f'Front Axle Lateral Force (Fy)\n{terrain_name} {model_label}')
            axes[0].legend()
            axes[0].grid(True)

            axes[1].plot(ft, self.actual_Fy_rear, alpha=0.6, label='Chrono (actual)')
            axes[1].plot(ft, self.pred_Fy_rear, alpha=0.8, label='Model (predicted)')
            axes[1].set_ylabel('Fy (N)')
            axes[1].set_xlabel('Time (s)')
            axes[1].set_title('Rear Axle Lateral Force (Fy)')
            axes[1].legend()
            axes[1].grid(True)

            fig.tight_layout()
            fig.savefig(os.path.join(plot_dir, 'tire_forces_Fy.png'), dpi=150)
            plt.close(fig)

            if any(v != 0 for v in self.pred_Fx_front):
                fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
                axes[0].plot(ft, self.actual_Fx_front, alpha=0.6, label='Chrono (actual)')
                axes[0].plot(ft, self.pred_Fx_front, alpha=0.8, label='Model (predicted)')
                axes[0].set_ylabel('Fx (N)')
                axes[0].set_title(f'Front Axle Longitudinal Force (Fx)\n{terrain_name} {model_label}')
                axes[0].legend()
                axes[0].grid(True)

                axes[1].plot(ft, self.actual_Fx_rear, alpha=0.6, label='Chrono (actual)')
                axes[1].plot(ft, self.pred_Fx_rear, alpha=0.8, label='Model (predicted)')
                axes[1].set_ylabel('Fx (N)')
                axes[1].set_xlabel('Time (s)')
                axes[1].set_title('Rear Axle Longitudinal Force (Fx)')
                axes[1].legend()
                axes[1].grid(True)

                fig.tight_layout()
                fig.savefig(os.path.join(plot_dir, 'tire_forces_Fx.png'), dpi=150)
                plt.close(fig)

        print(f"Plots saved to {plot_dir}")


# =============================================================================
# Tire history tracker for temporal NN
# =============================================================================

class TireHistoryTracker:
    """Track recent per-tire operating conditions for the temporal NN.

    Maintains a sliding window of the last (K-1) observations of
    [kappa, alpha, u, Fz, steering_rate] for front and rear tires.
    The history is flattened most-recent-first for the NLP parameter vector.
    """

    def __init__(self, K):
        self.K = K
        self.n_keep = K - 1
        self._front = collections.deque(maxlen=self.n_keep)
        self._rear = collections.deque(maxlen=self.n_keep)
        for _ in range(self.n_keep):
            self._front.append(np.zeros(5))
            self._rear.append(np.zeros(5))

    def update(self, kappa_f, alpha_f, u, Fz_f, sr_f,
               kappa_r, alpha_r, Fz_r, sr_r):
        self._front.appendleft(np.array([kappa_f, alpha_f, u, Fz_f, sr_f]))
        self._rear.appendleft(np.array([kappa_r, alpha_r, u, Fz_r, sr_r]))

    @property
    def front(self):
        return np.concatenate(list(self._front))

    @property
    def rear(self):
        return np.concatenate(list(self._rear))


class RateTracker:
    """Track per-axle operating condition rates (finite differences).

    Computes [dkappa/dt, dalpha/dt, du/dt] for front and rear axles
    from consecutive MPC iterations.
    """

    def __init__(self, dt):
        self.dt = dt
        self._prev_front = None
        self._prev_rear = None
        self._rates_front = np.zeros(3)
        self._rates_rear = np.zeros(3)

    def update(self, kappa_f, alpha_f, u_f,
               kappa_r, alpha_r, u_r):
        cur_f = np.array([kappa_f, alpha_f, u_f])
        cur_r = np.array([kappa_r, alpha_r, u_r])
        if self._prev_front is not None:
            self._rates_front = (cur_f - self._prev_front) / self.dt
            self._rates_rear = (cur_r - self._prev_rear) / self.dt
        self._prev_front = cur_f
        self._prev_rear = cur_r

    @property
    def front(self):
        return self._rates_front.copy()

    @property
    def rear(self):
        return self._rates_rear.copy()
