#!/usr/bin/env python3
"""
MPC Controller Node (Decoupled)
================================

Runs the Dallas MPC controller + trajectory generator as a standalone process,
communicating with the Chrono simulation via ZMQ.

Subscribes: VehicleState from simulation node
Publishes:  ControlCommand to simulation node

Transport delay compensation:
  The controller measures the round-trip delay τ_d between sending a command
  and receiving the next state update that reflects it. Before solving the MPC,
  it propagates the received state forward by τ_d steps using the dynamics model
  and the recently-sent control commands (which are "in the pipeline"). This way
  the MPC optimizes from the *predicted* state at the time the new command will
  actually be applied.

Usage:
    python mpc_controller_node.py --model nn --terrain sand --path sinusoidal
"""

import argparse
import collections
import csv
import math
import sys
import time as wall_time
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from hil_messages import (
    VehicleState, ControlCommand, SimStatus,
    ZMQPublisher, ZMQSubscriber,
    ctrl_pub_endpoint, sim_sub_endpoint,
)
from param_consistency import (
    get_vehicle_params_for_demo, get_terrain_preset,
    terrain_preset_to_internal, HMMWV_VEHICLE_PARAMS,
)
from dallas_mpc import DallasMPC, NNCasADi
from dallas_chrono_demo import make_path_function


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
        """
        Args:
            vehicle_params: Dict with M, Izz, Lf, Lr, etc.
            dt_prop: Integration step for prediction (s). Finer = more accurate.
        """
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
                           These are the controls that were sent but not yet applied.
            delay_s: Total delay to compensate (seconds).

        Returns:
            z_pred: Predicted 8-state vector at t + delay_s.
        """
        if delay_s <= 0:
            return z0.copy()

        z = z0.copy()
        n_steps = max(1, int(round(delay_s / self.dt)))
        dt = delay_s / n_steps

        # Build a simple time→control lookup from the buffer
        buf_list = list(control_buffer)  # [(time, delta_dot, Jx), ...]

        for step_i in range(n_steps):
            # Pick the latest control that was valid at this propagation time
            delta_dot_cmd = 0.0
            jx_cmd = 0.0
            if buf_list:
                delta_dot_cmd = buf_list[-1][1]
                jx_cmd = buf_list[-1][2]

            z = self._rk4_step(z, delta_dot_cmd, jx_cmd, dt)

        return z

    def _rk4_step(self, z, delta_dot, Jx, dt):
        """Single RK4 step of the bicycle dynamics."""
        k1 = self._dynamics(z, delta_dot, Jx)
        k2 = self._dynamics(z + 0.5 * dt * k1, delta_dot, Jx)
        k3 = self._dynamics(z + 0.5 * dt * k2, delta_dot, Jx)
        k4 = self._dynamics(z + dt * k3, delta_dot, Jx)
        return z + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

    def _dynamics(self, z, delta_dot, Jx):
        """Simplified bicycle model dynamics (linear cornering stiffness)."""
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
    """Integrates MPC rate commands (delta_dot, Jx) into steering/throttle/brake
    at the MPC update rate, matching the dallas_chrono_demo conversion logic."""

    def __init__(self, mpc: DallasMPC, v_target: float = 5.0):
        self.mpc = mpc
        self.v_target = v_target
        self.steering_angle = 0.0   # δ
        self.acceleration = 0.0     # ax
        self.steering_gain = 1.0 / mpc.delta_max
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
                                      -self.mpc.delta_max, self.mpc.delta_max)
        self.acceleration += Jx * dt
        self.acceleration = np.clip(self.acceleration,
                                    self.mpc.ax_min, self.mpc.ax_max)

        # Steering → normalised
        steering = np.clip(self.steering_angle * self.steering_gain, -1.0, 1.0)

        # Throttle / brake with dead-band to prevent oscillation near ax ≈ 0.
        # When ax is in [-dead_band, +dead_band], coast with only PI speed boost.
        dead_band = 0.1  # m/s² — small accelerations → coast
        speed_err = self.v_target - u
        if speed_err > 0:
            self.speed_err_integral += speed_err * dt
            self.speed_err_integral = min(self.speed_err_integral, 3.0)
            speed_boost = 0.15 * speed_err + 0.05 * self.speed_err_integral
        else:
            self.speed_err_integral = max(self.speed_err_integral - 0.5 * dt, 0.0)
            speed_boost = 0.0

        if self.acceleration > dead_band:
            base = self.acceleration / self.mpc.ax_max * self.throttle_gain
            throttle = min(base + speed_boost, 1.0)
            braking = 0.0
        elif self.acceleration < -dead_band:
            throttle = 0.0
            braking = min(-self.acceleration / abs(self.mpc.ax_min) * self.brake_gain, 1.0)
        else:
            # Dead-band: coast with only PI speed feedback
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

    def __init__(self, path_type: str, v_target: float,
                 lane_offset: float = 3.0,
                 sine_amplitude: float = 2.0,
                 sine_wavelength: float = 30.0,
                 rms_time_start: float = 0.0):
        self.path_type = path_type
        self.v_target = v_target
        self.lane_offset = lane_offset
        self.sine_amplitude = sine_amplitude
        self.sine_wavelength = sine_wavelength
        self.rms_time_start = rms_time_start

        # Raw sample storage
        self.times: list[float] = []
        self.crosstrack_errors: list[float] = []   # signed (y - y_ref)
        self.heading_errors: list[float] = []       # signed (psi - psi_ref)
        self.speed_errors: list[float] = []         # signed (u - v_target)
        self.xs: list[float] = []
        self.ys: list[float] = []
        self.us: list[float] = []

        # Control / MPC history (for plots)
        self.steerings: list[float] = []
        self.throttles: list[float] = []
        self.brakings: list[float] = []
        self.deltas: list[float] = []        # steering angle (rad)
        self.accelerations: list[float] = [] # longitudinal accel (m/s²)
        self.solve_times_ms: list[float] = []
        self.tau_comp_ms: list[float] = []
        self.ctrl_times: list[float] = []    # may differ from self.times

        # Lateral force comparison (Chrono actual vs NN predicted)
        self.fy_times: list[float] = []
        self.actual_Fy_front: list[float] = []
        self.actual_Fy_rear: list[float] = []
        self.nn_Fy_front: list[float] = []
        self.nn_Fy_rear: list[float] = []

        # Reference path arrays (for XY plot)
        self.y_refs: list[float] = []
        self.psi_refs: list[float] = []

        # Running statistics (for periodic reports)
        self._window: list[float] = []  # recent |crosstrack| for windowed RMS

    # ----- reference path (matches dallas_chrono_demo RMS logic) -----

    def _y_ref(self, x: float) -> float:
        if self.path_type == "lane_change":
            s, e = 10.0, 25.0
            if x < s:
                return 0.0
            elif x > e:
                return self.lane_offset
            t = (x - s) / (e - s)
            return self.lane_offset * t * t * (3 - 2 * t)
        elif self.path_type == "double_lane_change":
            z1s, z1e = 8.0, 18.0
            z2s, z2e = 28.0, 38.0
            if x < z1s:
                return 0.0
            elif x < z1e:
                t = (x - z1s) / (z1e - z1s)
                return self.lane_offset * t * t * (3 - 2 * t)
            elif x < z2s:
                return self.lane_offset
            elif x < z2e:
                t = (x - z2s) / (z2e - z2s)
                return self.lane_offset * (1 - t * t * (3 - 2 * t))
            else:
                return 0.0
        elif self.path_type == "sinusoidal":
            return self.sine_amplitude * np.sin(
                2 * np.pi * x / self.sine_wavelength)
        return 0.0

    def _psi_ref(self, x: float) -> float:
        if self.path_type == "lane_change":
            s, e = 10.0, 25.0
            if s < x < e:
                t = (x - s) / (e - s)
                dy_dx = self.lane_offset * 6 * t * (1 - t) / (e - s)
                return np.arctan(dy_dx)
            return 0.0
        elif self.path_type == "double_lane_change":
            z1s, z1e = 8.0, 18.0
            z2s, z2e = 28.0, 38.0
            if z1s < x < z1e:
                t = (x - z1s) / (z1e - z1s)
                dy_dx = self.lane_offset * 6 * t * (1 - t) / (z1e - z1s)
                return np.arctan(dy_dx)
            elif z2s < x < z2e:
                t = (x - z2s) / (z2e - z2s)
                dy_dx = -self.lane_offset * 6 * t * (1 - t) / (z2e - z2s)
                return np.arctan(dy_dx)
            return 0.0
        elif self.path_type == "sinusoidal":
            return np.arctan(
                2 * np.pi * self.sine_amplitude / self.sine_wavelength
                * np.cos(2 * np.pi * x / self.sine_wavelength))
        return 0.0

    # ----- public API -----

    def record(self, t: float, x: float, y: float, psi: float, u: float):
        """Record one sample."""
        y_ref = self._y_ref(x)
        psi_ref = self._psi_ref(x)

        ct_err = y - y_ref
        hd_err = psi - psi_ref
        # Wrap heading error to [-pi, pi]
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
        """Record control outputs (called after MPC solve)."""
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
                           nn_Fy_f: float, nn_Fy_r: float):
        """Record actual (Chrono) vs NN-predicted axle lateral forces."""
        self.fy_times.append(t)
        self.actual_Fy_front.append(actual_Fy_f)
        self.actual_Fy_rear.append(actual_Fy_r)
        self.nn_Fy_front.append(nn_Fy_f)
        self.nn_Fy_rear.append(nn_Fy_r)

    def periodic_summary(self, last_n: int = 20) -> str:
        """One-line summary of recent tracking performance."""
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
        """Full summary printed at end of run."""
        if not self.crosstrack_errors:
            return "  No tracking data collected."

        ct = np.array(self.crosstrack_errors)
        hd = np.array(self.heading_errors)
        sp = np.array(self.speed_errors)
        ts = np.array(self.times)

        # Apply RMS time window (skip startup transient)
        mask = ts >= self.rms_time_start
        if mask.sum() == 0:
            mask = np.ones(len(ts), dtype=bool)
        ct_w, hd_w, sp_w = ct[mask], hd[mask], sp[mask]

        rms_ct = np.sqrt(np.mean(ct_w ** 2))
        max_ct = np.max(np.abs(ct_w))
        rms_hd = np.degrees(np.sqrt(np.mean(hd_w ** 2)))
        max_hd = np.degrees(np.max(np.abs(hd_w)))
        rms_sp = np.sqrt(np.mean(sp_w ** 2))
        mean_sp = np.mean(np.array(self.us)[mask])
        min_sp = np.min(np.array(self.us)[mask])

        lines = [
            "",
            "  Tracking Analytics:",
            f"    Samples: {mask.sum()} (from t≥{self.rms_time_start:.1f}s)",
            f"    Cross-track error  — RMS: {rms_ct:.4f} m,  Max: {max_ct:.4f} m",
            f"    Heading error      — RMS: {rms_hd:.2f}°,  Max: {max_hd:.2f}°",
            f"    Speed error        — RMS: {rms_sp:.3f} m/s",
            f"    Speed              — Avg: {mean_sp:.2f},  Min: {min_sp:.2f} m/s",
        ]
        return "\n".join(lines)

    def plot_results(self, plot_dir: str, terrain_name: str,
                     model_label: str):
        """Generate end-of-run diagnostic plots and save to plot_dir."""
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        if len(self.times) < 2:
            print("  Not enough data for plots.")
            return

        model_tag = "nn" if "NN" in model_label else "pacejka"
        out = Path(plot_dir)
        out.mkdir(parents=True, exist_ok=True)
        tag = f"{terrain_name}_{self.path_type}_{model_tag}"

        ts = np.array(self.times)
        xs = np.array(self.xs)
        ys = np.array(self.ys)
        yr = np.array(self.y_refs)
        ct = np.array(self.crosstrack_errors)
        hd = np.degrees(np.array(self.heading_errors))
        us = np.array(self.us)

        # ---- Figure 1: XY trajectory + reference ----
        fig, ax = plt.subplots(figsize=(12, 5))
        ax.plot(xs, yr, 'k--', lw=1.5, label='Reference')
        sc = ax.scatter(xs, ys, c=np.abs(ct), cmap='RdYlGn_r',
                        s=8, vmin=0, vmax=max(0.3, np.percentile(np.abs(ct), 95)))
        plt.colorbar(sc, ax=ax, label='|Cross-track error| (m)')
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(f'XY Trajectory — {model_label} / {terrain_name} / {self.path_type}')
        ax.legend(loc='upper left')
        ax.set_aspect('equal', adjustable='datalim')
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(out / f'xy_trajectory_{tag}.png', dpi=150)
        plt.close(fig)

        # ---- Figure 2: Tracking errors (3 subplots) ----
        fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
        fig.suptitle(f'Tracking Errors — {model_label} / {terrain_name} / {self.path_type}',
                     fontsize=13)

        axes[0].plot(ts, ct * 100, 'b-', lw=0.8)
        axes[0].axhline(0, color='k', lw=0.5)
        axes[0].set_ylabel('Cross-track (cm)')
        axes[0].grid(True, alpha=0.3)
        # Add RMS annotation
        mask = ts >= self.rms_time_start
        if mask.sum() > 0:
            rms_ct = np.sqrt(np.mean(ct[mask] ** 2)) * 100
            axes[0].axhline(rms_ct, color='r', ls='--', lw=0.7, label=f'RMS={rms_ct:.1f}cm')
            axes[0].axhline(-rms_ct, color='r', ls='--', lw=0.7)
            axes[0].legend(fontsize=9)

        axes[1].plot(ts, hd, 'g-', lw=0.8)
        axes[1].axhline(0, color='k', lw=0.5)
        axes[1].set_ylabel('Heading error (°)')
        axes[1].grid(True, alpha=0.3)

        axes[2].plot(ts, us, 'r-', lw=0.8, label='Actual')
        axes[2].axhline(self.v_target, color='k', ls='--', lw=1, label=f'Target={self.v_target:.1f}')
        axes[2].set_ylabel('Speed (m/s)')
        axes[2].set_xlabel('Time (s)')
        axes[2].legend(fontsize=9)
        axes[2].grid(True, alpha=0.3)

        fig.tight_layout()
        fig.savefig(out / f'tracking_errors_{tag}.png', dpi=150)
        plt.close(fig)

        # ---- Figure 3: Control inputs + MPC timing (4 subplots) ----
        if len(self.ctrl_times) > 1:
            tc = np.array(self.ctrl_times)
            fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
            fig.suptitle(f'Controls & MPC — {model_label} / {terrain_name} / {self.path_type}',
                         fontsize=13)

            axes[0].plot(tc, np.degrees(self.deltas), 'b-', lw=0.8)
            axes[0].set_ylabel('Steering angle (°)')
            axes[0].grid(True, alpha=0.3)

            axes[1].plot(tc, self.throttles, 'g-', lw=0.8, label='Throttle')
            axes[1].plot(tc, self.brakings, 'r-', lw=0.8, label='Brake')
            axes[1].set_ylabel('Throttle / Brake')
            axes[1].set_ylim(-0.05, 1.05)
            axes[1].legend(fontsize=9)
            axes[1].grid(True, alpha=0.3)

            axes[2].plot(tc, self.solve_times_ms, 'purple', lw=0.8)
            axes[2].set_ylabel('MPC solve (ms)')
            axes[2].grid(True, alpha=0.3)

            axes[3].plot(tc, self.tau_comp_ms, 'orange', lw=0.8)
            axes[3].set_ylabel('τ_comp (ms)')
            axes[3].set_xlabel('Time (s)')
            axes[3].grid(True, alpha=0.3)

            fig.tight_layout()
            fig.savefig(out / f'controls_mpc_{tag}.png', dpi=150)
            plt.close(fig)

        # ---- Figure 4: Fy actual vs NN (if data available) ----
        if len(self.fy_times) > 1:
            tf = np.array(self.fy_times)
            fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
            fig.suptitle(
                f'Lateral Force: Chrono vs NN — {terrain_name} / {self.path_type}',
                fontsize=13)

            axes[0].plot(tf, self.actual_Fy_front, 'b-', lw=0.8, label='Chrono')
            axes[0].plot(tf, self.nn_Fy_front, 'r--', lw=0.8, label='NN')
            axes[0].set_ylabel('Fy (N)')
            axes[0].set_title('Front Axle Lateral Force')
            axes[0].legend(fontsize=9)
            axes[0].grid(True, alpha=0.3)

            axes[1].plot(tf, self.actual_Fy_rear, 'b-', lw=0.8, label='Chrono')
            axes[1].plot(tf, self.nn_Fy_rear, 'r--', lw=0.8, label='NN')
            axes[1].set_ylabel('Fy (N)')
            axes[1].set_xlabel('Time (s)')
            axes[1].set_title('Rear Axle Lateral Force')
            axes[1].legend(fontsize=9)
            axes[1].grid(True, alpha=0.3)

            fig.tight_layout()
            fig.savefig(out / f'Fy_actual_vs_nn_{tag}.png', dpi=150)
            plt.close(fig)

        print(f"  Plots saved to {out}/")


# =============================================================================
# Main controller loop
# =============================================================================

def run_controller_node(args):
    print("=" * 60)
    print("MPC Controller Node (Decoupled)")
    print("=" * 60)

    # ------------------------------------------------------------------
    # Wait for config from sim node
    # ------------------------------------------------------------------
    state_sub = ZMQSubscriber(sim_sub_endpoint(args.sim_host, args.sim_port))
    print(f"  Subscribing to state from {args.sim_host}:{args.sim_port}")

    config = None
    print("  Waiting for sim config...", end="", flush=True)
    for _ in range(200):  # Up to 20 seconds
        result = state_sub.recv(timeout_ms=100)
        if result is not None:
            topic, msg = result
            if isinstance(msg, SimStatus) and msg.event == "config":
                config = msg.config
                print(" received!")
                break
    if config is None:
        # Use defaults from args
        print(" timeout — using CLI arguments for config.")
        vehicle_params = get_vehicle_params_for_demo()
        tp = get_terrain_preset(args.terrain)
        terrain_params = terrain_preset_to_internal(tp)
        config = {
            "vehicle_params": vehicle_params,
            "terrain_params": terrain_params,
            "terrain_preset": args.terrain,
            "path_type": args.path,
            "v_target": args.speed,
            "sim_time": args.time,
            "sine_amplitude": args.sine_amplitude,
            "sine_wavelength": args.sine_wavelength,
        }

    vehicle_params = config["vehicle_params"]
    terrain_params = config["terrain_params"]
    v_target = config.get("v_target", args.speed)
    path_type = config.get("path_type", args.path)
    sine_amp = config.get("sine_amplitude", args.sine_amplitude)
    sine_wl = config.get("sine_wavelength", args.sine_wavelength)
    terrain_name = config.get("terrain_preset", args.terrain)

    print(f"  Terrain: {terrain_name}")
    print(f"  Path: {path_type}, v_target: {v_target} m/s")

    # ------------------------------------------------------------------
    # Build MPC
    # ------------------------------------------------------------------
    dt_mpc = 0.1
    N_horizon = 25

    nn_casadi = None
    if args.model == "nn":
        base_path = Path(__file__).parent.parent
        model_version = args.nn_model
        model_path = base_path / "nn_models" / model_version / "best_terrain_nn.pt"
        scaler_path = model_path.parent / "scalers.pkl"
        if model_path.exists():
            nn_casadi = NNCasADi(str(model_path), str(scaler_path), terrain_params)
        else:
            print(f"  WARNING: NN model not found at {model_path}, falling back to Pacejka")

    mpc = DallasMPC(
        nn_casadi=nn_casadi,
        params=vehicle_params,
        dt=dt_mpc,
        N=N_horizon,
        kappa_mode=args.kappa,
        lateral_load_transfer=not args.no_lat_transfer,
    )
    model_label = "NN" if mpc.use_nn else "Pacejka"
    print(f"  MPC built: {model_label}, N={N_horizon}, dt={dt_mpc}s")

    # Warmup MPC solver (JIT)
    print("  Warming up MPC solver...", end="", flush=True)
    z0_warm = np.array([0, 0, 0, 5.0, 0, 0, 0, 0])
    x_ref_w = np.linspace(0, 20, N_horizon + 1)
    y_ref_w = np.zeros(N_horizon + 1)
    psi_ref_w = np.zeros(N_horizon + 1)
    v_ref_w = 5.0 * np.ones(N_horizon + 1)
    for _ in range(3):
        mpc.solve(z0_warm, x_ref_w, y_ref_w, psi_ref_w, v_ref_w, x_ref_w[-1], 0, 0)
        print(".", end="", flush=True)
    print(" done!")

    # ------------------------------------------------------------------
    # Path generator
    # ------------------------------------------------------------------
    path_func = make_path_function(
        path_type=path_type,
        v_target=v_target,
        sine_amplitude=sine_amp,
        sine_wavelength=sine_wl,
        use_closest_point=not args.no_path_reindex,
    )

    # ------------------------------------------------------------------
    # Transport delay compensation
    # ------------------------------------------------------------------
    delay_est = DelayEstimator(initial_delay=args.initial_delay)
    state_predictor = StatePredictor(vehicle_params, dt_prop=0.005)
    control_buffer = collections.deque(maxlen=50)  # Recent (time, delta_dot, Jx)

    # ------------------------------------------------------------------
    # Control integrator
    # ------------------------------------------------------------------
    integrator = ControlIntegrator(mpc, v_target=v_target)

    # ------------------------------------------------------------------
    # Tracking analytics
    # ------------------------------------------------------------------
    analytics = TrackingAnalytics(
        path_type=path_type,
        v_target=v_target,
        sine_amplitude=sine_amp,
        sine_wavelength=sine_wl,
        rms_time_start=args.rms_time_start,
    )

    # ------------------------------------------------------------------
    # Publisher for control commands
    # ------------------------------------------------------------------
    ctrl_pub = ZMQPublisher(ctrl_pub_endpoint(args.ctrl_port))
    print(f"  Publishing controls on port {args.ctrl_port}")

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------
    seq = 0
    last_state: VehicleState = None
    solve_times = []
    n_terrain_est = terrain_params.get("n", 1.1)

    # ------------------------------------------------------------------
    # Timestamped run directory (shared by CSV + plots)
    # ------------------------------------------------------------------
    model_tag = "nn" if model_label == "NN" else "pacejka"
    run_ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(args.plot_dir) / f"{run_ts}_{terrain_name}_{path_type}_{model_tag}"
    run_dir.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------
    # Diagnostic CSV logger
    # ------------------------------------------------------------------
    csv_file = None
    csv_writer = None
    if not args.no_csv:
        csv_path = run_dir / f"diag_{terrain_name}_{path_type}_{model_tag}.csv"
        csv_file = open(csv_path, "w", newline="")
        csv_header = [
            # timing
            "sim_time", "wall_time", "seq",
            # measured state (noisy, as seen by controller)
            "x_fa_meas", "y_fa_meas", "psi_meas", "u_meas", "v_meas", "omega_meas",
            # ground truth (if available)
            "x_fa_true", "y_fa_true", "psi_true", "u_true",
            # delay-compensated state (input to MPC)
            "x_fa_comp", "y_fa_comp", "psi_comp", "u_comp", "v_comp", "omega_comp",
            "delta_state", "ax_state",
            # reference trajectory (first point)
            "x_ref_0", "y_ref_0", "psi_ref_0", "v_ref_0",
            # MPC output
            "delta_dot", "Jx", "mpc_cost", "solver_status", "solver_iters",
            # integrated controls
            "steering", "throttle", "braking", "steering_angle", "acceleration",
            # delay compensation
            "tau_one_way_ms", "tau_solve_ms", "tau_comp_ms", "solve_time_ms",
            # tracking errors
            "crosstrack_err", "heading_err_deg", "speed_err",
            # tire forces (if available)
            "actual_Fy_front", "actual_Fy_rear", "nn_Fy_front", "nn_Fy_rear",
            # slip angles & Fz used for NN
            "alpha_f", "alpha_r", "Fz_f_mean", "Fz_r_mean",
        ]
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(csv_header)
        print(f"  Diagnostic CSV: {csv_path}")

    print(f"  Delay compensation: {'ON' if not args.no_delay_comp else 'OFF'} "
          f"(initial τ={args.initial_delay * 1000:.0f}ms)")
    print(f"  Running controller loop...")

    running = True
    while running:
        # --- Receive state (blocking with timeout to allow clean exit) ---
        result = state_sub.recv(timeout_ms=200)
        if result is None:
            continue

        topic, msg = result

        if isinstance(msg, SimStatus):
            if msg.event == "stop":
                print("  Received stop signal from sim node.")
                running = False
                break
            continue

        if not isinstance(msg, VehicleState):
            continue

        last_state = msg
        recv_time = wall_time.time()

        # Update delay estimate
        delay_est.update_transport(msg.wall_time, recv_time)

        # --- Build state vector ---
        psi = quat_to_yaw(msg.quat_e0, msg.quat_e1, msg.quat_e2, msg.quat_e3)
        Lf = mpc.Lf

        # Transform CG → front axle
        x_fa = msg.x_cg + Lf * np.cos(psi)
        y_fa = msg.y_cg + Lf * np.sin(psi)

        z0_measured = np.array([
            x_fa, y_fa, psi,
            max(msg.u, 0.5),  # Clamp min speed (same as dallas_chrono_demo)
            msg.v, msg.omega,
            integrator.steering_angle,
            integrator.acceleration,
        ])

        # --- Delay compensation: propagate state forward ---
        if not args.no_delay_comp:
            tau = delay_est.compensation_delay
            z0 = state_predictor.propagate(z0_measured, control_buffer, tau)
        else:
            z0 = z0_measured
            tau = 0.0

        # --- Generate reference trajectory ---
        x_ref, y_ref, psi_ref, v_ref, x_goal, y_goal, psi_goal = path_func(
            msg.time, z0, mpc.N, mpc.dt
        )

        # --- Solve MPC ---
        t0_solve = wall_time.time()
        delta_dot, Jx, Z_opt, U_opt = mpc.solve(
            z0, x_ref, y_ref, psi_ref, v_ref,
            x_goal, y_goal, psi_goal,
            n_terrain=n_terrain_est,
            sr_meas=0.0,
        )
        t_solve = wall_time.time() - t0_solve
        solve_times.append(t_solve)
        delay_est.update_solve(t_solve)

        if Z_opt is None:
            delta_dot, Jx = 0.0, 0.0

        # --- Integrate controls ---
        steering, throttle, braking = integrator.update(
            delta_dot, Jx, dt_mpc, msg.u
        )

        # --- Record in control buffer for delay compensation ---
        control_buffer.append((msg.time, delta_dot, Jx))

        # --- Record tracking analytics (use ground truth if available) ---
        tf = msg.tire_forces or {}
        true_x = tf.get('true_x_cg')
        if true_x is not None:
            true_psi = tf['true_psi']
            true_x_fa = true_x + Lf * np.cos(true_psi)
            true_y_fa = tf['true_y_cg'] + Lf * np.sin(true_psi)
            true_u = tf['true_u']
            analytics.record(msg.time, true_x_fa, true_y_fa, true_psi, true_u)
        else:
            analytics.record(msg.time, z0_measured[0], z0_measured[1], psi, msg.u)
        analytics.record_control(
            msg.time, steering, throttle, braking,
            integrator.steering_angle, integrator.acceleration,
            t_solve * 1000.0, delay_est.compensation_delay * 1000.0,
        )

        # --- Record Fy: Chrono actual vs NN predicted (if NN enabled) ---
        if mpc.use_nn and msg.tire_forces is not None:
            tf = msg.tire_forces
            actual_Fy_f = tf.get('front_left_Fy', 0) + tf.get('front_right_Fy', 0)
            actual_Fy_r = tf.get('rear_left_Fy', 0) + tf.get('rear_right_Fy', 0)

            # Bicycle-model slip angles
            u_safe = max(abs(msg.u), 0.5)
            alpha_f = integrator.steering_angle - math.atan2(
                msg.v + mpc.Lf * msg.omega, u_safe)
            alpha_r = -math.atan2(
                msg.v - mpc.Lr * msg.omega, u_safe)

            # Dynamic Fz per wheel (longitudinal + lateral load transfer)
            g = 9.81
            ax = integrator.acceleration
            L = mpc.Lf + mpc.Lr
            Fz_f_mean = (mpc.M * g * mpc.Lr - mpc.M * ax * mpc.h_cg) / L / 2.0
            Fz_r_mean = (mpc.M * g * mpc.Lf + mpc.M * ax * mpc.h_cg) / L / 2.0

            if mpc.lateral_load_transfer:
                ay = msg.u * msg.omega
                dFz = mpc.M * ay * mpc.h_cg / mpc.T / 2.0
                Fz_fo = min(Fz_f_mean + dFz, 1.9 * Fz_f_mean)
                Fz_fi = max(Fz_f_mean - dFz, 0.1 * Fz_f_mean)
                Fz_ro = min(Fz_r_mean + dFz, 1.9 * Fz_r_mean)
                Fz_ri = max(Fz_r_mean - dFz, 0.1 * Fz_r_mean)
                _, Fy_fo = mpc.nn_casadi.predict_numeric(
                    alpha_f, Fz_fo, u_safe, n_terrain=n_terrain_est)
                _, Fy_fi = mpc.nn_casadi.predict_numeric(
                    alpha_f, Fz_fi, u_safe, n_terrain=n_terrain_est)
                _, Fy_ro = mpc.nn_casadi.predict_numeric(
                    alpha_r, Fz_ro, u_safe, n_terrain=n_terrain_est)
                _, Fy_ri = mpc.nn_casadi.predict_numeric(
                    alpha_r, Fz_ri, u_safe, n_terrain=n_terrain_est)
                nn_Fy_f = -(Fy_fo + Fy_fi)
                nn_Fy_r = -(Fy_ro + Fy_ri)
            else:
                _, Fy_fw = mpc.nn_casadi.predict_numeric(
                    alpha_f, Fz_f_mean, u_safe, n_terrain=n_terrain_est)
                _, Fy_rw = mpc.nn_casadi.predict_numeric(
                    alpha_r, Fz_r_mean, u_safe, n_terrain=n_terrain_est)
                nn_Fy_f = -2.0 * Fy_fw
                nn_Fy_r = -2.0 * Fy_rw

            analytics.record_tire_forces(
                msg.time, actual_Fy_f, actual_Fy_r, nn_Fy_f, nn_Fy_r)

        # --- Publish command ---
        cmd = ControlCommand(
            time=msg.time,
            wall_time=wall_time.time(),
            seq=seq,
            steering=steering,
            throttle=throttle,
            braking=braking,
            delta=integrator.steering_angle,
            acceleration=integrator.acceleration,
            delta_dot=delta_dot,
            jerk=Jx,
            solve_time_ms=t_solve * 1000.0,
        )
        ctrl_pub.send(cmd)

        # --- Write diagnostic CSV row ---
        if csv_writer is not None:
            tf = msg.tire_forces or {}
            true_x = tf.get('true_x_cg')
            if true_x is not None:
                true_psi_v = tf['true_psi']
                true_x_fa = true_x + Lf * np.cos(true_psi_v)
                true_y_fa = tf['true_y_cg'] + Lf * np.sin(true_psi_v)
                true_u_v = tf['true_u']
            else:
                true_x_fa = z0_measured[0]
                true_y_fa = z0_measured[1]
                true_psi_v = psi
                true_u_v = msg.u

            # Cross-track & heading from analytics (last recorded)
            ct_err = analytics.crosstrack_errors[-1] if analytics.crosstrack_errors else 0
            hd_err = np.degrees(analytics.heading_errors[-1]) if analytics.heading_errors else 0
            sp_err = analytics.speed_errors[-1] if analytics.speed_errors else 0

            # Fy data (last recorded, may be empty)
            fy_af = analytics.actual_Fy_front[-1] if analytics.actual_Fy_front else ''
            fy_ar = analytics.actual_Fy_rear[-1] if analytics.actual_Fy_rear else ''
            fy_nf = analytics.nn_Fy_front[-1] if analytics.nn_Fy_front else ''
            fy_nr = analytics.nn_Fy_rear[-1] if analytics.nn_Fy_rear else ''

            # Slip angles & Fz (compute inline)
            u_safe_csv = max(abs(msg.u), 0.5)
            alpha_f_csv = integrator.steering_angle - math.atan2(
                msg.v + mpc.Lf * msg.omega, u_safe_csv)
            alpha_r_csv = -math.atan2(
                msg.v - mpc.Lr * msg.omega, u_safe_csv)
            g_csv = 9.81
            L_csv = mpc.Lf + mpc.Lr
            Fz_f_csv = (mpc.M * g_csv * mpc.Lr - mpc.M * integrator.acceleration * mpc.h_cg) / L_csv / 2.0
            Fz_r_csv = (mpc.M * g_csv * mpc.Lf + mpc.M * integrator.acceleration * mpc.h_cg) / L_csv / 2.0

            mpc_cost = getattr(mpc, 'last_cost', float('nan'))
            solver_status = getattr(mpc, 'last_solver_status', '')
            solver_iters = getattr(mpc, 'last_iter_count', -1)

            csv_writer.writerow([
                f"{msg.time:.4f}", f"{recv_time:.6f}", seq,
                f"{z0_measured[0]:.6f}", f"{z0_measured[1]:.6f}",
                f"{psi:.6f}", f"{msg.u:.4f}", f"{msg.v:.4f}", f"{msg.omega:.6f}",
                f"{true_x_fa:.6f}", f"{true_y_fa:.6f}",
                f"{true_psi_v:.6f}", f"{true_u_v:.4f}",
                f"{z0[0]:.6f}", f"{z0[1]:.6f}", f"{z0[2]:.6f}",
                f"{z0[3]:.4f}", f"{z0[4]:.4f}", f"{z0[5]:.6f}",
                f"{z0[6]:.6f}", f"{z0[7]:.4f}",
                f"{x_ref[0]:.6f}", f"{y_ref[0]:.6f}",
                f"{psi_ref[0]:.6f}", f"{v_ref[0]:.4f}",
                f"{delta_dot:.6f}", f"{Jx:.6f}",
                f"{mpc_cost:.4f}", solver_status, solver_iters,
                f"{steering:.6f}", f"{throttle:.4f}", f"{braking:.4f}",
                f"{integrator.steering_angle:.6f}", f"{integrator.acceleration:.4f}",
                f"{delay_est.one_way_delay*1000:.2f}",
                f"{delay_est.solve_time*1000:.2f}",
                f"{delay_est.compensation_delay*1000:.2f}",
                f"{t_solve*1000:.2f}",
                f"{ct_err:.6f}", f"{hd_err:.4f}", f"{sp_err:.4f}",
                fy_af, fy_ar, fy_nf, fy_nr,
                f"{alpha_f_csv:.6f}", f"{alpha_r_csv:.6f}",
                f"{Fz_f_csv:.1f}", f"{Fz_r_csv:.1f}",
            ])

        seq += 1

        # --- Periodic report ---
        if seq % 20 == 0:
            mean_ms = np.mean(solve_times[-20:]) * 1000
            tau_ms = delay_est.compensation_delay * 1000
            trk = analytics.periodic_summary()
            print(f"  t={msg.time:.1f}s  solve={mean_ms:.1f}ms  "
                  f"τ_comp={tau_ms:.1f}ms  {trk}  "
                  f"u={msg.u:.2f}m/s")

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    if solve_times:
        st = np.array(solve_times)
        print(f"\n  Controller Summary ({model_label}):")
        print(f"    Total solves: {len(st)}")
        print(f"    Mean solve:   {np.mean(st)*1000:.2f} ms")
        print(f"    Max solve:    {np.max(st)*1000:.2f} ms")
        print(f"    Effective rate: {1.0/np.mean(st):.1f} Hz")
        print(f"    Final τ_comp: {delay_est.compensation_delay*1000:.1f} ms")

    print(analytics.final_summary())

    # Close CSV
    if csv_file is not None:
        csv_file.close()
        print(f"  Diagnostic CSV written: {csv_path} ({seq} rows)")

    if not args.no_plot:
        analytics.plot_results(
            plot_dir=str(run_dir),
            terrain_name=terrain_name,
            model_label=model_label,
        )

    ctrl_pub.close()
    state_sub.close()


# =============================================================================
# Entry point
# =============================================================================

def main():
    p = argparse.ArgumentParser(description="MPC Controller Node (decoupled)")

    # Model
    p.add_argument("--model", default="nn", choices=["nn", "linear"],
                   help="nn = NN tire model, linear = Pacejka")
    p.add_argument("--nn-model", default="v6", help="NN model version directory")
    p.add_argument("--kappa", default="zero", choices=["zero", "approx"])
    p.add_argument("--no-lat-transfer", action="store_true",
                   help="Disable lateral load transfer (2 NN calls vs 4)")

    # Path
    p.add_argument("--path", default="lane_change",
                   choices=["lane_change", "double_lane_change", "sinusoidal"])
    p.add_argument("--speed", type=float, default=5.0, help="Target speed (m/s)")
    p.add_argument("--sine-amplitude", type=float, default=2.0)
    p.add_argument("--sine-wavelength", type=float, default=30.0)
    p.add_argument("--no-path-reindex", action="store_true")

    # Terrain (used as fallback if no config received from sim)
    p.add_argument("--terrain", default="sand", choices=["sand", "clay", "dirt"])
    p.add_argument("--time", type=float, default=15.0, help="Expected sim duration")

    # Delay compensation
    p.add_argument("--no-delay-comp", action="store_true",
                   help="Disable transport delay compensation in MPC")
    p.add_argument("--initial-delay", type=float, default=0.02,
                   help="Initial one-way delay estimate (s)")

    # Analytics
    p.add_argument("--rms-time-start", type=float, default=2.0,
                   help="Start time for RMS calculation, skips startup (s)")
    p.add_argument("--no-plot", action="store_true",
                   help="Skip generating end-of-run plots")
    p.add_argument("--no-csv", action="store_true",
                   help="Skip diagnostic CSV output")
    p.add_argument("--plot-dir", default="plots",
                   help="Directory for output plots (default: plots/)")

    # Network
    p.add_argument("--sim-host", default="localhost", help="Sim node host")
    p.add_argument("--sim-port", type=int, default=5555, help="Sim state port")
    p.add_argument("--ctrl-port", type=int, default=5556, help="Control command port")

    args = p.parse_args()
    run_controller_node(args)


if __name__ == "__main__":
    main()
