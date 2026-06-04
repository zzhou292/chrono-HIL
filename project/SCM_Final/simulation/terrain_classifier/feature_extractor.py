#!/usr/bin/env python3
"""
Feature Extractor for Terrain Classification
==============================================

Computes physically meaningful features from a sliding window of VehicleState
messages.  All inputs are quantities that can be measured by real-world sensors:

  Sensor            Signal                  Proxy in VehicleState
  ────────────────  ──────────────────────  ──────────────────────
  Wheel encoders    per-wheel ω_wheel       tire_forces[*_long_slip] (slip ratio)
  IMU               ax, ay, ω_z, ω̇_z       finite differences of u, v, omega
  GPS/INS           x, y, ψ, V             x_cg, y_cg, quat → ψ, u
  Steering sensor   δ                       ControlCommand.steering (passed in)

Features (per window):
  - Slip ratio: mean, std, max of |κ| for front/rear axles
  - Acceleration vibration: std(ax), std(ay), std(ω̇_z)
  - Lateral dynamics: mean |v|/u (side-slip ratio), mean |ω|
  - Speed statistics: mean u, std u
  - Vertical dynamics: std(az) — measures road roughness / wheel bounce
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np


@dataclass
class FeatureVector:
    """Named feature vector for one classification window."""
    timestamp: float

    # Slip ratio features (from wheel speed vs vehicle speed)
    slip_front_mean: float
    slip_front_std: float
    slip_front_max: float
    slip_rear_mean: float
    slip_rear_std: float
    slip_rear_max: float

    # IMU vibration features (std of accelerations over window)
    yaw_accel_std: float   # yaw acceleration vibration

    # Vertical acceleration (road roughness proxy)
    az_std: float

    # Lateral dynamics (body-frame)
    sideslip_ratio_mean: float   # mean |v|/max(|u|, 0.5)
    yaw_rate_mean: float         # mean |omega|

    # Longitudinal dynamics
    ax_std: float                # longitudinal acceleration vibration
    ay_std: float                # lateral acceleration vibration

    # Context features (included for normalization / ML)
    speed_mean: float            # mean u
    steering_std: float          # std of steering input over window

    # Logged but NOT in ML array
    speed_std: float             # std u

    def to_array(self) -> np.ndarray:
        """Return feature values as a flat numpy array (excludes timestamp).

        Excludes speed_mean (confound with sand's speed ceiling) and
        speed_std.  steering_std is kept for driving-intensity normalization.
        """
        return np.array([
            self.slip_front_mean, self.slip_front_std, self.slip_front_max,
            self.slip_rear_mean, self.slip_rear_std, self.slip_rear_max,
            self.yaw_accel_std, self.az_std,
            self.sideslip_ratio_mean, self.yaw_rate_mean,
            self.ax_std, self.ay_std,
            self.steering_std,
        ], dtype=np.float64)

    @staticmethod
    def feature_names() -> List[str]:
        return [
            "slip_front_mean", "slip_front_std", "slip_front_max",
            "slip_rear_mean", "slip_rear_std", "slip_rear_max",
            "yaw_accel_std", "az_std",
            "sideslip_ratio_mean", "yaw_rate_mean",
            "ax_std", "ay_std",
            "steering_std",
        ]


class FeatureExtractor:
    """Sliding-window feature extraction from VehicleState stream.

    Args:
        window_sec: Window duration in seconds (default 1.0s = 100 samples @ 100 Hz).
        stride_sec: How often to emit a new feature vector (default 0.25s).
        min_speed: Minimum speed (m/s) to compute valid features (avoids division
                   by near-zero speed for slip ratio).
    """

    def __init__(self, window_sec: float = 1.0, stride_sec: float = 0.25,
                 min_speed: float = 1.0):
        self.window_sec = window_sec
        self.stride_sec = stride_sec
        self.min_speed = min_speed

        # Circular buffer of raw samples: (t, u, v, omega, x, y, z, psi,
        #   slip_fl, slip_fr, slip_rl, slip_rr, steering)
        self._buf: deque = deque()
        self._last_emit_time: float = -1e9

    @staticmethod
    def _yaw_from_quat(e0, e1, e2, e3) -> float:
        return math.atan2(2 * (e0 * e3 + e1 * e2),
                          1 - 2 * (e2 * e2 + e3 * e3))

    def push(self, state, steering: float = 0.0) -> Optional[FeatureVector]:
        """Ingest a VehicleState and optionally return a FeatureVector.

        Args:
            state: VehicleState dataclass from hil_messages.
            steering: Current normalized steering command [-1,1].

        Returns:
            FeatureVector if a new window just completed, else None.
        """
        t = state.time
        psi = self._yaw_from_quat(state.quat_e0, state.quat_e1,
                                   state.quat_e2, state.quat_e3)

        # Extract per-wheel slip ratios from tire_forces (if available)
        tf = state.tire_forces or {}
        slip_fl = tf.get("front_left_long_slip", 0.0)
        slip_fr = tf.get("front_right_long_slip", 0.0)
        slip_rl = tf.get("rear_left_long_slip", 0.0)
        slip_rr = tf.get("rear_right_long_slip", 0.0)

        sample = (t, state.u, state.v, state.omega,
                  state.x_cg, state.y_cg, state.z_cg, psi,
                  slip_fl, slip_fr, slip_rl, slip_rr, steering)
        self._buf.append(sample)

        # Trim old samples outside the window
        while self._buf and self._buf[0][0] < t - self.window_sec:
            self._buf.popleft()

        # Check if it's time to emit
        if t - self._last_emit_time < self.stride_sec:
            return None

        # Need at least ~20 samples for meaningful statistics
        if len(self._buf) < 20:
            return None

        # Check minimum speed (avoid bad slip ratio features at standstill)
        speeds = [s[1] for s in self._buf]
        if np.mean(np.abs(speeds)) < self.min_speed:
            return None

        self._last_emit_time = t
        return self._compute_features(t)

    def _compute_features(self, current_time: float) -> FeatureVector:
        """Compute features from the current sliding window."""
        arr = np.array(list(self._buf))
        # columns: t=0, u=1, v=2, omega=3, x=4, y=5, z=6, psi=7,
        #          slip_fl=8, slip_fr=9, slip_rl=10, slip_rr=11, steer=12

        ts = arr[:, 0]
        u = arr[:, 1]
        v = arr[:, 2]
        omega = arr[:, 3]
        z = arr[:, 6]
        slip_fl = arr[:, 8]
        slip_fr = arr[:, 9]
        slip_rl = arr[:, 10]
        slip_rr = arr[:, 11]

        # ---- Slip ratio features (front/rear axle averages) ----
        slip_front = (np.abs(slip_fl) + np.abs(slip_fr)) / 2.0
        slip_rear = (np.abs(slip_rl) + np.abs(slip_rr)) / 2.0

        # ---- Low-pass filter to suppress sensor noise before differentiation ----
        # Moving average with window of 5 samples (~50ms at 100 Hz).
        # Prevents noise from being amplified 100x by finite differences.
        kern = 5
        if len(u) >= kern:
            kernel = np.ones(kern) / kern
            u_filt = np.convolve(u, kernel, mode='valid')
            v_filt = np.convolve(v, kernel, mode='valid')
            om_filt = np.convolve(omega, kernel, mode='valid')
            z_filt = np.convolve(z, kernel, mode='valid')
            ts_filt = np.convolve(ts, kernel, mode='valid')
        else:
            u_filt, v_filt, om_filt, z_filt, ts_filt = u, v, omega, z, ts

        # ---- Acceleration by finite differences (on filtered signals) ----
        dt = np.diff(ts_filt)
        dt = np.where(dt < 1e-6, 1e-6, dt)  # avoid division by zero

        ax = np.diff(u_filt) / dt   # longitudinal acceleration
        ay = np.diff(v_filt) / dt   # lateral acceleration
        omega_dot = np.diff(om_filt) / dt  # yaw acceleration

        # Vertical "acceleration" proxy: second derivative of z
        vz = np.diff(z_filt) / dt
        if len(vz) > 1:
            dt2 = dt[:-1]
            dt2 = np.where(dt2 < 1e-6, 1e-6, dt2)
            az = np.diff(vz) / dt2
        else:
            az = np.array([0.0])

        # ---- Side-slip ratio (|v| / max(|u|, 0.5)) ----
        safe_u = np.maximum(np.abs(u), 0.5)
        sideslip = np.abs(v) / safe_u

        steering = arr[:, 12]

        return FeatureVector(
            timestamp=current_time,
            slip_front_mean=float(np.mean(slip_front)),
            slip_front_std=float(np.std(slip_front)),
            slip_front_max=float(np.max(slip_front)),
            slip_rear_mean=float(np.mean(slip_rear)),
            slip_rear_std=float(np.std(slip_rear)),
            slip_rear_max=float(np.max(slip_rear)),
            yaw_accel_std=float(np.std(omega_dot)),
            az_std=float(np.std(az)),
            sideslip_ratio_mean=float(np.mean(sideslip)),
            yaw_rate_mean=float(np.mean(np.abs(omega))),
            ax_std=float(np.std(ax)),
            ay_std=float(np.std(ay)),
            speed_mean=float(np.mean(u)),
            steering_std=float(np.std(steering)),
            speed_std=float(np.std(u)),
        )

    def reset(self):
        """Clear the buffer (e.g. between runs)."""
        self._buf.clear()
        self._last_emit_time = -1e9
