"""
DOB-CBF Safety Filter
=====================

Disturbance-Observer-based Control Barrier Function (DOB-CBF) safety filter
for the HMMWV, adapted from the ROS2 bridge ``DobCBFHelper.py``.

Key features:
1. **Obstacle avoidance via weighted ellipsoidal CBF**: Uses a heading-aligned
   ellipsoidal barrier so that lateral evasion is cheaper than braking — the
   vehicle steers around obstacles rather than just stopping.
2. **Disturbance Observer (DOB)**: A first-order observer estimates unmodeled
   longitudinal forces (terrain drag, grade, SCM sinkage resistance) and feeds
   the estimate into the CBF constraint for robustness.
3. **Directional bias**: A forward-shifted ellipse center makes the filter more
   proactive for obstacles ahead of the vehicle.
4. **Terrain-aware speed limiting**: Reduces safe speed on rough terrain using
   the NN tire model traction predictions.
5. **Latency compensation**: Predicts vehicle state forward through actuation
   delay via a discrete predictor with derivative feedback.

Mathematical Formulation
------------------------
State: ``x = [x, y, psi, v, beta]`` where beta is the current road-wheel angle.

Barrier function (per obstacle):
    ``h(x) = (p - p_obs)^T P (p - p_obs) - r_safe^2``

where ``P = R(psi)^T diag(w_long, w_lat) R(psi)`` is a heading-aligned
ellipsoidal weight matrix.  Setting ``w_long < w_lat`` (e.g. 1/100 vs 1/9)
elongates the safe zone along the heading -> lateral escape is "closer" in
barrier-space -> the QP naturally prefers steering over braking.

Control inputs: ``u = [dbeta (steering angle rate), alpha (normalized throttle)]``

The QP minimizes deviation from the driver's desired input:
    min  ||u - u_des||^2
    s.t. -psi1_i @ u <= psi0_i   for each obstacle i
         actuator limits

DOB update:
    ``hdv0 = p0v + a_v * v``   (velocity disturbance estimate)
    ``dp0v = -a_v * (f_nom(v, alpha) + hdv0)``

Usage:
    from safety import CBFSafetyFilter

    cbf = CBFSafetyFilter(vehicle_params, nn_casadi=nn_model)

    # In loop:
    result = cbf.filter(
        desired_steering, desired_throttle, desired_brake,
        vehicle_state, obstacles, terrain_roughness
    )
"""

import numpy as np
from scipy.optimize import minimize
from typing import List, Tuple, Optional, Dict
from dataclasses import dataclass
import time


@dataclass
class SafetyFilterResult:
    """Output of the CBF safety filter."""
    steering: float       # Filtered steering input [-1, 1]
    throttle: float       # Filtered throttle input [0, 1]
    braking: float        # Filtered braking input [0, 1]
    was_modified: bool    # True if inputs were changed by safety filter
    active_constraints: int  # Number of active CBF constraints
    solve_time_ms: float  # QP solve time in milliseconds
    v_max_terrain: float  # Terrain-limited max safe speed (m/s)
    safety_margin: float  # Minimum h(x) across all obstacles
    dob_norm: float = 0.0 # Norm of DOB disturbance estimate


class DelayCompensator:
    """
    Discrete predictor for latency compensation.

    Implements the delay compensator from the CCTA bridge:
    Uses a FIFO buffer of past control commands and a discrete predictor
    with derivative feedback to estimate the current actual vehicle state
    given known actuation delay.

    Args:
        delay_steps: Number of time steps of actuation delay
        dt: Control loop time step (s)
        k1: Derivative feedback gain (default: 0.6)
        k2: Proportional feedback gain (default: 2.0)
    """

    def __init__(self, delay_steps: int = 5, dt: float = 0.02,
                 k1: float = 0.6, k2: float = 2.0):
        self.td = max(int(delay_steps), 0)
        self.dt = dt
        self.k1 = k1
        self.k2 = k2

        from collections import deque
        maxlen = self.td + 1
        self.throttle_hist = deque([0.0] * maxlen, maxlen=maxlen)
        self.steer_hist = deque([0.0] * maxlen, maxlen=maxlen)
        self.throttle_pred = deque([0.0] * maxlen, maxlen=maxlen)
        self.steer_pred = deque([0.0] * maxlen, maxlen=maxlen)

    def update(self, throttle: float, steering: float) -> Tuple[float, float]:
        """
        Push new command and return delay-compensated estimate.

        Returns:
            (compensated_throttle, compensated_steering)
        """
        self.throttle_hist.append(throttle)
        self.steer_hist.append(steering)

        if self.td == 0 or len(self.throttle_hist) <= self.td:
            self.throttle_pred.append(throttle)
            self.steer_pred.append(steering)
            return throttle, steering

        th = list(self.throttle_hist)
        sh = list(self.steer_hist)
        tp = list(self.throttle_pred)
        sp = list(self.steer_pred)

        v_td = th[-self.td - 1]
        dv_td = (th[-self.td] - v_td) / self.dt
        vp_td = tp[-self.td - 1]
        dvp_td = (tp[-self.td] - vp_td) / self.dt

        vp_now = tp[-1] + self.dt * (
            dv_td + self.k1 * (dv_td - dvp_td) + self.k2 * (v_td - vp_td)
        )
        self.throttle_pred.append(vp_now)

        s_td = sh[-self.td - 1]
        ds_td = (sh[-self.td] - s_td) / self.dt
        sp_td = sp[-self.td - 1]
        dsp_td = (sp[-self.td] - sp_td) / self.dt

        sp_now = sp[-1] + self.dt * (
            ds_td + self.k1 * (ds_td - dsp_td) + self.k2 * (s_td - sp_td)
        )
        self.steer_pred.append(sp_now)

        return float(vp_now), float(sp_now)


class DisturbanceObserver:
    """
    First-order velocity disturbance observer (DOB).

    Estimates the unmodeled longitudinal disturbance force (terrain drag,
    SCM sinkage resistance, grade, wind) as a lumped acceleration term.

    The observer dynamics are:
        hdv0 = p0v + a_v * v          (disturbance estimate)
        dp0v = -a_v * (f_nom + hdv0)  (observer state update)

    where f_nom is the nominal acceleration from the powertrain model.

    The higher-dimensional DOB (position x/y, heading) from the full
    reference is omitted because our obstacle positions come from ground
    truth, not dead-reckoning, so position disturbance estimation is
    unnecessary.

    Args:
        a_v: Observer bandwidth for velocity (higher = faster tracking,
             but more noise sensitivity). Default 10.0.
        max_accel: Maximum powertrain acceleration for nominal model (m/s^2).
        max_decel: Maximum braking deceleration for nominal model (m/s^2).
    """

    def __init__(self, a_v: float = 10.0,
                 max_accel: float = 3.0, max_decel: float = -6.0):
        self.a_v = a_v
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.p0v = 0.0  # Observer internal state
        self._initialized = False

    def update(self, v: float, alpha: float, dt: float,
               f_nom_override: Optional[float] = None) -> float:
        """
        Update the DOB and return the velocity disturbance estimate.

        Args:
            v: Current longitudinal speed (m/s)
            alpha: Current normalized throttle/brake input [-1, 1]
                   (positive = throttle, negative = brake)
            dt: Time step (s)
            f_nom_override: If provided, use this as the nominal longitudinal
                acceleration instead of the internal linear model. This allows
                the NN tire model to supply a physics-based f_nom so the DOB
                only estimates *true* unmodeled disturbances.

        Returns:
            hdv0: Estimated longitudinal disturbance acceleration (m/s^2).
                  Positive = unexplained acceleration, negative = drag/resistance.
        """
        # On first call, initialize p0v so that hdv0 = 0 (no initial disturbance)
        if not self._initialized:
            self.p0v = -self.a_v * v
            self._initialized = True

        hdv0 = self.p0v + self.a_v * v

        # Nominal acceleration model
        if f_nom_override is not None:
            f_nom = f_nom_override
        elif alpha >= 0:
            f_nom = self.max_accel * alpha
        else:
            f_nom = self.max_decel * abs(alpha)

        # Observer dynamics: dp0v = -a_v * (f_nom + hdv0)
        dp0v = -self.a_v * (f_nom + hdv0)
        self.p0v += dp0v * dt

        return hdv0

    @property
    def disturbance_estimate(self) -> float:
        """Current disturbance estimate without updating."""
        return self.p0v

    def reset(self):
        self.p0v = 0.0
        self._initialized = False


class CBFSafetyFilter:
    """
    DOB-CBF safety filter with weighted ellipsoidal barrier.

    Filters manual or MPC control inputs to enforce safety constraints:
    1. Obstacle avoidance via heading-aligned ellipsoidal CBF (prefers steering)
    2. Terrain-aware speed limits (from roughness + NN traction prediction)
    3. Disturbance observer for longitudinal force estimation
    4. Latency compensation (forward prediction of vehicle state)

    The filter solves a QP at each step:
        min  ||u - u_des||^2_W
        s.t. -psi1_i @ u <= psi0_i  (CBF constraints per obstacle)
             actuator limits

    Control inputs: u = [dbeta (steering rate), alpha (throttle)]

    The ellipsoidal barrier uses P = R^T diag(w_long, w_lat) R where w_long << w_lat
    so that lateral avoidance is "cheaper" than braking. This is the key difference
    from isotropic barriers that tend to only reduce throttle.

    Args:
        vehicle_params: Dict with M, Lf, Lr, Izz, h_cg, T
        nn_casadi: Optional NNCasADi for tire force prediction
        max_steering_rate: Maximum steering angle rate (rad/s)
        cbf_alpha: First CBF class-K gain (l1cbf). Higher = more conservative.
        cbf_alpha2: Second CBF class-K gain (l2cbf).
        obstacle_buffer: Extra safety margin around obstacles (m)
        vehicle_radius: Effective vehicle collision radius (m)
        max_speed: Absolute maximum allowed speed (m/s)
        delay_steps: Actuation delay in control steps for compensator
        control_dt: Safety filter update period (s)
        w_long: Barrier weight in longitudinal (heading) direction.
                Smaller values -> larger "safe zone" ahead -> earlier braking.
        w_lat: Barrier weight in lateral direction.
               Larger values -> tighter lateral clearance -> stronger steer signal.
        forward_bias: Forward shift of barrier center (m). Makes filter more
                      proactive for obstacles ahead. 0 = no bias.
        dob_bandwidth: DOB velocity observer bandwidth. Higher = faster.
        cbf_flavor: 'balance' (equal cost to modify steering/throttle),
                    'steer_priority' (strongly prefers throttle modification),
                    'throttle_priority' (strongly preserves throttle).
        teleop_delay: Estimated one-way network delay for teleoperation (s).
                      0 = no teleop compensation (local control). When > 0,
                      the barrier is inflated by v * RTT to account for operator
                      reaction lag, and stale commands trigger emergency braking.
        stale_cmd_timeout: Maximum command age before auto-brake (s).
                           Only active when teleop_delay > 0.
    """

    def __init__(self,
                 vehicle_params: dict,
                 nn_casadi=None,
                 max_steering_rate: float = 2.0,
                 cbf_alpha: float = 1.0,
                 cbf_alpha2: float = 0.8,
                 obstacle_buffer: float = 0.25,
                 vehicle_radius: float = 1.0,
                 max_speed: float = 15.0,
                 delay_steps: int = 5,
                 control_dt: float = 0.02,
                 w_long: float = 0.15,
                 w_lat: float = 0.50,
                 forward_bias: float = 1.5,
                 dob_bandwidth: float = 10.0,
                 cbf_flavor: str = 'balance',
                 teleop_delay: float = 0.0,
                 stale_cmd_timeout: float = 2.0):

        # Vehicle parameters
        self.M = vehicle_params['M']
        self.Lf = vehicle_params['Lf']
        self.Lr = vehicle_params['Lr']
        self.L = self.Lf + self.Lr
        self.Izz = vehicle_params['Izz']
        self.h_cg = vehicle_params.get('h_cg', 0.65)
        self.T = vehicle_params.get('T', 1.8194)

        # NN tire model for traction-aware limits
        self.nn_casadi = nn_casadi

        # CBF parameters
        self.alpha1 = cbf_alpha      # l1cbf in reference
        self.alpha2 = cbf_alpha2     # l2cbf in reference
        self.obstacle_buffer = obstacle_buffer
        self.vehicle_radius = vehicle_radius
        self.max_speed = max_speed
        self.max_steer_rate = max_steering_rate
        self.control_dt = control_dt

        # Ellipsoidal barrier weights (from reference: w1=1/100, w2=1/9)
        self.w_long = w_long
        self.w_lat = w_lat
        self.forward_bias = forward_bias

        # CBF flavor controls QP cost weighting
        self.cbf_flavor = cbf_flavor

        # Teleop delay compensation
        self._teleop_enabled = teleop_delay > 0.0  # Only activate if explicitly set
        self._teleop_delay = max(teleop_delay, 0.0)
        self._stale_cmd_timeout = stale_cmd_timeout
        self._cmd_age = 0.0        # Latest measured command age (s)
        self._last_cmd_wall = None  # Wall-clock time of last received command
        self._delay_ema = teleop_delay  # EMA-smoothed one-way delay estimate
        self._delay_ema_alpha = 0.15    # EMA smoothing factor

        # Obstacle filtering range — must cover the ellipsoidal barrier's
        # longitudinal reach: reach ~ forward_bias + sqrt(max_safe_r^2 / w_long)
        max_safe_r = vehicle_radius + obstacle_buffer + 5.0
        self.r_precpt = forward_bias + np.sqrt(max_safe_r**2 / w_long) + 10.0

        # Delay compensator
        self.delay_comp = DelayCompensator(
            delay_steps=delay_steps, dt=control_dt
        )

        # Disturbance observer
        self.dob = DisturbanceObserver(
            a_v=dob_bandwidth,
            max_accel=3.0,
            max_decel=-6.0,
        )

        # Steering angle conversion
        self.max_road_steer_angle = 0.49  # rad (HMMWV)

        # Actuator limits
        self.max_accel = 3.0   # m/s^2 throttle
        self.max_decel = -6.0  # m/s^2 braking

        # Current steering angle state (integrated from dbeta)
        self._beta = 0.0  # road wheel angle (rad)
        self._alpha = 0.0  # last throttle command

        # Rear-approach avoidance: when a vehicle closes from behind, never let
        # the driver slow down (braking raises rear-end risk), and accelerate to
        # try to escape when the path ahead is clear -- including from a standstill.
        self._rear_threat_dist = 12.0    # m: a closing rear obstacle within this is a threat
        self._rear_half_w = 2.5          # m: lateral half-width counted as "same lane"
        self._prev_rear_dist = float('inf')

        # State for logging
        self._last_result = None
        self._filter_count = 0
        self._modify_count = 0

        # CSV logging for diagnostics
        self._csv_file = None
        self._csv_writer = None
        self._obs_csv_file = None
        self._obs_csv_writer = None
        self._init_csv_logging()

        # Cache for NN tire force queries (avoid redundant calls per step)
        self._nn_cache = {}

    # ------------------------------------------------------------------
    # Teleop delay API
    # ------------------------------------------------------------------

    def set_teleop_delay(self, delay_s: float):
        """Set estimated one-way teleop network delay (seconds)."""
        self._teleop_delay = max(delay_s, 0.0)
        self._delay_ema = max(delay_s, 0.0)
        self._teleop_enabled = self._teleop_delay > 0.0

    def update_command_age(self, cmd_wall_time: float):
        """
        Update teleop delay estimate from a received command's wall-clock stamp.

        Call this each time a ControlCommand arrives.  Computes one-way
        latency as ``time.time() - cmd_wall_time`` and feeds an EMA filter.
        Also tracks the wall-clock of the most recent command for staleness.

        Args:
            cmd_wall_time: The ``wall_time`` field from the ControlCommand.
        """
        now = time.time()
        one_way = max(now - cmd_wall_time, 0.0)
        self._cmd_age = one_way
        self._last_cmd_wall = now  # record *when* we last got a command
        # Only update teleop delay estimate if teleop mode was explicitly
        # enabled at construction (delay > 0).  Local ZMQ latency (~1-2ms)
        # should NOT activate the teleop prediction / stale-command logic.
        if self._teleop_enabled:
            a = self._delay_ema_alpha
            self._delay_ema = a * one_way + (1.0 - a) * self._delay_ema
            self._teleop_delay = self._delay_ema

    def _effective_obstacle_buffer(self, v: float) -> float:
        """
        Compute obstacle buffer inflated by teleop round-trip delay.

        At speed *v* with one-way delay *tau*, the vehicle travels
        ``v * 2 * tau`` metres during the round-trip before the operator
        can react.  We add this distance (scaled by 0.5 for tuning head-room)
        to the static obstacle_buffer.

        Returns the effective buffer in metres.
        """
        if self._teleop_delay <= 0.0:
            return self.obstacle_buffer
        rtt = 2.0 * self._teleop_delay
        return self.obstacle_buffer + v * rtt * 0.5

    def _is_command_stale(self) -> bool:
        """True if no command received within stale_cmd_timeout (teleop only)."""
        if self._teleop_delay <= 0.0 or self._last_cmd_wall is None:
            return False
        age = time.time() - self._last_cmd_wall
        return age > self._stale_cmd_timeout

    def _init_csv_logging(self):
        """Initialize CSV files for safety filter diagnostics."""
        import csv
        import os
        # Per-run dir under parallel sweeps (set by benchmarking/common.py);
        # falls back to the historical global logs/ for live/manual runs.
        log_dir = os.environ.get('HIL_RUN_LOG_DIR') or os.path.join(
            os.path.dirname(__file__), '..', 'logs')
        os.makedirs(log_dir, exist_ok=True)

        # Main filter log: one row per filter() call
        main_path = os.path.join(log_dir, 'cbf_filter_log.csv')
        self._csv_file = open(main_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'step', 'x', 'y', 'psi_deg', 'v', 'beta_deg', 'hdv0',
            'n_obs', 'n_constraints', 'min_h',
            'steer_in', 'throttle_in', 'brake_in',
            'steer_out', 'throttle_out', 'brake_out',
            'alpha_cmd', 'alpha_out',
            'was_modified', 'active_constraints', 'solve_ms',
            'qp_success', 'v_max_terrain',
        ])

        # Per-obstacle log: one row per obstacle per filter() call
        obs_path = os.path.join(log_dir, 'cbf_obstacle_log.csv')
        self._obs_csv_file = open(obs_path, 'w', newline='')
        self._obs_csv_writer = csv.writer(self._obs_csv_file)
        self._obs_csv_writer.writerow([
            'step', 'obs_x', 'obs_y', 'obs_r', 'dist',
            'd_along', 'd_cross', 'd_along_unbiased',
            'h', 'h_dot', 'h_ddot_auto',
            'A_steer', 'A_alpha', 'psi0',
            'psi1_steer', 'psi1_alpha',
            'skipped_hdot', 'skipped_far',
        ])
        print(f"  [CBF] CSV logging to {log_dir}/cbf_*.csv")

    def _log_csv_obstacle(self, step, obs_x, obs_y, obs_r, dist,
                          d_along, d_cross, d_along_unbiased,
                          h, h_dot, h_ddot_auto,
                          A_steer, A_alpha, psi0, psi1_steer, psi1_alpha,
                          skipped_hdot, skipped_far):
        if self._obs_csv_writer:
            self._obs_csv_writer.writerow([
                step, f'{obs_x:.3f}', f'{obs_y:.3f}', f'{obs_r:.3f}', f'{dist:.2f}',
                f'{d_along:.3f}', f'{d_cross:.3f}', f'{d_along_unbiased:.3f}',
                f'{h:.4f}', f'{h_dot:.4f}', f'{h_ddot_auto:.4f}',
                f'{A_steer:.6f}', f'{A_alpha:.6f}', f'{psi0:.4f}',
                f'{psi1_steer:.6f}', f'{psi1_alpha:.6f}',
                int(skipped_hdot), int(skipped_far),
            ])

    def _log_csv_main(self, step, x, y, psi, v, beta, hdv0,
                      n_obs, n_constraints, min_h,
                      steer_in, throttle_in, brake_in,
                      steer_out, throttle_out, brake_out,
                      alpha_cmd, alpha_out,
                      was_modified, active_constraints, solve_ms,
                      qp_success, v_max_terrain):
        if self._csv_writer:
            self._csv_writer.writerow([
                step, f'{x:.3f}', f'{y:.3f}', f'{np.degrees(psi):.2f}',
                f'{v:.3f}', f'{np.degrees(beta):.2f}', f'{hdv0:.4f}',
                n_obs, n_constraints, f'{min_h:.4f}',
                f'{steer_in:.4f}', f'{throttle_in:.4f}', f'{brake_in:.4f}',
                f'{steer_out:.4f}', f'{throttle_out:.4f}', f'{brake_out:.4f}',
                f'{alpha_cmd:.4f}', f'{alpha_out:.4f}',
                int(was_modified), active_constraints, f'{solve_ms:.2f}',
                int(qp_success), f'{v_max_terrain:.2f}',
            ])
            # Flush periodically so we can read mid-run
            if step % 100 == 0:
                self._csv_file.flush()
                self._obs_csv_file.flush()

    def _compute_nn_tire_forces(self, u: float, v_lat: float, omega: float,
                                 delta: float, kappa: float = 0.0
                                 ) -> Optional[Dict[str, float]]:
        """
        Query NN tire model for front/rear Fx, Fy and derived quantities.

        Uses bicycle-model slip angles to query the NN at front and rear axles.
        Returns None if nn_casadi is not available.

        Returns dict with:
            Fx_total: Total longitudinal force (N), both axles, both sides
            Fy_total: Total lateral force (N), both axles, both sides
            Mz_yaw:   Yaw moment from lateral forces (N·m)
            ax_nn:     Longitudinal acceleration (m/s²)
            ay_nn:     Lateral acceleration (m/s²)
            alpha_dot: Yaw angular acceleration (rad/s²)
            Fy_f:      Front axle lateral force per wheel (N)
            Fy_r:      Rear axle lateral force per wheel (N)
            alpha_f:   Front slip angle (rad)
            alpha_r:   Rear slip angle (rad)
        """
        if self.nn_casadi is None:
            return None

        g = 9.81
        u_safe = max(u, 0.5)  # Avoid division by zero

        # Bicycle model slip angles
        alpha_f = delta - np.arctan2(v_lat + self.Lf * omega, u_safe)
        alpha_r = -np.arctan2(v_lat - self.Lr * omega, u_safe)

        # Clamp slip angles to training-data range
        _alpha_max = 0.55
        alpha_f = float(max(-_alpha_max, min(_alpha_max, alpha_f)))
        alpha_r = float(max(-_alpha_max, min(_alpha_max, alpha_r)))

        # Normal forces (static weight distribution, per wheel)
        Fz_f = self.M * g * self.Lr / self.L / 2.0
        Fz_r = self.M * g * self.Lf / self.L / 2.0

        try:
            Fx_f, Fy_f = self.nn_casadi.predict_numeric(alpha_f, Fz_f, u_safe, kappa)
            Fx_r, Fy_r = self.nn_casadi.predict_numeric(alpha_r, Fz_r, u_safe, kappa)

            # Total forces (×2 for left+right wheels)
            Fx_total = 2.0 * (Fx_f + Fx_r)
            Fy_total = 2.0 * (Fy_f + Fy_r)
            # Yaw moment: front pushes one way, rear the other
            Mz_yaw = 2.0 * (self.Lf * Fy_f - self.Lr * Fy_r)

            return {
                'Fx_total': Fx_total,
                'Fy_total': Fy_total,
                'Mz_yaw': Mz_yaw,
                'ax_nn': Fx_total / self.M,
                'ay_nn': Fy_total / self.M,
                'alpha_dot': Mz_yaw / self.Izz,
                'Fy_f': Fy_f,
                'Fy_r': Fy_r,
                'alpha_f': alpha_f,
                'alpha_r': alpha_r,
            }
        except Exception:
            return None

    def _compute_nn_steering_sensitivity(self, u: float, v_lat: float,
                                          omega: float, delta: float
                                          ) -> Optional[Tuple[float, float]]:
        """
        Estimate dFy/ddelta and dMz/ddelta via finite difference on the NN.

        Returns (dFy_ddelta, dMz_ddelta) or None if NN unavailable.
        These tell the CBF how much lateral force / yaw moment one unit of
        steering rate buys — the steering control authority.
        """
        if self.nn_casadi is None:
            return None

        eps = 0.005  # ~0.3 deg perturbation
        forces_plus = self._compute_nn_tire_forces(u, v_lat, omega, delta + eps)
        forces_minus = self._compute_nn_tire_forces(u, v_lat, omega, delta - eps)

        if forces_plus is None or forces_minus is None:
            return None

        dFy_ddelta = (forces_plus['Fy_total'] - forces_minus['Fy_total']) / (2 * eps)
        dMz_ddelta = (forces_plus['Mz_yaw'] - forces_minus['Mz_yaw']) / (2 * eps)

        return dFy_ddelta, dMz_ddelta

    def _compute_reactive_steering(self, vehicle_state: dict,
                                    obstacles: List[Tuple[float, float, float]]) -> float:
        """
        Compute a reactive steering avoidance command for nearby obstacles.

        The second-order CBF cannot steer away from head-on obstacles because
        the barrier gradient is orthogonal to the steering effect (geometric
        singularity). This layer handles steering directly: for each nearby
        obstacle ahead, it contributes a steering command away from it,
        proportional to proximity.

        Returns:
            Steering adjustment in [-1, 1] (normalized).
        """
        x = vehicle_state.get('x', 0.0)
        y = vehicle_state.get('y', 0.0)
        psi = vehicle_state.get('psi', 0.0)
        v = vehicle_state.get('u', 0.0)

        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)

        steer_cmd = 0.0

        for (obs_x, obs_y, obs_r) in obstacles:
            # Transform obstacle to body frame
            dx_world = obs_x - x
            dy_world = obs_y - y
            dx_body = dx_world * cos_psi + dy_world * sin_psi   # forward
            dy_body = -dx_world * sin_psi + dy_world * cos_psi  # left-positive

            # Only care about obstacles ahead
            if dx_body < 1.0:
                continue

            safe_r = obs_r + self.vehicle_radius + self.obstacle_buffer
            dist = np.sqrt(dx_body**2 + dy_body**2)

            # Intervention range: from safe_r out to react_range
            react_range = safe_r + 8.0 + v * 0.5  # scales slightly with speed
            if dist > react_range:
                continue

            # Proximity: 0 at react_range, 1 at safe_r
            proximity = np.clip(1.0 - (dist - safe_r) / max(react_range - safe_r, 0.1), 0.0, 1.0)

            # Steer AWAY from obstacle
            # dy_body > 0 → obstacle to the left → steer right (negative normalized steering)
            # dy_body < 0 → obstacle to the right → steer left (positive normalized steering)
            if abs(dy_body) > 0.3:
                direction = -np.sign(dy_body)
            else:
                # Nearly head-on: pick the side with more clearance, default right
                direction = -1.0

            steer_cmd += direction * proximity * 0.5

        return np.clip(steer_cmd, -1.0, 1.0)

    def filter(self,
               desired_steering: float,
               desired_throttle: float,
               desired_brake: float,
               vehicle_state: dict,
               obstacles: List[Tuple[float, float, float]] = None,
               terrain_roughness: float = 0.0) -> SafetyFilterResult:
        """
        Apply DOB-CBF safety filter to desired control inputs.

        Args:
            desired_steering: Desired steering input [-1, 1] (normalized)
            desired_throttle: Desired throttle input [0, 1]
            desired_brake: Desired brake input [0, 1]
            vehicle_state: Dict with keys:
                'x', 'y': world position (m)
                'psi': heading angle (rad)
                'u': longitudinal velocity (m/s)
                'v': lateral velocity (m/s)
                'omega': yaw rate (rad/s)
                'delta': current steering angle (rad)
            obstacles: List of (x, y, radius) tuples
            terrain_roughness: Terrain roughness metric (m)

        Returns:
            SafetyFilterResult with filtered inputs and diagnostics.
        """
        t_start = time.time()
        self._filter_count += 1

        if obstacles is None:
            obstacles = []

        # Extract vehicle state
        x = vehicle_state.get('x', 0.0)
        y = vehicle_state.get('y', 0.0)
        psi = vehicle_state.get('psi', 0.0)
        v = max(vehicle_state.get('u', 0.5), 0.1)  # Avoid /0
        v_lat = vehicle_state.get('v', 0.0)
        omega = vehicle_state.get('omega', 0.0)
        delta = vehicle_state.get('delta', 0.0)

        # Update internal beta from measured steering angle
        self._beta = delta

        # Teleop: stale command detection — emergency brake if no recent cmds
        if self._is_command_stale():
            self._modify_count += 1
            safe_result = SafetyFilterResult(
                steering=desired_steering,  # hold last steering
                throttle=0.0,
                braking=1.0,
                was_modified=True,
                active_constraints=0,
                solve_time_ms=0.0,
                v_max_terrain=0.0,
                safety_margin=0.0,
                dob_norm=0.0,
            )
            self._last_result = safe_result
            if self._filter_count % 20 == 0:
                age = time.time() - self._last_cmd_wall if self._last_cmd_wall else float('inf')
                print(f"  [CBF #{self._filter_count}] STALE COMMAND — age={age:.2f}s > "
                      f"timeout={self._stale_cmd_timeout:.1f}s  ** EMERGENCY BRAKE **")
            return safe_result

        # Teleop: compute delay-inflated obstacle buffer for this step
        effective_buffer = self._effective_obstacle_buffer(v)

        # Apply delay compensation
        comp_throttle, comp_steering = self.delay_comp.update(
            desired_throttle, desired_steering
        )

        # Compute net throttle command for DOB: alpha in [-1, 1]
        if desired_brake > 0.05:
            alpha_cmd = -desired_brake
        else:
            alpha_cmd = desired_throttle

        # Query NN tire model for current state (used by DOB + CBF)
        nn_forces = self._compute_nn_tire_forces(v, v_lat, omega, delta)
        nn_steer_sens = self._compute_nn_steering_sensitivity(v, v_lat, omega, delta)

        # NN-informed nominal acceleration for DOB
        f_nom_nn = nn_forces['ax_nn'] if nn_forces is not None else None

        # Update DOB (with NN nominal model if available)
        hdv0 = self.dob.update(v, alpha_cmd, self.control_dt,
                               f_nom_override=f_nom_nn)

        # Terrain-aware speed limit
        v_max_terrain = self._compute_terrain_speed_limit(v, terrain_roughness, delta)

        # ---- Position-based QP formulation ----
        # QP variables: u = [steer_out, alpha_out]
        #   steer_out: normalized steering position [-1, 1]
        #   alpha_out: throttle/brake [-1, 1]
        # u_desired = [desired_steering, alpha_cmd]
        #
        # This avoids the rate-based (dbeta) formulation where the kblf
        # feedback loop fights CBF corrections, causing the QP to always
        # prefer braking over steering.

        desired_alpha = alpha_cmd
        u_desired = np.array([desired_steering, desired_alpha])

        # Current road-wheel angle for barrier linearization
        # Use last applied steering as best estimate of current vehicle state
        current_beta = self._beta

        # Build heading-aligned weight matrix P
        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)
        R = np.array([[cos_psi, sin_psi],
                       [-sin_psi, cos_psi]])
        Q_barrier = np.diag([self.w_long, self.w_lat])
        P = R.T @ Q_barrier @ R

        # NN-informed dynamics for CBF constraints
        # Autonomous lateral acceleration and yaw moment from NN
        if nn_forces is not None:
            # NN provides actual tire-generated accelerations at current state
            ay_tire = nn_forces['ay_nn']          # lateral accel from Fy (m/s^2)
            alpha_dot_tire = nn_forces['alpha_dot']  # yaw accel from Mz (rad/s^2)
        else:
            ay_tire = 0.0
            alpha_dot_tire = 0.0

        # Steering sensitivity: how much lateral force / yaw per unit delta change
        if nn_steer_sens is not None:
            dFy_ddelta, dMz_ddelta = nn_steer_sens
            # Convert to accelerations per unit dbeta (dbeta = ddelta/dt, so
            # sensitivity is per unit delta; multiply by control_dt handled in QP)
            day_ddelta = dFy_ddelta / self.M
            dalpha_dot_ddelta = dMz_ddelta / self.Izz
        else:
            # Kinematic fallback: d(omega)/dt ~ v/L * ddelta
            day_ddelta = 0.0
            dalpha_dot_ddelta = v / self.L

        # Longitudinal control authority (throttle -> Fx -> ax)
        if nn_forces is not None:
            # NN-based: query at slightly higher/lower throttle to get sensitivity
            # Use current ax_nn as baseline; scale by throttle fraction
            ax_nn = nn_forces['ax_nn']
            if abs(ax_nn) > 0.01:
                accel_gain = abs(ax_nn / max(abs(alpha_cmd), 0.1))
            else:
                accel_gain = self.max_accel if desired_alpha >= 0 else abs(self.max_decel)
        else:
            accel_gain = self.max_accel if desired_alpha >= 0 else abs(self.max_decel)

        # Build CBF constraints for each nearby obstacle
        A_ineq_list = []
        b_ineq_list = []
        min_h = float('inf')
        self._pending_obs_logs = []

        # Expand obstacle filtering range to cover delay-inflated buffer
        delay_inflate = effective_buffer - self.obstacle_buffer
        r_precpt_eff = self.r_precpt + delay_inflate

        for (obs_x, obs_y, obs_r) in obstacles:
            # Distance check -- skip far obstacles
            dd = (x - obs_x)**2 + (y - obs_y)**2
            if dd > r_precpt_eff**2:
                continue

            safe_r = obs_r + self.vehicle_radius + effective_buffer

            # ----------------------------------------------------------
            # Position-based CBF: steering enters through h_dot (1st
            # derivative) via omega = v/L * tan(beta).  Throttle enters
            # through h_ddot (2nd derivative) via acceleration.
            #
            # Constraint (2nd-order CBF):
            #   h_ddot + alpha2 * h_dot(steer_out) + alpha1 * h >= 0
            #
            # h_dot is linearized around current_beta:
            #   h_dot(s) ≈ h_dot_0 + A_steer * (s - s_current)
            # where s is the normalized steering output and
            #   A_steer = dh_dot/ds = dh_dot/d(omega) * d(omega)/d(s)
            # ----------------------------------------------------------

            beta = current_beta
            w1 = self.w_long
            w2 = self.w_lat
            bias = self.forward_bias

            # Body-frame decomposition
            raw_dx = x - obs_x
            raw_dy = y - obs_y
            d_along = cos_psi * raw_dx + sin_psi * raw_dy + bias
            d_cross = -sin_psi * raw_dx + cos_psi * raw_dy

            # Barrier value
            h = w1 * d_along**2 + w2 * d_cross**2 - safe_r**2
            min_h = min(min_h, h)

            # Euclidean distance from vehicle CG to obstacle center
            dist_eucl = np.sqrt(dd)

            # Kinematic quantities at current beta
            # Use actual measured yaw rate for barrier dynamics — the
            # kinematic bicycle model (omega_kin = v/L*tan(delta)) is
            # inaccurate on low-friction surfaces where tire slip causes
            # the real yaw rate to diverge from the kinematic prediction.
            d_along_unbiased = d_along - bias

            d_along_dot = v + omega * d_cross
            d_cross_dot = v_lat - omega * d_along_unbiased

            h_dot = 2 * w1 * d_along * d_along_dot + 2 * w2 * d_cross * d_cross_dot

            # Skip obstacles we're moving away from (h increasing and already safe)
            if h_dot > 0 and h > 0:
                continue

            # Skip obstacles that are BEHIND the vehicle (already passed).
            # d_along_unbiased < 0 means obstacle is ahead; > 0 means behind.
            if d_along_unbiased > 1.0:
                continue

            # Skip obstacles we're moving away from even with h < 0
            # (we already passed but barrier still overlaps)
            if h_dot > 0 and d_along_unbiased > -0.5:
                continue

            # Clamp h from below to prevent deadlock when deep inside barrier.
            # Physical collision radius is safe_r; barrier violation (h < 0)
            # just means we're inside the safety margin, not colliding.
            # Use a softer constraint: only require h_dot >= 0 (stop getting
            # closer) instead of full recovery when deep in the barrier.
            if h < 0 and dist_eucl > safe_r:
                # Inside barrier but not colliding — use first-order constraint
                # only: require h_dot + alpha1 * h_clamped >= 0
                # This gives a "slow down and steer" rather than "emergency brake"
                h_eff = max(h, -0.5 * safe_r**2)
            else:
                h_eff = h

            # Steering sensitivity: dh_dot / d(steer_normalized)
            # Uses kinematic model for control authority (how steering
            # changes omega), which is a vehicle design property.
            tan_beta = np.tan(beta) if abs(beta) < 1.5 else np.sign(beta) * 1e3
            sec2_beta = 1.0 + tan_beta**2
            d_omega_d_steer = v / self.L * sec2_beta * self.max_road_steer_angle

            # dh_dot/d(omega) = 2*d_cross*(w1*d_along - w2*d_along_unbiased)
            dh_dot_d_omega = 2 * d_cross * (w1 * d_along - w2 * d_along_unbiased)

            A_steer = dh_dot_d_omega * d_omega_d_steer

            # Autonomous h_ddot (second derivative at constant control)
            omega_dot_auto = hdv0 / self.L * tan_beta if abs(v) > 0.1 else 0.0
            d_along_ddot_auto = (hdv0
                                 + d_cross_dot * omega
                                 + d_cross * omega_dot_auto)
            d_cross_ddot_auto = (-d_along_dot * omega
                                 - d_along_unbiased * omega_dot_auto)
            # Drop velocity² (centrifugal) terms from h_ddot_auto.
            # The full chain-rule expansion of d²h/dt² includes
            # d_along_dot² and d_cross_dot² which are always non-negative.
            # These inflate h_ddot_auto — especially d_cross_dot² during
            # turns — making the QP constraint non-binding: the CBF
            # "thinks" the barrier is naturally improving when the vehicle
            # is merely rotating the body frame, not actually escaping.
            # Keeping only the position×acceleration terms makes the
            # constraint conservative enough to trigger braking/steering
            # before it's too late.
            h_ddot_auto = (2 * w1 * d_along * d_along_ddot_auto
                         + 2 * w2 * d_cross * d_cross_ddot_auto)

            # Throttle sensitivity: alpha affects h_ddot through acceleration
            A_alpha = 2 * w1 * d_along * accel_gain

            # Position-based CBF constraint using effective barrier:
            # h_ddot_auto + A_alpha*alpha + alpha2*(h_dot_0 + A_steer*(s - s_cur)) + alpha1*h_eff >= 0
            #
            # Rearrange to: -[alpha2*A_steer, A_alpha] @ [s, alpha] <= psi0
            s_current = self._beta / self.max_road_steer_angle
            psi0 = (h_ddot_auto
                    + self.alpha2 * (h_dot - A_steer * s_current)
                    + self.alpha1 * h_eff)
            psi1 = np.array([self.alpha2 * A_steer, A_alpha])

            A_ineq_list.append(-psi1)
            b_ineq_list.append(psi0)

            # Buffer obstacle data for CSV (logged only if intervention occurs)
            self._pending_obs_logs.append((
                self._filter_count, obs_x, obs_y, obs_r, np.sqrt(dd),
                d_along, d_cross, d_along_unbiased,
                h, h_dot, h_ddot_auto,
                A_steer, A_alpha, psi0,
                psi1[0], psi1[1],
            ))

        # Speed limit constraint (terrain-aware)
        if v > v_max_terrain * 0.9:
            speed_margin = v_max_terrain - v
            accel_for_speed = accel_gain
            A_ineq_list.append(np.array([0.0, -accel_for_speed]))
            b_ineq_list.append(self.alpha1 * speed_margin)

        # Low-speed recovery: if vehicle is nearly stopped and min_h < 0
        # but no real collision (Euclidean distance > safe_r for all nearby
        # obstacles), drop CBF constraints and rely on reactive steering.
        # This prevents the vehicle from getting permanently stuck inside
        # the barrier zone when surrounded by multiple obstacles.
        # Use hysteresis: enter creep at v < 1.0, exit at v > 3.0
        if not hasattr(self, '_creep_mode'):
            self._creep_mode = False
        if min_h < -0.1:
            if v < 1.0:
                self._creep_mode = True
            elif v > 3.0:
                self._creep_mode = False
        else:
            self._creep_mode = False

        if self._creep_mode and len(A_ineq_list) > 0:
            # Check if ANY obstacle is actually within physical collision radius
            physical_collision = False
            for (obs_x, obs_y, obs_r) in obstacles:
                dd = (x - obs_x)**2 + (y - obs_y)**2
                hard_safe_r = obs_r + self.vehicle_radius
                if dd < hard_safe_r**2:
                    physical_collision = True
                    break
            if not physical_collision:
                # Safe to creep: replace CBF constraints with a speed limit
                A_ineq_list.clear()
                b_ineq_list.clear()
                # Limit creeping speed to 3 m/s
                creep_max = 3.0
                if v > creep_max * 0.9:
                    A_ineq_list.append(np.array([0.0, -accel_gain]))
                    b_ineq_list.append(self.alpha1 * (creep_max - v))

        # Solve QP
        was_modified = False
        active_constraints = 0
        qp_success = True

        if len(A_ineq_list) > 0:
            A_ineq = np.array(A_ineq_list)
            b_ineq = np.array(b_ineq_list)

            # Actuator limits: steer_out in [-1, 1], alpha_out in [-1, 1]
            A_limits = np.array([
                [1.0, 0.0],   # steer_out <= 1
                [-1.0, 0.0],  # -steer_out <= 1
                [0.0, 1.0],   # alpha_out <= 1
                [0.0, -1.0],  # -alpha_out <= 1
            ])
            b_limits = np.array([1.0, 1.0, 1.0, 1.0])

            A_all = np.vstack([A_ineq, A_limits])
            b_all = np.hstack([b_ineq, b_limits])

            # QP cost: min ||u - u_desired||^2
            # u = [steer_out, alpha_out], u_desired = [desired_steering, alpha_cmd]
            if self.cbf_flavor == 'balance':
                # Steering 10x cheaper than throttle: QP prefers to steer
                # around obstacles rather than brake
                W = np.diag([1.0, 10.0])
                H = 2.0 * W
                f = -2.0 * W @ u_desired
            elif self.cbf_flavor == 'steer_priority':
                W = np.diag([500.0, 1.0])
                H = 2.0 * W
                f = -2.0 * W @ u_desired
            elif self.cbf_flavor == 'throttle_priority':
                W = np.diag([1.0, 500.0])
                H = 2.0 * W
                f = -2.0 * W @ u_desired
            else:
                H = 2.0 * np.eye(2)
                f = -2.0 * u_desired

            def qp_objective(u_var):
                return 0.5 * u_var @ H @ u_var + f @ u_var

            constraints = [{
                'type': 'ineq',
                'fun': lambda u_var: b_all - A_all @ u_var
            }]

            result = minimize(
                qp_objective, u_desired,
                method='SLSQP',
                constraints=constraints,
                options={'maxiter': 50, 'ftol': 1e-8}
            )

            if result.success:
                u_safe = result.x
                diff = np.linalg.norm(u_safe - u_desired)
                was_modified = diff > 1e-4
                if was_modified:
                    self._modify_count += 1
                    active_constraints = sum(
                        1 for i in range(len(A_ineq))
                        if A_ineq[i] @ u_safe > b_ineq[i] - 1e-3
                    )
            else:
                # Infeasible: emergency brake, hold steering
                u_safe = np.array([desired_steering, -1.0])
                was_modified = True
                qp_success = False
                self._modify_count += 1
                active_constraints = len(A_ineq_list)
        else:
            u_safe = u_desired

        # Extract filtered controls directly (position-based, no integration)
        safe_steering = np.clip(u_safe[0], -1.0, 1.0)
        safe_alpha = u_safe[1]

        # Reactive steering layer: add avoidance commands for nearby obstacles.
        # This handles head-on obstacles where the CBF geometric singularity
        # prevents the QP from choosing steering, and provides guidance during
        # low-speed creeping through obstacle fields.
        if len(obstacles) > 0 and min_h < 2.0:
            reactive_steer = self._compute_reactive_steering(vehicle_state, obstacles)
            if abs(reactive_steer) > 0.01:
                safe_steering = np.clip(safe_steering + reactive_steer, -1.0, 1.0)
                if not was_modified:
                    was_modified = True
                    self._modify_count += 1

        # Teleop: forward-predict over delay horizon and emergency-brake
        # if the QP-safe output still leads to a collision within the RTT
        if self._teleop_delay > 0.0 and len(obstacles) > 0:
            pred_horizon = 2.0 * self._teleop_delay + 0.3  # RTT + reaction
            pred_steps = max(int(pred_horizon / 0.1), 1)
            preds = self.predict_state(
                vehicle_state, safe_steering,
                max(safe_alpha, 0.0), dt=0.1, steps=pred_steps)
            collision, t_collide = self.check_predicted_collision(preds, obstacles)
            if collision:
                # Override to emergency brake; keep steering
                safe_alpha = -1.0
                was_modified = True
                if not qp_success or active_constraints == 0:
                    active_constraints = 1

        # Track applied steering for next call's linearization point
        self._beta = safe_steering * self.max_road_steer_angle
        self._alpha = safe_alpha

        if safe_alpha >= 0:
            safe_throttle = min(safe_alpha, 1.0)
            safe_brake = 0.0
        else:
            safe_throttle = 0.0
            safe_brake = min(abs(safe_alpha), 1.0)

        # --- Rear-approach avoidance ---
        # A vehicle closing from behind cannot always be avoided, but the worst
        # response is to slow down. Find the nearest obstacle BEHIND us (in our
        # lane) and, if it is closing inside the threat distance, refuse any
        # deceleration and -- if the path ahead is clear -- accelerate to escape
        # (ramping with proximity), even from a standstill.
        rear_dist = float('inf')
        fwd_dist = float('inf')
        cps, sps = np.cos(psi), np.sin(psi)
        for (obs_x, obs_y, obs_r) in obstacles:
            rx, ry = obs_x - x, obs_y - y
            lon = rx * cps + ry * sps            # +forward in body frame
            lat = -rx * sps + ry * cps
            d = float(np.hypot(rx, ry)) - obs_r
            if abs(lat) < self._rear_half_w:
                if lon < -1.0:
                    rear_dist = min(rear_dist, d)
                elif lon > 0.0:
                    fwd_dist = min(fwd_dist, d)
        rear_closing = rear_dist < self._prev_rear_dist - 0.02
        self._prev_rear_dist = rear_dist if np.isfinite(rear_dist) else float('inf')
        if rear_dist < self._rear_threat_dist and rear_closing:
            safe_brake = 0.0                     # never brake into a rear approach
            if fwd_dist > self._rear_threat_dist:  # path ahead clear -> accelerate away
                escape = float(np.clip(1.2 - rear_dist / self._rear_threat_dist, 0.4, 1.0))
                safe_throttle = max(safe_throttle, desired_throttle, escape)
            else:                                  # blocked ahead -> at least don't slow
                safe_throttle = max(safe_throttle, desired_throttle)
            was_modified = True

        solve_time = (time.time() - t_start) * 1000

        self._last_result = SafetyFilterResult(
            steering=safe_steering,
            throttle=safe_throttle,
            braking=safe_brake,
            was_modified=was_modified,
            active_constraints=active_constraints,
            solve_time_ms=solve_time,
            v_max_terrain=v_max_terrain,
            safety_margin=min_h if min_h < float('inf') else float('inf'),
            dob_norm=abs(hdv0),
        )

        # CSV log only when filter intervenes
        if was_modified:
            self._log_csv_main(
                self._filter_count, x, y, psi, v, self._beta, hdv0,
                len(obstacles), len(A_ineq_list), min_h,
                desired_steering, desired_throttle, desired_brake,
                safe_steering, safe_throttle, safe_brake,
                alpha_cmd, safe_alpha,
                was_modified, active_constraints, solve_time,
                qp_success, v_max_terrain,
            )
            # Flush buffered obstacle logs for this intervention
            for obs_log in self._pending_obs_logs:
                self._log_csv_obstacle(*obs_log, skipped_hdot=False, skipped_far=False)

        # Diagnostic logging (only when filter intervenes)
        if was_modified and self._filter_count % 20 == 0:
            n_obs = len(obstacles)
            mod_tag = " ** MODIFIED **" if was_modified else ""
            n_constraints = len(A_ineq_list)
            delay_tag = f" delay={self._teleop_delay*1000:.0f}ms buf={effective_buffer:.2f}m" if self._teleop_delay > 0 else ""
            print(f"  [CBF #{self._filter_count}] v={v:.2f} pos=({x:.1f},{y:.1f}) psi={np.degrees(psi):.1f}°"
                  f" | obs={n_obs} constraints={n_constraints} min_h={min_h:.2f} dob={hdv0:+.2f}"
                  f" | IN steer={desired_steering:+.3f} thr={desired_throttle:.3f} brk={desired_brake:.3f}"
                  f" | OUT steer={safe_steering:+.3f} thr={safe_throttle:.3f} brk={safe_brake:.3f}"
                  f" | alpha_cmd={alpha_cmd:+.3f} -> safe_alpha={safe_alpha:+.3f}"
                  f" | v_max_t={v_max_terrain:.1f}{delay_tag}{mod_tag}")

        return self._last_result

    def _compute_terrain_speed_limit(self, speed: float, roughness: float,
                                      delta: float) -> float:
        """
        Compute terrain-aware maximum safe speed.

        Uses terrain roughness and (optionally) NN tire force
        predictions to determine the maximum speed that maintains vehicle
        stability on the current terrain.
        """
        # Base speed limit from terrain roughness
        if roughness > 0.01:
            roughness_factor = 1.0 / (1.0 + 10.0 * roughness)
            v_max_rough = self.max_speed * roughness_factor
        else:
            v_max_rough = self.max_speed

        # NN tire model traction check (if available)
        v_max_traction = self.max_speed
        if self.nn_casadi is not None:
            v_max_traction = self._nn_traction_speed_limit(speed, delta)

        return max(min(v_max_rough, v_max_traction, self.max_speed), 2.0)

    def _nn_traction_speed_limit(self, speed: float, delta: float) -> float:
        """Use NN tire model to estimate maximum speed before traction loss.
        
        Uses a fixed moderate steering angle (typical evasion maneuver) rather
        than the driver's current steering angle, so the speed limit reflects
        traction budget for emergency avoidance, not normal cornering.
        """
        g = 9.81
        Fz_f = self.M * g * self.Lr / self.L / 2.0
        Fz_r = self.M * g * self.Lf / self.L / 2.0

        alpha_max = 0.15  # ~8.6 deg

        try:
            _, Fy_f = self.nn_casadi.predict_numeric(alpha_max, Fz_f, max(speed, 2.0), 0.0)
            _, Fy_r = self.nn_casadi.predict_numeric(alpha_max, Fz_r, max(speed, 2.0), 0.0)

            Fy_max = 2.0 * (abs(Fy_f) + abs(Fy_r))
            ay_max = Fy_max / self.M

            # Use a fixed moderate evasion angle (~5 deg) to set the speed
            # limit.  This avoids penalizing the driver's actual steering.
            evasion_delta = 0.09  # ~5 deg road wheel
            R = self.L / max(np.tan(evasion_delta), 0.01)
            v_max = np.sqrt(ay_max * R)

            return min(v_max, self.max_speed)

        except Exception:
            return self.max_speed

    def predict_state(self, state: dict, steering: float, throttle: float,
                      dt: float = 0.1, steps: int = 5) -> List[dict]:
        """Forward-simulate vehicle state for look-ahead collision checking."""
        x = state.get('x', 0.0)
        y_pos = state.get('y', 0.0)
        psi = state.get('psi', 0.0)
        u = state.get('u', 0.5)

        delta = steering * self.max_road_steer_angle
        ax = 3.0 * throttle

        predictions = []
        for _ in range(steps):
            x += u * np.cos(psi) * dt
            y_pos += u * np.sin(psi) * dt
            psi += (u / self.L) * np.tan(delta) * dt
            u += ax * dt
            u = max(u, 0.0)

            predictions.append({
                'x': x, 'y': y_pos, 'psi': psi, 'u': u
            })

        return predictions

    def check_predicted_collision(self, predictions: List[dict],
                                   obstacles: List[Tuple[float, float, float]]) -> Tuple[bool, float]:
        """Check if predicted trajectory collides with any obstacle."""
        for i, pred in enumerate(predictions):
            for obs_x, obs_y, obs_r in obstacles:
                safe_r = obs_r + self.vehicle_radius + self.obstacle_buffer
                dist_sq = (pred['x'] - obs_x)**2 + (pred['y'] - obs_y)**2
                if dist_sq < safe_r**2:
                    return True, i * 0.1
        return False, float('inf')

    @property
    def intervention_rate(self) -> float:
        """Fraction of filter calls that modified the input (0-1)."""
        if self._filter_count == 0:
            return 0.0
        return self._modify_count / self._filter_count

    def get_diagnostics(self) -> dict:
        """Get diagnostic info for logging."""
        result = self._last_result
        return {
            'filter_calls': self._filter_count,
            'interventions': self._modify_count,
            'intervention_rate': self.intervention_rate,
            'last_solve_ms': result.solve_time_ms if result else 0.0,
            'last_modified': result.was_modified if result else False,
            'last_v_max_terrain': result.v_max_terrain if result else self.max_speed,
            'last_safety_margin': result.safety_margin if result else float('inf'),
            'last_active_constraints': result.active_constraints if result else 0,
            'last_dob_norm': result.dob_norm if result else 0.0,
        }


# ============================================================================
# Predictive shields (MPPI and NMPC comparison) — see predictive_shield.py
# ============================================================================

from .predictive_shield import MPPIShield, NMPCShield  # noqa: E402

SAFETY_FLAVORS = ('mppi', 'nmpc', 'dob_cbf')


def make_safety_filter(flavor: str,
                       vehicle_params: dict,
                       nn_model=None,
                       terrain_params: dict | None = None,
                       **flavor_kwargs):
    """Factory for the three safety-filter flavors.

    Args:
        flavor: one of ``SAFETY_FLAVORS``.  ``'dob_cbf'`` is the
            minimum-deviation, intent-preserving CBF filter;
            ``'mppi'`` is the predictive learned-dynamics shield; and
            ``'nmpc'`` is the gradient-based finite-horizon comparison.
        vehicle_params: dict with ``M, Lf, Lr, Izz, ...``.
        nn_model: a loaded ``NNTireModel`` (required for ``mppi`` and
            ``nmpc``; optional for ``dob_cbf`` — falls back to kinematic).
        terrain_params: terrain preset dict (``Kphi, Kc, n, c, phi`` in
            **degrees**, ``k``).  Required for ``mppi`` and ``nmpc``.
        **flavor_kwargs: forwarded verbatim to the chosen filter's
            constructor.  The caller is responsible for using
            flavor-appropriate keys (see each class docstring).

    Returns:
        A filter instance exposing ``.filter(...)``,
        ``.update_command_age(...)``, ``.set_teleop_delay(...)``,
        and ``.get_diagnostics()`` — interchangeable across flavors.
    """
    f = (flavor or '').lower()
    if f in ('mppi', 'mppi_shield'):
        if nn_model is None or terrain_params is None:
            raise ValueError("flavor='mppi' requires nn_model and terrain_params")
        return MPPIShield(vehicle_params=vehicle_params,
                          nn_model=nn_model,
                          terrain_params=terrain_params,
                          **flavor_kwargs)
    if f in ('nmpc', 'nmpc_shield'):
        if nn_model is None or terrain_params is None:
            raise ValueError("flavor='nmpc' requires nn_model and terrain_params")
        return NMPCShield(vehicle_params=vehicle_params,
                          nn_model=nn_model,
                          terrain_params=terrain_params,
                          **flavor_kwargs)
    if f in ('dob_cbf', 'cbf', 'legacy', 'dob-cbf'):
        return CBFSafetyFilter(vehicle_params=vehicle_params,
                               nn_casadi=nn_model,
                               **flavor_kwargs)
    raise ValueError(f"Unknown safety flavor {flavor!r}; "
                     f"expected one of {SAFETY_FLAVORS}")
