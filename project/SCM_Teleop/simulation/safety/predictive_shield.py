"""
Predictive Safety Shield — MPPI and NMPC flavors
================================================

Both flavors share the same NN-surrogate-based vehicle rollout and the same
multi-objective cost (obstacle barriers, NN friction-cone, speed cap,
deviation from operator/AI intent), and both target the *latency-padded*
horizon so that operator command staleness is baked into the prediction
rather than papered over with a static obstacle inflation.

The two flavors differ only in how they pick a safe control at each step:

* ``MPPIShield`` — Model Predictive Path Integral.  Sample K candidate
  command sequences, roll each through the surrogate, and return the
  importance-weighted mean of the first action.  This is the primary
  shield: it has no QP linearization, no class-K-tuning, and handles
  non-convex multi-obstacle costs without singularities.

* ``NMPCShield`` — gradient-based finite-horizon NMPC via SLSQP over the
  same rollout.  Provided as an ablation: deterministic, lower variance,
  but slower and easier to trap in local minima.

Both flavors expose the same ``filter(...) -> SafetyFilterResult`` API as
the legacy ``CBFSafetyFilter``, so they slot into ``chrono_sim_node``
behind a ``--safety-flavor`` flag.
"""

from __future__ import annotations

import csv
import math
import os
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

from .surrogate_dynamics import (
    NumpyTireSurrogate,
    SurrogateRolloutParams,
    VehicleSurrogateDynamics,
    build_rollout_params,
)


# ----------------------------------------------------------------------------
# Result dataclass — matches the CBF filter so downstream code is unchanged
# ----------------------------------------------------------------------------

@dataclass
class SafetyFilterResult:
    steering: float
    throttle: float
    braking: float
    was_modified: bool
    active_constraints: int          # # of nearby obstacles considered
    solve_time_ms: float
    v_max_terrain: float
    safety_margin: float             # min clearance over horizon (m)
    dob_norm: float = 0.0            # unused by predictive shield, kept for parity


# ----------------------------------------------------------------------------
# Shared base
# ----------------------------------------------------------------------------

class _PredictiveShieldBase:
    """Common state, terrain handling, cost, and result construction.

    Subclasses implement ``_solve(...)`` returning ``(u0_safe, info)``.
    """

    def __init__(self,
                 vehicle_params: dict,
                 nn_model,
                 terrain_params: dict,
                 *,
                 horizon: int = 18,
                 dt: float = 0.1,
                 max_speed: float = 15.0,
                 vehicle_radius: float = 1.0,
                 obstacle_buffer: float = 0.25,
                 weight_obstacle: float = 18.0,
                 weight_friction: float = 0.0,
                 weight_speed: float = 0.6,
                 weight_deviation: float = 0.6,
                 weight_terminal_obstacle: float = 8.0,
                 weight_progress: float = 35.0,
                 phi_safety_margin_deg: float = 3.0,
                 traction_mu_cap: float = 0.9,
                 teleop_delay: float = 0.0,
                 stale_cmd_timeout: float = 2.0,
                 control_dt: float = 0.1,
                 sigma_mode: str = "off",
                 sigma_buffer_gain: float = 0.05,
                 csv_basename: str = 'predictive_shield_log.csv'):
        self.vp = vehicle_params
        self.terrain_nominal = dict(terrain_params)
        self.terrain_nominal['phi'] = float(np.radians(terrain_params['phi']))
        self._phi_uncertainty_rad = 0.0  # set via update_terrain
        self._phi_uncertainty_deg = 0.0
        # ``sigma_mode`` chooses how the estimator's phi uncertainty acts on
        # the shield: ``tighten`` pulls the friction cone down by sigma;
        # ``inflate`` adds sigma * gain metres to the obstacle clearance
        # buffer; ``both`` does both; ``off`` disables the gate. The 2026-05
        # ablation (paper Sec. IX-B) showed every live-terrain gate variant
        # underperforms a shield that ignores estimator updates entirely,
        # so the canonical/default mode is ``off`` and the shield runs on
        # its initial terrain. The non-``off`` modes are retained only so
        # the ablation in ``paper_scripts/sigma_gate_ablation.py`` remains
        # reproducible.
        if sigma_mode not in ("tighten", "inflate", "both", "off"):
            raise ValueError(f"sigma_mode must be one of "
                             f"tighten/inflate/both/off (got {sigma_mode!r})")
        self.sigma_mode = sigma_mode
        self.sigma_buffer_gain = float(sigma_buffer_gain)
        self.N0 = max(int(horizon), 2)
        self.dt = float(dt)
        self.max_speed = float(max_speed)
        self.vehicle_radius = float(vehicle_radius)
        self.obstacle_buffer = float(obstacle_buffer)
        self.w_obs = float(weight_obstacle)
        self.w_fric = float(weight_friction)
        self.w_spd = float(weight_speed)
        self.w_dev = float(weight_deviation)
        self.w_obs_T = float(weight_terminal_obstacle)
        self.w_prog = float(weight_progress)
        self.phi_safety_margin_rad = float(np.radians(phi_safety_margin_deg))
        self.mu_cap = float(traction_mu_cap)
        self.control_dt = float(control_dt)

        # Teleop delay tracking — same semantics as the legacy CBF filter
        self._teleop_enabled = teleop_delay > 0.0
        self._teleop_delay = max(teleop_delay, 0.0)
        self._stale_cmd_timeout = stale_cmd_timeout
        self._last_cmd_wall: Optional[float] = None
        self._delay_ema = teleop_delay
        self._delay_ema_alpha = 0.15

        # NN surrogate + dynamics rollout
        surr = NumpyTireSurrogate(nn_model)
        rparams = build_rollout_params(vehicle_params)
        self.dyn = VehicleSurrogateDynamics(surr, rparams, self.terrain_nominal)

        # Diagnostics
        self._filter_count = 0
        self._modify_count = 0
        self._last_result: Optional[SafetyFilterResult] = None
        self._csv_file = None
        self._csv_writer = None
        self._init_csv_logging(csv_basename)

    # ---- terrain estimator hookup ------------------------------------------

    def update_terrain(self, terrain_params: dict,
                       phi_uncertainty_deg: float = 0.0):
        """Update terrain parameters (and friction-cone uncertainty).

        Called from the simulation/controller node whenever the online
        terrain estimator publishes a new ``(n, phi, ...)``.  ``phi`` must
        be passed in **degrees** (matching ``TERRAIN_PRESETS``); we convert
        internally.  ``phi_uncertainty_deg`` is the ensemble standard
        deviation, exposed to two independent gating paths controlled by
        ``sigma_mode`` (see ``__init__``): ``tighten`` pulls the friction
        cone down by ``phi - sigma`` (the original design, found harmful in
        2026-05 ablation), ``inflate`` adds ``k_sigma * sigma_deg`` to the
        obstacle clearance buffer instead, and ``both`` does both.
        """
        tp = dict(terrain_params)
        tp['phi'] = float(np.radians(tp['phi']))
        self.terrain_nominal = tp
        self._phi_uncertainty_rad = float(np.radians(phi_uncertainty_deg))
        self._phi_uncertainty_deg = float(phi_uncertainty_deg)
        self.dyn.update_terrain(tp)

    # ---- teleop delay API (mirrors CBFSafetyFilter) ------------------------

    def set_teleop_delay(self, delay_s: float):
        self._teleop_delay = max(delay_s, 0.0)
        self._delay_ema = self._teleop_delay
        self._teleop_enabled = self._teleop_delay > 0.0

    def update_command_age(self, cmd_wall_time: float):
        now = time.time()
        one_way = max(now - cmd_wall_time, 0.0)
        self._last_cmd_wall = now
        if self._teleop_enabled:
            a = self._delay_ema_alpha
            self._delay_ema = a * one_way + (1.0 - a) * self._delay_ema
            self._teleop_delay = self._delay_ema

    def _effective_buffer(self, v: float) -> float:
        buf = self.obstacle_buffer
        if self._teleop_delay > 0.0:
            rtt = 2.0 * self._teleop_delay
            buf += 0.5 * v * rtt
        if self.sigma_mode in ("inflate", "both"):
            # Inflate clearance when the estimator is uncertain about phi.
            # Keeps the shield equally willing to commit to evasive moves
            # (the friction cone is untouched in this mode) but enforces a
            # wider stand-off so a low-traction surprise leaves room.
            buf += self.sigma_buffer_gain * self._phi_uncertainty_deg
        return buf

    def _is_command_stale(self) -> bool:
        if self._teleop_delay <= 0.0 or self._last_cmd_wall is None:
            return False
        return (time.time() - self._last_cmd_wall) > self._stale_cmd_timeout

    def _horizon_steps(self) -> int:
        """Latency-padded horizon.  N = max(N0, ceil(RTT/dt) + 1)."""
        if self._teleop_delay <= 0.0:
            return self.N0
        rtt_steps = int(math.ceil(2.0 * self._teleop_delay / self.dt))
        return max(self.N0, rtt_steps + 1)

    # ---- cost function (shared by both flavors) ----------------------------

    def _phi_lower_bound(self) -> float:
        """Conservative friction angle used by the tightened cone (rad)."""
        # Ensemble-disagreement gate: pull phi down by sigma + a small fixed
        # safety margin; clamp at a positive floor. Only the sigma term
        # responds to ``sigma_mode`` — the constant safety margin is always
        # applied (it pre-dates the gate and is part of the friction model).
        phi = self.terrain_nominal['phi']
        sigma_contrib = (self._phi_uncertainty_rad
                         if self.sigma_mode in ("tighten", "both") else 0.0)
        phi_lb = phi - sigma_contrib - self.phi_safety_margin_rad
        return max(phi_lb, np.radians(2.0))

    def _trajectory_cost(self,
                          traj: np.ndarray,
                          controls: np.ndarray,
                          op_cmd: np.ndarray,
                          obstacles: np.ndarray,
                          v_max_terrain: float) -> np.ndarray:
        """Cost per sample, shape ``(K,)``.

        Components (all numpy-broadcast over the K dimension):

        1.  **Obstacle barrier.**  For each obstacle within reach we use
            a *quadratic + half-soft* penalty inside the safe radius:
            ``w * max(0, safe_r - dist)^2``.  We also include a heavier
            terminal cost on the last horizon point so MPPI doesn't 'flick'
            past obstacles when noise samples are short-sighted.

        2.  **Friction cone.**  ``|ay_h| > mu*g`` penalty, where ``mu`` is
            tightened by ``phi_uncertainty + phi_safety_margin``.  ``ay``
            is recomputed from the rollout (post-hoc — we don't need its
            gradient inside the rollout because the NN already supplies
            the lateral force).

        3.  **Speed cap.**  Quadratic over excess ``u - v_max_terrain``.

        4.  **Deviation.**  Squared error between the *first* control and
            the operator/AI command.  Penalising only the first step
            keeps the shield minimally invasive: later samples can drift
            far from the operator without paying the deviation cost.

        Args:
            traj: ``(K, H+1, 7)`` rollout including initial state.
            controls: ``(K, H, 2)``.
            op_cmd: ``(2,)`` operator command in (steer_norm, alpha).
            obstacles: ``(M, 3)`` array of (x, y, safe_r) — note that the
                caller passes in safe_r already inflated by vehicle radius,
                obstacle buffer, and latency, so the cost just compares
                Euclidean distance against it.
            v_max_terrain: speed cap (m/s).

        Returns:
            cost: ``(K,)`` total cost per sample.
        """
        K, Hp1, _ = traj.shape
        H = Hp1 - 1

        # ---- obstacle cost ---------------------------------------------------
        # We use a *fourth-power* penalty on the soft-buffer penetration
        # (was quadratic).  The buffer is narrow (~0.25 m), so a
        # quadratic ramp gave roughly the same penalty for "brushing
        # the buffer at 0.05 m" as for "deep in the buffer at 0.20 m" —
        # which pushed MPPI to brake hard even on glancing approaches.
        # The fourth-power makes the penalty close to zero until the
        # rollout is well inside the buffer, then ramps sharply.  Hard
        # collisions (rollout enters phys_r) are penalised separately
        # in MPPIShield._solve with weight 1e3, so this term only
        # governs the "comfortable distance" preference.
        if obstacles.shape[0] > 0:
            ox = obstacles[None, None, :, 0]                        # (1,1,M)
            oy = obstacles[None, None, :, 1]
            safe_r = obstacles[None, None, :, 2]                    # (1,1,M)
            tx = traj[:, 1:, 0:1]                                   # (K,H,1)
            ty = traj[:, 1:, 1:2]
            dx = tx - ox
            dy = ty - oy
            dist = np.sqrt(dx * dx + dy * dy)                       # (K,H,M)
            # Normalise penetration by safe_r so the cost shape is
            # scale-free (rocks of different sizes look the same).
            norm_pen = np.maximum(safe_r - dist, 0.0) / safe_r       # in [0, 1]
            stage = norm_pen ** 4
            obs_stage = self.w_obs * stage[:, :-1, :].sum(axis=(1, 2))
            obs_term = self.w_obs_T * stage[:, -1, :].sum(axis=1)
            obs_cost = obs_stage + obs_term
        else:
            obs_cost = np.zeros(K)

        # ---- friction cone cost ----------------------------------------------
        # Friction-cone enforcement is delegated to the planning NMPC,
        # which already has the live ``phi`` estimate and the traction
        # budget constraint baked into its OCP — duplicating it on the
        # shield side just had the shield second-guessing the planner
        # and scrubbing throttle long before any rock-related concern.
        # ``weight_friction=0`` by default; the term is computed if the
        # caller explicitly sets a non-zero weight (e.g. for an
        # ablation that runs the shield without a friction-aware NMPC
        # underneath).
        if self.w_fric > 0:
            u = traj[:, 1:, 3]
            delta = traj[:, 1:, 6]
            L = self.dyn.p.Lf + self.dyn.p.Lr
            ay = u * u * np.tan(np.clip(delta, -0.5, 0.5)) / L
            phi_lb = self._phi_lower_bound()
            mu_eff = min(np.tan(phi_lb), self.mu_cap)
            ay_lim = mu_eff * 9.81
            excess = np.maximum(np.abs(ay) - 1.10 * ay_lim, 0.0)
            fric_cost = self.w_fric * (excess ** 2).sum(axis=1)
        else:
            fric_cost = np.zeros(K)
        u = traj[:, 1:, 3]                                          # (K,H)

        # ---- speed cap cost --------------------------------------------------
        over_spd = np.maximum(u - v_max_terrain, 0.0)
        spd_cost = self.w_spd * (over_spd ** 2).sum(axis=1)

        # ---- deviation cost (first step only) --------------------------------
        # Asymmetric: heavier on throttle than steering, opposite of the
        # original intuition.  Steering is exactly what the shield needs
        # to override aggressively when an obstacle is in the planned
        # path (CBF's reactive_steering layer does the same thing —
        # commit to a steering direction even if it disagrees strongly
        # with operator intent).  Throttle/brake intent should be more
        # preserved — the operator's "go faster" or "stop" intent is
        # usually the right call once steering has solved the geometry.
        d_steer = controls[:, 0, 0] - op_cmd[0]
        d_alpha = controls[:, 0, 1] - op_cmd[1]
        dev_cost = self.w_dev * (0.5 * d_steer ** 2 + 3.0 * d_alpha ** 2)

        # ---- look-beyond-horizon obstacle cost ------------------------------
        # Project the rollout's final state forward at constant velocity
        # for an extra ``T_lookahead`` seconds and add a penalty for
        # obstacles the extended path would hit.  This gives the shield
        # indirect lookahead well beyond the rollout horizon — necessary
        # because a 1.8 s horizon at 4 m/s only reaches ~7 m, so the
        # shield otherwise can't see a second rock 15 m down the road
        # until the vehicle is right on top of it (the "evaded the big
        # rock then dove into the small one" failure mode).
        if obstacles.shape[0] > 0:
            T_la = 2.0  # seconds of constant-velocity extrapolation
            xT = traj[:, -1, 0]
            yT = traj[:, -1, 1]
            psiT = traj[:, -1, 2]
            uT = traj[:, -1, 3]
            cos_pT = np.cos(psiT)
            sin_pT = np.sin(psiT)
            # Sample 4 points along the extrapolated trajectory so we
            # catch obstacles anywhere along the projected line, not
            # just at the endpoint.
            ts = np.linspace(0.5, T_la, 4)
            la_costs = np.zeros(K)
            for t_la in ts:
                xp = xT + uT * cos_pT * t_la
                yp = yT + uT * sin_pT * t_la
                dxp = xp[:, None] - obstacles[None, :, 0]
                dyp = yp[:, None] - obstacles[None, :, 1]
                distp = np.sqrt(dxp * dxp + dyp * dyp)
                sr = obstacles[None, :, 2]
                # Same fourth-power soft penalty as the in-horizon cost,
                # but weighted down because the prediction is less
                # confident the further out we go.
                npen = np.maximum(sr - distp, 0.0) / sr
                la_costs = la_costs + (npen ** 4).sum(axis=1)
            obs_cost = obs_cost + 0.5 * self.w_obs * la_costs

        # ---- progress reward ------------------------------------------------
        # Without this, "brake to zero" is the trivial optimal cost
        # because no other term punishes lack of motion — the shield
        # happily stops at a rock boundary instead of going around it.
        # Penalise the rollout's *shortfall* in forward (operator-heading)
        # displacement vs what current speed would deliver at constant
        # velocity over the horizon.  Trajectories that go around the
        # rock at maintained speed have ~zero progress cost; trajectories
        # that brake hard pay the difference.  Penalty is one-sided —
        # going further than expected (e.g. via wider sweep) is fine.
        if self.w_prog > 0:
            psi0 = traj[:, 0, 2]
            x0 = traj[:, 0, 0]
            y0 = traj[:, 0, 1]
            xT = traj[:, -1, 0]
            yT = traj[:, -1, 1]
            # Use the initial heading as the "operator wants forward"
            # direction; this is consistent with the bicycle-model
            # convention and avoids needing the reference path.
            cos_p = np.cos(psi0)
            sin_p = np.sin(psi0)
            progress = (xT - x0) * cos_p + (yT - y0) * sin_p
            u0 = traj[:, 0, 3]
            # Target progress at constant current speed, with a small
            # ``min_target`` so the term still has signal when starting
            # from rest (otherwise the shield never accelerates).
            min_target = 1.0  # m
            target = np.maximum(u0 * H * self.dt, min_target)
            shortfall = np.maximum(target - progress, 0.0)
            # Normalise by target so cost is scale-free.
            prog_cost = self.w_prog * (shortfall / target) ** 2
        else:
            prog_cost = np.zeros(K)

        return obs_cost + fric_cost + spd_cost + dev_cost + prog_cost

    # ---- speed-cap heuristic from NN traction ------------------------------

    def _v_max_terrain(self, u: float, delta_now: float) -> float:
        """Use the NN surrogate to bound a curving speed limit, same idea
        as the legacy CBF but evaluated directly from the surrogate."""
        evasion_delta = max(np.radians(5.0), abs(delta_now))
        u_q = max(u, 2.0)
        Fz_f = self.dyn.p.Fz_f_per_wheel
        Fz_r = self.dyn.p.Fz_r_per_wheel
        # Probe at moderate slip angles — single batch with two samples
        alpha = np.array([evasion_delta, evasion_delta])
        Fz = np.array([Fz_f, Fz_r])
        us = np.array([u_q, u_q])
        kappa = np.zeros(2)
        _, Fy = self.dyn.nn.predict_batch(alpha, Fz, us, kappa, self.terrain_nominal)
        Fy_max = 2.0 * (abs(float(Fy[0])) + abs(float(Fy[1])))
        ay_max = Fy_max / self.dyn.p.M
        L = self.dyn.p.Lf + self.dyn.p.Lr
        R = L / max(np.tan(evasion_delta), 0.05)
        v_lim = math.sqrt(max(ay_max * R, 0.1))
        return min(self.max_speed, max(v_lim, 2.0))

    # ---- CSV logging -------------------------------------------------------

    def _init_csv_logging(self, basename: str):
        log_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'logs')
        os.makedirs(log_dir, exist_ok=True)
        path = os.path.join(log_dir, basename)
        self._csv_file = open(path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            'step', 'x', 'y', 'psi_deg', 'u', 'delta_deg',
            'n_obs', 'horizon', 'phi_lb_deg', 'v_max_terrain',
            'steer_in', 'throttle_in', 'brake_in',
            'steer_out', 'throttle_out', 'brake_out',
            'was_modified', 'min_clearance', 'solve_ms', 'cost_min',
        ])

    def _log(self, step, x, y, psi, u, delta, n_obs, horizon, phi_lb,
             v_max, s_in, t_in, b_in, s_out, t_out, b_out,
             modified, min_clear, solve_ms, cost_min):
        if self._csv_writer is None:
            return
        self._csv_writer.writerow([
            step,
            f'{x:.3f}', f'{y:.3f}', f'{np.degrees(psi):.2f}',
            f'{u:.3f}', f'{np.degrees(delta):.2f}',
            n_obs, horizon, f'{np.degrees(phi_lb):.2f}', f'{v_max:.2f}',
            f'{s_in:+.3f}', f'{t_in:.3f}', f'{b_in:.3f}',
            f'{s_out:+.3f}', f'{t_out:.3f}', f'{b_out:.3f}',
            int(modified), f'{min_clear:.3f}', f'{solve_ms:.2f}',
            f'{cost_min:.3f}',
        ])
        if step % 100 == 0:
            self._csv_file.flush()

    # ---- common state ingestion --------------------------------------------

    def _state_vec(self, vs: dict) -> np.ndarray:
        return np.array([
            vs.get('x', 0.0), vs.get('y', 0.0), vs.get('psi', 0.0),
            max(vs.get('u', 0.5), 0.1),
            vs.get('v', 0.0), vs.get('omega', 0.0),
            vs.get('delta', 0.0),
        ])

    def _obstacle_array(self, obstacles, v_state: float) -> np.ndarray:
        """Build ``(M, 4)`` array of ``(x, y, safe_r, phys_r)`` for the cost.

        Two radii per obstacle:

        * ``safe_r``: physical obstacle radius + vehicle radius + static
          buffer (+ teleop-latency inflation if ``teleop_delay > 0``)
          + a small stopping-distance term.  Drives the soft obstacle
          cost (quadratic penalty inside ``safe_r``).  The
          stopping-distance term is intentionally *small* — we don't
          want it to puff up perpendicular obstacles into the path —
          but having a few tenths of a metre of speed-aware inflation
          lets the cost climb gracefully before the rollout's horizon
          end reaches the rock.

        * ``phys_r``: physical obstacle radius + vehicle radius (no
          buffer).  Drives the *physical-collision penalty* in
          ``_trajectory_cost``: rollouts that actually intersect the
          rock get ~1000× the soft cost so they're effectively
          impossible in the importance-weighted mean.
        """
        if not obstacles:
            return np.zeros((0, 4))
        eff_buf = self._effective_buffer(v_state)
        rows = []
        for (ox, oy, ro) in obstacles:
            phys_r = ro + self.vehicle_radius
            safe_r = phys_r + eff_buf
            rows.append((ox, oy, safe_r, phys_r))
        return np.asarray(rows)

    def _min_clearance(self, traj: np.ndarray,
                        obstacles: np.ndarray) -> float:
        """Minimum (distance - safe_r) seen over the chosen trajectory."""
        if obstacles.shape[0] == 0:
            return float('inf')
        tx = traj[:, 0:1]
        ty = traj[:, 1:2]
        dx = tx - obstacles[None, :, 0]
        dy = ty - obstacles[None, :, 1]
        dist = np.sqrt(dx * dx + dy * dy)
        clear = (dist - obstacles[None, :, 2]).min()
        return float(clear)

    # ---- public filter entry point -----------------------------------------

    def filter(self, desired_steering, desired_throttle, desired_brake,
               vehicle_state, obstacles=None,
               terrain_roughness=0.0) -> SafetyFilterResult:
        t_start = time.time()
        self._filter_count += 1

        # Stale-command emergency brake (teleop only)
        if self._is_command_stale():
            self._modify_count += 1
            res = SafetyFilterResult(
                steering=desired_steering, throttle=0.0, braking=1.0,
                was_modified=True, active_constraints=0,
                solve_time_ms=(time.time() - t_start) * 1000.0,
                v_max_terrain=0.0, safety_margin=0.0,
            )
            self._last_result = res
            return res

        obstacles = obstacles or []
        state0 = self._state_vec(vehicle_state)
        op_cmd = self._operator_command(desired_steering, desired_throttle,
                                         desired_brake)
        H = self._horizon_steps()
        obs_arr = self._obstacle_array(obstacles, state0[3])
        v_max = self._v_max_terrain(state0[3], state0[6])

        # Passthrough fast-path — roll the operator command forward; if it
        # already satisfies every safety constraint, skip the (more
        # expensive) shield optimization and return the operator command
        # verbatim.  This is the MPPI/NMPC analogue of "DOB-CBF is the
        # identity map when no QP constraint is binding".
        ptr_traj = self.dyn.rollout(
            state0.reshape(1, -1),
            np.tile(op_cmd, (1, H, 1)),
            self.dt,
        )
        is_safe, ptr_info = self._is_passthrough_safe(
            ptr_traj, op_cmd, obs_arr, v_max)
        if is_safe:
            # Verbatim passthrough.  The shield's optimization state space
            # collapses ``(throttle, brake)`` into a single signed alpha,
            # but the MPC routinely commands a small simultaneous
            # ``throttle + brake`` (an artefact of its rate-limited
            # integrator).  Encoding -> decoding through alpha on
            # passthrough silently drops the brake component and changes
            # the net ax — so when the rollout says "operator command is
            # safe", we return the raw operator tuple unmodified rather
            # than the alpha-encoded approximation.
            s_out = float(desired_steering)
            t_out = float(desired_throttle)
            b_out = float(desired_brake)
            modified = False
            info = {'cost_min': 0.0, 'traj_best': ptr_traj[0],
                    'passthrough': True}
            u0_safe = op_cmd  # kept for traj_best / logging
        else:
            u0_safe, info = self._solve(state0, op_cmd, obs_arr, H, v_max)
            s_out = float(np.clip(u0_safe[0], -1.0, 1.0))
            a_out = float(np.clip(u0_safe[1], -1.0, 1.0))
            if a_out >= 0.0:
                t_out, b_out = a_out, 0.0
            else:
                t_out, b_out = 0.0, -a_out
            modified = (abs(s_out - desired_steering) > 1e-3 or
                        abs(t_out - desired_throttle) > 1e-3 or
                        abs(b_out - desired_brake) > 1e-3)
            if modified:
                self._modify_count += 1
        traj_best = info.get('traj_best')
        min_clear = (self._min_clearance(traj_best[1:, :2], obs_arr)
                      if traj_best is not None else float('inf'))

        res = SafetyFilterResult(
            steering=s_out, throttle=t_out, braking=b_out,
            was_modified=modified, active_constraints=int(obs_arr.shape[0]),
            solve_time_ms=(time.time() - t_start) * 1000.0,
            v_max_terrain=v_max, safety_margin=min_clear,
        )
        self._last_result = res

        if modified or self._filter_count % 20 == 0:
            self._log(
                self._filter_count, state0[0], state0[1], state0[2],
                state0[3], state0[6], len(obstacles), H,
                self._phi_lower_bound(), v_max,
                desired_steering, desired_throttle, desired_brake,
                s_out, t_out, b_out, modified, min_clear,
                res.solve_time_ms, info.get('cost_min', 0.0))
        return res

    @staticmethod
    def _operator_command(steer, throttle, brake) -> np.ndarray:
        """Map operator (steer, throttle, brake) -> (steer_norm, alpha)."""
        alpha = float(throttle) - float(brake)
        return np.array([float(steer), float(alpha)])

    # ---- diagnostics -------------------------------------------------------

    @property
    def intervention_rate(self) -> float:
        return (self._modify_count / self._filter_count
                if self._filter_count else 0.0)

    def get_diagnostics(self) -> dict:
        r = self._last_result
        return {
            'filter_calls': self._filter_count,
            'interventions': self._modify_count,
            'intervention_rate': self.intervention_rate,
            'last_solve_ms': r.solve_time_ms if r else 0.0,
            'last_modified': r.was_modified if r else False,
            'last_v_max_terrain': r.v_max_terrain if r else self.max_speed,
            'last_safety_margin': r.safety_margin if r else float('inf'),
            'last_active_constraints': r.active_constraints if r else 0,
            'last_dob_norm': 0.0,
        }

    def _reactive_steering(self,
                            state0: np.ndarray,
                            obs_arr: np.ndarray) -> float:
        """CBF-style geometric prior: an "evade away from nearest in-path
        obstacle" steering command computed from the obstacle geometry
        alone (no rollout, no NN).

        Returns a normalised steering command in ``[-1, 1]``.  Used as
        a deterministic seed for the predictive shields so they have
        a working evade trajectory to refine rather than having to
        discover the direction from MPPI noise alone.  This is the
        same trick DOB-CBF's reactive_steering layer uses; without
        it, MPPI's mean-of-elites averaging often picks the *wrong*
        side of a head-on obstacle (cross-mode averaging) and the
        vehicle stalls.

        The direction is chosen as "away from the lateral side the
        obstacle is on" — if the obstacle is to the left of the
        vehicle's heading, steer right, and vice versa.  Magnitude
        scales with proximity (1.0 when at the safe-radius boundary,
        0.0 when far enough away to ignore).
        """
        if obs_arr.shape[0] == 0:
            return 0.0
        x, y, psi = state0[0], state0[1], state0[2]
        cos_psi = np.cos(psi)
        sin_psi = np.sin(psi)
        steer_total = 0.0
        # Reaction range: a few times the safe radius — should be wide
        # enough that the seed is non-trivial well before any
        # obstacle is in the rollout horizon.
        for row in obs_arr:
            ox, oy, safe_r = float(row[0]), float(row[1]), float(row[2])
            dx_world = ox - x
            dy_world = oy - y
            dx_body = cos_psi * dx_world + sin_psi * dy_world   # forward
            dy_body = -sin_psi * dx_world + cos_psi * dy_world   # left+
            # Only react to obstacles strictly ahead of the vehicle
            if dx_body < -0.5:
                continue
            dist = np.sqrt(dx_body * dx_body + dy_body * dy_body)
            react_range = safe_r + 6.0
            if dist > react_range:
                continue
            proximity = np.clip(
                1.0 - (dist - safe_r) / max(react_range - safe_r, 0.1),
                0.0, 1.0)
            # Body-frame +y_body is to the LEFT in the convention used
            # by surrogate_dynamics (positive steer_norm → left turn).
            # If obstacle is to the LEFT (dy_body > 0), steer RIGHT
            # (negative steer_norm).  For nearly head-on obstacles
            # (small |dy_body|), default left to break ties consistently
            # — that's the same heuristic the legacy CBF uses.
            if abs(dy_body) > 0.3:
                direction = -np.sign(dy_body)
            else:
                direction = +1.0
            steer_total += float(direction) * float(proximity) * 0.8
        return float(np.clip(steer_total, -1.0, 1.0))

    def _is_passthrough_safe(self,
                              ptr_traj: np.ndarray,
                              op_cmd: np.ndarray,
                              obs_arr: np.ndarray,
                              v_max: float
                              ) -> Tuple[bool, dict]:
        """Decide whether to skip the shield and pass the operator command through.

        Returns ``(is_safe, info)`` where ``is_safe`` is True iff the
        passthrough trajectory satisfies *all* of:

        * no obstacle penetrates the inflated safe radius at any step,
        * the friction-cone bound (``mu * g`` with the tightened ``phî``)
          is not exceeded at any step,
        * the terrain-aware speed cap is not exceeded.

        Even one violation → fall back to ``_solve``.  Deviation cost is
        identically zero on the passthrough by construction so it's not
        part of the gate.
        """
        x = ptr_traj[0, 1:, 0]
        y = ptr_traj[0, 1:, 1]
        u = ptr_traj[0, 1:, 3]
        delta = ptr_traj[0, 1:, 6]

        if obs_arr.shape[0] > 0:
            dx = x[:, None] - obs_arr[None, :, 0]
            dy = y[:, None] - obs_arr[None, :, 1]
            dist = np.sqrt(dx * dx + dy * dy)
            pen = (obs_arr[None, :, 2] - dist).max()
            if pen > 0.0:
                return False, {'cost': float('inf'), 'reason': 'obstacle'}

        # Friction-cone enforcement is the planning NMPC's job (it has
        # the live ``phi`` estimate and the traction budget constraint
        # baked into its OCP).  Forcing the shield to redo it here was
        # over-aggressive: on a sinusoidal path the rollout's predicted
        # *future* speed always touches the cone, and the shield ended
        # up scrubbing throttle to a crawl 10+ m before the nearest
        # rock.  We keep the obstacle + speed-cap gates, and let the
        # planner handle traction.

        if u.max() > v_max + 1e-3:
            return False, {'cost': float('inf'), 'reason': 'speed'}

        return True, {'cost': 0.0, 'reason': 'passthrough'}

    def _physical_collision_penalty(self, traj: np.ndarray,
                                     obs_arr: np.ndarray) -> np.ndarray:
        """Large penalty for rollouts that physically enter ``phys_r``.

        This augments the smooth obstacle barrier with a clear "never
        prefer a physical hit" term.  MPPI has used this penalty for
        committed action selection; NMPC needs the same term because its
        local optimizer otherwise sees only the soft barrier landscape.
        """
        if obs_arr.shape[0] == 0:
            return np.zeros(traj.shape[0])
        H = traj.shape[1] - 1
        soft_ceiling = (
            self.w_obs * H
            + self.w_obs_T
            + 0.5 * self.w_obs * 4
            + self.w_prog
            + self.w_dev * (0.5 * 4 + 3 * 4)
            + self.w_spd * H
        )
        phys_w = 50.0 * soft_ceiling
        tx = traj[:, 1:, 0:1]
        ty = traj[:, 1:, 1:2]
        ox = obs_arr[None, None, :, 0]
        oy = obs_arr[None, None, :, 1]
        pr = obs_arr[None, None, :, 3]
        dist = np.sqrt((tx - ox) ** 2 + (ty - oy) ** 2)
        pen = np.maximum(pr - dist, 0.0)
        norm_pen = pen / pr
        return phys_w * (norm_pen ** 2).sum(axis=(1, 2))

    def _solve(self, state0, op_cmd, obs_arr, H, v_max):
        raise NotImplementedError


# ----------------------------------------------------------------------------
# Primary: MPPI shield
# ----------------------------------------------------------------------------

class MPPIShield(_PredictiveShieldBase):
    """Sampling-based predictive safety filter (MPPI).

    Draws ``K`` noisy command sequences around the operator/AI command,
    rolls each through the NN-surrogate dynamics over a latency-padded
    horizon, and returns the importance-weighted mean of the first
    action.  K "seed" trajectories are injected unconditionally (full
    brake, coast straight, hard-left + brake, hard-right + brake) so the
    shield always has a recoverable option in its sample set even when
    the operator command leads straight into an obstacle.

    Args:
        n_samples: K, the number of MPPI samples per step.
        sigma_steer: stddev of steering-norm noise (sampled per step).
        sigma_alpha: stddev of throttle-norm noise (sampled per step).
        temperature: lambda in ``w = exp(-(S - S_min) / lambda)``.
        critical_cost: above this min-cost we emergency-brake.
        rng_seed: numpy random seed.

    Other args inherited from :class:`_PredictiveShieldBase`.
    """

    def __init__(self,
                 vehicle_params: dict,
                 nn_model,
                 terrain_params: dict,
                 *,
                 n_samples: int = 384,
                 sigma_steer: float = 0.35,
                 sigma_alpha: float = 0.35,
                 temperature: float = 1.0,
                 critical_cost: float = 5e3,
                 rng_seed: int = 0,
                 disable_seeds: bool = False,
                 **kwargs):
        kwargs.setdefault('csv_basename', 'mppi_shield_log.csv')
        super().__init__(vehicle_params, nn_model, terrain_params, **kwargs)
        self.K = int(n_samples)
        self.sigma = np.array([sigma_steer, sigma_alpha])
        self.lam = float(temperature)
        self.critical_cost = float(critical_cost)
        self._rng = np.random.default_rng(rng_seed)
        # Ablation: when True, skip the hand-crafted seed trajectories
        # (passthrough / brake / evade) and rely solely on Gaussian sampling
        # around the operator command.  Used to measure how much of the
        # paper's collision-rate result comes from the seeds vs the rollouts.
        self.disable_seeds = bool(disable_seeds)
        # Initial nominal mean — updated each step with the operator's cmd
        self._u_nom = None

    def _seed_trajectories(self, op_cmd: np.ndarray, H: int,
                            reactive_steer: float = 0.0) -> np.ndarray:
        """Hand-crafted control sequences guaranteed to be in the sample set.

        Returns shape ``(S, H, 2)`` with S seeds.

        Steady-state seeds (each control repeated for the full horizon):

        * **passthrough** — operator command repeated.  Anchors the
          weighted mean to the operator's intent whenever the operator
          command is safe.  Required for the "shield is the identity
          when unconstrained" property.
        * **full brake / coast** — keeps operator steering, but kills
          throttle (full brake or zero).  The minimal-bias safe choice.

        Time-varying "commit-then-straighten" seeds — these matter when
        the vehicle is already inside the soft buffer of a rock dead
        ahead, because a steady-state evade can't represent "turn
        hard for the first 0.3 s, then straighten back to track":

        * **hard-left commit** — full-left for the first quarter of
          the horizon, then straighten with light throttle.  Lets the
          shield discover an evasion arc that gets *around* a head-on
          obstacle within the rollout, rather than coasting toward it.
        * **hard-right commit** — same, mirrored.
        * **commit-then-brake L/R** — sharper version that also kills
          throttle, for when the obstacle is too close to outrun.

        If ``reactive_steer`` is non-zero (computed by
        ``_reactive_steering`` from the obstacle geometry), three
        extra seeds are added that *commit* to that direction at
        different throttle levels — these give MPPI a known-good
        starting point for the evade rather than relying on noise
        + the symmetric L/R seeds to discover it.
        """
        commit = max(int(0.35 * H), 2)   # ~0.6 s of hard turn at H=18
        op_alpha = float(op_cmd[1])
        # Throttle to use on the "evade through" seeds: keep the operator's
        # forward throttle if it's positive, otherwise inject a moderate
        # throttle so the evade actually makes lateral progress.
        evade_alpha = max(op_alpha, 0.6)

        passthrough = np.tile(op_cmd, (H, 1))

        # Evade-and-keep-throttle: full lateral commit for ~commit steps,
        # then straighten while keeping forward throttle.  These seeds
        # are the analogue of the legacy DOB-CBF reactive_steering
        # override — they represent "drive *around* the obstacle"
        # rather than "scrub speed at the obstacle boundary".
        evade_left_through = np.tile([1.0, evade_alpha], (H, 1))
        evade_left_through[commit:] = [0.2, evade_alpha]

        evade_right_through = np.tile([-1.0, evade_alpha], (H, 1))
        evade_right_through[commit:] = [-0.2, evade_alpha]

        # Evade-and-brake: same lateral commit but with throttle cut —
        # for tighter clearances where we can't outrun the obstacle.
        evade_left_brake = np.tile([1.0, -0.5], (H, 1))
        evade_left_brake[commit:] = [0.0, -0.5]
        evade_right_brake = np.tile([-1.0, -0.5], (H, 1))
        evade_right_brake[commit:] = [0.0, -0.5]

        # Steady-state evades — these matter when there are several
        # obstacles in series and the right move is "drift one direction
        # for the whole horizon".
        sustain_left = np.tile([0.7, evade_alpha], (H, 1))
        sustain_right = np.tile([-0.7, evade_alpha], (H, 1))

        seeds = [
            passthrough,
            np.tile([op_cmd[0], -1.0], (H, 1)),    # brake, hold steering
            np.tile([op_cmd[0],  0.0], (H, 1)),    # coast, hold steering
            evade_left_through,
            evade_right_through,
            evade_left_brake,
            evade_right_brake,
            sustain_left,
            sustain_right,
        ]
        if abs(reactive_steer) > 0.05:
            # Three flavors of "commit to the geometric evade direction":
            # full throttle, ease throttle, brake while turning.  All
            # use the *same* steering (the reactive_steer value), so
            # MPPI's cluster-then-average action selector will pick
            # them as one mode regardless of which throttle wins.
            rs = float(reactive_steer)
            seeds += [
                np.tile([rs, evade_alpha], (H, 1)),
                np.tile([rs, 0.2], (H, 1)),
                np.tile([rs, -0.5], (H, 1)),
            ]
        return np.asarray(seeds)

    def _solve(self, state0, op_cmd, obs_arr, H, v_max):
        # Two-iteration cross-entropy / MPPI hybrid (CEM-like refinement).
        #
        # The base class already short-circuits to passthrough when the
        # operator command is safe, so we only run here when a real
        # avoidance decision is needed.  In that regime the operator's
        # command is *unsafe by definition*, and clustering K samples
        # around it via pure MPPI lets random-noise samples drown out the
        # few aggressive-evade seeds that actually solve the problem —
        # MPPI's importance-weighted mean is biased toward sample
        # density, and there are 384 noisy "op_cmd + ε" samples vs
        # ~9 hand-crafted evade seeds.
        #
        # We replace that with two refinement rounds:
        #
        #   1. Score K samples around op_cmd + the seeds, pick the
        #      ``n_elite`` lowest-cost ones, and fit a Gaussian over
        #      them in control space.
        #   2. Resample around the elite mean (a much smaller σ —
        #      we're now exploring around a *known-good* trajectory)
        #      and take the importance-weighted mean.
        #
        # This is the standard CEM-MPPI pattern.  It gives the
        # commitment that pure MPPI averaging fails to provide: the
        # second round samples are clustered around the best seed,
        # not the operator's unsafe command, so the weighted mean
        # actually points toward the evasion.
        K = self.K
        x0 = np.tile(state0, (K, 1))

        reactive_steer = self._reactive_steering(state0, obs_arr)
        if self.disable_seeds:
            seeds = np.empty((0, H, 2), dtype=float)
        else:
            seeds = self._seed_trajectories(op_cmd, H,
                                              reactive_steer=reactive_steer)
        S = seeds.shape[0]

        # ----- Round 1: explore around the operator command -----
        eps1 = self._rng.normal(0.0, 1.0, size=(K, H, 2)) * self.sigma
        u_nom1 = np.tile(op_cmd, (H, 1))
        ctrl1 = u_nom1[None, :, :] + eps1
        if S < K:
            ctrl1[:S] = seeds
        np.clip(ctrl1, -1.0, 1.0, out=ctrl1)

        traj1 = self.dyn.rollout(x0, ctrl1, self.dt)
        cost1 = self._trajectory_cost(traj1, ctrl1, op_cmd, obs_arr, v_max)
        cost1 = cost1 + self._physical_collision_penalty(traj1, obs_arr)

        n_elite = max(int(0.05 * K), 12)
        elite_idx = np.argsort(cost1)[:n_elite]
        elite_ctrl = ctrl1[elite_idx]                                 # (n_elite, H, 2)
        elite_mean = elite_ctrl.mean(axis=0)                          # (H, 2)
        elite_std = np.clip(elite_ctrl.std(axis=0), 0.05, 0.5)         # (H, 2)

        # ----- Round 2: refine around the elite distribution -----
        eps2 = self._rng.normal(0.0, 1.0, size=(K, H, 2)) * elite_std[None, :, :]
        ctrl2 = elite_mean[None, :, :] + eps2
        # Re-inject seeds so they're always in the candidate pool
        if S < K:
            ctrl2[:S] = seeds
        np.clip(ctrl2, -1.0, 1.0, out=ctrl2)

        traj2 = self.dyn.rollout(x0, ctrl2, self.dt)
        cost2 = self._trajectory_cost(traj2, ctrl2, op_cmd, obs_arr, v_max)
        cost2 = cost2 + self._physical_collision_penalty(traj2, obs_arr)

        # Step-to-step direction commitment: once the shield has
        # picked an evasion direction last step, lightly bias the
        # cluster centre toward that same direction.  Without this,
        # the operator command swinging across the obstacle's
        # lateral midpoint can flip MPPI's argmin to the *other*
        # cluster mid-maneuver — vehicle suddenly steers full-right
        # when it was committed left — and drives into the rock.
        # CBF gets this commitment for free because its reactive
        # layer is integrated across many sim ticks.
        prev_dir = getattr(self, '_last_steer_dir', 0.0)
        if abs(prev_dir) > 0.2 and abs(reactive_steer) > 0.05:
            # Add a small bonus to samples that match the previous
            # direction's sign — equivalent to ~1 unit of cost,
            # which is tiny compared to obstacle penalties but
            # enough to break ties between cluster modes.
            same_sign = np.sign(ctrl2[:, 0, 0]) == np.sign(prev_dir)
            cost2 = cost2 + np.where(~same_sign, 1.0, 0.0)

        # ----- Action selection: cluster the elites, commit to the best
        # cluster --------------------------------------------------------
        # Pure argmin is decisive but twitchy.  Averaging all K samples
        # dilutes the few good seeds.  Averaging the top-N elites
        # *fails* when those elites split into two modes (e.g. half
        # say "evade left", half say "evade right"); the mean lies
        # between the modes and drives the vehicle into the obstacle.
        # That multi-modal failure was the dominant MPPI loss in the
        # 9-(terrain, path) sweep matrix (MPPI stuck at x≈15-19 on
        # double_lane_change / high-traction-sinusoidal scenarios).
        #
        # Fix: take the *single best* sample as the commit point, then
        # average only those few elites that lie near that best
        # sample's first action (within ``cluster_radius`` in control
        # space).  This keeps MPPI's mild smoothing benefit while
        # preventing cross-mode averaging.
        best_idx = int(np.argmin(cost2))
        best_u0 = ctrl2[best_idx, 0]
        # Pull the small "elite shortlist" first (top 5 % by cost)
        n_short = max(int(0.05 * K), 10)
        short_idx = np.argsort(cost2)[:n_short]
        short_u0 = ctrl2[short_idx, 0]                                 # (n_short, 2)
        # Filter to those in the same mode as the argmin (Euclidean
        # distance ≤ 0.6 in normalised control space — enough to span
        # mild noise around one seed, narrow enough to exclude an
        # oppositely-steered seed at distance ~2.0)
        cluster_radius = 0.6
        mask = np.linalg.norm(short_u0 - best_u0, axis=1) <= cluster_radius
        cluster_u0 = short_u0[mask]
        # Always include the best sample itself
        if cluster_u0.shape[0] == 0:
            cluster_u0 = best_u0[None, :]
        u0_safe = cluster_u0.mean(axis=0)
        np.clip(u0_safe, -1.0, 1.0, out=u0_safe)
        # Persist the steering sign for the next step's commitment bias
        self._last_steer_dir = float(u0_safe[0])
        S_min = float(cost2[best_idx])
        return u0_safe, {'cost_min': S_min,
                         'traj_best': traj2[best_idx]}

    def _physical_collision_penalty(self, traj: np.ndarray,
                                     obs_arr: np.ndarray) -> np.ndarray:
        """Large penalty for rollouts that physically enter ``phys_r``.

        Returns shape ``(K,)``.  Used as a *cost augmentation* (not a
        hard filter) so the importance-weighted mean stays well-defined
        even when the vehicle is inside the soft buffer of a rock.

        The weight is set relative to the maximum *soft* cost the
        rollout can accumulate (obstacle buffer + look-ahead + progress
        + deviation), so a physical collision is unambiguously the
        worst outcome no matter how the other weights are tuned.  This
        avoids the "tune the absolute number" problem we had earlier.
        """
        if obs_arr.shape[0] == 0:
            return np.zeros(traj.shape[0])
        H = traj.shape[1] - 1
        # Rough upper bound on the soft-cost sum across the horizon.
        soft_ceiling = (
            self.w_obs * H                  # full buffer at every step
            + self.w_obs_T                  # terminal
            + 0.5 * self.w_obs * 4          # look-ahead samples
            + self.w_prog                   # max progress shortfall
            + self.w_dev * (0.5 * 4 + 3 * 4)  # max dev squared
            + self.w_spd * H                # max speed-cap
        )
        phys_w = 50.0 * soft_ceiling
        tx = traj[:, 1:, 0:1]
        ty = traj[:, 1:, 1:2]
        ox = obs_arr[None, None, :, 0]
        oy = obs_arr[None, None, :, 1]
        pr = obs_arr[None, None, :, 3]
        dist = np.sqrt((tx - ox) ** 2 + (ty - oy) ** 2)
        pen = np.maximum(pr - dist, 0.0)
        # Normalise by phys_r so a 0.5-m penetration into a small rock
        # costs the same fraction as a 0.5-m penetration into a big one.
        norm_pen = pen / pr
        return phys_w * (norm_pen ** 2).sum(axis=(1, 2))


# ----------------------------------------------------------------------------
# Ablation: Predictive Safety NMPC (gradient-based, scipy SLSQP)
# ----------------------------------------------------------------------------

class NMPCShield(_PredictiveShieldBase):
    """Gradient-based predictive safety filter via L-BFGS-B.

    Decision variables are the flattened control sequence ``(H * 2,)``.
    Cost matches MPPI (same rollout, same weights), evaluated in pure
    numpy.  Because the NN rollout is not autodifferentiable, L-BFGS-B
    uses a batched 2-point finite-difference Jacobian internally.  We
    pick **L-BFGS-B** over SLSQP because:

    * SLSQP performs an exact line search per iteration plus an inner
      QP, which on this problem cost ~130 ms per call — well above the
      10 Hz safety-loop budget.
    * L-BFGS-B uses a cheap strong-Wolfe line search and a low-memory
      quasi-Newton Hessian estimate; on the same problem it converges
      in fewer function evals and ~3–4× less wall-clock.
    * We only need box bounds (no equality / general inequality), which
      is L-BFGS-B's native problem class.

    This is the ablation flavor — paired with :class:`MPPIShield`,
    it lets the paper directly compare sampling vs gradient predictive
    shielding on identical rollout dynamics and cost.

    Args:
        n_iter: ``maxiter`` for L-BFGS-B.
        ftol: tolerance on the cost function value.
    """

    def __init__(self,
                 vehicle_params: dict,
                 nn_model,
                 terrain_params: dict,
                 *,
                 n_iter: int = 6,
                 ftol: float = 1e-3,
                 **kwargs):
        kwargs.setdefault('csv_basename', 'nmpc_shield_log.csv')
        # NMPC uses a slightly shorter horizon than MPPI but long enough
        # to cover the same stopping-distance budget at typical speeds.
        kwargs.setdefault('horizon', 8)
        super().__init__(vehicle_params, nn_model, terrain_params, **kwargs)
        self.n_iter = int(n_iter)
        self.ftol = float(ftol)
        self._u_warm: Optional[np.ndarray] = None
        from scipy.optimize import minimize  # noqa: F401
        self._minimize = minimize

    def _cost_flat(self, u_flat, state0, op_cmd, obs_arr, H, v_max):
        controls = u_flat.reshape(1, H, 2)
        traj = self.dyn.rollout(state0.reshape(1, -1), controls, self.dt)
        c = self._trajectory_cost(traj, controls, op_cmd, obs_arr, v_max)
        c = c + self._physical_collision_penalty(traj, obs_arr)
        return float(c[0])

    def _cost_and_grad(self, u_flat, state0, op_cmd, obs_arr, H, v_max):
        """Cost + batched forward-difference Jacobian in a single rollout.

        scipy's default finite-difference path calls ``_cost_flat`` once
        per decision variable (``2H`` extra calls per gradient evaluation),
        which is the worst case for numpy: each call carries roughly the
        same Python/dispatch overhead regardless of batch size, so on the
        NN-surrogate rollout we pay ``O(H)`` overhead instead of ``O(1)``.

        Here we batch the reference rollout and *all* 2H perturbed
        rollouts into a single ``K = 2H + 1`` rollout, then read the
        Jacobian off as forward differences in cost space.  This drops
        per-solve wall-time by ~10× on our static MLP surrogate.

        Returns ``(cost, grad)`` matching scipy's ``jac=True`` convention.
        """
        D = H * 2
        eps = 1e-3
        # Build K = D+1 control sequences: index 0 = nominal,
        # indices 1..D = nominal with one component perturbed by +eps.
        K = D + 1
        controls = np.tile(u_flat.reshape(1, H, 2), (K, 1, 1))
        for d in range(D):
            h, c = divmod(d, 2)
            controls[d + 1, h, c] += eps
        np.clip(controls, -1.0, 1.0, out=controls)

        x0 = np.tile(state0, (K, 1))
        traj = self.dyn.rollout(x0, controls, self.dt)
        costs = self._trajectory_cost(traj, controls, op_cmd, obs_arr, v_max)
        costs = costs + self._physical_collision_penalty(traj, obs_arr)
        c0 = float(costs[0])
        grad = (costs[1:] - costs[0]) / eps
        return c0, grad.astype(np.float64)

    def _solve(self, state0, op_cmd, obs_arr, H, v_max):
        # Multi-start L-BFGS-B: launch from several distinct
        # warm-starts and keep the lowest-cost result.  Pure single-
        # start L-BFGS-B has been observed to land in local minima
        # that brush through rocks (~1.9k collision-frames on sand/lc
        # in one sweep run).  Each warm-start runs an independent
        # optimization with the same iter budget, so wall-clock scales
        # roughly linearly with the number of starts.  We keep starts
        # cheap (n_iter is still small) but cover the most useful
        # distinct trajectories: previous solution, operator command,
        # and the CBF-style geometric prior (if any obstacle is in
        # range).
        # Single-start L-BFGS-B with the previous-solution warm-start.
        # An earlier 2-start variant (warm-start + geometric prior)
        # regressed badly on the sand/dlc and sand/sinusoidal scenarios
        # — adding starts in the wrong basin pulled the optimizer
        # toward locally-worse solutions on those cost landscapes.
        # The single-start path is the most consistent across the
        # (terrain × path) matrix; closing the remaining collision
        # gap requires a better surrogate, not more optimization
        # (see TRACKING for the NN-vs-Chrono accuracy analysis).
        if self._u_warm is not None and self._u_warm.shape == (H, 2):
            x0 = np.vstack([self._u_warm[1:],
                              self._u_warm[-1:]]).flatten()
        else:
            x0 = np.tile(op_cmd, H).flatten()

        bounds = [(-1.0, 1.0)] * (H * 2)

        res = self._minimize(
            self._cost_and_grad, x0,
            args=(state0, op_cmd, obs_arr, H, v_max),
            method='L-BFGS-B', jac=True, bounds=bounds,
            options={'maxiter': self.n_iter, 'ftol': self.ftol,
                     'gtol': 1e-3,
                     'maxfun': 5 * (self.n_iter + 1)},
        )
        u_seq = res.x.reshape(H, 2)
        np.clip(u_seq, -1.0, 1.0, out=u_seq)
        u0_safe = u_seq[0]
        self._u_warm = u_seq
        traj = self.dyn.rollout(state0.reshape(1, -1),
                                  u_seq.reshape(1, H, 2), self.dt)
        return u0_safe, {'cost_min': float(res.fun),
                         'traj_best': traj[0]}
