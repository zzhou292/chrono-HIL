"""Modular collision-warning system for the HIL teleoperation stack.

Unlike the DOB-CBF filter (``CBFSafetyFilter`` in ``safety/__init__.py``),
this component does **not**
modify commands. It runs in parallel with whatever filter the operator
has selected (or none), and emits a discrete *warning signal* that is
piped back to the remote driver.

The warning is **terrain-aware** and **latency-aware**:

  * On soft terrain (low Bekker ``n``) the vehicle's effective braking
    deceleration is lower → required stopping distance is longer →
    warning fires earlier for the same obstacle distance.
  * When teleop latency or jitter is high, the operator's reaction is
    effectively delayed → required stopping distance grows by
    ``v * 2 * tau_RTT`` → warning fires earlier.

Severity levels (suggested mapping; downstream code decides what to do):
    0 GREEN  — no obstacle within ``react_horizon_s`` × v ahead
    1 YELLOW — TTC > brake_horizon_s; visual prompt only
    2 ORANGE — TTC ≤ brake_horizon_s but > 0.5 s; haptic/audio prompt
    3 RED    — TTC ≤ 0.5 s or stopping distance > clearance; commit warning

The component exposes the same ``update_command_age()`` /
``set_teleop_delay()`` API as the safety filters so the existing ZMQ
wiring in ``acados_mpc_controller_node`` and ``chrono_sim_node`` can
keep it fed with latency stats.
"""
from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple


@dataclass
class CollisionWarning:
    """Warning signal emitted by ``CollisionWarningSystem.evaluate``."""

    severity: int            # 0 (green) → 3 (red)
    ttc: float               # time-to-collision against worst obstacle (s)
    closest_obstacle: Optional[Tuple[float, float, float]]   # x, y, r
    stopping_distance: float # current required stopping distance (m)
    clearance: float         # current Euclidean clearance to nearest obstacle (m)
    margin: float            # clearance - stopping_distance (m); negative ⇒ red
    latency_inflation_m: float  # extra stopping margin added due to latency (m)
    terrain_n_used: float    # the n value used to compute the friction floor
    message: str             # human-readable description for HMI overlay


_SEVERITY_NAMES = {0: "GREEN", 1: "YELLOW", 2: "ORANGE", 3: "RED"}


class CollisionWarningSystem:
    """Terrain + latency-aware forward collision warning.

    Args:
        vehicle_radius: Effective collision radius of the chassis (m).
        max_brake_decel_dry: Linear-interp fallback ceiling, only used
            when no ``tire_model_dir`` is supplied. Default 6.0 m/s²
            matches the ``CBFSafetyFilter`` actuator model.
        min_brake_decel_soft: Linear-interp fallback floor (soft soil).
        tire_model_dir: Path to a tire-NN surrogate (e.g.
            ``nn_models/rig_rate_64_32``). When provided, the brake-decel
            estimate is computed *analytically* by querying the surrogate
            for peak |F_x| at peak-braking slip across a small terrain
            grid built at init; the linear-interp fallback above is then
            unused.
        vehicle_mass: HMMWV gross mass (kg) used for `a = F/m`.
        Lf, Lr: CG-to-front-axle and CG-to-rear-axle distances (m).
            Together with mass they determine static per-wheel F_z.
        reaction_time_s: Operator + actuator latency baseline (s).
        jitter_window_s: How far back to look when estimating latency
            jitter standard deviation (s).
        latency_safety_factor: Multiplier on (RTT + jitter) added to the
            stopping budget. Higher = warn earlier.
        brake_horizon_s: TTC threshold above which warning is at most YELLOW.
        commit_horizon_s: TTC threshold below which warning is RED.
        update_interval_s: Minimum interval between ``evaluate`` calls; if
            called more often the cached result is returned to keep the
            warning stable (avoids flicker).
        verbose: Print warning transitions.
    """

    def __init__(self,
                 *,
                 vehicle_radius: float = 1.5,
                 max_brake_decel_dry: float = 6.0,
                 min_brake_decel_soft: float = 2.5,
                 tire_model_dir: Optional[str] = None,
                 vehicle_mass: float = 2500.0,
                 Lf: float = 1.685,
                 Lr: float = 1.685,
                 reaction_time_s: float = 0.20,
                 jitter_window_s: float = 1.0,
                 latency_safety_factor: float = 1.5,
                 brake_horizon_s: float = 2.5,
                 commit_horizon_s: float = 0.5,
                 update_interval_s: float = 0.05,
                 verbose: bool = False):
        self.vehicle_radius = float(vehicle_radius)
        self.a_dry = float(max_brake_decel_dry)
        self.a_soft = float(min_brake_decel_soft)
        self.reaction_time_s = float(reaction_time_s)
        self.jitter_window_s = float(jitter_window_s)
        self.latency_safety_factor = float(latency_safety_factor)
        self.brake_horizon_s = float(brake_horizon_s)
        self.commit_horizon_s = float(commit_horizon_s)
        self.update_interval_s = float(update_interval_s)
        self.verbose = bool(verbose)
        self._vehicle_mass = float(vehicle_mass)
        self._Lf = float(Lf); self._Lr = float(Lr)
        self._L = self._Lf + self._Lr
        # Static per-wheel normal loads (front + rear axle, 2 wheels each).
        g = 9.81
        self._Fz_front_static = vehicle_mass * g * self._Lr / self._L / 2.0
        self._Fz_rear_static  = vehicle_mass * g * self._Lf / self._L / 2.0

        # Analytical brake-decel cache built from the tire NN surrogate.
        # Maps n ∈ [0.40, 1.30] (step 0.05) → peak |F_x| per wheel × axle.
        self._brake_table: list[tuple[float, float]] = []   # (n, a_brake)
        if tire_model_dir is not None:
            self._build_brake_table(Path(tire_model_dir))

        # Latency tracking
        self._delay_history: deque = deque()  # (wall_time, one_way_delay)
        self._delay_ema = 0.0
        self._delay_ema_alpha = 0.20
        self._teleop_enabled = False
        self._last_cmd_wall: Optional[float] = None

        # Sticky severity to suppress 1-step flicker around thresholds
        self._last_severity = 0
        self._last_eval_time = -1.0
        self._last_warning: Optional[CollisionWarning] = None

    # ----- Latency API (mirrors CBFSafetyFilter) -----
    def set_teleop_delay(self, delay_s: float) -> None:
        self._delay_ema = max(delay_s, 0.0)
        self._teleop_enabled = self._delay_ema > 0.0

    def update_command_age(self, cmd_wall_time: float) -> None:
        """Record the wall-clock arrival of a new ControlCommand so we can
        track jitter and detect stale commands."""
        now = time.time()
        one_way = max(now - cmd_wall_time, 0.0)
        self._last_cmd_wall = now
        if self._teleop_enabled:
            a = self._delay_ema_alpha
            self._delay_ema = a * one_way + (1.0 - a) * self._delay_ema
        # Track for jitter window
        self._delay_history.append((now, one_way))
        cutoff = now - self.jitter_window_s
        while self._delay_history and self._delay_history[0][0] < cutoff:
            self._delay_history.popleft()

    def _jitter_estimate(self) -> float:
        """Std-dev of recent one-way delays over the jitter window."""
        if len(self._delay_history) < 4:
            return 0.0
        delays = [d for _, d in self._delay_history]
        mean = sum(delays) / len(delays)
        var = sum((d - mean) ** 2 for d in delays) / len(delays)
        return math.sqrt(var)

    # ----- Internal helpers -----
    def _build_brake_table(self, tire_model_dir) -> None:
        """At init time, sweep the tire NN surrogate across (n, slip-ratio)
        to find peak |F_x| per wheel for each axle, convert to maximum
        body deceleration, and store as a sorted lookup table over n.

        The model is queried at static per-wheel F_z (no load transfer)
        and a fixed reference speed (5 m/s — F_x weakly depends on u in
        the rate-augmented surrogate). Reload-per-n is acceptable here
        because the table is built once at init.
        """
        import sys as _sys
        from pathlib import Path as _Path
        _SIM = _Path(__file__).resolve().parents[1]
        if str(_SIM) not in _sys.path:
            _sys.path.insert(0, str(_SIM))
        from nn_tire_model import load_nn_tire_model     # noqa
        from param_consistency import (                   # noqa
            TERRAIN_PRESETS, terrain_preset_to_internal,
        )
        # Use the manifold interpolator from the runtime estimator so the
        # full Bekker vector at each `n` matches what the controller's
        # MPC sees.
        try:
            from learned_terrain_estimator import _terrain_params_for_n
        except Exception:
            # Fallback: just interp between presets directly
            presets = sorted(
                [(float(p["n"]), p) for p in
                 (terrain_preset_to_internal(TERRAIN_PRESETS[t])
                  for t in ("clay", "dirt", "sand"))],
                key=lambda x: x[0],
            )
            def _terrain_params_for_n(n_val):
                for i in range(len(presets) - 1):
                    n_a, p_a = presets[i]; n_b, p_b = presets[i + 1]
                    if n_a <= n_val <= n_b:
                        t = (n_val - n_a) / max(n_b - n_a, 1e-6)
                        return {k: float(p_a[k] + t * (p_b[k] - p_a[k]))
                                for k in p_a if isinstance(p_a[k], (int, float))}
                return presets[-1][1] if n_val > presets[-1][0] else presets[0][1]

        # Slip-ratio sweep range — go a bit past the rig's training edge
        # to make sure we capture the peak.
        kappas = [-0.40, -0.35, -0.30, -0.25, -0.20, -0.15, -0.10, -0.05, 0.0]
        n_grid = [round(0.40 + 0.05 * i, 2) for i in range(int((1.30 - 0.40) / 0.05) + 1)]
        u_ref = 5.0
        table = []
        for n_val in n_grid:
            try:
                terrain_params = _terrain_params_for_n(n_val)
                model = load_nn_tire_model(tire_model_dir,
                                            terrain_params=terrain_params)
                best_Fx_front = 0.0
                best_Fx_rear  = 0.0
                for k in kappas:
                    Fx_f, _ = model.predict_numeric(
                        0.0, self._Fz_front_static, u_ref, float(k))
                    Fx_r, _ = model.predict_numeric(
                        0.0, self._Fz_rear_static,  u_ref, float(k))
                    best_Fx_front = max(best_Fx_front, abs(float(Fx_f)))
                    best_Fx_rear  = max(best_Fx_rear,  abs(float(Fx_r)))
                # 2 wheels per axle × 2 axles
                total_Fx_max = 2.0 * best_Fx_front + 2.0 * best_Fx_rear
                a_brake = total_Fx_max / self._vehicle_mass
                table.append((float(n_val), float(a_brake)))
            except Exception:
                continue
        self._brake_table = table
        if self.verbose and table:
            print(f"  [warn] brake-decel table built from "
                  f"{tire_model_dir.name}: "
                  f"a(n=0.4)={table[0][1]:.2f}  "
                  f"a(n=0.7)={self._brake_decel_for_terrain(0.7):.2f}  "
                  f"a(n=1.1)={self._brake_decel_for_terrain(1.1):.2f}")

    def _brake_decel_for_terrain(self, n_value: Optional[float]) -> float:
        """Predicted maximum body deceleration for the given Bekker n.

        If the tire-NN backed lookup table is present, returns a linear
        interpolation of analytically computed peak braking force /
        vehicle mass; otherwise falls back to a hand-tuned linear
        interpolation between ``a_soft`` (soft soil) and ``a_dry``
        (firm soil) anchors.
        """
        if n_value is None or not math.isfinite(n_value):
            return self.a_dry
        if self._brake_table:
            n_val = float(n_value)
            # Clamp + linear interpolate the analytical table
            n_lo, a_lo = self._brake_table[0]
            n_hi, a_hi = self._brake_table[-1]
            if n_val <= n_lo:
                return a_lo
            if n_val >= n_hi:
                return a_hi
            for i in range(len(self._brake_table) - 1):
                n_a, a_a = self._brake_table[i]
                n_b, a_b = self._brake_table[i + 1]
                if n_a <= n_val <= n_b:
                    t = (n_val - n_a) / max(n_b - n_a, 1e-9)
                    return float(a_a + t * (a_b - a_a))
            return a_hi
        # Fallback: linear interp between anchor points
        t = max(0.0, min(1.0, (float(n_value) - 0.4) / (1.2 - 0.4)))
        return self.a_soft + t * (self.a_dry - self.a_soft)

    def _stopping_distance(self, v: float, a_brake: float,
                            tau_reaction_s: float) -> float:
        """Required distance to come to rest:
            d = v * tau_reaction + v² / (2 a_brake)
        where ``tau_reaction`` already includes baseline operator delay
        plus the latency-inflation term computed by the caller.
        """
        v = max(0.0, float(v))
        a = max(0.1, float(a_brake))
        return v * tau_reaction_s + (v * v) / (2.0 * a)

    def _severity_from_ttc_margin(self, ttc: float, margin: float) -> int:
        if margin <= 0.0 or ttc <= self.commit_horizon_s:
            return 3
        if ttc <= self.brake_horizon_s:
            return 2
        # Yellow band: closer than 2x brake horizon
        if ttc <= 2.0 * self.brake_horizon_s:
            return 1
        return 0

    # ----- Main API -----
    def evaluate(self,
                 vehicle_state: Dict[str, float],
                 obstacles: List[Tuple[float, float, float]],
                 terrain_n: Optional[float] = None,
                 ) -> CollisionWarning:
        """Compute the warning signal.

        Args:
            vehicle_state: dict with at minimum ``x, y, psi, u``. ``u`` is
                longitudinal velocity (m/s).
            obstacles: list of ``(x, y, radius)`` tuples.
            terrain_n: current estimated Bekker n. If ``None``, defaults to
                a conservative middle value.
        """
        now = time.time()
        if self._last_warning is not None and (
            now - self._last_eval_time < self.update_interval_s
        ):
            return self._last_warning
        self._last_eval_time = now

        x = float(vehicle_state.get("x", 0.0))
        y = float(vehicle_state.get("y", 0.0))
        psi = float(vehicle_state.get("psi", 0.0))
        u = max(0.0, float(vehicle_state.get("u", 0.0)))

        a_brake = self._brake_decel_for_terrain(terrain_n)
        jitter_s = self._jitter_estimate()
        # Total operator reaction time: baseline + RTT + jitter padding
        rtt = 2.0 * (self._delay_ema if self._teleop_enabled else 0.0)
        tau_reaction = (self.reaction_time_s
                        + rtt
                        + self.latency_safety_factor * jitter_s)
        latency_inflation_m = u * (rtt + self.latency_safety_factor * jitter_s)

        d_stop = self._stopping_distance(u, a_brake, tau_reaction)

        # Find the worst obstacle in the forward cone
        cos_psi = math.cos(psi)
        sin_psi = math.sin(psi)
        worst_ttc = float("inf")
        worst_obs: Optional[Tuple[float, float, float]] = None
        worst_clearance = float("inf")
        worst_margin = float("inf")
        for ox, oy, oradius in obstacles:
            dx_world = ox - x
            dy_world = oy - y
            d_along = dx_world * cos_psi + dy_world * sin_psi    # forward
            d_cross = -dx_world * sin_psi + dy_world * cos_psi   # left-positive
            safe_r = float(oradius) + self.vehicle_radius
            # Skip obstacles already behind the vehicle
            if d_along < -0.5:
                continue
            # Lateral miss (already going to clear) — small cross influence
            if abs(d_cross) > 1.5 * safe_r + 1.5:
                continue
            clearance = math.sqrt(dx_world ** 2 + dy_world ** 2) - safe_r
            ttc = (d_along - safe_r) / max(u, 0.1)
            margin = clearance - d_stop
            if ttc < worst_ttc or (ttc == worst_ttc and margin < worst_margin):
                worst_ttc = ttc
                worst_obs = (float(ox), float(oy), float(oradius))
                worst_clearance = clearance
                worst_margin = margin

        if worst_obs is None:
            severity = 0
            ttc = float("inf")
            clearance = float("inf")
            margin = float("inf")
        else:
            severity = self._severity_from_ttc_margin(worst_ttc, worst_margin)
            ttc = worst_ttc
            clearance = worst_clearance
            margin = worst_margin

        msg = (f"{_SEVERITY_NAMES[severity]}  v={u:.1f}m/s  "
               f"ttc={ttc:.2f}s  clr={clearance:.2f}m  "
               f"stop={d_stop:.2f}m  margin={margin:+.2f}m")

        warning = CollisionWarning(
            severity=int(severity),
            ttc=float(ttc),
            closest_obstacle=worst_obs,
            stopping_distance=float(d_stop),
            clearance=float(clearance),
            margin=float(margin),
            latency_inflation_m=float(latency_inflation_m),
            terrain_n_used=float(terrain_n if terrain_n is not None else 1.0),
            message=msg,
        )
        # Verbose-print only when severity changes
        if self.verbose and severity != self._last_severity:
            print(f"  [WARN] {_SEVERITY_NAMES[self._last_severity]} → "
                  f"{_SEVERITY_NAMES[severity]}  {msg}")
        self._last_severity = severity
        self._last_warning = warning
        return warning


# ---------- Module factory (mirrors safety.make_safety_filter) ---------
def make_collision_warning_system(flavor: str = "ttc", **kwargs):
    """Factory for collision-warning flavors.

    Current flavors:
        * ``"ttc"`` — terrain + latency-aware TTC warning (the default).

    Additional flavors can be added (Bayesian, learned-risk, ...) and
    selected here without touching the call sites that consume the
    warning signal.
    """
    f = (flavor or "ttc").lower()
    if f in ("ttc", "default"):
        return CollisionWarningSystem(**kwargs)
    raise ValueError(f"Unknown collision-warning flavor {flavor!r}; "
                     f"expected one of: 'ttc'")
