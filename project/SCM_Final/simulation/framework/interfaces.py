"""Protocol-based interfaces for the six swap points of the SCM_Final stack.

Each ``Protocol`` documents the *minimum* surface that a flavor must
expose to participate as that role. The runtime never instantiates the
Protocol — it instantiates a concrete class (`CBFSafetyFilter`,
`CollisionWarningSystem`, …) and the protocol exists so ``mypy``,
``pyright``, or a unit test can prove the swap is structural rather
than a duck-type accident.

All protocols are ``@runtime_checkable`` so callers can also do
``isinstance(thing, SafetyFilter)`` at runtime when they want a defence
in depth.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Protocol, Tuple, runtime_checkable


# ---------------------------------------------------------------------------
# Result / signal value objects shared across roles
# ---------------------------------------------------------------------------


@dataclass
class DriverCommand:
    """Normalised actuator command produced by a ``CommandSource``.

    ``steering`` is in ``[-1, 1]``, throttle and braking in ``[0, 1]``.
    ``wall_time`` is the producer's monotonic timestamp; the sim node
    uses it to compute command age for latency-aware filters.
    """

    steering: float
    throttle: float
    braking: float
    wall_time: float


@dataclass
class SafetyFilterResult:
    """Output of any ``SafetyFilter.filter(...)`` call.

    Mirrors the existing ``simulation.safety.SafetyFilterResult`` so
    that the protocol layer is binary-compatible with the legacy
    factory output. Extra diagnostic fields are optional.
    """

    steering: float
    throttle: float
    braking: float
    was_modified: bool
    active_constraints: int = 0
    solve_time_ms: float = 0.0
    safety_margin: float = 0.0
    diagnostics: Optional[Dict[str, float]] = None


@dataclass
class CollisionWarningSignal:
    """Output of any ``CollisionWarning.evaluate(...)`` call.

    Mirrors :class:`simulation.safety.collision_warning.CollisionWarning`
    but is re-declared here so the framework layer has no dependency on
    the safety package.
    """

    severity: int                                       # 0..3
    ttc: float                                          # s
    closest_obstacle: Optional[Tuple[float, float, float]]
    stopping_distance: float                            # m
    clearance: float                                    # m
    margin: float                                       # clearance - d_stop
    terrain_n_used: float
    message: str


# ---------------------------------------------------------------------------
# Role protocols
# ---------------------------------------------------------------------------


@runtime_checkable
class CommandSource(Protocol):
    """Anything that can produce a :class:`DriverCommand`.

    Implementations: ``acados_mpc_controller_node`` (over ZMQ; adapted
    by the sim-node receive loop), ``g29_controller.G29Controller``,
    ``g29_controller.WASDController``.
    """

    def next_command(self, state: Any) -> DriverCommand: ...  # pragma: no cover


@runtime_checkable
class SafetyFilter(Protocol):
    """Minimum-deviation screen of one driver command.

    The shipped implementation is :class:`CBFSafetyFilter` (DOB-CBF) in
    :mod:`simulation.safety`; the factory
    :func:`simulation.safety.make_safety_filter` returns it keyed by
    ``--safety-flavor dob_cbf``. (The MPPI/NMPC shields were archived
    2026-06-21; the registry remains swappable for new filters.)
    """

    def filter(
        self,
        desired_steering: float,
        desired_throttle: float,
        desired_brake: float,
        vehicle_state: Any,
        obstacles: List[Any],
    ) -> SafetyFilterResult: ...  # pragma: no cover

    def update_command_age(self, command_wall_time: float) -> None: ...  # pragma: no cover

    def set_teleop_delay(self, delay_s: float) -> None: ...  # pragma: no cover

    def get_diagnostics(self) -> Dict[str, float]: ...  # pragma: no cover


@runtime_checkable
class CollisionWarning(Protocol):
    """Forward collision-warning advisory — never modifies commands.

    Implementations live in
    :mod:`simulation.safety.collision_warning`. The default flavor is
    ``CollisionWarningSystem`` (TTC + analytical brake table +
    latency-inflated reaction time). Live terrain ``n̂`` is passed
    directly as an argument to ``evaluate(...)`` so the warning can
    re-index its brake-decel table per tick.
    """

    def evaluate(
        self,
        *,
        x: float,
        y: float,
        yaw: float,
        v: float,
        obstacles: List[Any],
        terrain_n: Optional[float] = None,
    ) -> CollisionWarningSignal: ...  # pragma: no cover

    def set_teleop_delay(self, delay_s: float) -> None: ...  # pragma: no cover

    def update_command_age(self, cmd_wall_time: float) -> None: ...  # pragma: no cover


@runtime_checkable
class TireModel(Protocol):
    """Differentiable tire force surrogate consumed by the NMPC.

    Implementations live in :mod:`simulation.nn_tire_model`
    (``StaticMLP``, ``RateMLP``, ``AxleRateMLP``, ...) and
    :mod:`simulation.analytical_tire_models` (Pacejka, TMeasy).
    The acados solver pulls a CasADi expression through ``get_casadi``.
    """

    def predict(
        self,
        alpha: float,
        Fz: float,
        u: float,
        kappa: float = 0.0,
        n_terrain: Optional[float] = None,
        steering_rate: float = 0.0,
    ) -> Tuple[float, float]: ...  # pragma: no cover


@runtime_checkable
class TerrainEstimator(Protocol):
    """Online soil-parameter estimator (n-only deployed, joint optional).

    Implementations live in :mod:`simulation.learned_terrain_estimator`.
    The deployed flavor is the sliding-window MLP returning Bekker `n`.
    The controller maps `n̂` along the retained clay-dirt-sand manifold
    to recover the full Bekker-Mohr vector.
    """

    def observe(self, state: Any, t: float) -> None: ...  # pragma: no cover

    def estimate(self) -> Tuple[Dict[str, float], float]: ...  # pragma: no cover

    def get_bekker_n(self) -> float: ...  # pragma: no cover

    def get_terrain_mpc_params(self) -> Dict[str, float]: ...  # pragma: no cover


@runtime_checkable
class LatencyProfile(Protocol):
    """Time-varying delay model applied per channel.

    Implementations live in :mod:`simulation.latency_profile`:
    constant delay, replay of a recorded trace, and the learned N-HiTS
    5G generator. The sim node calls ``delay(t, channel)`` once per
    tick to retrieve the current delay on a named channel
    (``"control"``, ``"manual"``, ``"camera"``).
    """

    def delay(self, t: float, channel: str) -> float: ...  # pragma: no cover
