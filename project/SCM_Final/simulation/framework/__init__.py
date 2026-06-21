"""SCM_Final framework — explicit swap-point interfaces and registry.

The whole stack is organised around six runtime roles. Each role has a
single ``Protocol`` class in :mod:`simulation.framework.interfaces` and
a single ``Registry`` in :mod:`simulation.framework.registry`. Adding a
new implementation is one decorator and zero edits to the consumers:

    >>> from simulation.framework import SAFETY_FILTERS, SafetyFilter
    >>> @SAFETY_FILTERS.register("my_new_filter")
    ... class MyFilter:
    ...     def filter(self, s, t, b, state, obs): ...
    ...     # plus update_command_age, set_teleop_delay, get_diagnostics
    >>> SAFETY_FILTERS.create("my_new_filter", vehicle_params=..., ...)

The roles are:

* ``COMMAND_SOURCES``  — ``CommandSource``  (NMPC, G29, WASD, remote)
* ``SAFETY_FILTERS``   — ``SafetyFilter``   (DOB-CBF; registry is swappable)
* ``COLLISION_WARNINGS`` — ``CollisionWarning`` (TTC + variants)
* ``TIRE_MODELS``      — ``TireModel``      (NN surrogates + analytical)
* ``TERRAIN_ESTIMATORS`` — ``TerrainEstimator`` (n-only, joint, UKF...)
* ``LATENCY_PROFILES`` — ``LatencyProfile`` (constant, learned 5G, replay)

``Registry.create(name, **kwargs)`` is the single entry point for
``launch_decoupled.py`` and the benchmarking harness. The CLI flags
that select a flavor (``--safety-flavor``, ``--cw-flavor`` etc.) map
1:1 onto registered names.
"""
from .interfaces import (
    CommandSource,
    SafetyFilter,
    CollisionWarning,
    TireModel,
    TerrainEstimator,
    LatencyProfile,
    SafetyFilterResult,
    CollisionWarningSignal,
    DriverCommand,
)
from .registry import Registry, REGISTRIES
from .registry import (
    COMMAND_SOURCES,
    SAFETY_FILTERS,
    COLLISION_WARNINGS,
    TIRE_MODELS,
    TERRAIN_ESTIMATORS,
    LATENCY_PROFILES,
)

__all__ = [
    "CommandSource",
    "SafetyFilter",
    "CollisionWarning",
    "TireModel",
    "TerrainEstimator",
    "LatencyProfile",
    "SafetyFilterResult",
    "CollisionWarningSignal",
    "DriverCommand",
    "Registry",
    "REGISTRIES",
    "COMMAND_SOURCES",
    "SAFETY_FILTERS",
    "COLLISION_WARNINGS",
    "TIRE_MODELS",
    "TERRAIN_ESTIMATORS",
    "LATENCY_PROFILES",
]
