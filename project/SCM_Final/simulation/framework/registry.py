"""Plain decorator-based registries — one per role.

The registry stores ``name -> factory_callable``. A factory is anything
that takes ``**kwargs`` and returns an instance of the role; this is
deliberately wider than "must be a class" so that legacy
``make_safety_filter`` / ``make_collision_warning_system`` factories
can be wrapped without rewriting them.

Usage:

    >>> from simulation.framework import SAFETY_FILTERS
    >>> @SAFETY_FILTERS.register("custom")
    ... class CustomFilter:
    ...     ...

    >>> # In launch_decoupled.py:
    >>> shield = SAFETY_FILTERS.create("custom", vehicle_params=...)

The benchmark suite asks each registry for ``list_flavors()`` so
``--help`` always reflects the actually registered set, not a hardcoded
choices list.
"""
from __future__ import annotations

from typing import Any, Callable, Dict, List


class Registry:
    """Name-keyed factory registry. One instance per role.

    Args:
        role: short human label used in error messages (e.g. ``"safety
            filter"``).
    """

    def __init__(self, role: str) -> None:
        self.role = role
        self._factories: Dict[str, Callable[..., Any]] = {}
        # name aliases let us preserve historical CLI strings without
        # duplicating registrations (e.g. "dob-cbf" -> "dob_cbf").
        self._aliases: Dict[str, str] = {}

    # ------------------------------------------------------------------
    # registration
    # ------------------------------------------------------------------

    def register(self, name: str, *aliases: str) -> Callable[[Callable[..., Any]], Callable[..., Any]]:
        """Decorator that registers a class or factory under ``name``.

        Optional positional ``aliases`` map alternate CLI strings onto
        the same canonical entry.
        """

        canonical = name.lower()

        def _wrap(factory: Callable[..., Any]) -> Callable[..., Any]:
            if canonical in self._factories:
                raise ValueError(
                    f"{self.role} flavor {canonical!r} is already registered"
                )
            self._factories[canonical] = factory
            for alias in aliases:
                self._aliases[alias.lower()] = canonical
            return factory

        return _wrap

    def register_value(self, name: str, factory: Callable[..., Any], *aliases: str) -> None:
        """Imperative form of :meth:`register`. Useful when you already
        have the class object and don't want to add a decorator at the
        definition site.
        """
        self.register(name, *aliases)(factory)

    # ------------------------------------------------------------------
    # lookup / construction
    # ------------------------------------------------------------------

    def _resolve(self, name: str) -> str:
        key = (name or "").lower()
        key = self._aliases.get(key, key)
        if key not in self._factories:
            raise ValueError(
                f"Unknown {self.role} flavor {name!r}. "
                f"Registered: {sorted(self._factories)}"
            )
        return key

    def get(self, name: str) -> Callable[..., Any]:
        """Return the registered factory for ``name`` without invoking it."""
        return self._factories[self._resolve(name)]

    def create(self, name: str, **kwargs: Any) -> Any:
        """Resolve ``name`` and instantiate the factory with ``kwargs``."""
        return self.get(name)(**kwargs)

    def list_flavors(self) -> List[str]:
        """Sorted list of *canonical* names (excludes aliases)."""
        return sorted(self._factories)

    def __contains__(self, name: object) -> bool:
        if not isinstance(name, str):
            return False
        try:
            self._resolve(name)
        except ValueError:
            return False
        return True

    def __repr__(self) -> str:
        return f"Registry({self.role!r}: {self.list_flavors()})"


# ---------------------------------------------------------------------------
# One registry per role. ``launch_decoupled.py`` and the benchmarking
# suite only ever import these names.
# ---------------------------------------------------------------------------

COMMAND_SOURCES     = Registry("command source")
SAFETY_FILTERS      = Registry("safety filter")
COLLISION_WARNINGS  = Registry("collision warning")
TIRE_MODELS         = Registry("tire model")
TERRAIN_ESTIMATORS  = Registry("terrain estimator")
LATENCY_PROFILES    = Registry("latency profile")


REGISTRIES: Dict[str, Registry] = {
    "command_source":     COMMAND_SOURCES,
    "safety_filter":      SAFETY_FILTERS,
    "collision_warning":  COLLISION_WARNINGS,
    "tire_model":         TIRE_MODELS,
    "terrain_estimator":  TERRAIN_ESTIMATORS,
    "latency_profile":    LATENCY_PROFILES,
}
