"""Register every shipped concrete class against its role registry.

Importing this module has the side effect of populating
:data:`SAFETY_FILTERS`, :data:`COLLISION_WARNINGS`,
:data:`LATENCY_PROFILES`, and the tire / terrain registries with every
flavor the framework currently knows about. The wiring is intentionally
done here (and not inside each concrete module) so the legacy modules
in ``simulation/safety/`` and ``simulation/latency_profile.py`` keep
their existing import-time behaviour and have no new dependency on the
framework package.

Idempotent: importing twice is a no-op.

External code only has to do::

    import simulation.framework.builtins  # registers everything

and then::

    from simulation.framework import SAFETY_FILTERS
    shield = SAFETY_FILTERS.create("dob_cbf", vehicle_params=...)
"""
from __future__ import annotations

import sys

from .registry import (
    SAFETY_FILTERS,
    COLLISION_WARNINGS,
    LATENCY_PROFILES,
)


_REGISTERED_FLAG = "_scm_framework_builtins_registered"
_already = getattr(sys.modules[__name__], _REGISTERED_FLAG, False)


if not _already:
    # ----- safety filters -------------------------------------------------
    # DOB-CBF is the only shipped filter (intent-preserving). The MPPI and NMPC
    # shields were archived 2026-06-21 (archive/2026-06-21_mppi_nmpc_removal/);
    # the registry stays swappable so a new filter can be registered here.
    from simulation.safety import CBFSafetyFilter, make_safety_filter

    SAFETY_FILTERS.register_value(
        "dob_cbf",
        lambda **kw: make_safety_filter("dob_cbf", **kw),
        "cbf", "dob-cbf", "legacy",
    )

    # ----- collision warning ---------------------------------------------
    from simulation.safety.collision_warning import make_collision_warning_system

    COLLISION_WARNINGS.register_value(
        "ttc",
        lambda **kw: make_collision_warning_system("ttc", **kw),
        "default", "terrain_latency",
    )

    # ----- latency profiles ----------------------------------------------
    # `simulation.latency_profile` exposes a single factory keyed on a
    # profile JSON; the registry surfaces it under the names the CLI uses.
    from simulation.latency_profile import LatencyProfile  # type: ignore

    LATENCY_PROFILES.register_value(
        "json",
        lambda profile_json, **kw: LatencyProfile(profile_json=profile_json, **kw),
        "5g_nhits", "trace", "replay",
    )

    setattr(sys.modules[__name__], _REGISTERED_FLAG, True)
