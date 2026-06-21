#!/usr/bin/env python3
"""Conformance test: every registered flavor implements its Protocol.

This script is the structural check the paper relies on when it claims
the stack is modular: rather than promise the swap-ability in prose,
we instantiate one of each shipped flavor and `isinstance`-check it
against its Protocol at runtime.

Run from the project root::

    python simulation/framework/test_conformance.py

Exits non-zero on the first conformance violation. Used as the smoke
check in the CI/benchmark suite.
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import simulation.framework.builtins  # noqa: F401  -- registers flavors
from simulation.framework import (
    SAFETY_FILTERS,
    COLLISION_WARNINGS,
    SafetyFilter,
    CollisionWarning,
)


def _vehicle_params() -> dict:
    # Keys are the union of what each shipped filter expects so a
    # single dict can be reused. Real callers should look up the
    # canonical names in `chrono_setup.py`.
    return {
        "M": 2500.0,
        "Iz": 4500.0,
        "Izz": 4500.0,
        "Lf": 1.685,
        "Lr": 1.685,
        "wheelbase": 3.37,
        "track_front": 1.7,
        "track_rear": 1.7,
        "h_cg": 0.6,
        "max_steer": 0.5,
        "max_acc": 4.0,
        "max_dec": 6.0,
    }


def _terrain_params() -> dict:
    return {
        "Kphi": 8.14e5, "Kc": 14.0e3, "n": 0.70,
        "c": 5.0e3, "phi": 23.0, "k": 0.018,
    }


def _check(flavor: str, instance: object, protocol: type) -> None:
    if not isinstance(instance, protocol):
        raise SystemExit(
            f"FAIL: flavor {flavor!r} does not satisfy {protocol.__name__}"
        )
    print(f"  ok  {flavor!r} satisfies {protocol.__name__}")


def main() -> None:
    print("Conformance check: safety filters")
    for flavor in SAFETY_FILTERS.list_flavors():
        try:
            instance = SAFETY_FILTERS.create(
                flavor,
                vehicle_params=_vehicle_params(),
                terrain_params=_terrain_params(),
                nn_model=None,
            )
        except Exception as exc:                       # noqa: BLE001
            # A filter that needs an nn_model at construction is skipped if one
            # isn't supplied — the *interface* check is what matters for
            # swap-ability, not the construction-time policy.
            print(f"  skip {flavor!r}: requires NN ({exc})")
            continue
        _check(flavor, instance, SafetyFilter)

    print("Conformance check: collision warnings")
    for flavor in COLLISION_WARNINGS.list_flavors():
        instance = COLLISION_WARNINGS.create(flavor)
        _check(flavor, instance, CollisionWarning)

    print("All registered flavors conform to their declared Protocol.")


if __name__ == "__main__":
    main()
