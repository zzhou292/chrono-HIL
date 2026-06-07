"""Spatially-varying SCM soil: one preset, a blend zone, then another preset.

PyChrono's :class:`SCMTerrain` queries soil parameters per contact location
through a registered ``SoilParametersCallback``, so a terrain that changes soil
type partway across the patch is *simulated in the physics* rather than faked by
a global parameter switch. The vehicle drives forward along +x; the soil is one
preset for small x, blends linearly over a short zone, and becomes a second
preset for large x.

Two pieces share a single blend function so they can never disagree:

* :func:`local_soil_at` / :func:`local_n_at` -- the pure ground-truth soil
  field as a function of x. A benchmark reconstructs the exact n(x) the
  simulator used by calling these with the vehicle's logged position.
* :class:`TransitionSoilCallback` -- the PyChrono callback that applies that
  same field at every SCM contact node during the run.
"""

from __future__ import annotations

from dataclasses import dataclass

import pychrono.vehicle as veh

from param_consistency import TERRAIN_PRESETS

# The eight scalars SCMTerrain.SetSoilParameters / the callback expect, in order.
# friction_angle is in DEGREES (Chrono's convention), not radians.
SOIL_KEYS = (
    "Kphi", "Kc", "n", "cohesion", "friction_angle", "janosi_shear",
    "elastic_stiffness", "damping",
)

_DEFAULTS = {"elastic_stiffness": 2e8, "damping": 3e4}


@dataclass(frozen=True)
class SpatialTransitionSpec:
    """Describes a soil transition along the +x driving direction.

    Attributes:
        start_preset: Soil preset for x below the blend zone (e.g. ``"clay"``).
        end_preset:   Soil preset for x above the blend zone (e.g. ``"sand"``).
        transition_x: Center of the blend zone, in terrain/global x (m).
        transition_width: Full width of the linear blend (m). ``0`` is a hard
            step at ``transition_x``; a few metres gives a physically realistic
            boundary and avoids an SCM contact-force discontinuity.
    """

    start_preset: str
    end_preset: str
    transition_x: float
    transition_width: float = 2.0

    def __post_init__(self):
        for name in (self.start_preset, self.end_preset):
            if name not in TERRAIN_PRESETS:
                raise ValueError(
                    f"Unknown terrain preset: {name!r}. "
                    f"Available: {list(TERRAIN_PRESETS.keys())}"
                )
        if self.transition_width < 0:
            raise ValueError("transition_width must be >= 0")


def _preset_vec(name: str) -> dict[str, float]:
    """The eight soil scalars for a preset, filling Chrono defaults if absent."""
    preset = TERRAIN_PRESETS[name]
    return {k: float(preset.get(k, _DEFAULTS.get(k, 0.0))) for k in SOIL_KEYS}


def blend_fraction(x: float, spec: SpatialTransitionSpec) -> float:
    """Fraction of the way from start to end soil at position ``x`` (0..1)."""
    if spec.transition_width <= 0.0:
        return 0.0 if x < spec.transition_x else 1.0
    half = spec.transition_width / 2.0
    s = (x - (spec.transition_x - half)) / spec.transition_width
    if s < 0.0:
        return 0.0
    if s > 1.0:
        return 1.0
    return s


def local_soil_at(x: float, spec: SpatialTransitionSpec) -> dict[str, float]:
    """Ground-truth soil parameter vector at position ``x`` (linear blend)."""
    s = blend_fraction(x, spec)
    a = _preset_vec(spec.start_preset)
    b = _preset_vec(spec.end_preset)
    return {k: (1.0 - s) * a[k] + s * b[k] for k in SOIL_KEYS}


def local_n_at(x: float, spec: SpatialTransitionSpec) -> float:
    """Ground-truth Bekker sinkage exponent ``n`` at position ``x``."""
    s = blend_fraction(x, spec)
    a = _preset_vec(spec.start_preset)["n"]
    b = _preset_vec(spec.end_preset)["n"]
    return (1.0 - s) * a + s * b


class TransitionSoilCallback(veh.SoilParametersCallback):
    """SCM per-location soil callback applying a :class:`SpatialTransitionSpec`.

    Register on an ``SCMTerrain`` with ``terrain.RegisterSoilParametersCallback``.
    Keep a Python reference to the instance alive for the lifetime of the
    terrain (the SWIG director is owned by Python).
    """

    def __init__(self, spec: SpatialTransitionSpec):
        veh.SoilParametersCallback.__init__(self)
        self.spec = spec
        self._a = _preset_vec(spec.start_preset)
        self._b = _preset_vec(spec.end_preset)

    def Set(self, loc, Bekker_Kphi, Bekker_Kc, Bekker_n, Mohr_cohesion,
            Mohr_friction, Janosi_shear, elastic_K, damping_R):
        s = blend_fraction(loc.x, self.spec)
        a, b = self._a, self._b

        def mix(key):
            return (1.0 - s) * a[key] + s * b[key]

        veh.doublep_assign(Bekker_Kphi, mix("Kphi"))
        veh.doublep_assign(Bekker_Kc, mix("Kc"))
        veh.doublep_assign(Bekker_n, mix("n"))
        veh.doublep_assign(Mohr_cohesion, mix("cohesion"))
        veh.doublep_assign(Mohr_friction, mix("friction_angle"))
        veh.doublep_assign(Janosi_shear, mix("janosi_shear"))
        veh.doublep_assign(elastic_K, mix("elastic_stiffness"))
        veh.doublep_assign(damping_R, mix("damping"))
