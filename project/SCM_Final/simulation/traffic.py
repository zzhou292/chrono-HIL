#!/usr/bin/env python3
"""PID-driven traffic vehicles for convoy safety scenarios.

Spawns additional HMMWVs into the *same* ``ChSystem`` as the ego (so they
share the SCM terrain and collision system and physically interact), each
driven by a ``ChPathFollowerDriver`` along a straight lane at a target speed.
Scriptable *hazards* perturb the driver output to reproduce dangerous convoy
behaviour the ego must avoid: a lead vehicle slamming the brakes, a cut-in, a
stall in the lane, or erratic swerving.

This is deliberately single-system multi-vehicle (not SynChrono): SynChrono is
for *distributed* multi-agent simulation where other agents are zombie copies,
which does not give the physical ego-vs-traffic contact dynamics a collision-
avoidance study needs.

Usage (from chrono_sim_node, after the ego vehicle + terrain exist):
    mgr = TrafficManager.from_preset("lead_brake", ego_lane_y=0.0)
    mgr.build(system, terrain)
    # per step, inside the ego loop:
    mgr.synchronize(t); ... ; mgr.advance(step)
    obstacles = mgr.obstacles()   # [(x, y, radius), ...] dynamic obstacles
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import pychrono as chrono
import pychrono.vehicle as veh


def _viz(name: str):
    """Resolve a VisualizationType enum across PyChrono API variants."""
    attr = f"VisualizationType_{name}"
    return getattr(veh, attr, None) or getattr(chrono, attr)

# A HMMWV is ~4.8 m x 2.1 m; this conservative bounding radius is what the ego's
# obstacle pipeline / safety filter sees for each traffic vehicle.
TRAFFIC_RADIUS = 2.2


@dataclass
class Hazard:
    """A scripted perturbation of a traffic vehicle's driver output.

    kind:
      'brake'  -- throttle 0, full brake (lead slams the brakes)
      'stop'   -- decelerate and hold (stall in the lane)
      'cut_in' -- steer toward ``params['toward_y']`` (lateral lunge into a lane)
      'swerve' -- sinusoidal steering of amplitude ``params['amp']``
      'slow'   -- scale throttle by ``params['factor']``
    """
    start_t: float
    dur: float
    kind: str
    params: dict = field(default_factory=dict)

    def active(self, t: float) -> bool:
        return self.start_t <= t < self.start_t + self.dur


@dataclass
class TrafficSpec:
    init_x: float
    init_y: float
    speed: float
    heading_deg: float = 0.0
    hazards: list[Hazard] = field(default_factory=list)


class TrafficVehicle:
    """One PID-driven HMMWV on the shared system, with hazard scripting."""

    def __init__(self, spec: TrafficSpec):
        self.spec = spec
        self.vehicle = None
        self.driver = None

    def build(self, system, terrain, add_patch, detail: str = "mesh") -> None:
        s = self.spec
        yaw = math.radians(s.heading_deg)
        tv = veh.HMMWV_Reduced(system)
        tv.SetChassisFixed(False)
        tv.SetChassisCollisionType(veh.CollisionType_PRIMITIVES)
        tv.SetInitPosition(chrono.ChCoordsysd(
            chrono.ChVector3d(s.init_x, s.init_y, 1.5),
            chrono.ChQuaterniond(math.cos(yaw / 2), 0, 0, math.sin(yaw / 2))))
        tv.SetEngineType(veh.EngineModelType_SHAFTS)
        tv.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
        tv.SetDriveType(veh.DrivelineTypeWV_AWD)
        tv.SetTireType(veh.TireModelType_RIGID)
        tv.Initialize()
        # detail: 'mesh' (pretty), 'primitives' (cheap boxes -> real-time for
        # big multi-vehicle scenes), or 'none' (headless, no visual assets).
        if detail in ("mesh", "primitives"):
            vt = _viz("MESH" if detail == "mesh" else "PRIMITIVES")
            tv.SetChassisVisualizationType(vt)
            tv.SetWheelVisualizationType(vt)
            tv.SetTireVisualizationType(vt)
            tv.SetSuspensionVisualizationType(_viz("PRIMITIVES"))
            tv.SetSteeringVisualizationType(_viz("PRIMITIVES"))
        self.vehicle = tv

        if add_patch is not None:
            for ax in tv.GetVehicle().GetAxles():
                for w in (ax.m_wheels[0], ax.m_wheels[1]):
                    add_patch(w.GetSpindle(), chrono.ChVector3d(0, 0, 0),
                              chrono.ChVector3d(1, 0.5, 1))

        # Straight lane path along the heading from the spawn point.
        dx, dy = math.cos(yaw), math.sin(yaw)
        path = veh.StraightLinePath(
            chrono.ChVector3d(s.init_x, s.init_y, 0.5),
            chrono.ChVector3d(s.init_x + 300.0 * dx, s.init_y + 300.0 * dy, 0.5), 1)
        drv = veh.ChPathFollowerDriver(tv.GetVehicle(), path, "traffic", s.speed)
        drv.GetSteeringController().SetLookAheadDistance(5.0)
        drv.GetSteeringController().SetGains(0.8, 0, 0)
        drv.GetSpeedController().SetGains(0.4, 0.0, 0.0)
        drv.Initialize()
        self.driver = drv

    def _apply_hazards(self, t: float, inp) -> None:
        for hz in self.spec.hazards:
            if not hz.active(t):
                continue
            if hz.kind in ("brake", "stop"):
                inp.m_throttle = 0.0
                inp.m_braking = 1.0
            elif hz.kind == "slow":
                inp.m_throttle *= float(hz.params.get("factor", 0.3))
            elif hz.kind == "swerve":
                amp = float(hz.params.get("amp", 0.4))
                period = float(hz.params.get("period", 2.0))
                inp.m_steering = max(-1.0, min(1.0,
                    inp.m_steering + amp * math.sin(2 * math.pi * (t - hz.start_t) / period)))
            elif hz.kind == "cut_in":
                toward_y = float(hz.params.get("toward_y", 0.0))
                y = self.vehicle.GetVehicle().GetPos().y
                bias = float(hz.params.get("gain", 0.6)) * (toward_y - y)
                inp.m_steering = max(-1.0, min(1.0, inp.m_steering + bias))

    def synchronize(self, t: float, terrain) -> None:
        self.driver.Synchronize(t)
        inp = self.driver.GetInputs()
        self._apply_hazards(t, inp)
        self._last_inputs = inp
        self.vehicle.Synchronize(t, inp, terrain)

    def advance(self, step: float) -> None:
        self.driver.Advance(step)
        self.vehicle.Advance(step)

    def state(self) -> dict:
        v = self.vehicle.GetVehicle()
        p = v.GetPos()
        rot = v.GetRot()
        psi = math.atan2(2 * (rot.e0 * rot.e3 + rot.e1 * rot.e2),
                         1 - 2 * (rot.e2 * rot.e2 + rot.e3 * rot.e3))
        return {"x": p.x, "y": p.y, "z": p.z, "psi": psi, "speed": v.GetSpeed(), "r": TRAFFIC_RADIUS}


class TrafficManager:
    """Builds and steps a set of traffic vehicles; exposes them as obstacles."""

    def __init__(self, specs: list[TrafficSpec]):
        self.specs = specs
        self.vehicles: list[TrafficVehicle] = []

    @classmethod
    def from_preset(cls, name: str, ego_lane_y: float = 0.0) -> "TrafficManager":
        if name not in CONVOY_PRESETS:
            raise ValueError(f"unknown convoy preset '{name}'; have {sorted(CONVOY_PRESETS)}")
        return cls(CONVOY_PRESETS[name](ego_lane_y))

    def build(self, system, terrain, detail: str = "mesh") -> None:
        """detail: 'mesh' | 'primitives' | 'none' | 'auto' (mesh<=3 else primitives)."""
        if detail == "auto":
            detail = "mesh" if len(self.specs) <= 3 else "primitives"
        add_patch = (getattr(terrain, "AddActiveDomain", None)
                     or getattr(terrain, "AddMovingPatch", None))
        for spec in self.specs:
            tv = TrafficVehicle(spec)
            tv.build(system, terrain, add_patch, detail=detail)
            self.vehicles.append(tv)

    def synchronize(self, t: float, terrain) -> None:
        for tv in self.vehicles:
            tv.synchronize(t, terrain)

    def advance(self, step: float) -> None:
        for tv in self.vehicles:
            tv.advance(step)

    def obstacles(self) -> list[tuple[float, float, float]]:
        """Current traffic poses as (x, y, radius) dynamic obstacles."""
        return [(s["x"], s["y"], s["r"]) for s in (tv.state() for tv in self.vehicles)]

    def states(self) -> list[dict]:
        return [tv.state() for tv in self.vehicles]


# ---------------------------------------------------------------------------
# Convoy presets: each returns a list of TrafficSpec given the ego's lane y.
# The ego starts at (0,0) heading +x; traffic is placed ahead / alongside.
# ---------------------------------------------------------------------------
def _lead_brake(ego_y: float) -> list[TrafficSpec]:
    # A lead vehicle ahead in the ego's lane that slams the brakes at t=6s.
    return [TrafficSpec(init_x=18.0, init_y=ego_y, speed=4.0,
                        hazards=[Hazard(6.0, 4.0, "brake")])]


def _cut_in(ego_y: float) -> list[TrafficSpec]:
    # A vehicle in the next lane that lunges into the ego's lane at t=5s.
    return [TrafficSpec(init_x=14.0, init_y=ego_y + 3.5, speed=4.5,
                        hazards=[Hazard(5.0, 3.0, "cut_in", {"toward_y": ego_y})])]


def _stalled(ego_y: float) -> list[TrafficSpec]:
    # A stalled vehicle blocking the lane from the start.
    return [TrafficSpec(init_x=30.0, init_y=ego_y, speed=0.0,
                        hazards=[Hazard(0.0, 60.0, "stop")])]


def _swerver(ego_y: float) -> list[TrafficSpec]:
    # An erratic lead vehicle that swerves within the lane.
    return [TrafficSpec(init_x=18.0, init_y=ego_y, speed=3.5,
                        hazards=[Hazard(4.0, 12.0, "swerve", {"amp": 0.5, "period": 2.5})])]


def _convoy(ego_y: float) -> list[TrafficSpec]:
    # A 3-vehicle convoy ahead; the lead brakes, rippling back.
    return [
        TrafficSpec(init_x=16.0, init_y=ego_y, speed=4.0,
                    hazards=[Hazard(8.0, 4.0, "brake")]),
        TrafficSpec(init_x=26.0, init_y=ego_y, speed=4.0,
                    hazards=[Hazard(9.0, 4.0, "brake")]),
        TrafficSpec(init_x=12.0, init_y=ego_y + 3.5, speed=4.5,
                    hazards=[Hazard(6.0, 3.0, "cut_in", {"toward_y": ego_y})]),
    ]


def _platoon(ego_y: float, n: int = 5) -> list[TrafficSpec]:
    # An n-vehicle platoon in the ego's lane; the front brakes and it ripples
    # back (stop-and-go shockwave the ego must not rear-end).
    return [TrafficSpec(init_x=16.0 + 10.0 * i, init_y=ego_y, speed=4.0,
                        hazards=[Hazard(7.0 + 0.7 * i, 5.0, "brake")])
            for i in range(n)]


def _oncoming(ego_y: float) -> list[TrafficSpec]:
    # Two vehicles approaching in the adjacent (left) lane (head-on pass).
    return [TrafficSpec(init_x=50.0, init_y=ego_y + 3.5, speed=4.0, heading_deg=180.0),
            TrafficSpec(init_x=72.0, init_y=ego_y + 3.5, speed=4.5, heading_deg=180.0)]


def _double_cut(ego_y: float) -> list[TrafficSpec]:
    # Cut-ins from both sides into the ego's lane.
    return [TrafficSpec(init_x=14.0, init_y=ego_y + 3.5, speed=4.5,
                        hazards=[Hazard(5.0, 3.0, "cut_in", {"toward_y": ego_y})]),
            TrafficSpec(init_x=20.0, init_y=ego_y - 3.5, speed=4.5,
                        hazards=[Hazard(7.0, 3.0, "cut_in", {"toward_y": ego_y})])]


def _stop_and_go(ego_y: float) -> list[TrafficSpec]:
    # A 3-vehicle convoy doing repeated stop-and-go.
    pulses = [Hazard(5.0, 2.0, "brake"), Hazard(9.0, 2.0, "brake"),
              Hazard(13.0, 2.0, "brake")]
    return [TrafficSpec(init_x=16.0 + 10.0 * i, init_y=ego_y, speed=4.0, hazards=list(pulses))
            for i in range(3)]


def _jam(ego_y: float) -> list[TrafficSpec]:
    # Dense, mostly-stopped traffic across three lanes the ego must thread.
    return [
        TrafficSpec(init_x=22.0, init_y=ego_y, speed=0.0, hazards=[Hazard(0.0, 60.0, "stop")]),
        TrafficSpec(init_x=30.0, init_y=ego_y + 3.5, speed=1.0, hazards=[Hazard(0.0, 60.0, "slow", {"factor": 0.2})]),
        TrafficSpec(init_x=34.0, init_y=ego_y - 3.5, speed=1.0, hazards=[Hazard(0.0, 60.0, "slow", {"factor": 0.2})]),
        TrafficSpec(init_x=42.0, init_y=ego_y, speed=0.0, hazards=[Hazard(0.0, 60.0, "stop")]),
        TrafficSpec(init_x=48.0, init_y=ego_y + 3.5, speed=0.0, hazards=[Hazard(0.0, 60.0, "stop")]),
    ]


def _overtake(ego_y: float) -> list[TrafficSpec]:
    # A faster vehicle overtakes in the left lane, cuts in, then brakes.
    return [TrafficSpec(init_x=6.0, init_y=ego_y + 3.5, speed=6.0,
                        hazards=[Hazard(4.0, 2.5, "cut_in", {"toward_y": ego_y}),
                                 Hazard(6.5, 3.0, "brake")])]


def _gauntlet(ego_y: float) -> list[TrafficSpec]:
    # Mixed 5-vehicle stress scene: lead brake + cut-in + stalled + swerver +
    # oncoming. The hardest combined avoidance test.
    return [
        TrafficSpec(init_x=16.0, init_y=ego_y, speed=4.0,
                    hazards=[Hazard(6.0, 4.0, "brake")]),
        TrafficSpec(init_x=13.0, init_y=ego_y + 3.5, speed=4.5,
                    hazards=[Hazard(4.5, 3.0, "cut_in", {"toward_y": ego_y})]),
        TrafficSpec(init_x=34.0, init_y=ego_y, speed=0.0,
                    hazards=[Hazard(0.0, 60.0, "stop")]),
        TrafficSpec(init_x=24.0, init_y=ego_y - 3.5, speed=3.5,
                    hazards=[Hazard(8.0, 8.0, "swerve", {"amp": 0.4, "period": 2.0})]),
        TrafficSpec(init_x=58.0, init_y=ego_y + 3.5, speed=4.0, heading_deg=180.0),
    ]


# name -> builder(ego_lane_y). The bigger scenes (platoon/jam/gauntlet, ~5
# vehicles) are real-time with the camera at mesh 0.12 but near the budget;
# the autonomous eval runs them headless where vehicle count is cheap.
CONVOY_PRESETS = {
    "lead_brake": _lead_brake,
    "cut_in": _cut_in,
    "stalled": _stalled,
    "swerver": _swerver,
    "convoy": _convoy,
    "platoon": _platoon,
    "oncoming": _oncoming,
    "double_cut": _double_cut,
    "stop_and_go": _stop_and_go,
    "jam": _jam,
    "overtake": _overtake,
    "gauntlet": _gauntlet,
}
