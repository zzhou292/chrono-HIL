#!/usr/bin/env python3
"""
HIL Message Definitions & ZMQ Transport
========================================

Shared message types and serialization for decoupled Chrono simulation ↔ MPC controller
communication over ZMQ (pub/sub pattern).

Topics:
  - vehicle_state  : Sim → Controller  (PUB/SUB)
  - control_cmd    : Controller → Sim   (PUB/SUB)
  - sim_status     : Sim → Controller   (PUB/SUB, lifecycle events)

All messages are serialized as msgpack for minimal overhead (~2-5μs round-trip
on localhost, vs ~50μs for JSON).
"""

import struct
import time as _time
from dataclasses import dataclass, field, asdict
from typing import Optional, Dict, Any, List

import numpy as np

try:
    import msgpack

    def _serialize(obj: dict) -> bytes:
        return msgpack.packb(obj, use_bin_type=True)

    def _deserialize(data: bytes) -> dict:
        return msgpack.unpackb(data, raw=False)

except ImportError:
    # Fallback to JSON if msgpack not available
    import json

    def _serialize(obj: dict) -> bytes:
        return json.dumps(obj).encode()

    def _deserialize(data: bytes) -> dict:
        return json.loads(data.decode())


# =============================================================================
# Message types
# =============================================================================

@dataclass
class VehicleState:
    """Published by sim node at physics rate (or decimated)."""
    time: float           # Chrono simulation time (s)
    wall_time: float      # Wall-clock timestamp for latency measurement (s)
    # CG position in global frame
    x_cg: float
    y_cg: float
    z_cg: float
    # Orientation (quaternion e0, e1, e2, e3)
    quat_e0: float
    quat_e1: float
    quat_e2: float
    quat_e3: float
    # Velocity in body (local) frame
    u: float              # Longitudinal speed (m/s)
    v: float              # Lateral speed (m/s)
    # Angular velocity
    omega: float          # Yaw rate (rad/s)
    # Body-frame acceleration (IMU)
    ax: float = 0.0       # Longitudinal acceleration (m/s²)
    ay: float = 0.0       # Lateral acceleration (m/s²)
    # Wheel angular velocities (wheel encoders, rad/s)
    wheel_omega_fl: float = 0.0
    wheel_omega_fr: float = 0.0
    wheel_omega_rl: float = 0.0
    wheel_omega_rr: float = 0.0
    # Road-wheel steering angle (steering sensor, rad)
    steering_angle: float = 0.0
    # Tire info (optional, for diagnostics)
    tire_forces: Optional[Dict[str, float]] = None
    # Nearby obstacles for MPC planning: flat list [x0,y0,r0, x1,y1,r1, ...]
    # Sorted by distance; at most N_OBS entries. Empty when no rocks present.
    obstacles: Optional[List[float]] = None

    def to_bytes(self) -> bytes:
        d = asdict(self)
        return _topic_frame(b"vehicle_state", _serialize(d))

    @classmethod
    def from_dict(cls, d: dict) -> "VehicleState":
        return cls(**{k: d[k] for k in cls.__dataclass_fields__ if k in d})


@dataclass
class ControlCommand:
    """Published by controller node at MPC rate."""
    time: float           # Simulation time this command targets
    wall_time: float      # Wall-clock timestamp
    seq: int              # Sequence number
    steering: float       # Normalized steering [-1, 1]
    throttle: float       # Throttle [0, 1]
    braking: float        # Braking [0, 1]
    # MPC internal state (for warm-start continuity)
    delta: float          # Steering angle state (rad)
    acceleration: float   # Longitudinal acceleration state (m/s²)
    delta_dot: float      # Steering rate command (rad/s)
    jerk: float           # Longitudinal jerk command (m/s³)
    # Controller diagnostics
    solve_time_ms: float = 0.0
    mpc_cost: float = 0.0
    # Optional live terrain estimate forwarded to the sim-side safety
    # shield. Piggybacked on ControlCommand because the ctrl ZMQ socket is
    # conflated (latest-only); a separate TerrainUpdate channel would be
    # dropped under CONFLATE whenever ControlCommands arrived between
    # estimator ticks. ``terrain_n=None`` means "no live estimate yet".
    terrain_n: Optional[float] = None
    terrain_phi_deg: Optional[float] = None
    terrain_phi_sigma_deg: Optional[float] = None
    terrain_Kphi: Optional[float] = None
    terrain_Kc: Optional[float] = None
    terrain_c: Optional[float] = None
    terrain_k: Optional[float] = None
    terrain_class: Optional[str] = None
    terrain_confidence: Optional[float] = None
    terrain_update_seq: int = 0  # monotone counter; shield acts on change

    def to_bytes(self) -> bytes:
        d = asdict(self)
        return _topic_frame(b"control_cmd", _serialize(d))

    @classmethod
    def from_dict(cls, d: dict) -> "ControlCommand":
        return cls(**{k: d[k] for k in cls.__dataclass_fields__ if k in d})


@dataclass
class SimStatus:
    """Lifecycle / configuration messages from sim node."""
    event: str            # "start", "stop", "config"
    time: float
    wall_time: float
    config: Optional[Dict[str, Any]] = None  # terrain params, vehicle params, etc.

    def to_bytes(self) -> bytes:
        d = asdict(self)
        return _topic_frame(b"sim_status", _serialize(d))

    @classmethod
    def from_dict(cls, d: dict) -> "SimStatus":
        return cls(**{k: d[k] for k in cls.__dataclass_fields__ if k in d})


@dataclass
class TerrainUpdate:
    """Live terrain-estimator output, controller -> sim-side safety shield.

    The shield reads this each tick to (a) re-condition its NN surrogate's
    terrain context and (b) tighten the friction-cone gate by phi_sigma_deg.
    Sent on the existing controller-publishing ZMQ socket, multiplexed on
    the ``terrain_update`` topic.
    """
    time: float
    wall_time: float
    n: float
    phi_deg: float
    phi_sigma_deg: float      # ensemble/EMA-residual uncertainty
    terrain_class: str = "estimated"
    confidence: float = 0.0
    Kphi: float = 0.0
    Kc: float = 0.0
    c: float = 0.0
    k: float = 0.0

    def to_bytes(self) -> bytes:
        d = asdict(self)
        return _topic_frame(b"terrain_update", _serialize(d))

    @classmethod
    def from_dict(cls, d: dict) -> "TerrainUpdate":
        return cls(**{k: d[k] for k in cls.__dataclass_fields__ if k in d})


# =============================================================================
# ZMQ topic framing helpers
# =============================================================================

def _topic_frame(topic: bytes, payload: bytes) -> bytes:
    """Prefix payload with a 2-byte topic length + topic (ZMQ multipart alternative)."""
    return struct.pack("!H", len(topic)) + topic + payload


def parse_message(raw: bytes):
    """Parse a topic-framed message. Returns (topic_str, dataclass_instance)."""
    topic_len = struct.unpack("!H", raw[:2])[0]
    topic = raw[2:2 + topic_len].decode()
    payload = _deserialize(raw[2 + topic_len:])

    _registry = {
        "vehicle_state": VehicleState,
        "control_cmd": ControlCommand,
        "sim_status": SimStatus,
        "terrain_update": TerrainUpdate,
    }

    # Check extended registry (terrain classifier, etc.)
    cls = _registry.get(topic)
    if cls is None:
        cls = _EXTENDED_REGISTRY.get(topic)
    if cls is None:
        return topic, payload
    return topic, cls.from_dict(payload)


# Extended message registry — populated by downstream modules
_EXTENDED_REGISTRY: Dict[str, type] = {}


# =============================================================================
# ZMQ Transport wrapper (thin layer over raw ZMQ)
# =============================================================================

class ZMQPublisher:
    """Publish messages on a ZMQ PUB socket."""

    def __init__(self, endpoint: str = "tcp://*:5555"):
        import time as _time
        import zmq as _zmq
        self._zmq = _zmq
        self._ctx = _zmq.Context.instance()
        self._sock = self._ctx.socket(_zmq.PUB)
        self._sock.setsockopt(_zmq.SNDHWM, 2)  # Drop old messages if consumer is slow
        self._sock.setsockopt(_zmq.LINGER, 0)   # Release port immediately on close
        # Bind retries with linear backoff. The orchestrated sweeps reuse port
        # blocks rapidly; a freshly-closed socket can leave the port in
        # TIME_WAIT for a few seconds, so retry up to ~8s before giving up.
        # Without this, every long sweep loses ~1% of runs to flaky bind
        # failures even though the port is logically free.
        last_err = None
        for attempt in range(16):
            try:
                self._sock.bind(endpoint)
                last_err = None
                break
            except _zmq.error.ZMQError as e:
                last_err = e
                _time.sleep(0.5)
        if last_err is not None:
            raise last_err
        self.endpoint = endpoint

    def send(self, msg) -> None:
        """Send a dataclass message (must have .to_bytes())."""
        self._sock.send(msg.to_bytes(), self._zmq.NOBLOCK)

    def close(self):
        self._sock.close()


class ZMQSubscriber:
    """Subscribe to messages on a ZMQ SUB socket."""

    def __init__(self, endpoint: str = "tcp://localhost:5555",
                 topics: Optional[List[str]] = None):
        import zmq as _zmq
        self._zmq = _zmq
        self._ctx = _zmq.Context.instance()
        self._sock = self._ctx.socket(_zmq.SUB)
        self._sock.setsockopt(_zmq.RCVHWM, 2)
        self._sock.setsockopt(_zmq.CONFLATE, 1)  # Keep only latest message
        self._sock.connect(endpoint)
        # Subscribe to everything (topic filtering is done at application level
        # since we use a custom framing, not ZMQ's native topic prefix)
        self._sock.setsockopt(_zmq.SUBSCRIBE, b"")
        self.endpoint = endpoint

    def recv(self, timeout_ms: int = 0):
        """Receive a message. Returns (topic, msg) or None if no message available.

        Args:
            timeout_ms: 0 = non-blocking, -1 = block forever, >0 = wait up to N ms.
        """
        if timeout_ms == 0:
            flags = self._zmq.NOBLOCK
        else:
            self._sock.setsockopt(self._zmq.RCVTIMEO, timeout_ms)
            flags = 0
        try:
            raw = self._sock.recv(flags)
            return parse_message(raw)
        except Exception:
            return None

    def close(self):
        self._sock.close()


# =============================================================================
# Default port assignments
# =============================================================================

# Sim publishes vehicle state on this port
SIM_PUB_PORT = 5555
# Controller publishes control commands on this port
CTRL_PUB_PORT = 5556

def sim_pub_endpoint(port: int = SIM_PUB_PORT) -> str:
    return f"tcp://*:{port}"

def sim_sub_endpoint(host: str = "localhost", port: int = SIM_PUB_PORT) -> str:
    return f"tcp://{host}:{port}"

def ctrl_pub_endpoint(port: int = CTRL_PUB_PORT) -> str:
    return f"tcp://*:{port}"

def ctrl_sub_endpoint(host: str = "localhost", port: int = CTRL_PUB_PORT) -> str:
    return f"tcp://{host}:{port}"
