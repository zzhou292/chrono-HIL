#!/usr/bin/env python3
"""
Terrain Classifier Message Definitions
========================================

Extends the HIL message system with a TerrainEstimate message published
by the terrain classifier node.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, asdict
from typing import Dict, Optional

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from hil_messages import (
    _topic_frame, _serialize, _deserialize,
    parse_message as _base_parse, _EXTENDED_REGISTRY,
)


# Default port for terrain classifier
TERRAIN_PUB_PORT = 5557


def terrain_pub_endpoint(port: int = TERRAIN_PUB_PORT) -> str:
    return f"tcp://*:{port}"


def terrain_sub_endpoint(host: str = "localhost", port: int = TERRAIN_PUB_PORT) -> str:
    return f"tcp://{host}:{port}"


@dataclass
class TerrainEstimate:
    """Published by the terrain classifier node."""
    time: float              # Simulation time of the latest state used
    wall_time: float         # Wall-clock publish timestamp
    terrain_class: str       # Predicted class: "clay", "sand", "dirt"
    confidence: float        # Confidence of predicted class [0, 1]
    probabilities: Dict[str, float]  # Per-class probabilities

    def to_bytes(self) -> bytes:
        d = asdict(self)
        return _topic_frame(b"terrain_estimate", _serialize(d))

    @classmethod
    def from_dict(cls, d: dict) -> "TerrainEstimate":
        return cls(**{k: d[k] for k in cls.__dataclass_fields__ if k in d})


def parse_terrain_message(raw: bytes):
    """Parse a terrain_estimate message, or fall through to base parser."""
    topic_len = struct.unpack("!H", raw[:2])[0]
    topic = raw[2:2 + topic_len].decode()
    if topic == "terrain_estimate":
        payload = _deserialize(raw[2 + topic_len:])
        return topic, TerrainEstimate.from_dict(payload)
    # Fall through to base HIL parser
    return _base_parse(raw)


# Auto-register so any ZMQSubscriber can deserialize TerrainEstimate
_EXTENDED_REGISTRY["terrain_estimate"] = TerrainEstimate
