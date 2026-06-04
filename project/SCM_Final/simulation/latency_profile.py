#!/usr/bin/env python3
"""JSON-configurable 5G-like latency profiles for teleoperation experiments.

The external 5G-Traffic-Generator repository provides ML traffic generation
code and open bitrate traces, but it does not ship trained checkpoints.  This
module keeps the sim self-contained by accepting those generated/open traces
when available and mapping bitrate load to one-way latency with configurable
good/poor/outage windows.
"""

from __future__ import annotations

import csv
import json
import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


REGIMES: dict[str, dict[str, float]] = {
    "good": {"delay_scale": 0.70, "capacity_scale": 1.60, "jitter_scale": 0.80},
    "nominal": {"delay_scale": 1.00, "capacity_scale": 1.00, "jitter_scale": 1.00},
    "poor": {"delay_scale": 1.80, "capacity_scale": 0.60, "jitter_scale": 1.70},
    "outage": {"delay_scale": 3.00, "capacity_scale": 0.25, "jitter_scale": 2.50},
}


@dataclass
class LatencyChannel:
    name: str
    trace_s: np.ndarray
    sample_period_s: float
    loop: bool

    def delay(self, sim_time_s: float) -> float:
        if self.trace_s.size == 0:
            return 0.0
        idx = int(max(sim_time_s, 0.0) / self.sample_period_s)
        if self.loop:
            idx %= self.trace_s.size
        else:
            idx = min(idx, self.trace_s.size - 1)
        return float(self.trace_s[idx])


class LatencyProfile:
    """Time-varying one-way latency profile loaded from JSON."""

    def __init__(
        self,
        *,
        path: Path,
        description: str,
        channels: dict[str, LatencyChannel],
        sample_period_s: float,
        loop: bool,
    ):
        self.path = path
        self.description = description
        self.channels = channels
        self.sample_period_s = sample_period_s
        self.loop = loop

    @classmethod
    def from_json(cls, path: str | Path) -> "LatencyProfile":
        cfg_path = Path(path).expanduser().resolve()
        with cfg_path.open("r", encoding="utf-8") as f:
            cfg = json.load(f)

        sample_period_s = float(cfg.get("sample_period_s", 0.05))
        if sample_period_s <= 0:
            raise ValueError("sample_period_s must be positive")
        loop = bool(cfg.get("loop", False))
        duration_s = float(cfg.get("duration_s", _infer_duration(cfg)))
        if duration_s <= 0:
            raise ValueError("duration_s must be positive or inferable from segments")

        channel_cfgs = cfg.get("channels", {})
        if not channel_cfgs:
            raise ValueError("latency profile JSON must define at least one channel")

        built: dict[str, LatencyChannel] = {}
        pending = dict(channel_cfgs)
        while pending:
            progressed = False
            for name, channel_cfg in list(pending.items()):
                copy_from = channel_cfg.get("copy_from")
                if copy_from:
                    if copy_from not in built:
                        continue
                    scale = float(channel_cfg.get("scale", 1.0))
                    offset_ms = float(channel_cfg.get("offset_ms", 0.0))
                    trace_s = np.maximum(
                        0.0,
                        built[copy_from].trace_s * scale + offset_ms / 1000.0,
                    )
                    built[name] = LatencyChannel(name, trace_s, sample_period_s, loop)
                else:
                    trace_s = _build_channel_trace(
                        name=name,
                        cfg=channel_cfg,
                        root_cfg=cfg,
                        cfg_path=cfg_path,
                        sample_period_s=sample_period_s,
                        duration_s=duration_s,
                    )
                    built[name] = LatencyChannel(name, trace_s, sample_period_s, loop)
                pending.pop(name)
                progressed = True
            if not progressed:
                unresolved = ", ".join(sorted(pending))
                raise ValueError(f"unresolved latency channel copy_from dependency: {unresolved}")

        return cls(
            path=cfg_path,
            description=str(cfg.get("description", cfg_path.name)),
            channels=built,
            sample_period_s=sample_period_s,
            loop=loop,
        )

    def delay(self, sim_time_s: float, channel: str = "control") -> float:
        if channel in self.channels:
            return self.channels[channel].delay(sim_time_s)
        if "control" in self.channels:
            return self.channels["control"].delay(sim_time_s)
        first = next(iter(self.channels.values()))
        return first.delay(sim_time_s)

    def describe(self) -> str:
        names = ", ".join(sorted(self.channels))
        duration = max(ch.trace_s.size for ch in self.channels.values()) * self.sample_period_s
        return (
            f"{self.description} ({duration:.1f}s, dt={self.sample_period_s:.3f}s, "
            f"loop={self.loop}, channels=[{names}])"
        )


def _infer_duration(cfg: dict[str, Any]) -> float:
    duration = 0.0
    for channel_cfg in cfg.get("channels", {}).values():
        for seg in channel_cfg.get("segments", []):
            duration = max(duration, float(seg.get("end_s", 0.0)))
    return duration if duration > 0 else 60.0


def _build_channel_trace(
    *,
    name: str,
    cfg: dict[str, Any],
    root_cfg: dict[str, Any],
    cfg_path: Path,
    sample_period_s: float,
    duration_s: float,
) -> np.ndarray:
    n = max(1, int(math.ceil(duration_s / sample_period_s)))
    rng = random.Random(int(cfg.get("seed", root_cfg.get("seed", 1))) + _stable_name_offset(name))

    base_delay_ms = float(cfg.get("base_delay_ms", 20.0))
    jitter_ms = float(cfg.get("jitter_ms", 4.0))
    min_ms = float(cfg.get("min_delay_ms", 0.0))
    max_ms = float(cfg.get("max_delay_ms", 500.0))
    queue_gain_ms = float(cfg.get("queue_gain_ms", 18.0))
    capacity_bps = float(cfg.get("capacity_bps", 40e6))
    eps = 1e-3

    bitrate = _load_or_generate_bitrate(cfg, cfg_path, n, rng)
    bitrate = bitrate * float(cfg.get("traffic_scale", 1.0))
    if bitrate.size < n:
        bitrate = np.resize(bitrate, n)
    bitrate = bitrate[:n]

    trace_ms = np.zeros(n, dtype=float)
    segments = cfg.get("segments") or [{"start_s": 0.0, "end_s": duration_s, "regime": "nominal"}]

    for idx in range(n):
        t = idx * sample_period_s
        seg = _segment_for_time(segments, t, duration_s)
        regime = dict(REGIMES.get(str(seg.get("regime", "nominal")).lower(), REGIMES["nominal"]))
        regime.update({k: float(seg[k]) for k in ("delay_scale", "capacity_scale", "jitter_scale") if k in seg})

        if "fixed_delay_ms" in seg:
            mean_ms = float(seg["fixed_delay_ms"])
            load_queue_ms = 0.0
        else:
            cap = max(capacity_bps * regime["capacity_scale"], 1.0)
            load = min(max(float(bitrate[idx]) / cap, 0.0), 0.995)
            load_queue_ms = queue_gain_ms * (load * load) / max(1.0 - load, eps)
            mean_ms = (
                float(seg.get("base_delay_ms", base_delay_ms)) * regime["delay_scale"]
                + float(seg.get("extra_ms", 0.0))
                + load_queue_ms
            )

        sigma_ms = max(0.0, float(seg.get("jitter_ms", jitter_ms)) * regime["jitter_scale"])
        jitter = rng.gauss(0.0, sigma_ms)
        trace_ms[idx] = min(max(mean_ms + jitter, min_ms), max_ms)

    return trace_ms / 1000.0


def _load_or_generate_bitrate(
    cfg: dict[str, Any],
    cfg_path: Path,
    n: int,
    rng: random.Random,
) -> np.ndarray:
    source = cfg.get("source", {})
    if source.get("kind") == "5g_repo_csv":
        raw_path = Path(str(source.get("path", ""))).expanduser()
        if not raw_path.is_absolute():
            raw_path = (cfg_path.parent / raw_path).resolve()
        if raw_path.exists():
            direction = str(source.get("direction", "DL_bitrate"))
            start_index = int(source.get("start_index", 0))
            values = _read_bitrate_column(raw_path, direction)
            if values.size:
                values = values.astype(float)
                if start_index >= values.size:
                    start_index = 0
                return np.resize(values[start_index:], n)
        else:
            print(f"  [LATENCY] Source CSV not found, using synthetic load: {raw_path}")

    mean_bps = float(cfg.get("synthetic_mean_bps", 14e6))
    sigma = float(cfg.get("synthetic_log_sigma", 0.65))
    floor_bps = float(cfg.get("synthetic_floor_bps", 0.2e6))
    vals = [max(floor_bps, rng.lognormvariate(math.log(max(mean_bps, 1.0)), sigma)) for _ in range(n)]
    return np.asarray(vals, dtype=float)


def _read_bitrate_column(path: Path, direction: str) -> np.ndarray:
    with path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if direction not in (reader.fieldnames or []):
            raise ValueError(f"{path} has no column {direction!r}; columns={reader.fieldnames}")
        vals = []
        for row in reader:
            try:
                vals.append(float(row[direction]))
            except (TypeError, ValueError):
                continue
    return np.asarray(vals, dtype=float)


def _segment_for_time(segments: list[dict[str, Any]], t: float, duration_s: float) -> dict[str, Any]:
    for seg in segments:
        start = float(seg.get("start_s", 0.0))
        end = float(seg.get("end_s", duration_s))
        if start <= t < end:
            return seg
    return segments[-1]


def _stable_name_offset(name: str) -> int:
    return sum((i + 1) * ord(ch) for i, ch in enumerate(name))
