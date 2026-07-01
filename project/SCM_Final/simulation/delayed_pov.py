#!/usr/bin/env python3
"""Software frame-delay buffer for the driver POV, so the operator actually
SEES the camera/downlink latency.

Why this exists: Chrono's ``ChCameraSensor.SetLag`` only gates when the sensor
buffer becomes available to *data consumers*; ``ChFilterVisualize`` draws each
frame at render time, so the displayed POV is NOT delayed by SetLag (verified in
the fork: ``ChSensor.h`` documents lag as "time ... until data becomes available
to the user", and ``ChOptixEngine`` uses it only for render scheduling). For a
human teleoperator that means the dominant difficulty factor -- delayed video --
never reached the screen.

This module instead pulls the rendered RGBA8 frame (via ``ChFilterRGBA8Access``),
timestamps it into a ring buffer, and displays the frame from ``delay_s`` ago in
its own pygame window. The delay is applied EXPLICITLY here (not via SetLag), so
it honours the time-varying per-frame ``camera`` latency exactly.

Fail-safe: if the pygame display cannot be created (e.g. headless), ``ok`` is
False and the caller keeps the live ``ChFilterVisualize`` path -- never worse
than before.
"""
from __future__ import annotations

from collections import deque

import numpy as np


class DelayedPOV:
    def __init__(self, width: int, height: int, *, fullscreen: bool = False,
                 flip_vertical: bool = False, max_delay_s: float = 2.5,
                 frame_period_s: float = 1.0 / 30.0):
        self.width = int(width)
        self.height = int(height)
        self.flip_vertical = bool(flip_vertical)
        self.ok = False
        self._pg = None
        self._screen = None
        # Ring buffer sized to hold slightly more than the worst-case delay.
        depth = max(8, int(max_delay_s / max(frame_period_s, 1e-3)) + 8)
        self._buf: deque[tuple[float, np.ndarray]] = deque(maxlen=depth)
        try:
            import pygame  # already a project dependency (G29 input)
            self._pg = pygame
            if not pygame.get_init():
                pygame.init()
            pygame.display.init()
            flags = pygame.FULLSCREEN if fullscreen else 0
            self._screen = pygame.display.set_mode((self.width, self.height), flags)
            pygame.display.set_caption("Driver POV (delayed downlink)")
            self.ok = True
        except Exception as e:  # headless / no SDL video / no pygame
            print(f"  [delayed-POV] display unavailable ({e}); "
                  f"keeping live ChFilterVisualize (camera delay will NOT be shown)")
            self.ok = False

    def capture(self, sim_time_s: float, driver_cam) -> None:
        """Grab the freshly rendered frame and timestamp it into the ring."""
        if not self.ok:
            return
        try:
            b = driver_cam.GetMostRecentRGBA8Buffer()
            if b.HasData():
                d = b.GetRGBA8Data()  # (H, W, 4) uint8
                self._buf.append((float(sim_time_s),
                                  np.ascontiguousarray(d[..., :3])))
        except Exception:
            pass

    def show(self, sim_time_s: float, delay_s: float) -> None:
        """Display the newest buffered frame at least ``delay_s`` old."""
        if not self.ok or not self._buf:
            return
        target = sim_time_s - max(float(delay_s), 0.0)
        frame = self._buf[0][1]
        for ts, fr in self._buf:
            if ts <= target:
                frame = fr
            else:
                break
        img = frame[::-1] if self.flip_vertical else frame
        pg = self._pg
        # (H, W, 3) row-major -> surface of size (W, H).
        surf = pg.image.frombuffer(np.ascontiguousarray(img).tobytes(),
                                   (self.width, self.height), "RGB")
        self._screen.blit(surf, (0, 0))
        pg.display.flip()
        pg.event.pump()

    def close(self) -> None:
        if self.ok and self._pg is not None:
            try:
                self._pg.display.quit()
            except Exception:
                pass
        self.ok = False
