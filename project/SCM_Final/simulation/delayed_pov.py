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
its own pygame window.

Design notes learned on hardware:
  * The Chrono RGBA8 buffer is bottom-up, so we flip vertically by default.
  * The window uses ``SCALED | FULLSCREEN`` so the camera surface scales to fill
    the physical display (otherwise a cam-sized window sits in a screen corner).
  * Frame selection is MONOTONIC in display time: with a time-varying camera
    delay a naive "newest frame older than now-delay" rule lets the shown frame
    jump backwards when the delay grows, which looks like heavy jitter. We clamp
    the display clock so it never rewinds, and skip re-blitting an unchanged
    frame.

Fail-safe: if the pygame display cannot be created (e.g. headless), ``ok`` is
False and the caller keeps the live ``ChFilterVisualize`` path.
"""
from __future__ import annotations

from collections import deque

import numpy as np


class DelayedPOV:
    def __init__(self, width: int, height: int, *, fullscreen: bool = False,
                 flip_vertical: bool = True, max_delay_s: float = 1.5,
                 frame_period_s: float = 1.0 / 30.0):
        self.width = int(width)
        self.height = int(height)
        self.flip_vertical = bool(flip_vertical)
        self.ok = False
        self._pg = None
        self._screen = None
        self._last_shown_ts = -1.0   # monotonic display clock (sim seconds)
        depth = max(8, int(max_delay_s / max(frame_period_s, 1e-3)) + 8)
        self._buf: deque[tuple[float, np.ndarray]] = deque(maxlen=depth)
        try:
            import pygame  # already a project dependency (G29 input)
            self._pg = pygame
            if not pygame.get_init():
                pygame.init()
            pygame.display.init()
            fs = pygame.FULLSCREEN if fullscreen else 0
            # Prefer SCALED (GPU-scales the cam-sized surface to fill the display,
            # so it isn't a small window in a screen corner); fall back to plain
            # if the renderer can't do SCALED.
            for flags in (pygame.SCALED | fs, fs):
                try:
                    self._screen = pygame.display.set_mode((self.width, self.height), flags)
                    break
                except Exception:
                    self._screen = None
            if self._screen is None:
                raise RuntimeError("set_mode failed for all flag combinations")
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
                d = b.GetRGBA8Data()  # (H, W, 4) uint8, bottom-up
                self._buf.append((float(sim_time_s),
                                  np.ascontiguousarray(d[..., :3])))
        except Exception:
            pass

    def show(self, sim_time_s: float, delay_s: float) -> None:
        """Display the buffered frame ~delay_s old, monotonically (never rewind)."""
        if not self.ok or not self._buf:
            return
        # Monotonic display clock: never let the target time move backwards, so a
        # growing delay freezes the view briefly instead of jumping to an older
        # frame (which reads as violent jitter).
        target = sim_time_s - max(float(delay_s), 0.0)
        if target <= self._last_shown_ts:
            return  # nothing newer to show yet; leave the current frame up
        frame, chosen_ts = self._buf[0][1], self._buf[0][0]
        for ts, fr in self._buf:
            if ts <= target:
                frame, chosen_ts = fr, ts
            else:
                break
        if chosen_ts <= self._last_shown_ts:
            return  # same frame already displayed
        self._last_shown_ts = chosen_ts
        pg = self._pg
        # (H, W, 3) row-major RGB -> surface of size (W, H).
        surf = pg.image.frombuffer(frame.tobytes(), (self.width, self.height), "RGB")
        if self.flip_vertical:
            surf = pg.transform.flip(surf, False, True)
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
