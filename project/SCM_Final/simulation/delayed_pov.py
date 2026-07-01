#!/usr/bin/env python3
"""Software frame-delay buffer for the driver POV, so the operator actually
SEES the camera/downlink latency -- smoothly.

Why this exists: Chrono's ``ChCameraSensor.SetLag`` only gates when the sensor
buffer becomes available to *data consumers*; ``ChFilterVisualize`` draws each
frame at render time, so the displayed POV is NOT delayed by SetLag. For a human
teleoperator that removes the dominant difficulty factor -- delayed video.

Design (mirrors the C++ ``ChCameraDelaySim`` reference in
``chrono_hil/network/sim/``, which is driveable):

  * **Wall-clock timing.** Each captured frame is stamped with a wall-clock
    ``apply_time = now + delay``. It is displayed once ``apply_time`` has passed
    in wall-clock time. Timing the delay in *sim* time (an earlier bug) coupled
    the display to sim-step jitter and made it undriveable.
  * **Release every loop iteration**, not just at sim/sensor ticks, so frames
    appear on screen at their correct real-world moment (smooth cadence)
    regardless of how the sim-step wall time fluctuates.
  * **Anti-rewind + monotonic apply times** so a growing delay never shows an
    older frame after a newer one (that reads as violent jitter).

Fail-safe: if the pygame display cannot be created (headless), ``ok`` is False
and the caller keeps the live ``ChFilterVisualize`` path.
"""
from __future__ import annotations

import time as _time
from collections import deque

import numpy as np


class DelayedPOV:
    def __init__(self, width: int, height: int, *, fullscreen: bool = False,
                 flip_vertical: bool = True, max_delay_s: float = 2.0,
                 frame_period_s: float = 1.0 / 30.0, debug: bool = False):
        self.width = int(width)
        self.height = int(height)
        self.flip_vertical = bool(flip_vertical)
        self.debug = bool(debug)
        self.ok = False
        self._pg = None
        self._screen = None
        # buffer holds (apply_wall, source_wall, rgb_frame); FIFO in capture order
        self._buf: deque[tuple[float, float, np.ndarray]] = deque()
        self._max_frames = max(8, int(max_delay_s / max(frame_period_s, 1e-3)) + 8)
        self._last_apply = -1.0        # last apply_time pushed (keep monotonic)
        self._last_shown_apply = -1.0  # last apply_time displayed (anti-rewind)
        self._n_cap = 0
        self._n_show = 0
        try:
            import pygame
            self._pg = pygame
            if not pygame.get_init():
                pygame.init()
            pygame.display.init()
            fs = pygame.FULLSCREEN if fullscreen else 0
            for flags in (pygame.SCALED | fs, fs):
                try:
                    self._screen = pygame.display.set_mode((self.width, self.height), flags)
                    break
                except Exception:
                    self._screen = None
            if self._screen is None:
                raise RuntimeError("set_mode failed")
            pygame.display.set_caption("Driver POV (delayed downlink)")
            self.ok = True
        except Exception as e:
            print(f"  [delayed-POV] display unavailable ({e}); "
                  f"keeping live ChFilterVisualize (camera delay will NOT be shown)")
            self.ok = False

    def capture(self, driver_cam, delay_s: float) -> None:
        """Grab the freshly rendered frame; schedule it to appear ``delay_s`` from now."""
        if not self.ok:
            return
        try:
            b = driver_cam.GetMostRecentRGBA8Buffer()
            if not b.HasData():
                return
            d = b.GetRGBA8Data()  # (H, W, 4) uint8, bottom-up
            now = _time.monotonic()
            apply_t = now + max(float(delay_s), 0.0)
            # keep apply times monotonic so frames never reorder in the buffer
            if apply_t <= self._last_apply:
                apply_t = self._last_apply + 1e-4
            self._last_apply = apply_t
            self._buf.append((apply_t, now, np.ascontiguousarray(d[..., :3])))
            while len(self._buf) > self._max_frames:
                self._buf.popleft()
            self._n_cap += 1
        except Exception:
            pass

    def show(self) -> None:
        """Display the newest frame whose wall-clock apply time has passed."""
        if not self.ok or not self._buf:
            return
        now = _time.monotonic()
        chosen = None
        # release every frame now due, keeping the newest (drop skipped ones so
        # the view never lags behind wall-clock)
        while self._buf and self._buf[0][0] <= now:
            chosen = self._buf.popleft()
        if chosen is None:
            return
        apply_t, source_t, frame = chosen
        if apply_t <= self._last_shown_apply:
            return
        self._last_shown_apply = apply_t
        pg = self._pg
        surf = pg.image.frombuffer(frame.tobytes(), (self.width, self.height), "RGB")
        if self.flip_vertical:
            surf = pg.transform.flip(surf, False, True)
        self._screen.blit(surf, (0, 0))
        pg.display.flip()
        pg.event.pump()
        self._n_show += 1
        if self.debug and (self._n_show <= 5 or self._n_show % 30 == 0):
            print(f"  [POV-dbg] shown#{self._n_show} realized_delay="
                  f"{(now - source_t) * 1000:5.0f}ms  buffered={len(self._buf)}  "
                  f"captured={self._n_cap}", flush=True)

    def close(self) -> None:
        if self.ok and self._pg is not None:
            try:
                self._pg.display.quit()
            except Exception:
                pass
        self.ok = False
