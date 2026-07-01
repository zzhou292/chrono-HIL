#!/usr/bin/env python3
"""Software frame-delay buffer for the driver POV, so the operator actually
SEES the camera/downlink latency -- smoothly and driveably.

Why this exists: Chrono's ``ChCameraSensor.SetLag`` only gates when the sensor
buffer becomes available to *data consumers*; ``ChFilterVisualize`` draws each
frame at render time, so the displayed POV is NOT delayed by SetLag. For a human
teleoperator that removes the dominant difficulty factor -- delayed video.
(NB: SetLag *does* delay ``GetMostRecentRGBA8Buffer``, so the caller must keep
SetLag at ~0 while this buffer owns the delay, or the two stack and double it.)

Design (mirrors the driveable C++ ``ChCameraDelaySim`` in ``chrono_hil``):
  * **Wall-clock timing.** Each captured frame gets ``apply_time = now + delay``
    and is shown once that wall-clock deadline passes. Timing in sim time (an
    earlier bug) coupled the display to sim-step jitter and made it undriveable.
  * **Release every loop iteration** so frames appear at their real-world moment
    (smooth cadence), not only at sim/sensor ticks.
  * **Anti-rewind + monotonic apply times** so a growing delay never shows an
    older frame after a newer one.
  * **Borderless desktop window** (not exclusive fullscreen) so the always-on-top
    HUD overlay still composits on top of the POV.
  * **Warm-up**: show the freshest frame while the delay buffer fills, so the
    screen isn't black for the first ``delay`` seconds.

Fail-safe: if the pygame display cannot be created (headless), ``ok`` is False
and the caller keeps the live ``ChFilterVisualize`` path.
"""
from __future__ import annotations

import os as _os
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
        self._win_w, self._win_h = self.width, self.height  # display window size
        self._buf: deque[tuple[float, float, np.ndarray]] = deque()
        self._max_frames = max(8, int(max_delay_s / max(frame_period_s, 1e-3)) + 8)
        self._last_apply = -1.0
        self._last_shown_apply = -1.0
        self._n_cap = 0
        self._n_show = 0
        self._n_miss = 0   # capture() calls where HasData() was False
        self._last_ts = None  # last captured buffer.TimeStamp (dedupe duplicates)
        try:
            import pygame
            self._pg = pygame
            if not pygame.get_init():
                pygame.init()
            pygame.display.init()
            # Fullscreen = a BORDERLESS window sized to the desktop (NOFRAME), NOT
            # exclusive FULLSCREEN. Exclusive fullscreen bypasses the window manager
            # and hides the always-on-top HUD overlay; a NOFRAME window stays
            # WM-composited so the HUD floats over it. The frame is scaled up to the
            # desktop in _blit (affordable now that dedupe caps the blit at ~30/s).
            if fullscreen:
                info = pygame.display.Info()
                if info.current_w > 0 and info.current_h > 0:
                    self._win_w, self._win_h = info.current_w, info.current_h
                _os.environ.setdefault("SDL_VIDEO_WINDOW_POS", "0,0")
                self._screen = pygame.display.set_mode((self._win_w, self._win_h),
                                                       pygame.NOFRAME)
            else:
                self._screen = pygame.display.set_mode((self._win_w, self._win_h))
            pygame.display.set_caption("Driver POV (delayed downlink)")
            self.ok = True
        except Exception as e:
            print(f"  [delayed-POV] display unavailable ({e}); "
                  f"keeping live ChFilterVisualize (camera delay will NOT be shown)")
            self.ok = False

    def _blit(self, frame: np.ndarray) -> None:
        # frame is stored already upright + contiguous (flip done at capture), so
        # this is just a buffer wrap + blit + flip -- no per-frame CPU scale/copy,
        # which is what keeps the sim at real-time. SCALED fits it to the display.
        pg = self._pg
        surf = pg.image.frombuffer(frame, (self.width, self.height), "RGB")
        if (self._win_w, self._win_h) != (self.width, self.height):
            surf = pg.transform.scale(surf, (self._win_w, self._win_h))
        self._screen.blit(surf, (0, 0))
        pg.display.flip()
        pg.event.pump()

    def capture(self, driver_cam, delay_s: float) -> None:
        """Grab the freshly rendered frame; schedule it ``delay_s`` from now."""
        if not self.ok:
            return
        try:
            b = driver_cam.GetMostRecentRGBA8Buffer()
            if not b.HasData():
                self._n_miss += 1
                return
            # The sensor block runs every physics step (sensor_interval=0 when the
            # IMU is active), but the camera only renders at cam_rate. Dedupe by the
            # buffer's render TimeStamp so we store one entry per real frame (~30/s),
            # not ~330/s duplicates -- otherwise the ring buffer holds < the delay
            # window and the delayed frame is evicted before it is due.
            ts = getattr(b, "TimeStamp", None)
            if ts is not None and ts == self._last_ts:
                return
            self._last_ts = ts
            d = b.GetRGBA8Data()  # (H, W, 4) uint8, bottom-up
            rgb = d[::-1, :, :3] if self.flip_vertical else d[..., :3]  # flip once, here
            now = _time.monotonic()
            apply_t = now + max(float(delay_s), 0.0)
            if apply_t <= self._last_apply:      # keep buffer ordered
                apply_t = self._last_apply + 1e-4
            self._last_apply = apply_t
            self._buf.append((apply_t, now, np.ascontiguousarray(rgb)))
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
        while self._buf and self._buf[0][0] <= now:
            chosen = self._buf.popleft()
        if chosen is None:
            # Warm-up: nothing is "due" yet (buffer still filling). Show the
            # freshest frame so the screen isn't black -- but don't advance the
            # anti-rewind clock, so normal delayed release takes over cleanly.
            if self._last_shown_apply < 0.0:
                self._blit(self._buf[-1][2])
            return
        apply_t, source_t, frame = chosen
        if apply_t <= self._last_shown_apply:
            return
        self._last_shown_apply = apply_t
        self._blit(frame)
        self._n_show += 1
        if self.debug and (self._n_show <= 5 or self._n_show % 30 == 0):
            print(f"  [POV-dbg] shown#{self._n_show} realized_delay="
                  f"{(now - source_t) * 1000:5.0f}ms  buffered={len(self._buf)}  "
                  f"captured={self._n_cap}", flush=True)

    def close(self) -> None:
        if self.debug:
            print(f"  [POV-dbg] final: captured={self._n_cap} shown={self._n_show} "
                  f"hasdata_misses={self._n_miss}", flush=True)
        if self.ok and self._pg is not None:
            try:
                self._pg.display.quit()
            except Exception:
                pass
        self.ok = False
