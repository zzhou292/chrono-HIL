#!/usr/bin/env python3
"""Live HMI overlay for HIL data collection: a virtual steering wheel + a
throttle/brake bar in a small always-on-top corner window, fed straight off
the ZMQ bus.

It subscribes only (one more SUB on the sim's ``vehicle_state`` PUB), so it
cannot perturb the sim/controller real-time loops. The sim publishes the
driver inputs on ``vehicle_state`` -- both the operator's raw command and the
applied (post-safety-filter) command -- so the HUD works in manual HIL mode
where there is *no* controller publishing ``control_cmd``:

  * ghost / dashed = the operator's raw command (steering_op / throttle_op);
  * solid          = what the vehicle applied (steering_app / throttle_app);
when they diverge, that is the safety-filter takeover, live.

The window is borderless, auto-positioned into a screen corner, and set
always-on-top (via wmctrl) so it sits over the sim window as a real overlay
instead of a separate window you have to raise by hand.

Usage (sim-port must match the launch_decoupled run):
  python hil_hud.py --sim-port 5965
"""
from __future__ import annotations
import argparse
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import zmq  # noqa: E402
from hil_messages import parse_message  # noqa: E402

WHEEL_LOCK_DEG = 450.0   # display rotation at |steer|=1; match the G29 range/2
APPLIED = (40, 199, 111)
GHOST = (154, 166, 178)
PANEL_BG = (17, 22, 28)
PANEL_EDGE = (42, 51, 64)
RED = (234, 84, 85)


def _sub(ctx, port):
    s = ctx.socket(zmq.SUB)
    s.setsockopt(zmq.CONFLATE, 1)      # keep only the latest (single-frame msgs)
    s.setsockopt_string(zmq.SUBSCRIBE, "")
    s.setsockopt(zmq.RCVHWM, 1)
    s.connect(f"tcp://127.0.0.1:{port}")
    return s


def _drain(sock):
    """Return the latest parsed message on a socket, or None."""
    last = None
    while True:
        try:
            last = sock.recv(zmq.NOBLOCK)
        except zmq.Again:
            break
    return parse_message(last) if last is not None else None


def _set_always_on_top(pg):
    """Best-effort: raise + pin the SDL window above others via wmctrl (X11)."""
    if not shutil.which("wmctrl"):
        return
    try:
        wid = pg.display.get_wm_info().get("window")
        if wid:
            subprocess.run(["wmctrl", "-i", "-r", hex(int(wid)), "-b", "add,above"],
                           check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass


def _wheel(surf, pg, cx, cy, r, steer, color, lw, ghost=False):
    rot = math.radians(steer * WHEEL_LOCK_DEG)
    pg.draw.circle(surf, color, (cx, cy), r, max(1, lw if not ghost else lw - 1))
    pg.draw.circle(surf, color, (cx, cy), max(3, r // 6))
    for a0 in (90, 210, 330):
        a = math.radians(a0) + rot
        ex, ey = cx + r * math.cos(a), cy - r * math.sin(a)
        pg.draw.line(surf, color, (cx, cy), (ex, ey), max(1, lw - (1 if ghost else 0)))
    top = math.pi / 2 + rot
    pg.draw.circle(surf, color, (int(cx + (r + 6) * math.cos(top)), int(cy - (r + 6) * math.sin(top))), 4)


def main():
    global WHEEL_LOCK_DEG
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--sim-port", type=int, required=True, help="vehicle_state PUB port (sim)")
    p.add_argument("--ctrl-port", type=int, default=0,
                   help="(legacy/unused) control_cmd port; the HUD now reads the "
                        "driver inputs from vehicle_state so it works in manual mode.")
    p.add_argument("--w", type=int, default=380); p.add_argument("--h", type=int, default=240)
    p.add_argument("--corner", choices=["br", "bl", "tr", "tl"], default="br",
                   help="Screen corner to dock the overlay into.")
    p.add_argument("--margin", type=int, default=24, help="Gap from the screen edge (px).")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--wheel-lock-deg", type=float, default=WHEEL_LOCK_DEG,
                   help="HUD wheel rotation at full steer (|steer|=1). Set to half "
                        "your G29's configured lock-to-lock range so the on-screen "
                        "wheel matches the physical one (default 450 = 900deg G29).")
    p.add_argument("--smoke", action="store_true", help="headless self-test (SDL dummy), then exit")
    args = p.parse_args()
    WHEEL_LOCK_DEG = args.wheel_lock_deg

    if args.smoke:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    import pygame as pg
    pg.init()
    # Dock into a screen corner (must set the env var before set_mode).
    if not args.smoke:
        info = pg.display.Info()
        sw, sh = info.current_w, info.current_h
        x = args.margin if args.corner in ("bl", "tl") else max(0, sw - args.w - args.margin)
        y = args.margin if args.corner in ("tl", "tr") else max(0, sh - args.h - args.margin)
        os.environ["SDL_VIDEO_WINDOW_POS"] = f"{x},{y}"
    screen = pg.display.set_mode((args.w, args.h), pg.NOFRAME)
    pg.display.set_caption("HIL HMI")
    font = pg.font.SysFont("DejaVu Sans", 16)
    clock = pg.time.Clock()

    ctx = zmq.Context.instance()
    sub_state = _sub(ctx, args.sim_port)

    steer_op = thr_op = brk_op = 0.0
    steer_app = thr_app = brk_app = 0.0
    running = True
    frames = 0
    rx = 0
    last_hb = 0.0
    while running:
        for e in pg.event.get():
            if e.type == pg.QUIT or (e.type == pg.KEYDOWN and e.key in (pg.K_ESCAPE, pg.K_q)):
                running = False
        msg = _drain(sub_state)
        if msg is not None and isinstance(msg, tuple):
            _, m = msg
            rx += 1
            steer_op = float(getattr(m, "steering_op", steer_op))
            thr_op = float(getattr(m, "throttle_op", thr_op))
            brk_op = float(getattr(m, "braking_op", brk_op))
            steer_app = float(getattr(m, "steering_app", steer_app))
            thr_app = float(getattr(m, "throttle_app", thr_app))
            brk_app = float(getattr(m, "braking_app", brk_app))

        screen.fill((0, 0, 0))
        pg.draw.rect(screen, PANEL_BG, (6, 6, args.w - 12, args.h - 12), border_radius=14)
        pg.draw.rect(screen, PANEL_EDGE, (6, 6, args.w - 12, args.h - 12), 2, border_radius=14)
        # wheel: ghost = operator command, solid = applied
        cx, cy, r = int(args.w * 0.34), int(args.h * 0.46), int(args.h * 0.30)
        _wheel(screen, pg, cx, cy, r, max(-1.5, min(1.5, steer_op)), GHOST, 3, ghost=True)
        _wheel(screen, pg, cx, cy, r, max(-1.5, min(1.5, steer_app)), APPLIED, 5)
        screen.blit(font.render("steer", True, GHOST), (cx - 22, args.h - 34))
        # accel/brake bar: center = 0, throttle up (green), brake down (red)
        bx, by, bh, bw = int(args.w * 0.74), int(args.h * 0.46), int(args.h * 0.30), 26
        pg.draw.rect(screen, (27, 37, 48), (bx - bw // 2, by - bh, bw, 2 * bh), border_radius=5)
        a_app = max(-1.0, min(1.0, thr_app - brk_app))
        col = APPLIED if a_app >= 0 else RED
        pg.draw.rect(screen, col, (bx - bw // 2, by - int(a_app * bh) if a_app >= 0 else by,
                                   bw, int(abs(a_app) * bh)))
        a_op = max(-1.0, min(1.0, thr_op - brk_op))      # operator ghost tick
        gy = by - int(a_op * bh)
        pg.draw.line(screen, GHOST, (bx - bw // 2 - 4, gy), (bx + bw // 2 + 4, gy), 2)
        pg.draw.line(screen, (58, 70, 84), (bx - bw // 2, by), (bx + bw // 2, by), 1)
        screen.blit(font.render("accel", True, GHOST), (bx - 26, args.h - 34))
        pg.display.flip()

        if frames == 1 and not args.smoke:
            _set_always_on_top(pg)   # window is mapped by now

        now = pg.time.get_ticks() / 1000.0
        if now - last_hb >= 1.0:      # heartbeat to the log: receive vs render
            print(f"hud: rx={rx} steer_app={steer_app:+.2f} thr_app={thr_app:.2f} "
                  f"steer_op={steer_op:+.2f}", flush=True)
            last_hb = now

        clock.tick(args.fps)
        frames += 1
        if args.smoke and frames >= 3:
            running = False

    pg.quit()
    if args.smoke:
        print("hil_hud smoke OK (3 frames rendered headless)")


if __name__ == "__main__":
    main()
