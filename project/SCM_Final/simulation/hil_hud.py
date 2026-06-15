#!/usr/bin/env python3
"""Live HMI overlay for HIL data collection: a virtual steering wheel + a
throttle bar in a small always-on-top window, fed straight off the ZMQ bus.

Decoupled by construction: it only *subscribes* (one more SUB on the existing
``vehicle_state`` and ``control_cmd`` PUBs), so it cannot perturb the sim or
controller real-time loops. Run it alongside ``launch_decoupled.py`` during a
manned round and screen-record the sim window + this HUD together.

  * ghost / dashed wheel = the operator's *commanded* steer (control_cmd);
  * solid wheel          = the *applied* road-wheel angle (vehicle_state);
when they diverge, that is the DOB-CBF takeover, live.

Throttle is the commanded value (control_cmd); the applied throttle is not on
the bus, so the brake-takeover needs a small sim-side publish to show here --
the post-hoc compositor (make_hil_overlay.py) can show it from sim_diag once
the *_cmd columns are logged.

Usage (ports must match the launch_decoupled run):
  python hil_hud.py --sim-port 5965 --ctrl-port 5966
"""
from __future__ import annotations
import argparse
import math
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import zmq  # noqa: E402
from hil_messages import parse_message  # noqa: E402

WHEEL_LOCK_DEG = 120.0
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


def _wheel(surf, pg, cx, cy, r, steer, color, lw, ghost=False):
    rot = math.radians(-steer * WHEEL_LOCK_DEG)
    pg.draw.circle(surf, color, (cx, cy), r, max(1, lw if not ghost else lw - 1))
    pg.draw.circle(surf, color, (cx, cy), max(3, r // 6))
    for a0 in (90, 210, 330):
        a = math.radians(a0) + rot
        ex, ey = cx + r * math.cos(a), cy - r * math.sin(a)
        pg.draw.line(surf, color, (cx, cy), (ex, ey), max(1, lw - (1 if ghost else 0)))
    top = math.pi / 2 + rot
    pg.draw.circle(surf, color, (int(cx + (r + 6) * math.cos(top)), int(cy - (r + 6) * math.sin(top))), 4)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--sim-port", type=int, required=True, help="vehicle_state PUB port (sim)")
    p.add_argument("--ctrl-port", type=int, required=True, help="control_cmd PUB port (controller)")
    p.add_argument("--max-steer", type=float, default=0.60, help="road-wheel angle (rad) at |cmd|=1, for normalising the applied wheel")
    p.add_argument("--w", type=int, default=380); p.add_argument("--h", type=int, default=240)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--smoke", action="store_true", help="headless self-test (SDL dummy), then exit")
    args = p.parse_args()

    if args.smoke:
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    import pygame as pg
    pg.init()
    screen = pg.display.set_mode((args.w, args.h), pg.NOFRAME)
    pg.display.set_caption("HIL HMI")
    font = pg.font.SysFont("DejaVu Sans", 16)
    clock = pg.time.Clock()

    ctx = zmq.Context.instance()
    sub_state = _sub(ctx, args.sim_port)
    sub_cmd = _sub(ctx, args.ctrl_port)

    steer_cmd = thr_cmd = steer_app = 0.0
    running = True
    frames = 0
    while running:
        for e in pg.event.get():
            if e.type == pg.QUIT or (e.type == pg.KEYDOWN and e.key in (pg.K_ESCAPE, pg.K_q)):
                running = False
        for sock, kind in ((sub_cmd, "cmd"), (sub_state, "state")):
            msg = _drain(sock)
            if msg is None:
                continue
            _, m = msg
            if kind == "cmd":
                steer_cmd = float(getattr(m, "steering", steer_cmd))
                thr_cmd = float(getattr(m, "throttle", thr_cmd))
            else:
                steer_app = float(getattr(m, "steering_angle", 0.0)) / max(args.max_steer, 1e-3)

        screen.fill((0, 0, 0))
        pg.draw.rect(screen, PANEL_BG, (6, 6, args.w - 12, args.h - 12), border_radius=14)
        pg.draw.rect(screen, PANEL_EDGE, (6, 6, args.w - 12, args.h - 12), 2, border_radius=14)
        # wheel: ghost = commanded, solid = applied
        cx, cy, r = int(args.w * 0.34), int(args.h * 0.46), int(args.h * 0.30)
        _wheel(screen, pg, cx, cy, r, max(-1.5, min(1.5, steer_cmd)), GHOST, 3, ghost=True)
        _wheel(screen, pg, cx, cy, r, max(-1.5, min(1.5, steer_app)), APPLIED, 5)
        screen.blit(font.render("steer", True, GHOST), (cx - 22, args.h - 34))
        # throttle bar (commanded)
        bx, by, bw, bh = int(args.w * 0.74), int(args.h * 0.46), 26, int(args.h * 0.30)
        pg.draw.rect(screen, (27, 37, 48), (bx - bw // 2, by - bh, bw, 2 * bh), border_radius=5)
        pg_fill = max(0.0, min(1.0, thr_cmd))
        pg.draw.rect(screen, APPLIED, (bx - bw // 2, by - int(pg_fill * bh), bw, int(pg_fill * bh)))
        pg.draw.line(screen, (58, 70, 84), (bx - bw // 2, by), (bx + bw // 2, by), 1)
        screen.blit(font.render("throttle", True, GHOST), (bx - 34, args.h - 34))
        pg.display.flip()
        clock.tick(args.fps)
        frames += 1
        if args.smoke and frames >= 3:
            running = False

    pg.quit()
    if args.smoke:
        print("hil_hud smoke OK (3 frames rendered headless)")


if __name__ == "__main__":
    main()
