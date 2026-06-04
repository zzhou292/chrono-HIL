#!/usr/bin/env python3
"""Live joystick / gamepad probe.

Lists every connected joystick, then continuously prints axis values
(snapshot, not just events, so resting positions are visible too) and
flags any button / hat / axis transition. Use it to discover the right
axis indices when wiring a new controller into ``g29_controller.py``.

Usage:
    python utilities/joystick_probe.py
    python utilities/joystick_probe.py --joystick 0   # select by index
    python utilities/joystick_probe.py --period-ms 50 # poll faster

Press Ctrl-C to exit. Designed to work on any pygame-supported gamepad,
not just the Logitech G29 -- on the 8BitDo Ultimate 2C wireless the
right trigger is usually axis 5 (rests near -1) and the left trigger
is axis 4 (rests near -1), with the right thumbstick on axes 2 / 3.
"""

from __future__ import annotations

import argparse
import os
import sys
import time


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--joystick", type=int, default=0,
                   help="Index of the joystick to probe (default 0).")
    p.add_argument("--period-ms", type=int, default=100,
                   help="Snapshot period (default 100 ms). Lower = busier output.")
    p.add_argument("--quiet-axis-threshold", type=float, default=0.02,
                   help="Suppress axis lines whose abs(value) is below this "
                        "in the rest-state snapshot (default 0.02).")
    args = p.parse_args()

    # Headless: pygame still needs a video driver init even with no display.
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    import pygame  # noqa: E402

    pygame.init()
    pygame.joystick.init()
    n = pygame.joystick.get_count()
    if n == 0:
        print("No joysticks detected. (Is the controller paired / plugged in?)")
        return 1
    print(f"Found {n} joystick(s):")
    for i in range(n):
        js = pygame.joystick.Joystick(i)
        js.init()
        print(f"  [{i}] {js.get_name()}  axes={js.get_numaxes()} "
              f"buttons={js.get_numbuttons()} hats={js.get_numhats()}")
    if args.joystick >= n:
        print(f"--joystick {args.joystick} out of range")
        return 1
    js = pygame.joystick.Joystick(args.joystick)
    js.init()
    print()
    print(f"=== Probing [{args.joystick}] {js.get_name()} ===")
    print("Move every stick + trigger + button + hat once. Each event is "
          "logged separately; the snapshot table below updates each tick.")
    print("(Ctrl-C to exit)")
    print()

    last_axis = [js.get_axis(i) for i in range(js.get_numaxes())]
    last_btn  = [js.get_button(i) for i in range(js.get_numbuttons())]
    last_hat  = [js.get_hat(i) for i in range(js.get_numhats())]

    tick = 0
    period = max(0.005, args.period_ms / 1000.0)
    try:
        while True:
            pygame.event.pump()

            # Event-style: print only what changed since last tick.
            for i in range(js.get_numaxes()):
                v = js.get_axis(i)
                if abs(v - last_axis[i]) > 0.05:
                    print(f"  AXIS[{i}] {last_axis[i]:+.3f} -> {v:+.3f}")
                    last_axis[i] = v
            for i in range(js.get_numbuttons()):
                b = js.get_button(i)
                if b != last_btn[i]:
                    state = "DOWN" if b else "UP"
                    print(f"  BUTTON[{i}] {state}")
                    last_btn[i] = b
            for i in range(js.get_numhats()):
                h = js.get_hat(i)
                if h != last_hat[i]:
                    print(f"  HAT[{i}] {last_hat[i]} -> {h}")
                    last_hat[i] = h

            # Snapshot once per second to show resting values too.
            tick += 1
            if tick * period >= 1.0:
                tick = 0
                snap = []
                for i in range(js.get_numaxes()):
                    v = js.get_axis(i)
                    if abs(v) >= args.quiet_axis_threshold or True:
                        snap.append(f"a{i}={v:+.2f}")
                btn_pressed = [str(i) for i in range(js.get_numbuttons())
                                if js.get_button(i)]
                hat_pressed = [f"h{i}={js.get_hat(i)}"
                               for i in range(js.get_numhats())]
                print(f"  [snap] {' '.join(snap)}"
                      + (f"   btn={','.join(btn_pressed)}" if btn_pressed else "")
                      + (f"   {' '.join(hat_pressed)}" if hat_pressed else ""))
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n(stopped)")
        return 0


if __name__ == "__main__":
    sys.exit(main())
