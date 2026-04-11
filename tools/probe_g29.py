#!/usr/bin/env python3
"""
G29 Axis/Button Probe
=====================
Move each control one at a time to see which SDL axis index it maps to.
Press Ctrl+C to quit.
"""

import pygame
import sys
import time

def main():
    pygame.init()
    pygame.joystick.init()

    n = pygame.joystick.get_count()
    if n == 0:
        print("No joysticks found.")
        sys.exit(1)

    print(f"Found {n} joystick(s):")
    for i in range(n):
        js = pygame.joystick.Joystick(i)
        js.init()
        print(f"  [{i}] {js.get_name()}  axes={js.get_numaxes()}  buttons={js.get_numbuttons()}  hats={js.get_numhats()}")

    # Pick first joystick (or pass index as arg)
    idx = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    js = pygame.joystick.Joystick(idx)
    js.init()
    print(f"\nUsing [{idx}] {js.get_name()}")
    print(f"  {js.get_numaxes()} axes, {js.get_numbuttons()} buttons, {js.get_numhats()} hats")
    print("\nMove controls one at a time. Only axes/buttons that change are printed.")
    print("Press Ctrl+C to quit.\n")

    num_axes = js.get_numaxes()
    num_buttons = js.get_numbuttons()
    num_hats = js.get_numhats()

    prev_axes = [0.0] * num_axes
    prev_buttons = [0] * num_buttons
    prev_hats = [(0, 0)] * num_hats

    # Let pygame settle
    for _ in range(10):
        pygame.event.pump()
        time.sleep(0.01)

    # Snapshot resting values
    for i in range(num_axes):
        prev_axes[i] = js.get_axis(i)

    print("Resting axis values:")
    for i in range(num_axes):
        print(f"  Axis {i}: {prev_axes[i]:+.4f}")
    print()

    try:
        while True:
            pygame.event.pump()

            for i in range(num_axes):
                val = js.get_axis(i)
                if abs(val - prev_axes[i]) > 0.05:
                    bar_len = int((val + 1) / 2 * 30)
                    bar = '#' * bar_len + '-' * (30 - bar_len)
                    print(f"  Axis {i}: {val:+.4f}  [{bar}]")
                    prev_axes[i] = val

            for i in range(num_buttons):
                val = js.get_button(i)
                if val != prev_buttons[i]:
                    print(f"  Button {i}: {'PRESSED' if val else 'released'}")
                    prev_buttons[i] = val

            for i in range(num_hats):
                val = js.get_hat(i)
                if val != prev_hats[i]:
                    print(f"  Hat {i}: {val}")
                    prev_hats[i] = val

            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nDone.")
    finally:
        pygame.quit()

if __name__ == '__main__':
    main()
