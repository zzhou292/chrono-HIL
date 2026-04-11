#!/usr/bin/env python3
"""
WASD Keyboard Driver Node
==========================

Standalone ZMQ node that reads WASD keyboard input and publishes
ControlCommand messages, replacing the MPC controller for manual driving.

Controls:
    W / Up    — Throttle
    S / Down  — Brake
    A / Left  — Steer left
    D / Right — Steer right
    Space     — Handbrake (full brake)
    Q / Esc   — Quit

Requires pygame for keyboard input.
"""

import argparse
import signal
import sys
import time

import pygame

from hil_messages import (
    ControlCommand,
    ZMQPublisher,
    ZMQSubscriber,
    ctrl_pub_endpoint,
    sim_sub_endpoint,
    parse_message,
)

# ---------------------------------------------------------------------------
# Tuning constants
# ---------------------------------------------------------------------------
PUBLISH_HZ = 50           # Control publish rate
STEER_RATE = 2.0          # Full lock in 0.5s
STEER_RETURN_RATE = 4.0   # Centre in 0.25s when released
THROTTLE_RATE = 3.0       # Full throttle in 0.33s
THROTTLE_DECAY = 5.0      # Release in 0.2s
BRAKE_RATE = 5.0          # Full brake in 0.2s
BRAKE_DECAY = 8.0         # Release in 0.125s
MAX_STEERING = 1.0
MAX_THROTTLE = 1.0
MAX_BRAKING = 1.0

# Pygame window size
WIN_W, WIN_H = 320, 200


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def draw_hud(screen, font, steering, throttle, braking, speed, sim_time):
    """Draw a simple HUD showing current inputs and vehicle speed."""
    screen.fill((30, 30, 30))

    # Title
    title = font.render("WASD Driver", True, (200, 200, 200))
    screen.blit(title, (10, 5))

    y = 35
    # Steering bar
    bar_w, bar_h = 200, 18
    bar_x = 100
    pygame.draw.rect(screen, (60, 60, 60), (bar_x, y, bar_w, bar_h))
    mid = bar_x + bar_w // 2
    steer_px = int(steering * (bar_w // 2))
    if steer_px > 0:
        pygame.draw.rect(screen, (100, 180, 255), (mid, y, steer_px, bar_h))
    elif steer_px < 0:
        pygame.draw.rect(screen, (100, 180, 255), (mid + steer_px, y, -steer_px, bar_h))
    pygame.draw.line(screen, (200, 200, 200), (mid, y), (mid, y + bar_h), 1)
    lbl = font.render(f"Steer: {steering:+.2f}", True, (200, 200, 200))
    screen.blit(lbl, (10, y))

    # Throttle bar
    y += 28
    pygame.draw.rect(screen, (60, 60, 60), (bar_x, y, bar_w, bar_h))
    tw = int(throttle * bar_w)
    pygame.draw.rect(screen, (80, 220, 80), (bar_x, y, tw, bar_h))
    lbl = font.render(f"Throt: {throttle:.2f}", True, (200, 200, 200))
    screen.blit(lbl, (10, y))

    # Brake bar
    y += 28
    pygame.draw.rect(screen, (60, 60, 60), (bar_x, y, bar_w, bar_h))
    bw = int(braking * bar_w)
    pygame.draw.rect(screen, (220, 80, 80), (bar_x, y, bw, bar_h))
    lbl = font.render(f"Brake: {braking:.2f}", True, (200, 200, 200))
    screen.blit(lbl, (10, y))

    # Speed & time
    y += 32
    spd_text = f"Speed: {speed:.1f} m/s ({speed * 3.6:.0f} km/h)"
    lbl = font.render(spd_text, True, (220, 220, 100))
    screen.blit(lbl, (10, y))

    y += 22
    lbl = font.render(f"Sim time: {sim_time:.1f}s", True, (160, 160, 160))
    screen.blit(lbl, (10, y))

    y += 22
    lbl = font.render("WASD/Arrows=drive  Space=brake  Q=quit", True, (120, 120, 120))
    screen.blit(lbl, (10, y))

    pygame.display.flip()


def run(args):
    # ZMQ setup
    ctrl_pub = ZMQPublisher(ctrl_pub_endpoint(args.ctrl_port))
    sim_sub = ZMQSubscriber(sim_sub_endpoint(args.sim_host, args.sim_port))

    # Pygame setup
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("WASD Driver")
    font = pygame.font.SysFont("monospace", 14)
    clock = pygame.time.Clock()

    steering = 0.0
    throttle = 0.0
    braking = 0.0
    seq = 0
    sim_time = 0.0
    speed = 0.0

    running = True
    dt = 1.0 / PUBLISH_HZ

    print("[wasd] WASD driver ready. Focus the pygame window and drive!")
    print("[wasd] W/Up=throttle  S/Down=brake  A/Left=steer left  D/Right=steer right")
    print("[wasd] Space=handbrake  Q/Esc=quit")

    while running:
        # --- Process pygame events ---
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_q, pygame.K_ESCAPE):
                    running = False

        # --- Read held keys ---
        keys = pygame.key.get_pressed()
        steer_left = keys[pygame.K_a] or keys[pygame.K_LEFT]
        steer_right = keys[pygame.K_d] or keys[pygame.K_RIGHT]
        accel = keys[pygame.K_w] or keys[pygame.K_UP]
        brake = keys[pygame.K_s] or keys[pygame.K_DOWN] or keys[pygame.K_SPACE]

        # --- Update steering (positive = left, SAE convention) ---
        if steer_left and not steer_right:
            steering = clamp(steering + STEER_RATE * dt, -MAX_STEERING, MAX_STEERING)
        elif steer_right and not steer_left:
            steering = clamp(steering - STEER_RATE * dt, -MAX_STEERING, MAX_STEERING)
        else:
            # Return to centre
            if steering > 0:
                steering = max(0.0, steering - STEER_RETURN_RATE * dt)
            elif steering < 0:
                steering = min(0.0, steering + STEER_RETURN_RATE * dt)

        # --- Update throttle ---
        if accel:
            throttle = clamp(throttle + THROTTLE_RATE * dt, 0.0, MAX_THROTTLE)
        else:
            throttle = clamp(throttle - THROTTLE_DECAY * dt, 0.0, MAX_THROTTLE)

        # --- Update braking ---
        if brake:
            braking = clamp(braking + BRAKE_RATE * dt, 0.0, MAX_BRAKING)
        else:
            braking = clamp(braking - BRAKE_DECAY * dt, 0.0, MAX_BRAKING)

        # --- Read latest vehicle state (non-blocking) ---
        result = sim_sub.recv(timeout_ms=0)
        if result is not None:
            topic, msg = result
            if topic == "vehicle_state":
                sim_time = msg.time
                speed = msg.u

        # --- Publish control command ---
        wall_now = time.time()
        cmd = ControlCommand(
            time=sim_time,
            wall_time=wall_now,
            seq=seq,
            steering=steering,
            throttle=throttle,
            braking=braking,
            delta=0.0,
            acceleration=0.0,
            delta_dot=0.0,
            jerk=0.0,
            solve_time_ms=0.0,
            mpc_cost=0.0,
        )
        ctrl_pub.send(cmd)
        seq += 1

        # --- Draw HUD ---
        draw_hud(screen, font, steering, throttle, braking, speed, sim_time)

        # --- Rate limit ---
        clock.tick(PUBLISH_HZ)

    # Cleanup
    print("[wasd] Shutting down.")
    pygame.quit()
    ctrl_pub.close()
    sim_sub.close()


def main():
    p = argparse.ArgumentParser(description="WASD keyboard driver for HMMWV simulation")
    p.add_argument("--sim-host", default="localhost")
    p.add_argument("--sim-port", type=int, default=5555)
    p.add_argument("--ctrl-port", type=int, default=5556)
    args = p.parse_args()

    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    run(args)


if __name__ == "__main__":
    main()
