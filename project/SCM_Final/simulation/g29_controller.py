#!/usr/bin/env python3
"""
G29 Steering Wheel Interface and Manual Driver
===============================================

Provides G29Controller (pygame/SDL) and ManualDriver (ChDriver subclass)
for manual control of the HMMWV in PyChrono simulations.
"""

import numpy as np
from typing import Tuple

import pychrono.vehicle as veh


class G29Controller:
    """
    Logitech G29 steering wheel controller using pygame/SDL.
    
    Provides steering, throttle, and brake inputs from the G29 racing wheel.
    Axis mappings match the chrono-HIL controller_G29.json config.
    
    Force feedback is provided via SDL2 haptic API (PySDL2) for self-aligning
    torque that increases with speed and steering angle.
    """
    
    # G29 axis mapping (from controller_G29.json). Used when the detected
    # joystick name matches a Logitech wheel; for generic gamepads the
    # _gamepad_stick profile re-maps below.
    STEERING_AXIS = 0   # Wheel rotation
    THROTTLE_AXIS = 2   # Right pedal (gas)
    BRAKE_AXIS = 3      # Left pedal (brake)
    CLUTCH_AXIS = 1     # Middle pedal (clutch) - optional

    # ---- 'gamepad_stick' profile (8BitDo Ultimate 2C, Xbox/PS-style pads) ----
    # The throttle/brake live on ONE stick axis: rest=0, push up=-1 (throttle),
    # push down=+1 (brake). The left-stick X (axis 0) drives steering.
    # Detected automatically when the joystick name does not match G29/G27/G920
    # /Logitech; can be forced by passing ``profile='gamepad_stick'`` to the
    # constructor.
    GAMEPAD_STEER_AXIS    = 3  # left stick X
    GAMEPAD_THROT_BRK_AXIS = 1  # left stick Y (negated: up = throttle)
    GAMEPAD_DEADZONE       = 0.05  # absolute value below this snaps to 0
    
    # Raw axis ranges
    AXIS_MIN = -32768
    AXIS_MAX = 32767
    
    # Force feedback parameters
    FF_MAX_FORCE = 32767  # SDL haptic max force
    FF_BASE_STRENGTH = 0.4  # Base self-aligning torque strength (0-1)
    FF_SPEED_FACTOR = 0.08  # Additional force per m/s of speed
    FF_FRICTION = 0.25  # Friction force that opposes steering velocity
    FF_ROAD_FEEL = 0.15  # Speed-dependent constant resistance ("road feel")
    
    def __init__(self, joystick_index: int = None, enable_force_feedback: bool = False,
                 profile: str = "auto"):
        """
        Initialize G29 controller.

        Args:
            joystick_index: Specific joystick index, or None to auto-detect G29
            enable_force_feedback: Whether to enable force feedback (default: True)
            profile: 'g29' for Logitech G29/G27/G920 (pedals on separate axes,
                rest=-1, pressed=+1), 'gamepad_stick' for Xbox/PS/8BitDo-style
                gamepads (throttle+brake combined on one stick Y axis, rest=0,
                up=throttle, down=brake), or 'auto' (default; chosen from the
                joystick name).
        """
        self.joystick = None
        self.steering = 0.0
        self.throttle = 0.0
        self.brake = 0.0
        self._initialized = False
        self._joystick_index = joystick_index
        self._enable_ff = enable_force_feedback
        self._profile_arg = profile  # 'auto' | 'g29' | 'gamepad_stick'
        self._profile = None         # resolved after init

        # Per-pedal released-baseline auto-calibration (G29 profile only).
        # The released rest value is a fixed hardware/driver property but is
        # NOT the same everywhere: this rig's G29 rests at +1 (pressed -> -1),
        # other setups rest at -1 (pressed -> +1). Do NOT assume a platform
        # convention (that was backwards on this rig) and do NOT sniff a single
        # early frame (an untouched axis reads ~0.0 under SDL2 until its first
        # motion event, and one spurious frame silently SWAPPED throttle/brake
        # between rounds). Instead DETECT each pedal's released baseline from a
        # stable, live, multi-frame reading; until calibrated, output 0
        # (released). throttle/brake = fraction the pedal has travelled from its
        # released rest toward the opposite (fully-pressed) extreme.
        self._pedal_cal = {
            "thr": {"rest": None, "sign": 0.0, "streak": 0},
            "brk": {"rest": None, "sign": 0.0, "streak": 0},
        }
        
        # Force feedback state (PySDL2)
        self._haptic = None
        self._ff_effect_id = -1
        self._ff_initialized = False
        self._last_steering = 0.0  # For steering velocity calculation
        
    def initialize(self) -> bool:
        """Initialize pygame and connect to G29. Returns True on success."""
        try:
            import pygame
            
            # Initialize pygame joystick subsystem only (no display needed)
            pygame.init()
            pygame.joystick.init()
            
            num_joysticks = pygame.joystick.get_count()
            if num_joysticks == 0:
                print("  No joysticks found!")
                return False
            
            print(f"  Found {num_joysticks} joystick(s):")
            
            # Find G29 or use specified index
            chosen_index = self._joystick_index
            for i in range(num_joysticks):
                js = pygame.joystick.Joystick(i)
                name = js.get_name()
                print(f"    [{i}] {name}")
                
                # Auto-detect G29
                if chosen_index is None and 'G29' in name.upper():
                    chosen_index = i
                # Also accept G27, G920 (similar Logitech wheels)
                elif chosen_index is None and any(x in name.upper() for x in ['G27', 'G920', 'LOGITECH']):
                    chosen_index = i
            
            # Fall back to first non-keyboard device
            if chosen_index is None:
                for i in range(num_joysticks):
                    js = pygame.joystick.Joystick(i)
                    name = js.get_name().upper()
                    if 'KEYBOARD' not in name and 'KVM' not in name:
                        chosen_index = i
                        break
            
            if chosen_index is None:
                chosen_index = 0
            
            # Initialize the chosen joystick
            self.joystick = pygame.joystick.Joystick(chosen_index)
            self.joystick.init()
            
            print(f"  Using: {self.joystick.get_name()}")
            print(f"    Axes: {self.joystick.get_numaxes()}, "
                  f"Buttons: {self.joystick.get_numbuttons()}")

            # Resolve controller profile -----------------------------------
            name_upper = self.joystick.get_name().upper()
            if self._profile_arg == "auto":
                if any(tag in name_upper for tag in ("G29", "G27", "G920", "LOGITECH")):
                    self._profile = "g29"
                else:
                    # Anything else (8BitDo, Xbox, PS, DualSense, ...) →
                    # treat as a gamepad with combined throttle/brake stick.
                    self._profile = "gamepad_stick"
            else:
                self._profile = self._profile_arg
            print(f"    Controller profile: {self._profile}")

            self._initialized = True
            
            # Initialize force feedback via PySDL2
            if self._enable_ff:
                self._init_force_feedback(chosen_index)
            
            return True
            
        except ImportError:
            print("  pygame not installed! Run: pip install pygame")
            return False
        except Exception as e:
            print(f"  G29 initialization error: {e}")
            return False
    
    def update(self) -> None:
        """Poll joystick and update steering/throttle/brake values."""
        if not self._initialized:
            return

        import pygame
        pygame.event.pump()  # Process events

        if self._profile == "gamepad_stick":
            # 8BitDo / Xbox / PS-style pad: left-stick X = steering,
            # left-stick Y = combined throttle/brake (up positive throttle,
            # down positive brake). Axis rests near 0, not -1.
            raw_steering = self.joystick.get_axis(self.GAMEPAD_STEER_AXIS)
            raw_y        = self.joystick.get_axis(self.GAMEPAD_THROT_BRK_AXIS)
            # Apply a small deadzone so spring imperfections at rest don't
            # produce phantom throttle/brake.
            if abs(raw_steering) < self.GAMEPAD_DEADZONE:
                raw_steering = 0.0
            if abs(raw_y) < self.GAMEPAD_DEADZONE:
                raw_y = 0.0
            # Steering: stick X is left-negative; the existing G29 path
            # negates as well so the chassis turns right when the operator
            # pushes the stick right.
            self.steering = -raw_steering
            # Throttle / brake split: stick UP (raw_y < 0) → throttle,
            # stick DOWN (raw_y > 0) → brake.
            if raw_y < 0.0:
                self.throttle = -raw_y
                self.brake = 0.0
            else:
                self.throttle = 0.0
                self.brake = raw_y
        else:
            # Original G29 / Logitech-wheel path.
            raw_steering = self.joystick.get_axis(self.STEERING_AXIS)
            raw_throttle = self.joystick.get_axis(self.THROTTLE_AXIS)
            raw_brake = self.joystick.get_axis(self.BRAKE_AXIS)

            self.steering = -raw_steering
            self.throttle = self._pedal_fraction("thr", raw_throttle)
            self.brake = self._pedal_fraction("brk", raw_brake)

        # Clamp to valid range
        self.steering = max(-1.0, min(1.0, self.steering))
        self.throttle = max(0.0, min(1.0, self.throttle))
        self.brake = max(0.0, min(1.0, self.brake))

        # Diagnostic: print raw + mapped values for first 60 frames (~2 s)
        if hasattr(self, '_diag_count'):
            self._diag_count += 1
        else:
            self._diag_count = 0
        if self._diag_count < 60 and self._diag_count % 10 == 0:
            if self._profile == "gamepad_stick":
                raw_y = self.joystick.get_axis(self.GAMEPAD_THROT_BRK_AXIS)
                raw_st = self.joystick.get_axis(self.GAMEPAD_STEER_AXIS)
                print(f"    [pad diag #{self._diag_count}] "
                      f"raw_steer={raw_st:+.3f} raw_y={raw_y:+.3f}  "
                      f"-> thr={self.throttle:.3f} brk={self.brake:.3f} "
                      f"steer={self.steering:+.3f}  [gamepad_stick]")
            else:
                raw_throttle = self.joystick.get_axis(self.THROTTLE_AXIS)
                raw_brake = self.joystick.get_axis(self.BRAKE_AXIS)
                _tr = self._pedal_cal["thr"]["rest"]
                _br = self._pedal_cal["brk"]["rest"]
                inv_tag = f"rest thr={_tr} brk={_br}"
                print(f"    [G29 diag #{self._diag_count}] "
                      f"raw_thr={raw_throttle:+.3f} raw_brk={raw_brake:+.3f}"
                      f" -> thr={self.throttle:.3f} brk={self.brake:.3f} "
                      f"steer={self.steering:+.3f} [{inv_tag}]")
    
    def _pedal_fraction(self, key: str, raw: float) -> float:
        """Map a raw pedal-axis reading to [0,1] travel, auto-calibrating the
        released baseline from a stable live reading.

        The released rest is +1 on some G29s and -1 on others; we detect it
        rather than assume it. A reading is a calibration vote only when it is
        clearly live (|raw| >= 0.5), so the cold ~0.0 reading an axis returns
        before its first SDL motion event never sets the baseline. The baseline
        locks after a short run of same-sign live frames, which rejects a lone
        spurious frame -- the cause of throttle/brake swapping between rounds.
        Until locked, the pedal reads released (0).
        """
        c = self._pedal_cal[key]
        if c["rest"] is None:
            if abs(raw) >= 0.5:
                sign = 1.0 if raw > 0.0 else -1.0
                c["streak"] = c["streak"] + 1 if sign == c["sign"] else 1
                c["sign"] = sign
                if c["streak"] >= 5:        # stable released baseline found
                    c["rest"] = sign
            else:
                c["streak"], c["sign"] = 0, 0.0
            if c["rest"] is None:
                return 0.0                  # not yet calibrated -> released
        rest = c["rest"]
        # released (raw==rest) -> 0; fully pressed (raw==-rest) -> 1.
        return max(0.0, min(1.0, (rest - raw) / (2.0 * rest)))

    def get_inputs(self) -> Tuple[float, float, float]:
        """Get current (steering, throttle, brake) values."""
        self.update()
        return self.steering, self.throttle, self.brake
    
    def is_button_pressed(self, button: int) -> bool:
        """Check if a button is currently pressed."""
        if not self._initialized:
            return False
        import pygame
        pygame.event.pump()
        return self.joystick.get_button(button)
    
    def _init_force_feedback(self, joystick_index: int) -> None:
        """Initialize SDL2 haptic device for force feedback."""
        try:
            import sdl2
            import ctypes
            
            # Initialize SDL haptic subsystem (separate from pygame's SDL)
            sdl2.SDL_Init(sdl2.SDL_INIT_HAPTIC)
            
            # Get number of haptic devices
            num_haptic = sdl2.SDL_NumHaptics()
            if num_haptic <= 0:
                print(f"    ⚠ No haptic devices found")
                return
            
            # Find the haptic device that matches our joystick
            # Usually it's the same index, but we search by name to be sure
            haptic_index = -1
            joystick_name = self.joystick.get_name().upper()
            
            for i in range(num_haptic):
                name = sdl2.SDL_HapticName(i)
                if name:
                    name = name.decode('utf-8', errors='ignore').upper()
                    if 'G29' in name or 'LOGITECH' in name:
                        haptic_index = i
                        break
            
            if haptic_index < 0:
                # Fall back to first haptic device
                haptic_index = 0
            
            # Open haptic device directly by index (not from joystick)
            self._haptic = sdl2.SDL_HapticOpen(haptic_index)
            if not self._haptic:
                print(f"    ⚠ Could not open haptic device: {sdl2.SDL_GetError()}")
                return
            
            # Check for constant force support
            caps = sdl2.SDL_HapticQuery(self._haptic)
            if not (caps & sdl2.SDL_HAPTIC_CONSTANT):
                print(f"    ⚠ Haptic device doesn't support constant force")
                sdl2.SDL_HapticClose(self._haptic)
                self._haptic = None
                return
            
            # Create constant force effect for self-aligning torque
            effect = sdl2.SDL_HapticEffect()
            ctypes.memset(ctypes.byref(effect), 0, ctypes.sizeof(effect))
            effect.type = sdl2.SDL_HAPTIC_CONSTANT
            effect.constant.direction.type = sdl2.SDL_HAPTIC_CARTESIAN
            effect.constant.direction.dir[0] = 1  # X-axis (steering)
            effect.constant.length = sdl2.SDL_HAPTIC_INFINITY  # Continuous
            effect.constant.level = 0  # Start with no force
            effect.constant.attack_length = 0
            effect.constant.fade_length = 0
            
            self._ff_effect_id = sdl2.SDL_HapticNewEffect(self._haptic, ctypes.byref(effect))
            if self._ff_effect_id < 0:
                print(f"    ⚠ Could not create haptic effect: {sdl2.SDL_GetError()}")
                sdl2.SDL_HapticClose(self._haptic)
                self._haptic = None
                return
            
            # Run the effect
            sdl2.SDL_HapticRunEffect(self._haptic, self._ff_effect_id, 1)
            
            self._ff_initialized = True
            print(f"    Force feedback enabled (self-aligning torque)")
            
        except ImportError:
            print(f"    ⚠ PySDL2 not installed - no force feedback")
        except Exception as e:
            print(f"    ⚠ Force feedback init error: {e}")
    
    def update_force_feedback(self, speed: float = 0.0, lateral_accel: float = 0.0) -> None:
        """
        Update force feedback based on vehicle state.
        
        Args:
            speed: Vehicle speed in m/s
            lateral_accel: Lateral acceleration in m/s^2 (for cornering feel)
        """
        if not self._ff_initialized or not self._haptic:
            return
        
        try:
            import sdl2
            import ctypes
            
            # === Force components ===
            
            # 1. Self-aligning torque: proportional to steering angle
            #    Pushes wheel back toward center, stronger at speed
            steer_force = -self.steering * self.FF_BASE_STRENGTH
            speed_mult = 1.0 + min(speed * self.FF_SPEED_FACTOR, 0.8)
            steer_force *= speed_mult
            
            # 2. Friction: opposes steering velocity (makes wheel feel heavy)
            #    Approximated from change in steering position
            steer_velocity = self.steering - self._last_steering
            friction_force = -steer_velocity * self.FF_FRICTION * 50  # Scale for responsiveness
            friction_force = max(-0.3, min(0.3, friction_force))  # Clamp friction
            self._last_steering = self.steering
            
            # 3. Road feel: constant resistance proportional to speed
            #    Creates "weight" in the wheel even when centered
            road_feel = self.FF_ROAD_FEEL * min(speed / 15.0, 1.0)  # Ramp up to 15 m/s
            # Apply road feel as centering toward current position (creates inertia)
            # This makes the wheel "sticky" and harder to move at speed
            
            # 4. Lateral acceleration feedback (understeer/oversteer feel)
            lat_force = -lateral_accel * 0.03
            
            # Combine forces
            total_force = steer_force + friction_force + lat_force
            
            # Add road feel as a bias toward holding position
            if abs(self.steering) < 0.1 and speed > 2.0:
                # Near center at speed: add resistance to movement
                total_force += -steer_velocity * road_feel * 30
            
            # Clamp and convert to SDL haptic range
            total_force = max(-1.0, min(1.0, total_force))
            haptic_level = int(total_force * self.FF_MAX_FORCE)
            
            # Update the effect
            effect = sdl2.SDL_HapticEffect()
            ctypes.memset(ctypes.byref(effect), 0, ctypes.sizeof(effect))
            effect.type = sdl2.SDL_HAPTIC_CONSTANT
            effect.constant.direction.type = sdl2.SDL_HAPTIC_CARTESIAN
            effect.constant.direction.dir[0] = 1
            effect.constant.length = sdl2.SDL_HAPTIC_INFINITY
            effect.constant.level = haptic_level
            
            sdl2.SDL_HapticUpdateEffect(self._haptic, self._ff_effect_id, ctypes.byref(effect))
            
        except Exception as e:
            pass  # Silently ignore force feedback errors during simulation
    
    def close(self) -> None:
        """Clean up pygame and haptic resources."""
        # Stop and close haptic
        if self._ff_initialized and self._haptic:
            try:
                import sdl2
                sdl2.SDL_HapticStopEffect(self._haptic, self._ff_effect_id)
                sdl2.SDL_HapticDestroyEffect(self._haptic, self._ff_effect_id)
                sdl2.SDL_HapticClose(self._haptic)
            except:
                pass
            self._haptic = None
            self._ff_initialized = False
        
        if self._initialized:
            import pygame
            pygame.joystick.quit()
            pygame.quit()
            self._initialized = False


class ManualDriver(veh.ChDriver):
    """
    Manual driver using G29 steering wheel or keyboard fallback.
    
    Provides direct control via Logitech G29 or similar steering wheel.
    """
    
    def __init__(self, vehicle, g29: G29Controller = None):
        """
        Args:
            vehicle: PyChrono WheeledVehicle
            g29: G29Controller instance (will be created if None)
        """
        super().__init__(vehicle.GetVehicle())
        
        self.vehicle = vehicle
        self.g29 = g29 if g29 is not None else G29Controller()
        
        # Control states
        self.m_steering = 0.0
        self.m_throttle = 0.0
        self.m_braking = 0.0
        
        # For compatibility with MPC driver cleanup code
        self.mpc_worker = None
        self.mp_worker = None
        self.state_history = []  # No state history for manual control
        
        # Initialize G29
        self._g29_available = self.g29.initialize()
        if not self._g29_available:
            print("  ⚠ G29 not available - vehicle will not respond to inputs")
    
    def Synchronize(self, time):
        """Called by Chrono to update driver inputs"""
        if self._g29_available:
            steering, throttle, brake = self.g29.get_inputs()
            self.m_steering = steering
            self.m_throttle = throttle
            self.m_braking = brake
            
            # Update force feedback based on vehicle state
            speed = self.vehicle.GetVehicle().GetSpeed()  # m/s
            # Get lateral acceleration for cornering feel
            try:
                chassis_acc = self.vehicle.GetChassisBody().GetPosDt2()  # Global frame accel
                lat_accel = chassis_acc.y  # Approximate lateral component
            except:
                lat_accel = 0.0
            self.g29.update_force_feedback(speed=speed, lateral_accel=lat_accel)
    
    def Advance(self, step):
        """Advance driver state (nothing needed for manual control)"""
        pass
    
    def GetSteering(self):
        return self.m_steering
    
    def GetThrottle(self):
        return self.m_throttle
    
    def GetBraking(self):
        return self.m_braking
    
    def shutdown(self):
        """Clean up G29 resources"""
        if self._g29_available:
            self.g29.close()
    
    def get_mpc_stats(self):
        """No MPC stats for manual control"""
        return None


# =========================================================================
# WASD Keyboard Driver
# =========================================================================

class WASDDriver(veh.ChDriver):
    """
    Manual driver using WASD / arrow keys via a small pygame window.

    Controls:
        W / Up    — Throttle
        S / Down  — Brake
        A / Left  — Steer left
        D / Right — Steer right
        Space     — Handbrake (full brake)
        Q / Esc   — Quit (zeros inputs)
    """

    # Tuning constants
    STEER_RATE = 2.0          # Full lock in 0.5s
    STEER_RETURN_RATE = 4.0   # Centre in 0.25s when released
    THROTTLE_RATE = 3.0       # Full throttle in 0.33s
    THROTTLE_DECAY = 5.0      # Release in 0.2s
    BRAKE_RATE = 5.0          # Full brake in 0.2s
    BRAKE_DECAY = 8.0         # Release in 0.125s

    def __init__(self, vehicle):
        super().__init__(vehicle.GetVehicle())
        self.vehicle = vehicle
        self.m_steering = 0.0
        self.m_throttle = 0.0
        self.m_braking = 0.0
        self._prev_time = 0.0
        self._quit = False

        # For compatibility with code that checks these
        self.mpc_worker = None
        self.mp_worker = None
        self.state_history = []

        # Defer pygame init to first Synchronize() so it doesn't conflict
        # with Irrlicht's window/display initialization.
        self._pygame = None
        self._screen = None
        self._font = None
        self._initialized = False

    def _init_pygame(self):
        """Lazy-initialize pygame after Irrlicht is already running."""
        import pygame
        self._pygame = pygame
        pygame.init()
        self._screen = pygame.display.set_mode((320, 120))
        pygame.display.set_caption("WASD Driver — focus this window")
        self._font = pygame.font.SysFont("monospace", 14)
        self._initialized = True
        print("  WASD driver: focus the 'WASD Driver' window to drive")

    def Synchronize(self, time):
        """Called by Chrono each physics step — read keys and update inputs."""
        if not self._initialized:
            self._init_pygame()
            return

        pg = self._pygame
        dt = time - self._prev_time if self._prev_time > 0 else 0.003
        self._prev_time = time

        # Pump events (needed for key state)
        for event in pg.event.get():
            if event.type == pg.QUIT:
                self._quit = True
            elif event.type == pg.KEYDOWN and event.key in (pg.K_q, pg.K_ESCAPE):
                self._quit = True

        if self._quit:
            self.m_throttle = 0.0
            self.m_braking = 0.0
            return

        keys = pg.key.get_pressed()
        steer_left = keys[pg.K_a] or keys[pg.K_LEFT]
        steer_right = keys[pg.K_d] or keys[pg.K_RIGHT]
        accel = keys[pg.K_w] or keys[pg.K_UP]
        brake = keys[pg.K_s] or keys[pg.K_DOWN] or keys[pg.K_SPACE]

        # Steering (positive = left, SAE convention)
        if steer_left and not steer_right:
            self.m_steering = min(1.0, self.m_steering + self.STEER_RATE * dt)
        elif steer_right and not steer_left:
            self.m_steering = max(-1.0, self.m_steering - self.STEER_RATE * dt)
        else:
            if self.m_steering > 0:
                self.m_steering = max(0.0, self.m_steering - self.STEER_RETURN_RATE * dt)
            elif self.m_steering < 0:
                self.m_steering = min(0.0, self.m_steering + self.STEER_RETURN_RATE * dt)

        # Throttle
        if accel:
            self.m_throttle = min(1.0, self.m_throttle + self.THROTTLE_RATE * dt)
        else:
            self.m_throttle = max(0.0, self.m_throttle - self.THROTTLE_DECAY * dt)

        # Braking
        if brake:
            self.m_braking = min(1.0, self.m_braking + self.BRAKE_RATE * dt)
        else:
            self.m_braking = max(0.0, self.m_braking - self.BRAKE_DECAY * dt)

        # Draw mini-HUD
        self._draw_hud()

    def _draw_hud(self):
        pg = self._pygame
        scr = self._screen
        scr.fill((30, 30, 30))
        y = 5
        speed = self.vehicle.GetVehicle().GetSpeed()

        lines = [
            (f"Steer: {self.m_steering:+.2f}", (100, 180, 255)),
            (f"Throt: {self.m_throttle:.2f}  Brake: {self.m_braking:.2f}",
             (80, 220, 80) if self.m_throttle > self.m_braking else (220, 80, 80)),
            (f"Speed: {speed:.1f} m/s  ({speed * 3.6:.0f} km/h)", (220, 220, 100)),
            ("WASD/Arrows=drive  Space=brake  Q=quit", (120, 120, 120)),
        ]
        for text, color in lines:
            surf = self._font.render(text, True, color)
            scr.blit(surf, (10, y))
            y += 22
        # Steering bar
        bar_y = y + 2
        bar_w, bar_h = 200, 10
        bar_x = 60
        pg.draw.rect(scr, (60, 60, 60), (bar_x, bar_y, bar_w, bar_h))
        mid = bar_x + bar_w // 2
        steer_px = int(self.m_steering * (bar_w // 2))
        if steer_px > 0:
            pg.draw.rect(scr, (100, 180, 255), (mid, bar_y, steer_px, bar_h))
        elif steer_px < 0:
            pg.draw.rect(scr, (100, 180, 255), (mid + steer_px, bar_y, -steer_px, bar_h))
        pg.draw.line(scr, (200, 200, 200), (mid, bar_y), (mid, bar_y + bar_h), 1)

        pg.display.flip()

    def Advance(self, step):
        pass

    def GetSteering(self):
        return self.m_steering

    def GetThrottle(self):
        return self.m_throttle

    def GetBraking(self):
        return self.m_braking

    def shutdown(self):
        if self._initialized:
            self._pygame.quit()

    def get_mpc_stats(self):
        return None
