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
    
    # G29 axis mapping (from controller_G29.json)
    STEERING_AXIS = 0   # Wheel rotation
    THROTTLE_AXIS = 2   # Right pedal (gas)
    BRAKE_AXIS = 3      # Left pedal (brake)
    CLUTCH_AXIS = 1     # Middle pedal (clutch) - optional
    
    # Raw axis ranges
    AXIS_MIN = -32768
    AXIS_MAX = 32767
    
    # Force feedback parameters
    FF_MAX_FORCE = 32767  # SDL haptic max force
    FF_BASE_STRENGTH = 0.4  # Base self-aligning torque strength (0-1)
    FF_SPEED_FACTOR = 0.08  # Additional force per m/s of speed
    FF_FRICTION = 0.25  # Friction force that opposes steering velocity
    FF_ROAD_FEEL = 0.15  # Speed-dependent constant resistance ("road feel")
    
    def __init__(self, joystick_index: int = None, enable_force_feedback: bool = False):
        """
        Initialize G29 controller.
        
        Args:
            joystick_index: Specific joystick index, or None to auto-detect G29
            enable_force_feedback: Whether to enable force feedback (default: True)
        """
        self.joystick = None
        self.steering = 0.0
        self.throttle = 0.0
        self.brake = 0.0
        self._initialized = False
        self._joystick_index = joystick_index
        self._enable_ff = enable_force_feedback
        
        # Pedal convention auto-detection
        # Linux G29: rest=-1, pressed=+1; Windows G29: rest=+1, pressed=-1
        self._pedal_inverted = None  # None=not yet detected
        self._pedal_detect_frames = 0
        
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
        
        # Read raw axis values
        raw_steering = self.joystick.get_axis(self.STEERING_AXIS)
        raw_throttle = self.joystick.get_axis(self.THROTTLE_AXIS)  
        raw_brake = self.joystick.get_axis(self.BRAKE_AXIS)
        
        # Steering: axis returns -1 to 1, negate for correct direction
        self.steering = -raw_steering
        
        # Auto-detect pedal convention on first few frames (pedals should be released)
        if self._pedal_inverted is None:
            self._pedal_detect_frames += 1
            if self._pedal_detect_frames >= 3:
                # After a few event pumps, check pedal rest position
                if raw_throttle < -0.5:
                    # Linux: rest=-1, pressed=+1
                    self._pedal_inverted = True
                    print(f"    G29 pedals: Linux convention detected (rest={raw_throttle:.2f})")
                else:
                    # Windows/default: rest=+1, pressed=-1
                    self._pedal_inverted = False
                    print(f"    G29 pedals: Windows convention detected (rest={raw_throttle:.2f})")
        
        # Map pedals to 0..1 based on detected convention
        if self._pedal_inverted:
            # Linux: -1 (released) to +1 (pressed)
            self.throttle = (raw_throttle + 1.0) / 2.0
            self.brake = (raw_brake + 1.0) / 2.0
        else:
            # Windows: +1 (released) to -1 (pressed)
            self.throttle = (1.0 - raw_throttle) / 2.0
            self.brake = (1.0 - raw_brake) / 2.0

        # Clamp to valid range
        self.steering = max(-1.0, min(1.0, self.steering))
        self.throttle = max(0.0, min(1.0, self.throttle))
        self.brake = max(0.0, min(1.0, self.brake))

        # Diagnostic: print raw + mapped values for first 60 frames (~2s)
        self._pedal_detect_frames += 0  # reuse counter safely
        if hasattr(self, '_diag_count'):
            self._diag_count += 1
        else:
            self._diag_count = 0
        if self._diag_count < 60 and self._diag_count % 10 == 0:
            inv_tag = "LINUX" if self._pedal_inverted else "WIN"
            print(f"    [G29 diag #{self._diag_count}] raw_thr={raw_throttle:+.3f} raw_brk={raw_brake:+.3f}"
                  f" -> thr={self.throttle:.3f} brk={self.brake:.3f} steer={self.steering:+.3f} [{inv_tag}]")
    
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
