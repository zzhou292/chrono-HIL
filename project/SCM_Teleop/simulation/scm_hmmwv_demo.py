#!/usr/bin/env python3
"""
SCM HMMWV MPC Demo  
==================

Full comparison of Dallas et al. MPC formulation with:
1. Linear (Pacejka-like) tire model - baseline
2. Neural Network tire model - terrain-adaptive

Tests on PyChrono HMMWV with SCM deformable terrain.

This demonstrates the key insight from Dallas et al.: 
Using a learned terramechanics model in the MPC prediction 
loop enables accurate trajectory tracking on deformable terrain.
"""

import numpy as np
import argparse
from pathlib import Path
import time
import sys
import yaml
import threading
import multiprocessing as mp
from queue import Queue, Empty
from dataclasses import dataclass
from typing import Optional, Tuple

# Add parent directory
sys.path.append(str(Path(__file__).parent))

# Import Chrono
import pychrono as chrono
import pychrono.vehicle as veh

# Import Dallas MPC and parameter consistency (vehicle params, training range, terrain presets)
from mpc_solver import DallasMPC, NNCasADi
from param_consistency import (
    get_vehicle_params_for_demo,
    check_terrain_in_training_range,
    get_static_fz_per_wheel,
    TERRAIN_PRESETS,
    EXCITATION_DEFAULTS,
    HMMWV_VEHICLE_PARAMS,
    TRAINING_RANGES_V6,
)


# =============================================================================
# G29 Steering Wheel Interface (pygame/SDL)
# =============================================================================

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
    
    def __init__(self, joystick_index: int = None, enable_force_feedback: bool = True):
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
        
        # Throttle/Brake: pygame normalizes to -1..1, but pedals rest at 1 (released)
        # and go to -1 (pressed). Convert to 0..1 range.
        self.throttle = (1.0 - raw_throttle) / 2.0
        self.brake = (1.0 - raw_brake) / 2.0
        
        # Clamp to valid range
        self.steering = max(-1.0, min(1.0, self.steering))
        self.throttle = max(0.0, min(1.0, self.throttle))
        self.brake = max(0.0, min(1.0, self.brake))
    
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


# =============================================================================
# Perlin Noise for Terrain Heightmaps
# =============================================================================

class PerlinNoise:
    """
    Perlin noise generator for procedural terrain heightmaps.
    
    Port of the C++ implementation from proj_HIL_scm_teleop.cpp.
    """
    
    def __init__(self, seed: int = 0):
        import random
        rng = random.Random(seed)
        self.p = list(range(256))
        rng.shuffle(self.p)
        self.p = self.p + self.p  # Double for wrap-around
    
    def _fade(self, t: float) -> float:
        return t * t * t * (t * (t * 6 - 15) + 10)
    
    def _lerp(self, t: float, a: float, b: float) -> float:
        return a + t * (b - a)
    
    def _grad(self, h: int, x: float, y: float) -> float:
        h = h & 15
        u = x if h < 8 else y
        v = y if h < 4 else (x if (h == 12 or h == 14) else 0)
        return (u if (h & 1) == 0 else -u) + (v if (h & 2) == 0 else -v)
    
    def noise(self, x: float, y: float) -> float:
        """Generate 2D Perlin noise for coordinates (x, y)."""
        import math
        X = int(math.floor(x)) & 255
        Y = int(math.floor(y)) & 255
        x -= math.floor(x)
        y -= math.floor(y)
        u = self._fade(x)
        v = self._fade(y)
        A = self.p[X] + Y
        B = self.p[X + 1] + Y
        return self._lerp(v, 
                         self._lerp(u, self._grad(self.p[A], x, y), 
                                       self._grad(self.p[B], x - 1, y)),
                         self._lerp(u, self._grad(self.p[A + 1], x, y - 1), 
                                       self._grad(self.p[B + 1], x - 1, y - 1)))
    
    def octave_noise(self, x: float, y: float, octaves: int, 
                     persistence: float = 0.5) -> float:
        """Generate multi-octave noise for smoother terrain."""
        total = 0.0
        frequency = 1.0
        amplitude = 1.0
        max_value = 0.0
        for _ in range(octaves):
            total += self.noise(x * frequency, y * frequency) * amplitude
            max_value += amplitude
            amplitude *= persistence
            frequency *= 2
        return total / max_value


def generate_heightmap_bmp(filename: str, width: int, height: int,
                           amplitude: float = 0.3, octaves: int = 4,
                           frequency: float = 0.05, seed: int = 12345,
                           max_slope: float = 0.3) -> str:
    """
    Generate a BMP heightmap file using Perlin noise.
    
    Args:
        filename: Path to save BMP file
        width: Image width in pixels
        height: Image height in pixels
        amplitude: Height amplitude (used for scaling in terrain.Initialize)
        octaves: Number of noise octaves (more = more detail, slower)
        frequency: Spatial frequency (lower = larger features)
        seed: Random seed for reproducibility
        max_slope: Maximum terrain slope (rise/run). 0.3 = 30% grade, ~17°
        
    Returns:
        Path to generated BMP file
    """
    import struct
    
    perlin = PerlinNoise(seed)
    
    # BMP row padding (rows must be multiple of 4 bytes)
    row_size = ((width * 3 + 3) // 4) * 4
    image_size = row_size * height
    file_size = 54 + image_size
    
    with open(filename, 'wb') as f:
        # BMP header (54 bytes)
        header = bytearray(54)
        header[0:2] = b'BM'                          # Signature
        struct.pack_into('<I', header, 2, file_size) # File size
        struct.pack_into('<I', header, 10, 54)       # Data offset
        struct.pack_into('<I', header, 14, 40)       # Info header size
        struct.pack_into('<I', header, 18, width)    # Width
        struct.pack_into('<I', header, 22, height)   # Height
        struct.pack_into('<H', header, 26, 1)        # Planes
        struct.pack_into('<H', header, 28, 24)       # Bits per pixel
        struct.pack_into('<I', header, 34, image_size) # Image size
        f.write(header)
        
        # First pass: generate raw heightmap values
        raw_heights = []
        for y in range(height):
            row_vals = []
            for x in range(width):
                nx = x * frequency
                ny = y * frequency
                noise_val = perlin.octave_noise(nx, ny, octaves)
                noise_val = (noise_val + 1.0) / 2.0  # Normalize to 0-1
                row_vals.append(noise_val)
            raw_heights.append(row_vals)
        
        # Second pass: apply slope limiting (Gaussian blur-like smoothing if needed)
        # Max slope in pixel space: max_slope * (terrain_meters / pixels)
        # Since we have 2 pixels per meter, pixel_spacing = 0.5m
        pixel_spacing = 0.5  # meters per pixel
        max_height_diff = max_slope * pixel_spacing  # max height change per pixel
        max_diff_normalized = max_height_diff / max(amplitude, 0.01)  # in 0-1 space
        
        # Iterative slope limiting (like erosion simulation)
        heights = [row[:] for row in raw_heights]  # copy
        for _ in range(3):  # few iterations to smooth steep slopes
            new_heights = [row[:] for row in heights]
            for y in range(1, height - 1):
                for x in range(1, width - 1):
                    center = heights[y][x]
                    # Check neighbors and limit slope
                    neighbors = [
                        heights[y-1][x], heights[y+1][x],
                        heights[y][x-1], heights[y][x+1]
                    ]
                    for n in neighbors:
                        diff = center - n
                        if abs(diff) > max_diff_normalized:
                            # Pull center toward acceptable slope
                            correction = (abs(diff) - max_diff_normalized) * 0.25
                            if diff > 0:
                                new_heights[y][x] -= correction
                            else:
                                new_heights[y][x] += correction
            heights = new_heights
        
        # Write to BMP
        row = bytearray(row_size)
        for y in range(height):
            for x in range(width):
                gray = int(max(0, min(255, heights[y][x] * 255)))
                row[x * 3 + 0] = gray  # B
                row[x * 3 + 1] = gray  # G
                row[x * 3 + 2] = gray  # R
            f.write(row)
    
    print(f"  Generated heightmap: {filename} ({width}x{height}, "
          f"amp={amplitude:.2f}m, oct={octaves}, freq={frequency:.3f}, "
          f"max_slope={max_slope*100:.0f}%, seed={seed})")
    return filename


# =============================================================================
# Async MPC Worker Thread
# =============================================================================

@dataclass
class MPCRequest:
    """State and reference data for MPC solve"""
    time: float
    z0: np.ndarray
    x_ref: np.ndarray
    y_ref: np.ndarray
    psi_ref: np.ndarray
    v_ref: np.ndarray
    x_goal: float
    y_goal: float
    psi_goal: float
    n_terrain: Optional[float] = None
    sr_meas: float = 0.0  # Measured steering rate (rad/s) from vehicle

@dataclass 
class MPCResult:
    """MPC solution result"""
    time: float
    delta_dot: float
    Jx: float
    solve_time_ms: float
    success: bool

class AsyncMPCWorker:
    """
    Runs MPC in a separate thread to avoid blocking physics loop.
    
    The physics loop sends state updates and receives the latest control.
    MPC solves asynchronously and results are applied when ready.
    """
    
    def __init__(self, mpc, max_queue_size=2):
        self.mpc = mpc
        self.request_queue = Queue(maxsize=max_queue_size)
        self.result_queue = Queue(maxsize=max_queue_size)
        
        self.latest_result: Optional[MPCResult] = None
        self.lock = threading.Lock()
        
        self.running = False
        self.thread: Optional[threading.Thread] = None
        
        # Stats
        self.solve_count = 0
        self.total_solve_time = 0.0
        self.dropped_requests = 0
    
    def start(self):
        """Start the MPC worker thread"""
        # Warmup MPC before starting thread (JIT compile on main thread)
        self._warmup()
        
        self.running = True
        self.thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.thread.start()
    
    def _warmup(self, n_warmup=3):
        """Run warmup solves to trigger IPOPT JIT compilation"""
        N = self.mpc.N
        z0 = np.array([0, 0, 0, 5.0, 0, 0, 0, 0])
        x_ref = np.linspace(0, 20, N+1)
        y_ref = np.zeros(N+1)
        psi_ref = np.zeros(N+1)
        v_ref = 5.0 * np.ones(N+1)
        
        for _ in range(n_warmup):
            _ = self.mpc.solve(z0, x_ref, y_ref, psi_ref, v_ref, x_ref[-1], 0, 0)
    
    def stop(self):
        """Stop the worker thread"""
        self.running = False
        # Send sentinel to unblock queue
        try:
            self.request_queue.put_nowait(None)
        except:
            pass
        if self.thread:
            self.thread.join(timeout=1.0)
    
    def submit_request(self, request: MPCRequest) -> bool:
        """
        Submit a new MPC request. Non-blocking.
        Returns True if queued, False if queue is full (request dropped).
        """
        try:
            # Clear old requests - we only care about the latest state
            while not self.request_queue.empty():
                try:
                    self.request_queue.get_nowait()
                    self.dropped_requests += 1
                except Empty:
                    break
            self.request_queue.put_nowait(request)
            return True
        except:
            self.dropped_requests += 1
            return False
    
    def get_latest_result(self) -> Optional[MPCResult]:
        """
        Get the most recent MPC result. Non-blocking.
        Returns None if no result available yet.
        """
        # Drain result queue to get latest
        while not self.result_queue.empty():
            try:
                result = self.result_queue.get_nowait()
                with self.lock:
                    self.latest_result = result
            except Empty:
                break
        
        with self.lock:
            return self.latest_result
    
    def _worker_loop(self):
        """Main worker loop - runs in separate thread"""
        while self.running:
            try:
                request = self.request_queue.get(timeout=0.1)
                
                if request is None:  # Sentinel
                    break
                
                # Solve MPC
                t0 = time.perf_counter()
                delta_dot, Jx, Z_opt, U_opt = self.mpc.solve(
                    request.z0, 
                    request.x_ref, request.y_ref, request.psi_ref, request.v_ref,
                    request.x_goal, request.y_goal, request.psi_goal,
                    n_terrain=request.n_terrain,
                    sr_meas=request.sr_meas
                )
                solve_time = (time.perf_counter() - t0) * 1000  # ms
                
                success = Z_opt is not None
                if not success:
                    delta_dot, Jx = 0.0, 0.0
                
                result = MPCResult(
                    time=request.time,
                    delta_dot=delta_dot,
                    Jx=Jx,
                    solve_time_ms=solve_time,
                    success=success
                )
                
                # Update stats
                self.solve_count += 1
                self.total_solve_time += solve_time
                
                # Post result
                try:
                    self.result_queue.put_nowait(result)
                except:
                    pass  # Result queue full, old result will be used
                
            except Empty:
                continue
            except Exception as e:
                print(f"[MPC Worker] Error: {e}")
    
    def get_stats(self) -> dict:
        """Get performance statistics"""
        avg_solve = self.total_solve_time / self.solve_count if self.solve_count > 0 else 0
        return {
            'solve_count': self.solve_count,
            'avg_solve_ms': avg_solve,
            'dropped_requests': self.dropped_requests,
        }


# =============================================================================
# Multiprocessing MPC Worker (bypasses GIL for true parallelism)
# =============================================================================

def _mpc_worker_process(request_queue: mp.Queue, result_queue: mp.Queue,
                        mpc_config: dict, stop_event: mp.Event, ready_event: mp.Event):
    """
    MPC worker process - runs in separate process to bypass Python GIL.
    
    Must rebuild MPC solver here since CasADi objects can't be pickled.
    """
    # Import MPC in child process
    from mpc_solver import DallasMPC, NNCasADi
    import numpy as np
    import time
    
    # Rebuild MPC solver
    nn_casadi = None
    if mpc_config.get('nn_model_path'):
        nn_casadi = NNCasADi(
            model_path=mpc_config['nn_model_path'],
            scaler_path=mpc_config['scaler_path'],
            terrain_params=mpc_config.get('terrain_params', {})
        )
    
    mpc = DallasMPC(
        nn_casadi=nn_casadi,
        params=mpc_config['vehicle_params'],
        dt=mpc_config['dt'],
        N=mpc_config['N'],
        nn_scale=mpc_config.get('nn_scale', 1.0),
        nn_sign=mpc_config.get('nn_sign', 1),
        kappa_mode=mpc_config.get('kappa_mode', 'approx'),
        lateral_load_transfer=mpc_config.get('lateral_load_transfer', True),
    )
    
    # Warmup solve to trigger Ipopt JIT compilation (first solve is slow)
    # Run multiple warmup solves - first solve triggers JIT, subsequent solves verify speed
    N = mpc_config['N']
    z0_warmup = np.array([0, 0, 0, 5.0, 0, 0, 0, 0])
    x_ref = np.linspace(0, 20, N+1)
    y_ref = np.zeros(N+1)
    psi_ref = np.zeros(N+1)
    v_ref = 5.0 * np.ones(N+1)
    
    for _ in range(3):  # 3 warmup solves
        _ = mpc.solve(z0_warmup, x_ref, y_ref, psi_ref, v_ref, x_ref[-1], 0, 0)
    
    # Signal ready
    ready_event.set()
    
    # Stats
    solve_count = 0
    total_solve_time = 0.0
    
    while not stop_event.is_set():
        try:
            # Get request - block indefinitely (up to 0.5s timeout)
            try:
                request = request_queue.get(timeout=0.5)
            except:
                # Queue empty - just wait
                continue
            
            if request is None:  # Sentinel to stop
                break
            
            # Solve MPC
            t0 = time.perf_counter()
            delta_dot, Jx, Z_opt, U_opt = mpc.solve(
                request['z0'],
                request['x_ref'], request['y_ref'], request['psi_ref'], request['v_ref'],
                request['x_goal'], request['y_goal'], request['psi_goal'],
                n_terrain=request.get('n_terrain'),
                sr_meas=request.get('sr_meas', 0.0)
            )
            solve_time = (time.perf_counter() - t0) * 1000  # ms
            
            success = Z_opt is not None
            if not success:
                delta_dot, Jx = 0.0, 0.0
            
            # Send result
            result = {
                'time': request['time'],
                'delta_dot': float(delta_dot),
                'Jx': float(Jx),
                'solve_time_ms': solve_time,
                'success': success,
            }
            
            # Blocking put - this should always work
            result_queue.put(result)
            
            solve_count += 1
            total_solve_time += solve_time
            
        except Exception as e:
            print(f"[MP MPC Worker] Error: {e}")
    
    # Send final stats
    try:
        result_queue.put({'_stats': {
            'solve_count': solve_count,
            'avg_solve_ms': total_solve_time / solve_count if solve_count > 0 else 0,
        }})
    except:
        pass


class MultiprocessMPCWorker:
    """
    Runs MPC in a separate PROCESS to bypass Python GIL.
    
    This provides true parallelism - MPC solving does not block
    physics or rendering at all.
    """
    
    def __init__(self, mpc_config: dict):
        """
        Args:
            mpc_config: Dict with keys:
                - vehicle_params: Vehicle parameter dict
                - dt: MPC timestep
                - N: Horizon length
                - nn_model_path: Optional path to NN model
                - scaler_path: Optional path to scalers
                - nn_scale, nn_sign, kappa_mode: MPC options
        """
        self.mpc_config = mpc_config
        
        # Use 'fork' context on Linux for simpler process creation
        # (spawn has issues with pickling functions from __main__)
        self.ctx = mp.get_context('fork')
        self.request_queue = self.ctx.Queue(maxsize=2)
        self.result_queue = self.ctx.Queue(maxsize=4)
        self.stop_event = self.ctx.Event()
        self.ready_event = self.ctx.Event()
        
        self.process: Optional[mp.Process] = None
        self.latest_result: Optional[dict] = None
        self.dropped_requests = 0
        self._final_stats = None
    
    def start(self, timeout=30.0):
        """Start the MPC worker process and wait for it to be ready"""
        self.process = self.ctx.Process(
            target=_mpc_worker_process,
            args=(self.request_queue, self.result_queue, self.mpc_config, 
                  self.stop_event, self.ready_event),
            daemon=True
        )
        self.process.start()
        
        # Wait for the process to be ready (MPC built + warmup solve done)
        print("  [MULTIPROCESS] Waiting for MPC process to initialize...")
        if not self.ready_event.wait(timeout=timeout):
            print("  [WARNING] MPC process didn't become ready in time")
        else:
            print("  [MULTIPROCESS] MPC process ready!")
    
    def stop(self):
        """Stop the worker process"""
        self.stop_event.set()
        try:
            self.request_queue.put_nowait(None)
        except:
            pass
        if self.process:
            self.process.join(timeout=2.0)
            if self.process.is_alive():
                self.process.terminate()
        
        # Poll for final stats from worker (sent just before exit)
        import time
        for _ in range(50):  # Try for up to 0.5s
            self.get_latest_result()
            if self._final_stats is not None:
                break
            time.sleep(0.01)
    
    def submit_request(self, request: MPCRequest) -> bool:
        """
        Submit MPC request. Non-blocking.
        Returns True if queued, False if dropped.
        """
        try:
            # Convert dataclass to dict for pickling (numpy arrays are picklable)
            req_dict = {
                'time': request.time,
                'z0': np.asarray(request.z0).copy(),
                'x_ref': np.asarray(request.x_ref).copy(),
                'y_ref': np.asarray(request.y_ref).copy(),
                'psi_ref': np.asarray(request.psi_ref).copy(),
                'v_ref': np.asarray(request.v_ref).copy(),
                'x_goal': float(request.x_goal),
                'y_goal': float(request.y_goal),
                'psi_goal': float(request.psi_goal),
                'n_terrain': request.n_terrain,
                'sr_meas': float(request.sr_meas),
            }
            # Use blocking put with short timeout
            self.request_queue.put(req_dict, timeout=0.001)
            return True
        except:
            self.dropped_requests += 1
            return False
    
    def get_latest_result(self) -> Optional[MPCResult]:
        """Get most recent MPC result. Non-blocking."""
        # Drain queue for latest
        result_found = False
        while True:
            try:
                result = self.result_queue.get_nowait()
                if '_stats' in result:
                    self._final_stats = result['_stats']
                    continue
                self.latest_result = result
                result_found = True
            except:
                break
        
        if self.latest_result is None:
            return None
        
        return MPCResult(
            time=self.latest_result['time'],
            delta_dot=self.latest_result['delta_dot'],
            Jx=self.latest_result['Jx'],
            solve_time_ms=self.latest_result['solve_time_ms'],
            success=self.latest_result['success'],
        )
    
    def get_stats(self) -> dict:
        """Get performance statistics"""
        # Drain any remaining results for stats
        self.get_latest_result()
        
        if self._final_stats:
            stats = self._final_stats.copy()
        else:
            stats = {'solve_count': 0, 'avg_solve_ms': 0}
        stats['dropped_requests'] = self.dropped_requests
        return stats


# =============================================================================
# PyChrono MPC Driver for Dallas Formulation
# =============================================================================

class DallasMPCDriver(veh.ChDriver):
    """
    PyChrono driver that uses Dallas MPC for vehicle control.
    
    Converts MPC outputs (steering rate, jerk) to Chrono inputs
    (steering, throttle, braking).
    
    Supports async mode where MPC runs in a separate thread or process.
    """
    
    # Default measurement noise standard deviations (set to None to disable)
    # Based on typical GPS/IMU sensor noise characteristics
    DEFAULT_MEAS_NOISE = {
        'x': 1.2,       # Global X position (m)
        'y': 1.2,       # Global Y position (m)
        'psi': 0.0175,  # Yaw angle (rad) ~1 degree
        'u': 0.25,      # Longitudinal speed (m/s)
        'v': 0.25,      # Lateral speed (m/s)
        'omega': 0.0175 # Yaw rate (rad/s)
    }
    
    def __init__(self, vehicle, mpc, dt_mpc=0.1, path_func=None, debug=False, ukf=None,
                 async_mpc=False, multiprocess_mpc=False, mpc_config=None,
                 measurement_noise=None, steer_excite_amp=0.0, steer_excite_freq=0.15,
                 steer_excite_ramp=3.0, step_size=3e-3):
        """
        Args:
            vehicle: PyChrono WheeledVehicle
            mpc: DallasMPC instance (not used if multiprocess_mpc=True)
            dt_mpc: MPC control interval
            path_func: Function(t) -> (x_ref, y_ref, psi_ref, v_ref) for N+1 points
            debug: Print debug info about tire forces
            ukf: Optional TerrainUKF for online sinkage exponent estimation
            async_mpc: If True, run MPC in a separate thread (non-blocking)
            multiprocess_mpc: If True, run MPC in separate PROCESS (bypasses GIL)
            mpc_config: Dict of MPC config for multiprocess mode
            measurement_noise: Dict with noise std devs for state measurements, or True for defaults
            steer_excite_amp: Sinusoidal steering excitation amplitude (rad) for UKF observability
            steer_excite_freq: Steering excitation frequency (Hz). Default 0.15 Hz (6.7s period)
            steer_excite_ramp: Ramp-up time (seconds) for smooth excitation onset
        """
        super().__init__(vehicle.GetVehicle())
        
        self.vehicle = vehicle
        self.mpc = mpc
        self.dt_mpc = dt_mpc
        self.step_size = step_size  # Physics timestep for rate integration
        self.path_func = path_func
        self.debug = debug
        self.debug_counter = 0
        self.ukf = ukf
        
        # UKF update rate — Dallas paper: sensors updated every 24ms
        self.dt_ukf = 0.024
        self.last_ukf_time = -self.dt_ukf
        self.n_terrain_est = None  # Latest UKF estimate (shared between UKF and MPC)
        
        # Async MPC mode (threading)
        self.async_mpc = async_mpc and not multiprocess_mpc
        self.mpc_worker: Optional[AsyncMPCWorker] = None
        
        # Multiprocess MPC mode (true parallelism)
        self.multiprocess_mpc = multiprocess_mpc
        self.mp_worker: Optional[MultiprocessMPCWorker] = None
        
        if self.multiprocess_mpc and mpc_config is not None:
            self.mp_worker = MultiprocessMPCWorker(mpc_config)
            self.mp_worker.start()
            print("  [MULTIPROCESS] MPC running in separate process")
        elif self.async_mpc:
            self.mpc_worker = AsyncMPCWorker(mpc)
            self.mpc_worker.start()
        
        self.last_mpc_time = -dt_mpc
        self.last_mpc_result_time = -1.0  # Track when we last got a result
        
        # Control states (from MPC state vector)
        self.steering_angle = 0.0  # δ
        self.acceleration = 0.0    # ax
        
        # Rate commands from MPC (used for integration between solves)
        self.delta_dot = 0.0
        self.Jx = 0.0
        
        # Current outputs
        self.m_steering = 0.0
        self.m_throttle = 0.0
        self.m_braking = 0.0
        
        # Gains for converting to vehicle inputs
        self.steering_gain = 1.0 / self.mpc.delta_max  # Normalize to [-1, 1]
        self.throttle_gain = 1.0
        self.brake_gain = 0.6
        self.v_target = None  # set by path_func for speed feedback
        
        # Measurement noise injection
        if measurement_noise is True:
            self.measurement_noise = self.DEFAULT_MEAS_NOISE.copy()
        elif measurement_noise:
            self.measurement_noise = measurement_noise
        else:
            self.measurement_noise = None
        self.speed_err_integral = 0.0
        
        # Steering excitation for terrain observability (Dallas paper uses sinusoidal steering)
        self.steer_excite_amp = steer_excite_amp
        self.steer_excite_freq = steer_excite_freq
        self.steer_excite_ramp = steer_excite_ramp  # smooth half-cosine ramp-up duration
        
        # State tracking
        self.state_history = []
        self.control_history = []
        
        # Timing stats for control loop profiling
        self.mpc_solve_times = []  # List of MPC solve durations (seconds)
        self.total_mpc_calls = 0
        self._last_timing_print = 0.0
        
        # Warmup MPC solver (sync mode only - multiprocess has its own warmup)
        if not self.multiprocess_mpc and not self.async_mpc and mpc is not None:
            self._warmup_solver()
    
    def _warmup_solver(self, n_warmup=3):
        """
        Run warmup solves to trigger IPOPT JIT compilation.
        First solve is typically 10-50x slower due to compilation.
        """
        import time as time_module
        print("  [WARMUP] Running MPC warmup solves...", end='', flush=True)
        
        N = self.mpc.N
        z0 = np.array([0, 0, 0, 5.0, 0, 0, 0, 0])
        x_ref = np.linspace(0, 20, N+1)
        y_ref = np.zeros(N+1)
        psi_ref = np.zeros(N+1)
        v_ref = 5.0 * np.ones(N+1)
        
        warmup_times = []
        for i in range(n_warmup):
            t0 = time_module.perf_counter()
            _ = self.mpc.solve(z0, x_ref, y_ref, psi_ref, v_ref, x_ref[-1], 0, 0)
            t1 = time_module.perf_counter()
            warmup_times.append((t1 - t0) * 1000)
            print('.', end='', flush=True)
        
        print(f" done! ({warmup_times[0]:.0f}ms -> {warmup_times[-1]:.0f}ms)")
    
    def _compute_excitation(self, time):
        """Compute smooth ramped sinusoidal steering excitation.
        
        Returns (delta, delta_dot) or (None, None) if excitation is disabled.
        Uses a half-cosine ramp-up envelope so the maneuver starts smoothly
        instead of jerking the wheel from 0 to full amplitude.
        """
        if self.steer_excite_amp <= 0:
            return None, None
        
        A = self.steer_excite_amp
        f = self.steer_excite_freq
        T_ramp = self.steer_excite_ramp
        w = 2.0 * np.pi * f
        
        # Smooth half-cosine envelope: 0 -> 1 over T_ramp seconds
        if T_ramp > 0 and time < T_ramp:
            env = 0.5 * (1.0 - np.cos(np.pi * time / T_ramp))
            env_dot = 0.5 * np.pi / T_ramp * np.sin(np.pi * time / T_ramp)
        else:
            env = 1.0
            env_dot = 0.0
        
        # delta(t) = env(t) * A * sin(w*t)
        sin_wt = np.sin(w * time)
        cos_wt = np.cos(w * time)
        delta = env * A * sin_wt
        # Product rule: d/dt [env * A * sin(wt)] = env_dot * A * sin(wt) + env * A * w * cos(wt)
        delta_dot = env_dot * A * sin_wt + env * A * w * cos_wt
        
        return delta, delta_dot
    
    def shutdown(self):
        """Clean up async/multiprocess worker if running"""
        if self.mpc_worker is not None:
            self.mpc_worker.stop()
        if self.mp_worker is not None:
            self.mp_worker.stop()
    
    def get_mpc_stats(self) -> Optional[dict]:
        """Get async MPC performance stats"""
        if self.mp_worker is not None:
            return self.mp_worker.get_stats()
        if self.mpc_worker is not None:
            return self.mpc_worker.get_stats()
        return None
    
    def Synchronize(self, time):
        """Called by Chrono to update driver inputs"""
        # UKF runs at its own rate (24ms), independent of MPC rate
        if self.ukf is not None and time - self.last_ukf_time >= self.dt_ukf:
            self._run_ukf_update(time)
            self.last_ukf_time = time
        
        if self.multiprocess_mpc:
            # Multiprocess mode: same as async but uses separate process
            self._sync_async_mpc(time, use_multiprocess=True)
        elif self.async_mpc:
            # Async mode: check for new results and submit new requests
            self._sync_async_mpc(time)
        else:
            # Sync mode: run MPC inline (blocking)
            if time - self.last_mpc_time >= self.dt_mpc:
                self._run_mpc_sync(time)
                self.last_mpc_time = time
    
    def _read_vehicle_state(self):
        """Read current vehicle state from Chrono, with optional noise injection.
        
        Returns:
            (x, y, psi, u, v, omega) — front-axle position and body-frame velocities.
            All values include measurement noise if enabled.
        """
        chassis = self.vehicle.GetChassisBody()
        pos = chassis.GetPos()
        vel = chassis.GetPosDt()
        rot = chassis.GetRot()
        omega_vec = chassis.GetAngVelLocal()
        
        x_cg, y_cg = pos.x, pos.y
        psi = self._quat_to_yaw(rot)
        Lf = self.mpc.Lf
        x = x_cg + Lf * np.cos(psi)
        y = y_cg + Lf * np.sin(psi)
        
        vel_loc = chassis.GetRot().RotateBack(vel)
        u, v = vel_loc.x, vel_loc.y
        omega = omega_vec.z
        
        # Inject measurement noise if enabled
        if self.measurement_noise:
            noise = self.measurement_noise
            x += np.random.normal(0, noise.get('x', 0))
            y += np.random.normal(0, noise.get('y', 0))
            psi += np.random.normal(0, noise.get('psi', 0))
            u += np.random.normal(0, noise.get('u', 0))
            v += np.random.normal(0, noise.get('v', 0))
            omega += np.random.normal(0, noise.get('omega', 0))
        
        return x, y, psi, u, v, omega
    
    def _run_ukf_update(self, time):
        """Run UKF terrain estimation at sensor rate (24ms), independent of MPC."""
        x, y, psi, u, v, omega = self._read_vehicle_state()
        
        exc_delta, exc_delta_dot = self._compute_excitation(time)
        if exc_delta is not None:
            actual_delta = exc_delta
            actual_delta_dot = exc_delta_dot
        else:
            actual_delta = self.steering_angle
            actual_delta_dot = self.delta_dot
        
        self.n_terrain_est = self.ukf.step(
            x_meas=x, y_meas=y, psi_meas=psi,
            u=max(u, 0.1), v=v, omega=omega,
            delta=actual_delta, ax=self.acceleration,
            time=time,
            steering_rate=actual_delta_dot,
        )
        
        if self.debug and int(time * 10) % 20 == 0:
            P_str = f" P_n={self.ukf.P[6,6]:.2e}" if hasattr(self.ukf, 'P') else ""
            print(f"    EST n_est={self.n_terrain_est:.4f}{P_str}")
    
    def _sync_async_mpc(self, time, use_multiprocess=False):
        """Handle async MPC: check results, submit requests, update controls"""
        # Get the appropriate worker
        worker = self.mp_worker if use_multiprocess else self.mpc_worker
        
        # Submit new request at MPC rate FIRST (minimize latency)
        if time - self.last_mpc_time >= self.dt_mpc:
            self._submit_mpc_request(time, use_multiprocess=use_multiprocess)
            self.last_mpc_time = time
        
        # Check for new MPC result (may arrive from previous or current request)
        result = worker.get_latest_result()
        if result is not None and result.time > self.last_mpc_result_time:
            # Got a new result - update rate commands
            self.delta_dot = result.delta_dot
            self.Jx = result.Jx
            self.last_mpc_result_time = result.time
        
        # Always update controls by integrating rate commands
        # This runs at physics rate, not MPC rate
        self._update_controls_from_rates(self.step_size, time)
    
    def _submit_mpc_request(self, time, use_multiprocess=False):
        """Build and submit MPC request to async/multiprocess worker"""
        # Get the appropriate worker
        worker = self.mp_worker if use_multiprocess else self.mpc_worker
        
        if worker is None:
            return
            
        # Get current vehicle state (with noise if enabled)
        x, y, psi, u, v, omega = self._read_vehicle_state()
        
        z0 = np.array([x, y, psi, max(u, 0.5), v, omega, 
                       self.steering_angle, self.acceleration])
        
        # Get reference trajectory
        if self.path_func is not None:
            x_ref, y_ref, psi_ref, v_ref, x_goal, y_goal, psi_goal = self.path_func(
                time, z0, self.mpc.N, self.mpc.dt)
        else:
            N = self.mpc.N
            x_ref = x + u * np.linspace(0, (N+1) * self.mpc.dt, N+1)
            y_ref = np.zeros(N+1)
            psi_ref = np.zeros(N+1)
            v_ref = 5.0 * np.ones(N+1)
            x_goal, y_goal, psi_goal = x_ref[-1], y_ref[-1], 0.0
        
        request = MPCRequest(
            time=time, z0=z0,
            x_ref=x_ref, y_ref=y_ref, psi_ref=psi_ref, v_ref=v_ref,
            x_goal=x_goal, y_goal=y_goal, psi_goal=psi_goal,
            n_terrain=self.n_terrain_est,  # From UKF running at 24ms
            sr_meas=0.0  # Always zero: v6 NN has ~4800 N/rad/s Fy sensitivity to sr;
                        # holding last delta_dot constant over the 2.5s horizon creates
                        # massive phantom lateral forces and a self-amplifying feedback loop.
        )
        worker.submit_request(request)
        
        # Record state
        self.state_history.append({
            'time': time, 'x': x, 'y': y, 'psi': psi,
            'u': u, 'v': v, 'omega': omega,
            'delta': self.steering_angle, 'ax': self.acceleration,
            'steering': self.m_steering, 'throttle': self.m_throttle, 'brake': self.m_braking
        })
    
    def _update_controls_from_rates(self, dt, time):
        """Integrate rate commands to get control values"""
        # Integrate steering rate and jerk
        self.steering_angle += self.delta_dot * dt
        self.steering_angle = np.clip(self.steering_angle, -self.mpc.delta_max, self.mpc.delta_max)
        
        self.acceleration += self.Jx * dt
        self.acceleration = np.clip(self.acceleration, self.mpc.ax_min, self.mpc.ax_max)
        
        # Convert to vehicle inputs - with optional steering excitation
        exc_delta, _ = self._compute_excitation(time)
        if exc_delta is not None:
            # Smooth sinusoidal steering + PI speed control
            self.m_steering = np.clip(exc_delta * self.steering_gain, -1.0, 1.0)

            # PI speed control instead of constant throttle
            u = 5.0
            if self.state_history:
                u = self.state_history[-1].get('u', 5.0)
            speed_err = (self.v_target or 5.0) - u
            if speed_err > 0:
                self.speed_err_integral += speed_err * dt
                self.speed_err_integral = min(self.speed_err_integral, 3.0)
            else:
                self.speed_err_integral = max(self.speed_err_integral - 0.5 * dt, 0.0)
            self.m_throttle = np.clip(0.3 + 0.15 * speed_err + 0.05 * self.speed_err_integral, 0.0, 1.0)
            self.m_braking = 0.0
        else:
            steer_total = self.steering_angle
            self.m_steering = np.clip(steer_total * self.steering_gain, -1.0, 1.0)
        
            # Get speed for feedback (approximation - uses last state)
            u = 5.0  # Default
            if self.state_history:
                u = self.state_history[-1].get('u', 5.0)
        
            speed_boost = 0.0
            if self.v_target is not None:
                speed_err = self.v_target - u
                if speed_err > 0:
                    self.speed_err_integral += speed_err * dt
                    self.speed_err_integral = min(self.speed_err_integral, 3.0)
                    speed_boost = 0.15 * speed_err + 0.05 * self.speed_err_integral
                else:
                    self.speed_err_integral = max(self.speed_err_integral - 0.5 * dt, 0.0)
        
            if self.acceleration >= 0:
                base_throttle = self.acceleration / self.mpc.ax_max * self.throttle_gain + 0.3
                self.m_throttle = min(base_throttle + speed_boost, 1.0)
                self.m_braking = 0.0
            else:
                self.m_throttle = 0.0
                self.m_braking = min(-self.acceleration / abs(self.mpc.ax_min) * self.brake_gain, 1.0)
    
    def _run_mpc_sync(self, time):
        """Execute MPC and update control states"""
        # Get current vehicle state
        chassis = self.vehicle.GetChassisBody()
        pos = chassis.GetPos()
        vel = chassis.GetPosDt()
        rot = chassis.GetRot()
        omega_vec = chassis.GetAngVelLocal()
        
        # Extract CG states
        x_cg = pos.x
        y_cg = pos.y
        psi = self._quat_to_yaw(rot)
        
        # Transform CG to front axle position for path tracking
        # MPC dynamics use front-axle kinematics (standard for path following)
        Lf = self.mpc.Lf
        x = x_cg + Lf * np.cos(psi)
        y = y_cg + Lf * np.sin(psi)
        
        # Velocity in vehicle frame
        vel_loc = chassis.GetRot().RotateBack(vel)
        u = vel_loc.x  # Longitudinal
        v = vel_loc.y  # Lateral
        omega = omega_vec.z  # Yaw rate
        
        # 8-state vector for Dallas MPC (front axle position)
        z0 = np.array([
            x, y, psi, 
            max(u, 0.5),  # Minimum speed
            v, omega,
            self.steering_angle,
            self.acceleration
        ])
        
        # Debug: Print slip info every 10 iterations (~1s)
        if self.debug:
            self.debug_counter += 1
            if self.debug_counter % 10 == 0:
                # Compute slip angles same way as MPC
                u_safe = max(abs(u), 0.5)
                Lf, Lr = self.mpc.Lf, self.mpc.Lr
                alpha_f = self.steering_angle - np.arctan2(v + Lf * omega, u_safe)
                alpha_r = -np.arctan2(v - Lr * omega, u_safe)
                
                print(f"\n=== Debug t={time:.1f}s ===")
                print(f"  Position: ({x:.1f}, {y:.1f}), psi={np.degrees(psi):.1f}°")
                print(f"  Velocity: u={u:.2f}, v={v:.2f}, omega={omega:.3f}")
                print(f"  Steering: {np.degrees(self.steering_angle):.1f}°")
                print(f"  Slip angles: front={np.degrees(alpha_f):.1f}°, rear={np.degrees(alpha_r):.1f}°")
                
                # If NN is available, compare forces
                if self.mpc.use_nn and self.mpc.nn_casadi is not None:
                    M = self.mpc.M
                    Fz_f = M * 9.81 * Lr / (Lf + Lr) / 2  # Per wheel
                    Fz_r = M * 9.81 * Lf / (Lf + Lr) / 2
                    kappa = np.clip(self.acceleration / (0.4 * 9.81), -0.3, 0.3)
                    
                    _, Fy_f_nn = self.mpc.nn_casadi.predict_numeric(alpha_f, Fz_f, u_safe, kappa)
                    _, Fy_r_nn = self.mpc.nn_casadi.predict_numeric(alpha_r, Fz_r, u_safe, kappa)
                    
                    # Expected axle forces (×2 for both wheels, negated)
                    Fyf_axle = -2 * Fy_f_nn
                    Fyr_axle = -2 * Fy_r_nn
                    
                    # Linear model comparison
                    Cf, Cr = self.mpc.Cf, self.mpc.Cr
                    Fyf_linear = Cf * alpha_f
                    Fyr_linear = Cr * alpha_r
                    
                    print(f"  NN forces (axle): Fyf={Fyf_axle:.0f}N, Fyr={Fyr_axle:.0f}N")
                    print(f"  Linear forces:    Fyf={Fyf_linear:.0f}N, Fyr={Fyr_linear:.0f}N")
                    print(f"  Ratio NN/Linear:  Fyf={Fyf_axle/Fyf_linear:.2f}, Fyr={Fyr_axle/Fyr_linear:.2f}" 
                          if abs(Fyf_linear) > 100 and abs(Fyr_linear) > 100 else "")
        
        # Get reference trajectory
        if self.path_func is not None:
            x_ref, y_ref, psi_ref, v_ref, x_goal, y_goal, psi_goal = self.path_func(
                time, z0, self.mpc.N, self.mpc.dt
            )
        else:
            # Default: straight line
            N = self.mpc.N
            x_ref = x + u * np.linspace(0, (N+1) * self.mpc.dt, N+1)
            y_ref = np.zeros(N+1)
            psi_ref = np.zeros(N+1)
            v_ref = 5.0 * np.ones(N+1)
            x_goal, y_goal, psi_goal = x_ref[-1], y_ref[-1], 0.0
        
        # Debug: print reference at first few iterations
        if self.debug and time < 0.5:
            print(f"\\n=== MPC Debug t={time:.2f}s ===")
            print(f"  z0: x={z0[0]:.2f}, y={z0[1]:.2f}, psi={np.degrees(z0[2]):.1f}°, u={z0[3]:.2f}")
            print(f"  x_ref[0..5]: {x_ref[:6]}")
            print(f"  y_ref[0..5]: {y_ref[:6]}")
            print(f"  psi_ref[0..5]: {np.degrees(psi_ref[:6])}")
            print(f"  goal: ({x_goal:.2f}, {y_goal:.2f}), psi={np.degrees(psi_goal):.1f}°")
        
        # UKF terrain estimate (updated at 24ms rate in _run_ukf_update)
        
        # Solve MPC (with timing)
        import time as time_module
        t_solve_start = time_module.perf_counter()
        
        delta_dot, Jx, Z_opt, U_opt = self.mpc.solve(
            z0, x_ref, y_ref, psi_ref, v_ref,
            x_goal, y_goal, psi_goal, n_terrain=self.n_terrain_est,
            sr_meas=0.0  # Zero: see comment in _submit_mpc_request re: v6 Fy sensitivity
        )
        
        t_solve_end = time_module.perf_counter()
        solve_duration = t_solve_end - t_solve_start
        self.mpc_solve_times.append(solve_duration)
        self.total_mpc_calls += 1
        
        # Print timing stats periodically (every 2 seconds of sim time)
        if self.debug and time - self._last_timing_print >= 2.0:
            self._last_timing_print = time
            recent_times = self.mpc_solve_times[-20:]  # Last 20 solves
            if recent_times:
                mean_ms = np.mean(recent_times) * 1000
                max_ms = np.max(recent_times) * 1000
                hz = 1.0 / np.mean(recent_times) if np.mean(recent_times) > 0 else 0
                model_type = 'NN' if self.mpc.use_nn else 'LINEAR'
                print(f"  [TIMING {model_type}] MPC solve: mean={mean_ms:.1f}ms, max={max_ms:.1f}ms, rate={hz:.0f}Hz")
        
        if Z_opt is None:
            # Fallback if solver fails
            delta_dot, Jx = 0.0, 0.0
        
        # Debug: print MPC output at first few iterations
        if self.debug and time < 0.5:
            print(f"  MPC output: delta_dot={np.degrees(delta_dot):.1f}°/s, Jx={Jx:.2f} m/s³")
            if Z_opt is not None:
                print(f"  Predicted trajectory:")
                for k in [0, 5, 10, 15, 20]:
                    if k < Z_opt.shape[1]:
                        print(f"    k={k}: x={Z_opt[0,k]:.2f}, y={Z_opt[1,k]:.2f}, psi={np.degrees(Z_opt[2,k]):.1f}°, delta={np.degrees(Z_opt[6,k]):.1f}°")
        
        # Integrate rate/jerk to get angle/acceleration
        self.steering_angle += delta_dot * self.dt_mpc
        self.steering_angle = np.clip(self.steering_angle, -self.mpc.delta_max, self.mpc.delta_max)
        
        self.acceleration += Jx * self.dt_mpc
        self.acceleration = np.clip(self.acceleration, self.mpc.ax_min, self.mpc.ax_max)
        
        # Convert to vehicle inputs - with optional steering excitation
        exc_delta, _ = self._compute_excitation(time)
        if exc_delta is not None:
            # Smooth sinusoidal steering + PI speed control
            self.m_steering = np.clip(exc_delta * self.steering_gain, -1.0, 1.0)

            # PI speed control instead of constant throttle
            speed_err = (self.v_target or 5.0) - u
            if speed_err > 0:
                self.speed_err_integral += speed_err * self.dt_mpc
                self.speed_err_integral = min(self.speed_err_integral, 3.0)
            else:
                self.speed_err_integral = max(self.speed_err_integral - 0.5 * self.dt_mpc, 0.0)
            self.m_throttle = np.clip(0.3 + 0.15 * speed_err + 0.05 * self.speed_err_integral, 0.0, 1.0)
            self.m_braking = 0.0
        else:
            steer_total = self.steering_angle
            self.m_steering = steer_total * self.steering_gain
            self.m_steering = np.clip(self.m_steering, -1.0, 1.0)
        
            speed_boost = 0.0
            if self.v_target is not None:
                speed_err = self.v_target - u
                if speed_err > 0:
                    self.speed_err_integral += speed_err * self.dt_mpc
                    self.speed_err_integral = min(self.speed_err_integral, 3.0)
                    speed_boost = 0.15 * speed_err + 0.05 * self.speed_err_integral
                else:
                    self.speed_err_integral = max(self.speed_err_integral - 0.5 * self.dt_mpc, 0.0)
        
            if self.acceleration >= 0:
                base_throttle = self.acceleration / self.mpc.ax_max * self.throttle_gain + 0.3
                self.m_throttle = min(base_throttle + speed_boost, 1.0)
                self.m_braking = 0.0
            else:
                self.m_throttle = 0.0
                self.m_braking = min(-self.acceleration / abs(self.mpc.ax_min) * self.brake_gain, 1.0)
        
        # Record state
        self.state_history.append({
            'time': time,
            'x': x, 'y': y, 'psi': psi,
            'u': u, 'v': v, 'omega': omega,
            'delta': self.steering_angle,
            'ax': self.acceleration,
            'steering': self.m_steering,
            'throttle': self.m_throttle,
            'brake': self.m_braking
        })
    
    def _quat_to_yaw(self, quat):
        """Extract yaw angle from quaternion"""
        e0, e1, e2, e3 = quat.e0, quat.e1, quat.e2, quat.e3
        yaw = np.arctan2(2 * (e0 * e3 + e1 * e2), 1 - 2 * (e2**2 + e3**2))
        return yaw
    
    def GetSteering(self):
        return self.m_steering
    
    def GetThrottle(self):
        return self.m_throttle
    
    def GetBraking(self):
        return self.m_braking


# =============================================================================
# Path Generator
# =============================================================================

def find_closest_point_on_sinusoid(x_veh, y_veh, amplitude, wavelength, search_range=10.0, n_samples=100,
                                    x_offset=0.0):
    """
    Find the closest point on a sinusoidal path y = amp*sin(2*pi*(x - x_offset)/wavelength) 
    to the vehicle's current position.  For x < x_offset the path is y=0 (lead-in).
    
    Returns:
        s_closest: The x-coordinate of the closest point on the path
        y_path: The y-coordinate of the closest point
        dist: The distance to the closest point
    """
    # Search window around vehicle's x position
    x_min = x_veh - search_range
    x_max = x_veh + search_range
    
    x_samples = np.linspace(x_min, x_max, n_samples)
    y_samples = np.where(x_samples >= x_offset,
                         amplitude * np.sin(2 * np.pi * (x_samples - x_offset) / wavelength),
                         0.0)
    
    # Find minimum distance point
    distances = np.sqrt((x_samples - x_veh)**2 + (y_samples - y_veh)**2)
    idx_min = np.argmin(distances)
    
    # Refine with local search (Newton-like)
    x_closest = x_samples[idx_min]
    for _ in range(3):  # Few iterations of refinement
        if x_closest < x_offset:
            # In the lead-in region: path is y=0, closest point is directly below
            y_path = 0.0
            dy_dx = 0.0
            d2y_dx2 = 0.0
        else:
            y_path = amplitude * np.sin(2 * np.pi * (x_closest - x_offset) / wavelength)
            dy_dx = amplitude * 2 * np.pi / wavelength * np.cos(2 * np.pi * (x_closest - x_offset) / wavelength)
            d2y_dx2 = -amplitude * (2 * np.pi / wavelength)**2 * np.sin(2 * np.pi * (x_closest - x_offset) / wavelength)
        
        # Gradient of distance^2 w.r.t. x_path
        grad = 2 * (x_closest - x_veh) + 2 * (y_path - y_veh) * dy_dx
        
        # Second derivative for Newton step
        hess = 2 + 2 * dy_dx**2 + 2 * (y_path - y_veh) * d2y_dx2
        
        if abs(hess) > 1e-6:
            x_closest = x_closest - 0.5 * grad / hess  # Damped Newton step
    
    if x_closest < x_offset:
        y_closest = 0.0
    else:
        y_closest = amplitude * np.sin(2 * np.pi * (x_closest - x_offset) / wavelength)
    dist = np.sqrt((x_closest - x_veh)**2 + (y_closest - y_veh)**2)
    
    return x_closest, y_closest, dist


def check_sinusoidal_feasibility(amplitude, wavelength, wheelbase=3.302, delta_max=0.5):
    """
    Check if a sinusoidal path is feasible for the vehicle.
    
    For y = A*sin(2πx/λ), max curvature κ = A*(2π/λ)² at the peaks.
    Vehicle min turning radius R_min = L/tan(δ_max).
    
    Returns:
        (is_feasible, required_R, achievable_R, margin_pct)
    """
    # Max curvature of sinusoid (at peaks where y'' is maximum)
    kappa_max = amplitude * (2 * np.pi / wavelength) ** 2
    required_R = 1.0 / kappa_max if kappa_max > 0 else float('inf')
    
    # Vehicle's minimum turning radius (Ackermann geometry)
    achievable_R = wheelbase / np.tan(delta_max)
    
    # Margin (positive = feasible, negative = impossible)
    margin_pct = (required_R - achievable_R) / achievable_R * 100
    is_feasible = required_R >= achievable_R
    
    return is_feasible, required_R, achievable_R, margin_pct


def suggest_feasible_sine_params(target_amplitude=2.0, wheelbase=3.302, delta_max=0.5, margin=1.2):
    """
    Suggest a feasible wavelength for a given amplitude.
    
    Args:
        target_amplitude: Desired amplitude (m)
        margin: Safety margin (1.2 = 20% easier than limit)
    
    Returns:
        min_wavelength: Minimum feasible wavelength (m)
    """
    achievable_R = wheelbase / np.tan(delta_max)
    # Add margin for controller tracking error
    min_R = achievable_R * margin
    
    # κ_max = A * (2π/λ)² = 1/R_min
    # λ = 2π * sqrt(A * R_min)
    min_wavelength = 2 * np.pi * np.sqrt(target_amplitude * min_R)
    
    return min_wavelength


def make_path_function(path_type='lane_change', lane_offset=3.0, v_target=8.0,
                        sine_amplitude=2.0, sine_wavelength=30.0,
                        use_closest_point=True, lead_in=0.0,
                        csv_dir=None, total_length=None):
    """
    Create a ReferencePath for the MPC driver.

    Generates dense waypoints for the chosen path type, fits an arc-length
    parameterised cubic spline, and returns a :class:`ReferencePath` whose
    :meth:`get_reference` method has the same ``(time, z0, N, dt)`` signature
    used by the controller loop.

    Args:
        path_type: 'lane_change', 'double_lane_change', or 'sinusoidal'
        lane_offset: Lateral offset for lane change maneuvers (m)
        v_target: Target longitudinal velocity (m/s)
        sine_amplitude: Amplitude for sinusoidal path (m)
        sine_wavelength: Wavelength for sinusoidal path (m)
        use_closest_point: (kept for CLI compat; now always True internally)
        lead_in: Straight lead-in distance (m) added before path geometry
        csv_dir: If given, save a reference_path_<type>.csv into this dir
        total_length: Override total path length (m); auto-computed if None

    Returns:
        ReferencePath object.  Use ``ref_path.get_reference`` as the
        path callable, and ``ref_path.evaluate_at_x`` for analytics.
    """
    from reference_path import ReferencePath, generate_path_waypoints

    # Check sinusoidal path feasibility
    if path_type == 'sinusoidal':
        is_feasible, req_R, ach_R, margin = check_sinusoidal_feasibility(
            sine_amplitude, sine_wavelength)
        if not is_feasible:
            min_wl = suggest_feasible_sine_params(sine_amplitude)
            print(f"\n  ⚠ WARNING: Sinusoidal path is INFEASIBLE!")
            print(f"     Required turning radius: {req_R:.2f} m")
            print(f"     Vehicle minimum radius:  {ach_R:.2f} m")
            print(f"     Path is {-margin:.0f}% beyond vehicle limits")
            print(f"     Suggestions:")
            print(f"       - Increase wavelength to ≥{min_wl:.1f}m (currently {sine_wavelength}m)")
            print(f"       - Or reduce amplitude to ≤{sine_amplitude * (req_R/ach_R):.2f}m")
            print()
        else:
            print(f"  Path feasibility: OK (margin: +{margin:.0f}%)")

    if lead_in > 0:
        print(f"  Lead-in: {lead_in:.0f}m straight before path starts")

    # Generate dense waypoints
    x_pts, y_pts = generate_path_waypoints(
        path_type, lead_in=lead_in, lane_offset=lane_offset,
        sine_amplitude=sine_amplitude, sine_wavelength=sine_wavelength,
        total_length=total_length,
    )

    # Build spline-based reference path
    ref_path = ReferencePath(x_pts, y_pts, v_target)
    print(f"  Reference path: {ref_path}")

    # Optionally save CSV
    if csv_dir is not None:
        import os
        os.makedirs(csv_dir, exist_ok=True)
        ref_path.save_csv(os.path.join(csv_dir,
                                       f'reference_path_{path_type}.csv'))

    return ref_path


# =============================================================================
# Chrono Simulation Setup
# =============================================================================

def setup_chrono_vehicle(visualize=True):
    """Setup PyChrono HMMWV vehicle."""
    
    # Set Chrono data path for mesh files
    chrono.SetChronoDataPath(chrono.GetChronoDataPath())
    veh.SetDataPath(chrono.GetChronoDataPath() + 'vehicle/')
    
    # Create vehicle FIRST (it creates its own system internally)
    vehicle = veh.HMMWV_Full()
    vehicle.SetContactMethod(chrono.ChContactMethod_SMC)
    vehicle.SetChassisFixed(False)
    vehicle.SetInitPosition(chrono.ChCoordsysd(
        chrono.ChVector3d(0, 0, 0.5),
        chrono.ChQuaterniond(1, 0, 0, 0)
    ))
    vehicle.SetEngineType(veh.EngineModelType_SHAFTS)
    vehicle.SetTransmissionType(veh.TransmissionModelType_AUTOMATIC_SHAFTS)
    vehicle.SetDriveType(veh.DrivelineTypeWV_AWD)
    vehicle.SetTireType(veh.TireModelType_RIGID)
    vehicle.Initialize()
    
    # Get system FROM vehicle after initialization  
    system = vehicle.GetSystem()
    system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    
    if visualize:
        # MESH for visual quality, PRIMITIVES for less important parts
        vehicle.SetChassisVisualizationType(veh.VisualizationType_MESH)
        vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
        vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
        vehicle.SetWheelVisualizationType(veh.VisualizationType_MESH)
        vehicle.SetTireVisualizationType(veh.VisualizationType_MESH)
    else:
        vehicle.SetChassisVisualizationType(veh.VisualizationType_PRIMITIVES)
        vehicle.SetWheelVisualizationType(veh.VisualizationType_PRIMITIVES)
        vehicle.SetTireVisualizationType(veh.VisualizationType_PRIMITIVES)
    
    return system, vehicle


def load_terrain_config(config_path):
    """
    Load terrain configuration from YAML file.
    
    Args:
        config_path: Path to YAML config file
        
    Returns:
        dict with terrain parameters (numeric values converted to float)
    """
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    
    # Validate required fields
    required = ['Kphi', 'Kc', 'n', 'cohesion', 'friction_angle', 'janosi_shear']
    for field in required:
        if field not in config:
            raise ValueError(f"Missing required terrain parameter: {field}")
    
    # Convert all values to float (handles scientific notation strings like '2.1e6')
    numeric_fields = ['Kphi', 'Kc', 'n', 'cohesion', 'friction_angle', 'janosi_shear',
                      'elastic_stiffness', 'damping', 'length', 'width', 'mesh_resolution']
    for field in numeric_fields:
        if field in config:
            config[field] = float(config[field])
    
    return config


def setup_scm_terrain(system, vehicle=None, visualize=True, terrain_preset='sand',
                      terrain_config=None, mesh_resolution=None,
                      bump_amplitude=0.0, bump_wavelength=20.0, 
                      bump_octaves=4, bump_seed=12345, bump_max_slope=0.3):
    """Setup SCM deformable terrain
    
    Args:
        system: Chrono system
        vehicle: Chrono vehicle (for moving patch optimization)
        visualize: Enable visualization
        terrain_preset: Preset name ('sand', 'clay', 'dirt') 
                       Ignored if terrain_config provided.
        terrain_config: Dict with terrain params from config file (overrides preset)
        mesh_resolution: Override mesh spacing (m). Default: 0.08 for headless, 0.05 for vis.
        bump_amplitude: Height variation amplitude (m). 0 = flat terrain.
        bump_wavelength: Distance between bump peaks in meters (larger = gentler rolling hills)
        bump_octaves: Noise octaves (more = more detail)  
        bump_seed: Random seed for reproducibility
        bump_max_slope: Maximum terrain slope (0.3 = 30% grade, safe for most vehicles)
    """
    import tempfile
    
    terrain = veh.SCMTerrain(system)
    
    # Load params from config or use presets
    if terrain_config is not None:
        Kphi = terrain_config['Kphi']
        Kc = terrain_config['Kc']
        n = terrain_config['n']
        c = terrain_config['cohesion']
        phi = terrain_config['friction_angle']
        k = terrain_config['janosi_shear']
        elastic_stiffness = terrain_config.get('elastic_stiffness', 2e8)
        damping = terrain_config.get('damping', 3e4)
        terrain_name = terrain_config.get('description', 'Custom config')
        print(f"  Terrain: {terrain_name}")
        print(f"    Kphi={Kphi:.2e}, Kc={Kc:.0f}, n={n:.2f}")
        print(f"    cohesion={c:.0f}, friction={phi:.0f}°, janosi={k:.3f}")
    else:
        # Use preset
        if terrain_preset not in TERRAIN_PRESETS:
            raise ValueError(f"Unknown terrain preset: {terrain_preset}. "
                           f"Available: {list(TERRAIN_PRESETS.keys())}")
        preset = TERRAIN_PRESETS[terrain_preset]
        Kphi = preset['Kphi']
        Kc = preset['Kc']
        n = preset['n']
        c = preset['cohesion']
        phi = preset['friction_angle']
        k = preset['janosi_shear']
        elastic_stiffness = preset.get('elastic_stiffness', 2e8)
        damping = preset.get('damping', 3e4)
        print(f"  Terrain: {terrain_preset} - {preset.get('description', '')}")
    
    # SetSoilParameters expects friction angle in DEGREES (not radians!)
    terrain.SetSoilParameters(
        Kphi, Kc, n, c, phi, k, elastic_stiffness, damping
    )
    
    # Mesh resolution: coarser = faster, all modes use 0.12m for real-time performance
    if mesh_resolution is not None:
        print(f"  Mesh: custom resolution {mesh_resolution}m")
        delta = mesh_resolution
    else:
        delta = 0.08  # Fine mesh for accurate terrain
        print(f"  Mesh: {delta}m")
    
    # Terrain dimensions: large visual area (moving patch keeps computation local)
    length, width = 200.0, 50.0  # Large terrain for visualization
    
    # Initialize terrain - flat or bumpy
    if bump_amplitude > 0:
        # Generate Perlin noise heightmap
        heightmap_file = tempfile.gettempdir() + '/scm_heightmap.bmp'
        # Image resolution: ~1 pixel per 0.5m for reasonable detail
        img_width = int(length * 2)
        img_height = int(width * 2)
        
        # Convert wavelength to frequency: 
        # wavelength is in meters, frequency is per-pixel
        # With 2 pixels per meter, freq = 1 / (wavelength * 2)
        pixel_frequency = 1.0 / (bump_wavelength * 2)
        
        generate_heightmap_bmp(heightmap_file, img_width, img_height,
                               amplitude=bump_amplitude, octaves=bump_octaves,
                               frequency=pixel_frequency, seed=bump_seed,
                               max_slope=bump_max_slope)
        # Initialize with heightmap: maps pixel values to height range
        terrain.Initialize(heightmap_file, length, width, 
                          0.0, bump_amplitude, delta)
        print(f"  Bumpy terrain: amplitude={bump_amplitude:.2f}m, "
              f"wavelength={bump_wavelength:.0f}m, max_slope={bump_max_slope*100:.0f}%")
    else:
        terrain.Initialize(length, width, delta)
    
    # Moving patch: only compute SCM deformation near the vehicle (huge speedup)
    if vehicle is not None:
        terrain.AddMovingPatch(vehicle.GetChassisBody(),
                               chrono.ChVector3d(0, 0, 0),
                               chrono.ChVector3d(6, 3, 1))
    
    if visualize:
        terrain.SetPlotType(veh.SCMTerrain.PLOT_SINKAGE, 0, 0.1)
    
    print(f"  SCM mesh: {delta}m, terrain: {length}x{width}m"
          + (", moving patch ON" if vehicle else ""))
    
    return terrain, {'Kphi': Kphi, 'Kc': Kc, 'n': n, 'c': c, 'phi': phi, 'k': k}


def add_trajectory_markers(system, path_type='lane_change', sim_time=10.0, 
                           v_target=8.0, lane_offset=3.0, marker_z=None,
                           sine_amplitude=2.0, sine_wavelength=30.0, lead_in=0.0):
    """
    Add visual markers on the ground to show the reference trajectory.
    
    Args:
        system: Chrono system
        path_type: 'lane_change', 'double_lane_change', or 'sinusoidal'
        sim_time: Duration to generate markers for
        v_target: Target velocity (used only for estimating marker count)
        lane_offset: Lane change offset (m)
        marker_z: Z height for markers (default: 0.15, set higher for bumpy terrain)
        sine_amplitude: Amplitude for sinusoidal path (m)
        sine_wavelength: Wavelength for sinusoidal path (m)
    """
    marker_spacing = 4.0  # meters between markers (sparser for performance)
    marker_radius = 0.15
    marker_height = marker_z if marker_z is not None else 0.15
    
    # Estimate total distance (just for marker count, not for path positions)
    total_dist = max(v_target * sim_time, 60.0)  # At least 60m to cover path
    n_markers = int(total_dist / marker_spacing) + 1
    
    print(f"  Adding {n_markers} trajectory markers for {path_type}...")
    
    # Path-specific parameters - ALL FIXED POSITIONS (shifted by lead_in)
    # Single lane change
    lc_start = 10.0 + lead_in
    lc_end = 25.0 + lead_in
    
    # Double lane change zones
    dlc_z1_start, dlc_z1_end = 8.0 + lead_in, 18.0 + lead_in
    dlc_z2_start, dlc_z2_end = 28.0 + lead_in, 38.0 + lead_in
    
    for i in range(n_markers):
        x = i * marker_spacing
        
        if path_type == 'lane_change':
            if x < lc_start:
                y = 0.0
                zone = 'start'
            elif x > lc_end:
                y = lane_offset
                zone = 'end'
            else:
                blend = (x - lc_start) / (lc_end - lc_start)
                blend = blend * blend * (3 - 2 * blend)
                y = blend * lane_offset
                zone = 'transition'
                
        elif path_type == 'double_lane_change':
            if x < dlc_z1_start:
                y = 0.0
                zone = 'start'
            elif x < dlc_z1_end:
                blend = (x - dlc_z1_start) / (dlc_z1_end - dlc_z1_start)
                blend = blend * blend * (3 - 2 * blend)
                y = blend * lane_offset
                zone = 'transition1'
            elif x < dlc_z2_start:
                y = lane_offset
                zone = 'middle'
            elif x < dlc_z2_end:
                blend = (x - dlc_z2_start) / (dlc_z2_end - dlc_z2_start)
                blend = blend * blend * (3 - 2 * blend)
                y = lane_offset * (1 - blend)
                zone = 'transition2'
            else:
                y = 0.0
                zone = 'end'
                
        elif path_type == 'sinusoidal':
            # Use parameters passed to function (with lead-in offset)
            if x < lead_in:
                y = 0.0
            else:
                y = sine_amplitude * np.sin(2 * np.pi * (x - lead_in) / sine_wavelength)
            zone = 'sine'
        else:
            y = 0.0
            zone = 'default'
        
        # Create marker
        marker = chrono.ChBodyEasySphere(marker_radius, 1000, True, False)
        marker.SetPos(chrono.ChVector3d(x, y, marker_height))
        marker.SetFixed(True)
        
        # Color by zone
        if zone == 'start':
            color = chrono.ChColor(0.2, 0.8, 0.2)  # Green
        elif zone == 'end':
            color = chrono.ChColor(0.2, 0.2, 0.8)  # Blue
        elif zone in ['transition', 'transition1']:
            color = chrono.ChColor(0.9, 0.9, 0.2)  # Yellow
        elif zone == 'middle':
            color = chrono.ChColor(0.8, 0.4, 0.1)  # Orange
        elif zone == 'transition2':
            color = chrono.ChColor(0.9, 0.5, 0.9)  # Pink
        elif zone == 'sine':
            # Rainbow based on sine phase (using actual amplitude)
            phase = (y / sine_amplitude + 1) / 2  # 0 to 1
            color = chrono.ChColor(0.8 * phase, 0.3, 0.8 * (1-phase))
        else:
            t = i / max(n_markers - 1, 1)
            color = chrono.ChColor(0.2 + 0.6 * t, 0.8 - 0.4 * t, 0.2)
        
        marker.GetVisualShape(0).SetColor(color)
        system.Add(marker)


# =============================================================================
# Main Simulation Loop
# =============================================================================

def run_simulation(controller_type='linear', visualize=True, sim_time=10.0, 
                   path_type='lane_change', terrain_preset='sand', debug=False,
                   nn_scale=1.0, nn_sign=1, terrain_config=None, v_target=5.0,
                   kappa_mode='zero', use_ukf=False, ukf_n_init=None, ukf_version='v3',
                   async_mpc=False,
                   multiprocess_mpc=False, nn_model_dir=None,
                   bump_amplitude=0.0, bump_wavelength=20.0, bump_octaves=4, 
                   bump_seed=12345, bump_max_slope=0.3, manual_control=False,
                   sine_amplitude=2.0, sine_wavelength=30.0, measurement_noise=True,
                   use_closest_point=True, rms_time_start=None, rms_time_end=None,
                   steer_excite_amp=0.0, steer_excite_freq=0.15, steer_excite_ramp=3.0,
                   lateral_load_transfer=True):
    """
    Run simulation with Dallas MPC controller or manual G29 control.
    
    Args:
        controller_type: 'linear' or 'nn'
        visualize: Enable Irrlicht visualization
        sim_time: Simulation duration (s)
        path_type: 'lane_change', 'double_lane_change', or 'sinusoidal'
        terrain_preset: 'sand' (dry sand), 'clay' (clayey soil), or 'dirt' (sandy loam)
        debug: Print tire force debug info
        use_ukf: Enable UKF terrain estimation (only with NN controller)
        measurement_noise: Dict with noise std devs or True for defaults. Keys:
            x (1.2m), y (1.2m), psi (0.0175 rad), u (0.25 m/s), v (0.25 m/s), omega (0.0175 rad/s)
        ukf_n_init: Initial guess for sinkage exponent (default: 0.7 away from truth)
        nn_scale: Scale factor for NN force predictions
        nn_sign: Sign multiplier for NN forces (+1 or -1)
        terrain_config: Dict with terrain params from config file
        kappa_mode: 'zero' or 'approx' — how to estimate longitudinal slip in MPC
        async_mpc: If True, run MPC in a separate thread (non-blocking)
        multiprocess_mpc: If True, run MPC in a separate PROCESS (bypasses GIL)
        bump_amplitude: Terrain bump height in meters (0 = flat)
        bump_wavelength: Distance between bump peaks in meters (larger = gentler)
        bump_octaves: Noise octaves (more = more detail)
        bump_seed: Random seed for reproducible terrain
        bump_max_slope: Maximum slope (0.3 = 30% grade)
        sine_amplitude: Amplitude for sinusoidal path (m)
        sine_wavelength: Wavelength for sinusoidal path (m)
        manual_control: If True, use G29 steering wheel for manual control (no MPC)
        use_closest_point: If True, use closest-point path re-indexing for recovery
        rms_time_start: Start time for RMS calculation (None = from beginning)
        rms_time_end: End time for RMS calculation (None = until end)
    """
    print(f"\n{'='*60}")
    if manual_control:
        print(f"Manual Control Mode: G29 Steering Wheel")
    else:
        print(f"Dallas MPC Simulation: {controller_type.upper()} Tire Model")
        if controller_type == 'nn':
            print(f"  NN scale={nn_scale}, sign={nn_sign}")
        if multiprocess_mpc:
            print(f"  MPC: MULTIPROCESS (separate process, bypasses GIL)")
        elif async_mpc:
            print(f"  MPC: ASYNC (threaded)")
    print(f"{'='*60}")
    
    # Timing for setup phases
    t0_setup = time.time()
    
    # Setup vehicle
    t0_vehicle = time.time()
    system, vehicle = setup_chrono_vehicle(visualize)
    t_vehicle = time.time() - t0_vehicle
    print(f"  [TIMING] Vehicle setup: {t_vehicle:.2f}s")
    
    # Setup terrain (pass vehicle for moving-patch optimization)
    t0_terrain = time.time()
    terrain, terrain_params = setup_scm_terrain(system, vehicle=vehicle, visualize=visualize,
                                                 terrain_preset=terrain_preset, terrain_config=terrain_config,
                                                 bump_amplitude=bump_amplitude, bump_wavelength=bump_wavelength,
                                                 bump_octaves=bump_octaves, bump_seed=bump_seed,
                                                 bump_max_slope=bump_max_slope)
    t_terrain = time.time() - t0_terrain
    print(f"  [TIMING] Terrain setup: {t_terrain:.2f}s")
    
    # Manual control mode: skip MPC setup, use G29 steering wheel
    if manual_control:
        print("  Initializing G29 steering wheel...")
        driver = ManualDriver(vehicle)
        
        # Add trajectory markers for reference (even in manual mode)
        if visualize:
            marker_z = bump_amplitude + 0.5 if bump_amplitude > 0 else 0.15
            add_trajectory_markers(system, path_type, sim_time, v_target=v_target, marker_z=marker_z,
                                   sine_amplitude=sine_amplitude, sine_wavelength=sine_wavelength)
    
    # Simulation step size: RigidTire + SCM is stable up to ~8ms
    # Use larger steps for speed, smaller only if accuracy needed
    step_size = 3e-3  # 3ms - balance speed and accuracy

    # MPC-based control (skip this entire block when manual_control=True)
    if not manual_control:
        # Warn if terrain is outside NN training range (NN predictions can be poor)
        if controller_type == 'nn':
            in_range, msgs = check_terrain_in_training_range(terrain_params)
            if not in_range:
                print("  ⚠ Terrain parameters are OUTSIDE NN training range:")
                for m in msgs:
                    print(f"     {m}")
                print("  NN predictions may be less accurate for out-of-range terrain parameters.")
    
        # Vehicle params: use HMMWV-aligned values so MPC Fz matches Chrono vehicle
        vehicle_params = get_vehicle_params_for_demo()
        
        # Create MPC controller
        # MPC runs at 10Hz with 2.5s horizon (25 steps × 0.1s)
        dt_mpc = 0.1   # 10Hz update rate
        N_horizon = 25  # 2.5s look-ahead (fast model allows longer horizon)
        
        # NN model directory (default: v6 - uses steering_rate as measured input)
        model_version = nn_model_dir if nn_model_dir else "v6"
        base_path = Path(__file__).parent.parent
        model_path = base_path / "nn_models" / model_version / "best_terrain_nn.pt"
        scaler_path = model_path.parent / "scalers.pkl"
        
        if controller_type == 'nn':
            # Load NN model (model_path and scaler_path already defined above)
            
            if not model_path.exists():
                print(f"⚠ NN model not found: {model_path}")
                print("  Falling back to linear model")
                controller_type = 'linear'
                nn_casadi = None
            else:
                nn_casadi = NNCasADi(model_path, scaler_path, terrain_params)
                
                # Diagnostic: test NN behavior at canonical slip angles (use same M as MPC)
                print("\n  NN Tire Model Diagnostic:")
                print("  -------------------------")
                Fz_f, Fz_r = get_static_fz_per_wheel(vehicle_params)
                Fz_wheel = (Fz_f + Fz_r) / 2  # representative per-wheel
                u_test = 5.0
                for alpha_deg in [-5, -2, 0, 2, 5]:
                    alpha_rad = np.radians(alpha_deg)
                    Fx, Fy = nn_casadi.predict_numeric(alpha_rad, Fz_wheel, u_test, 0.0)
                    # What the MPC uses: negate and double
                    Fy_mpc = -2.0 * Fy
                    print(f"    α={alpha_deg:+3.0f}°: NN wheel Fy={Fy:+7.0f}N → MPC axle Fy={Fy_mpc:+8.0f}N")
                print("")
        else:
            nn_casadi = None
        
        t0_mpc = time.time()
        mpc = DallasMPC(
            nn_casadi=nn_casadi,
            params=vehicle_params,
            dt=dt_mpc,
            N=N_horizon,
            nn_scale=nn_scale,
            nn_sign=nn_sign,
            kappa_mode=kappa_mode,
            lateral_load_transfer=lateral_load_transfer,
        )
        t_mpc = time.time() - t0_mpc
        nn_calls = 4 if lateral_load_transfer else 2
        print(f"  [TIMING] MPC setup: {t_mpc:.2f}s (NN calls/step: {nn_calls})")
        
        # Create reference path (spline-based)
        ref_path = make_path_function(path_type, v_target=v_target,
                                       sine_amplitude=sine_amplitude,
                                       sine_wavelength=sine_wavelength,
                                       use_closest_point=use_closest_point)
        path_func = ref_path.get_reference
        print(f"  Target speed: {v_target} m/s")
        if path_type == 'sinusoidal':
            print(f"  Sinusoidal: amp={sine_amplitude}m, wavelength={sine_wavelength}m")
        
        # UKF terrain estimation (Dallas Sec. IV)
        # V2 is proper UKF, V3 is trajectory-matching grid search
        # NOTE: V3 performs better with gentle path tracking due to low observability
        # Dallas paper used aggressive sinusoidal steering + throttle for excitation
        ukf = None
        if use_ukf and controller_type == 'nn' and nn_casadi is not None:
            import sys
            sys.path.insert(0, str(Path(__file__).parent.parent / "terrain_estimation"))
            if ukf_version == 'v2':
                from terrain_estimator_v2 import load_ukf
            else:
                from terrain_estimator_v3 import load_ukf
            ukf_model_path = base_path / "nn_models" / model_version / "best_terrain_nn.pt"
            ukf_scaler_path = ukf_model_path.parent / "scalers.pkl"
            
            n_init = ukf_n_init if ukf_n_init is not None else 0.7
            dt_ukf = 0.024  # Dallas paper: sensor measurements updated every 24ms
            ukf = load_ukf(
                model_path=str(ukf_model_path),
                scaler_path=str(ukf_scaler_path),
                vehicle_params=vehicle_params,
                base_terrain_params=terrain_params,
                dt=dt_ukf,
                n_init=n_init,
                debug=debug,
            )
            true_n = terrain_params.get('n', '?')
            print(f"  UKF enabled: n_init={n_init:.2f}, true n={true_n}")
        
        # Create driver
        # Build config for multiprocess mode (MPC will be rebuilt in child process)
        mpc_config = None
        if multiprocess_mpc:
            mp_model_path = model_path  # Already computed above
            mp_scaler_path = scaler_path
            mpc_config = {
                'vehicle_params': vehicle_params,
                'terrain_params': terrain_params,
                'dt': dt_mpc,
                'N': N_horizon,
                'nn_scale': nn_scale,
                'nn_sign': nn_sign,
                'kappa_mode': kappa_mode,
                'lateral_load_transfer': lateral_load_transfer,
                'nn_model_path': str(mp_model_path) if controller_type == 'nn' else None,
                'scaler_path': str(mp_scaler_path) if controller_type == 'nn' else None,
            }
        
        driver = DallasMPCDriver(vehicle, mpc, dt_mpc=dt_mpc, path_func=path_func,
                                 debug=debug, ukf=ukf, async_mpc=async_mpc,
                                 multiprocess_mpc=multiprocess_mpc, mpc_config=mpc_config,
                                 measurement_noise=measurement_noise,
                                 steer_excite_amp=steer_excite_amp,
                                 steer_excite_freq=steer_excite_freq,
                                 steer_excite_ramp=steer_excite_ramp,
                                 step_size=step_size)
        driver.v_target = v_target
        
        if steer_excite_amp > 0:
            print(f"  Steering excitation: amp={np.degrees(steer_excite_amp):.1f}°, freq={steer_excite_freq}Hz, ramp={steer_excite_ramp}s")
        
        if multiprocess_mpc:
            print(f"  MPC worker PROCESS started (bypasses GIL)")
        elif async_mpc:
            print(f"  MPC worker thread started")
        
        # Add trajectory markers BEFORE visualization (must be in system before vis.Initialize)
        if visualize:
            marker_z = bump_amplitude + 0.5 if bump_amplitude > 0 else 0.15
            add_trajectory_markers(system, path_type, sim_time, v_target=v_target, marker_z=marker_z,
                                   sine_amplitude=sine_amplitude, sine_wavelength=sine_wavelength)
    
    # Visualization - optimized for real-time performance
    vis = None
    if visualize:
        try:
            vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
            vis.SetWindowTitle(f"Dallas MPC - {controller_type.upper()}")
            vis.SetWindowSize(3440, 1440)  # Full ultrawide resolution
            vis.SetChaseCamera(chrono.ChVector3d(0, 0, 1.5), 6.0, 0.5)
            vis.Initialize()
            vis.AddLightDirectional()
            vis.AddSkyBox()
            vis.AttachVehicle(vehicle.GetVehicle())
            
        except Exception as e:
            print(f"⚠ Visualization failed: {e}")
            vis = None
    
    print(f"  Physics step: {step_size*1000:.0f}ms")
    
    # Total setup time
    t_setup_total = time.time() - t0_setup
    print(f"  [TIMING] Total setup: {t_setup_total:.2f}s")
    
    if manual_control:
        print(f"  Running simulation (close window to exit)...")
    else:
        print(f"  Running {sim_time}s simulation...")
    start_time = time.time()
    
    # Timing accumulators for simulation loop
    timing_stats = {
        'render': 0.0,
        'driver_sync': 0.0,
        'terrain_sync': 0.0,
        'vehicle_sync': 0.0,
        'driver_advance': 0.0,
        'terrain_advance': 0.0,
        'vehicle_advance': 0.0,
        'vis_advance': 0.0,
        'step_count': 0,
    }
    last_timing_report = 0.0
    
    # Tire force logging (for Task 5: actual vs NN force comparison)
    tire_force_history = []
    force_log_interval = 10  # Log every N-th physics step to avoid overhead
    force_log_counter = 0
    
    # Frame skipping for visualization: render at ~35 FPS, not every physics step
    # At 3ms step = 333 Hz physics, render every ~10 steps = ~35 FPS
    render_interval = 1.0 / 35.0  # Target 35 FPS
    last_render_time = -render_interval
    
    while True:
        time_chrono = vehicle.GetSystem().GetChTime()
        
        # In manual mode, run until window is closed (no time limit)
        if not manual_control and time_chrono >= sim_time:
            break
        
        # Check visualization window (but don't render every frame)
        if vis is not None and not vis.Run():
            break
        
        # Render at limited FPS (frame skipping for performance)
        should_render = vis is not None and (time_chrono - last_render_time >= render_interval)
        if should_render:
            t0 = time.time()
            vis.BeginScene()
            vis.Render()
            vis.EndScene()
            timing_stats['render'] += time.time() - t0
            last_render_time = time_chrono
        
        # Synchronize driver first to get latest inputs (critical for manual control)
        t0 = time.time()
        driver.Synchronize(time_chrono)
        timing_stats['driver_sync'] += time.time() - t0
        
        # Get driver inputs AFTER synchronize so we have current values
        driver_inputs = veh.DriverInputs()
        driver_inputs.m_steering = driver.GetSteering()
        driver_inputs.m_throttle = driver.GetThrottle()
        driver_inputs.m_braking = driver.GetBraking()
        
        t0 = time.time()
        terrain.Synchronize(time_chrono)
        timing_stats['terrain_sync'] += time.time() - t0
        
        t0 = time.time()
        vehicle.Synchronize(time_chrono, driver_inputs, terrain)
        timing_stats['vehicle_sync'] += time.time() - t0
        
        if vis is not None:
            vis.Synchronize(time_chrono, driver_inputs)
        
        # Advance
        t0 = time.time()
        driver.Advance(step_size)
        timing_stats['driver_advance'] += time.time() - t0
        
        t0 = time.time()
        terrain.Advance(step_size)
        timing_stats['terrain_advance'] += time.time() - t0
        
        t0 = time.time()
        vehicle.Advance(step_size)
        timing_stats['vehicle_advance'] += time.time() - t0
        
        # Tire force logging (every N-th step)
        force_log_counter += 1
        if force_log_counter >= force_log_interval:
            force_log_counter = 0
            veh_obj = vehicle.GetVehicle()
            record = {'time': time_chrono}
            for axle_idx, axle_name in enumerate(['front', 'rear']):
                for side_idx, side_name in [(veh.LEFT, 'left'), (veh.RIGHT, 'right')]:
                    tire = veh_obj.GetTire(axle_idx, side_idx)
                    tf = tire.ReportTireForce(terrain)
                    key = f'{axle_name}_{side_name}'
                    record[f'{key}_Fx'] = tf.force.x
                    record[f'{key}_Fy'] = tf.force.y
                    record[f'{key}_Fz'] = tf.force.z
                    record[f'{key}_slip_angle'] = tire.GetSlipAngle()
                    record[f'{key}_long_slip'] = tire.GetLongitudinalSlip()
            tire_force_history.append(record)
        
        if vis is not None:
            t0 = time.time()
            vis.Advance(step_size)
            timing_stats['vis_advance'] += time.time() - t0
        
        timing_stats['step_count'] += 1
        
        # Real-time throttling: spin-wait if simulation is running ahead of real-time
        if vis is not None:
            target_wall_time = start_time + time_chrono
            while time.time() < target_wall_time:
                pass  # Busy wait (spinning) for precise timing
        
        # Progress with timing breakdown every 2 seconds of sim time
        if time_chrono - last_timing_report >= 2.0:
            last_timing_report = time_chrono
            pos = vehicle.GetChassisBody().GetPos()
            elapsed = time.time() - start_time
            rt_factor = time_chrono / elapsed if elapsed > 0 else 0
            
            # Calculate percentages
            total_step_time = sum(v for k, v in timing_stats.items() if k != 'step_count')
            if total_step_time > 0:
                pct = lambda k: timing_stats[k] / total_step_time * 100
                print(f"    t={time_chrono:.1f}s pos=({pos.x:.1f},{pos.y:.1f}) RT={rt_factor:.2f}x | "
                      f"terrain:{pct('terrain_advance'):.0f}% drv:{pct('driver_sync'):.0f}% "
                      f"veh:{pct('vehicle_advance'):.0f}% render:{pct('render'):.0f}%")
            else:
                print(f"    t={time_chrono:.1f}s: pos=({pos.x:.1f}, {pos.y:.1f})")
    
    elapsed = time.time() - start_time
    print(f"  Simulation completed in {elapsed:.1f}s (RT factor: {sim_time/elapsed:.2f}x)")
    
    # MPC control loop timing summary (always print, not just debug)
    if not manual_control and hasattr(driver, 'mpc_solve_times') and driver.mpc_solve_times:
        solve_times = np.array(driver.mpc_solve_times)
        model_type = 'NN' if driver.mpc.use_nn else 'LINEAR (Pacejka)'
        print(f"\n  [CONTROL LOOP TIMING - {model_type}]")
        print(f"    Total MPC solves: {len(solve_times)}")
        print(f"    Mean solve time:  {np.mean(solve_times)*1000:.2f} ms")
        print(f"    Std solve time:   {np.std(solve_times)*1000:.2f} ms")
        print(f"    Min solve time:   {np.min(solve_times)*1000:.2f} ms")
        print(f"    Max solve time:   {np.max(solve_times)*1000:.2f} ms")
        print(f"    Median solve:     {np.median(solve_times)*1000:.2f} ms")
        print(f"    Effective rate:   {1.0/np.mean(solve_times):.1f} Hz (target: {1.0/driver.dt_mpc:.0f} Hz)")
        
        # Warn if MPC is too slow
        if np.mean(solve_times) > driver.dt_mpc:
            print(f"    ⚠ WARNING: MPC is slower than control rate! ({np.mean(solve_times)*1000:.1f}ms > {driver.dt_mpc*1000:.0f}ms)")
    
    # Async/Multiprocess MPC stats and cleanup
    if (async_mpc or multiprocess_mpc) and (driver.mpc_worker is not None or driver.mp_worker is not None):
        # Shutdown first to collect final stats from worker
        driver.shutdown()
        
        stats = driver.get_mpc_stats()
        if stats:
            mode = "MULTIPROCESS" if multiprocess_mpc else "ASYNC"
            print(f"\n  [{mode} MPC] Stats:")
            print(f"    Solves: {stats['solve_count']}, Avg solve: {stats['avg_solve_ms']:.1f}ms")
            print(f"    Dropped requests: {stats['dropped_requests']}")
    
    # Final timing breakdown
    total_step_time = sum(v for k, v in timing_stats.items() if k != 'step_count')
    if total_step_time > 0:
        print(f"\n  [TIMING] Step breakdown ({timing_stats['step_count']} steps):")
        for key in ['terrain_advance', 'driver_sync', 'vehicle_advance', 'render', 
                    'terrain_sync', 'vehicle_sync', 'driver_advance', 'vis_advance']:
            pct = timing_stats[key] / total_step_time * 100
            if pct > 1:  # Only show significant contributors
                print(f"    {key}: {timing_stats[key]:.2f}s ({pct:.1f}%)")
        print(f"    Total accounted: {total_step_time:.2f}s")
        overhead = elapsed - total_step_time
        print(f"    Overhead/other: {overhead:.2f}s ({overhead/elapsed*100:.1f}%)")
    
    # Results
    rms_error = None
    if len(driver.state_history) > 0:
        states = driver.state_history
        final_state = states[-1]
        
        print(f"\n  Results:")
        print(f"    Final position: ({final_state['x']:.2f}, {final_state['y']:.2f})")
        print(f"    Final heading: {np.degrees(final_state['psi']):.1f}°")
        print(f"    Final speed: {final_state['u']:.2f} m/s")
        
        # Calculate RMS tracking error
        # Filter by time window if specified (exclude startup transients)
        t_start = rms_time_start if rms_time_start is not None else 0.0
        t_end = rms_time_end if rms_time_end is not None else sim_time
        
        states_filtered = [s for s in states if t_start <= s['time'] <= t_end]
        
        if len(states_filtered) == 0:
            print(f"    Warning: No states in time window [{t_start}, {t_end}]s")
            states_filtered = states
        else:
            if rms_time_start is not None or rms_time_end is not None:
                print(f"    RMS window: [{t_start:.1f}, {t_end:.1f}]s ({len(states_filtered)} samples)")
        
        lane_offset = 3.0
        errors = []
        for s in states_filtered:
            x = s['x']
            y = s['y']
            if path_type == 'lane_change':
                lc_start, lc_end = 10, 25
                if x < lc_start:
                    y_ref = 0.0
                elif x > lc_end:
                    y_ref = lane_offset
                else:
                    t = (x - lc_start) / (lc_end - lc_start)
                    y_ref = lane_offset * t * t * (3 - 2 * t)
            elif path_type == 'double_lane_change':
                zone1_start, zone1_end = 8, 18
                zone2_start, zone2_end = 28, 38
                if x < zone1_start:
                    y_ref = 0.0
                elif x < zone1_end:
                    t = (x - zone1_start) / (zone1_end - zone1_start)
                    y_ref = lane_offset * t * t * (3 - 2 * t)
                elif x < zone2_start:
                    y_ref = lane_offset
                elif x < zone2_end:
                    t = (x - zone2_start) / (zone2_end - zone2_start)
                    y_ref = lane_offset * (1 - t * t * (3 - 2 * t))
                else:
                    y_ref = 0.0
            elif path_type == 'sinusoidal':
                y_ref = sine_amplitude * np.sin(2 * np.pi * x / sine_wavelength)
            else:
                y_ref = 0.0
            errors.append((y - y_ref)**2)
        
        rms_error = np.sqrt(np.mean(errors))
        print(f"    RMS tracking error: {rms_error:.3f} m")
        
        # Braking diagnostics
        speeds = [s['u'] for s in states]
        brakes = [s['brake'] for s in states]
        throttles = [s['throttle'] for s in states]
        brake_time = sum(1 for b in brakes if b > 0.1) * 0.1  # Assume 10Hz
        min_speed = min(speeds)
        avg_speed = np.mean(speeds)
        print(f"    Speed: min={min_speed:.2f}, avg={avg_speed:.2f} m/s")
        print(f"    Braking time: {brake_time:.1f}s ({brake_time/sim_time*100:.0f}% of sim)")
        
        if ukf is not None:
            true_n = terrain_params.get('n', None)
            est_n = ukf.n_estimated
            print(f"\n  UKF Terrain Estimation:")
            print(f"    Estimated n: {est_n:.4f}")
            if true_n is not None:
                err_pct = abs(est_n - true_n) / true_n * 100
                print(f"    True n:      {true_n:.4f}")
                print(f"    Error:       {err_pct:.1f}%")
            if hasattr(ukf, 'bias_v') and ukf.bias_v != 0:
                print(f"    Bias v:      {ukf.bias_v:+.5f} m/s")
                print(f"    Bias omega:  {ukf.bias_omega:+.5f} rad/s")
    
    if vis is not None:
        vis.GetDevice().closeDevice()
    
    # Save tire force history if any data was collected
    if tire_force_history:
        import json
        force_log_path = Path(__file__).parent.parent / "diagnostic_scripts" / "tire_force_log.json"
        force_log_path.parent.mkdir(parents=True, exist_ok=True)
        with open(force_log_path, 'w') as f:
            json.dump(tire_force_history, f)
        print(f"  Tire force log saved: {force_log_path} ({len(tire_force_history)} records)")
        
        # Also save state history for comparison
        if hasattr(driver, 'state_history') and driver.state_history:
            state_log_path = force_log_path.with_name("state_history_log.json")
            with open(state_log_path, 'w') as f:
                json.dump(driver.state_history, f)
            print(f"  State history saved: {state_log_path} ({len(driver.state_history)} records)")

        # Generate Chrono-actual vs NN-predicted lateral force plot
        if nn_casadi is not None and hasattr(driver, 'state_history') and driver.state_history:
            try:
                import matplotlib
                matplotlib.use("Agg")
                import matplotlib.pyplot as plt

                forces = tire_force_history
                states = driver.state_history

                times_f = np.array([r['time'] for r in forces])
                actual_Fy_front = np.array([r['front_left_Fy'] + r['front_right_Fy'] for r in forces])
                actual_Fy_rear = np.array([r['rear_left_Fy'] + r['rear_right_Fy'] for r in forces])

                # Interpolate state onto force time grid
                times_s = np.array([s['time'] for s in states])
                u_interp = np.interp(times_f, times_s, [s['u'] for s in states])
                v_interp = np.interp(times_f, times_s, [s['v'] for s in states])
                omega_interp = np.interp(times_f, times_s, [s['omega'] for s in states])
                delta_interp = np.interp(times_f, times_s, [s['delta'] for s in states])
                ax_interp = np.interp(times_f, times_s, [s.get('ax', 0.0) for s in states])

                vp = vehicle_params
                L = vp["Lf"] + vp["Lr"]
                h_cg = vp.get("h_cg", 0.65)
                T = vp.get("T", 1.8194)
                M = vp["M"]

                # Dynamic Fz with longitudinal + lateral load transfer
                Fz_f_dyn = (M * 9.81 * vp["Lr"] - M * ax_interp * h_cg) / L / 2.0
                Fz_r_dyn = (M * 9.81 * vp["Lf"] + M * ax_interp * h_cg) / L / 2.0
                ay = u_interp * omega_interp
                dFz = M * ay * h_cg / T / 2.0
                Fz_f_outer = np.minimum(Fz_f_dyn + dFz, Fz_f_dyn * 1.9)
                Fz_f_inner = np.maximum(Fz_f_dyn - dFz, Fz_f_dyn * 0.1)
                Fz_r_outer = np.minimum(Fz_r_dyn + dFz, Fz_r_dyn * 1.9)
                Fz_r_inner = np.maximum(Fz_r_dyn - dFz, Fz_r_dyn * 0.1)

                # Bicycle-model slip angles
                u_safe = np.maximum(np.abs(u_interp), 0.5)
                alpha_f = delta_interp - np.arctan2(v_interp + vp["Lf"] * omega_interp, u_safe)
                alpha_r = -np.arctan2(v_interp - vp["Lr"] * omega_interp, u_safe)

                # NN predictions — match MPC mode (lateral load transfer or not)
                nn_Fy_front = np.zeros(len(times_f))
                nn_Fy_rear = np.zeros(len(times_f))
                if lateral_load_transfer:
                    for i in range(len(times_f)):
                        _, Fy_fo = nn_casadi.predict_numeric(alpha_f[i], Fz_f_outer[i], u_safe[i])
                        _, Fy_fi = nn_casadi.predict_numeric(alpha_f[i], Fz_f_inner[i], u_safe[i])
                        _, Fy_ro = nn_casadi.predict_numeric(alpha_r[i], Fz_r_outer[i], u_safe[i])
                        _, Fy_ri = nn_casadi.predict_numeric(alpha_r[i], Fz_r_inner[i], u_safe[i])
                        nn_Fy_front[i] = -(Fy_fo + Fy_fi)
                        nn_Fy_rear[i] = -(Fy_ro + Fy_ri)
                else:
                    for i in range(len(times_f)):
                        _, Fy_fw = nn_casadi.predict_numeric(alpha_f[i], Fz_f_dyn[i], u_safe[i])
                        _, Fy_rw = nn_casadi.predict_numeric(alpha_r[i], Fz_r_dyn[i], u_safe[i])
                        nn_Fy_front[i] = -2.0 * Fy_fw
                        nn_Fy_rear[i] = -2.0 * Fy_rw

                fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
                for ax, name, actual, nn_pred in [
                    (axes[0], 'Front Axle', actual_Fy_front, nn_Fy_front),
                    (axes[1], 'Rear Axle', actual_Fy_rear, nn_Fy_rear),
                ]:
                    ax.plot(times_f, actual, 'b-', alpha=0.6, linewidth=0.8, label='Chrono actual')
                    ax.plot(times_f, nn_pred, 'r-', alpha=0.8, linewidth=1.2, label='NN predicted')
                    ax.set_ylabel('Fy (N)')
                    ax.set_title(f'{name} Lateral Force')
                    ax.legend()
                    ax.grid(True, alpha=0.3)
                axes[1].set_xlabel('Time (s)')

                terrain_name = terrain_preset if terrain_preset else 'custom'
                model_tag = 'nn' if controller_type == 'nn' else 'pacejka'
                lt_label = 'lat-xfer' if lateral_load_transfer else 'no-lat-xfer'
                fig.suptitle(f'Lateral Force: Chrono vs NN ({lt_label}) — {terrain_name} / {path_type}',
                             fontsize=13, y=1.01)
                plt.tight_layout()

                from datetime import datetime as _dt
                _ts = _dt.now().strftime("%Y%m%d_%H%M%S")
                plot_dir = Path(__file__).parent.parent / "diagnostic_scripts" / "plots" / f"{_ts}_{terrain_name}_{path_type}_{model_tag}"
                plot_dir.mkdir(parents=True, exist_ok=True)
                plot_path = plot_dir / f"Fy_actual_vs_nn_{terrain_name}_{path_type}_{model_tag}.png"
                plt.savefig(plot_path, dpi=150, bbox_inches='tight')
                plt.close()
                print(f"  Force comparison plot saved: {plot_path}")
            except Exception as e:
                print(f"  Warning: Could not generate force comparison plot: {e}")
    
    return rms_error


def compare_controllers(visualize=False, sim_time=10.0, path_type='lane_change',
                        terrain_config=None, terrain_preset='sand', v_target=5.0,
                        kappa_mode='zero', bump_amplitude=0.0, bump_wavelength=20.0,
                        bump_octaves=4, bump_seed=12345, bump_max_slope=0.3,
                        sine_amplitude=2.0, sine_wavelength=30.0, use_closest_point=True):
    """Compare linear vs NN tire models"""
    print("\n" + "="*70)
    print(f"DALLAS MPC COMPARISON: Linear vs NN Tire Model ({path_type})")
    if path_type == 'sinusoidal':
        print(f"  Sinusoidal: amp={sine_amplitude}m, wavelength={sine_wavelength}m, reindex={use_closest_point}")
    print("="*70)
    
    rms_linear = run_simulation(
        controller_type='linear', visualize=visualize, sim_time=sim_time,
        path_type=path_type, terrain_preset=terrain_preset,
        terrain_config=terrain_config, v_target=v_target, kappa_mode=kappa_mode,
        bump_amplitude=bump_amplitude, bump_wavelength=bump_wavelength,
        bump_octaves=bump_octaves, bump_seed=bump_seed, bump_max_slope=bump_max_slope,
        sine_amplitude=sine_amplitude, sine_wavelength=sine_wavelength,
        use_closest_point=use_closest_point,
    )
    
    rms_nn = run_simulation(
        controller_type='nn', visualize=visualize, sim_time=sim_time,
        path_type=path_type, terrain_preset=terrain_preset,
        terrain_config=terrain_config, v_target=v_target, kappa_mode=kappa_mode,
        bump_amplitude=bump_amplitude, bump_wavelength=bump_wavelength,
        bump_octaves=bump_octaves, bump_seed=bump_seed, bump_max_slope=bump_max_slope,
        sine_amplitude=sine_amplitude, sine_wavelength=sine_wavelength,
        use_closest_point=use_closest_point,
    )
    
    print("\n" + "="*70)
    print("COMPARISON RESULTS")
    print("="*70)
    
    if rms_linear is not None and rms_nn is not None:
        print(f"  Linear RMS: {rms_linear:.4f} m")
        print(f"  NN RMS:     {rms_nn:.4f} m")
        if rms_nn < rms_linear:
            improvement = (rms_linear - rms_nn) / rms_linear * 100
            print(f"\n  NN model is {improvement:.1f}% better!")
        else:
            worse = (rms_nn - rms_linear) / rms_linear * 100
            print(f"\n  Linear model is {worse:.1f}% better on this scenario")


# =============================================================================
# Simple Test Without Chrono
# =============================================================================

def test_mpc_only():
    """Test MPC controllers without Chrono"""
    from mpc_solver import test_dallas_mpc, test_with_nn
    
    test_dallas_mpc()
    test_with_nn()


# =============================================================================
# Entry Point
# =============================================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Dallas MPC PyChrono Demo")
    parser.add_argument('--linear', action='store_true', help='Run linear model only')
    parser.add_argument('--nn', action='store_true', help='Run NN model only')
    parser.add_argument('--both', action='store_true', help='Compare both models')
    parser.add_argument('--no-vis', action='store_true', help='Disable visualization')
    parser.add_argument('--time', type=float, default=15.0, help='Simulation time (s)')
    parser.add_argument('--path', type=str, default='lane_change',
                        choices=['lane_change', 'double_lane_change', 'sinusoidal'],
                        help='Path type: lane_change (easy), double_lane_change (medium), sinusoidal')
    parser.add_argument('--mpc-only', action='store_true', help='Test MPC without Chrono')
    parser.add_argument('--debug', action='store_true', help='Print tire force debug info')
    parser.add_argument('--nn-scale', type=float, default=1.0, help='Scale factor for NN forces (default: 1.0; use 0.5 if NN overpredicts)')
    parser.add_argument('--nn-sign', type=int, default=1, choices=[1, -1], help='Sign for NN forces (+1 or -1)')
    parser.add_argument('--kappa', type=str, default='zero', choices=['zero', 'approx'],
                        help='Longitudinal slip mode: zero (pure lateral) or approx (ax-based)')
    parser.add_argument('--sweep', action='store_true', help='Sweep through scale/sign combinations')
    parser.add_argument('--terrain-config', type=str, default=None, 
                        help='Path to YAML terrain config file (overrides --terrain)')
    parser.add_argument('--terrain', type=str, default='sand',
                        choices=['sand', 'clay', 'dirt'],
                        help='Terrain preset: sand (dry sand), clay (clayey soil), dirt (sandy loam)')
    parser.add_argument('--random-terrain', action='store_true', help='Random soil params within NN training range')
    parser.add_argument('--terrain-n', type=float, default=None, help='Override terrain sinkage exponent n (e.g. 1.0-1.4)')
    parser.add_argument('--ukf', action='store_true', help='Enable UKF terrain estimation (with --nn)')
    parser.add_argument('--ukf-n-init', type=float, default=None, help='UKF initial guess for sinkage exponent n (default: 0.7)')
    parser.add_argument('--ukf-version', type=str, default='v3', choices=['v2', 'v3'],
                        help='Estimator version: v2=actual UKF (7-state), v3=trajectory-matching grid search (default: v3)')
    parser.add_argument('--steer-excite', type=float, default=0.0,
                        help='Sinusoidal steering excitation amplitude in radians (recommended: 0.35 ~20deg). Overlays on MPC control for better terrain observability.')
    parser.add_argument('--steer-excite-freq', type=float, default=0.15,
                        help='Steering excitation frequency in Hz (default: 0.15, i.e. 6.7s period — smooth transitions)')
    parser.add_argument('--steer-excite-ramp', type=float, default=3.0,
                        help='Ramp-up time for excitation envelope in seconds (default: 3.0)')
    parser.add_argument('--speed', type=float, default=5.0, help='Target speed (m/s)')
    parser.add_argument('--async', dest='async_mpc', action='store_true', 
                        help='Run MPC in a separate thread (non-blocking)')
    parser.add_argument('--multiprocess', action='store_true',
                        help='Run MPC in separate PROCESS (bypasses GIL, true parallelism)')
    parser.add_argument('--no-lat-transfer', action='store_true',
                        help='Disable lateral load transfer (2 NN calls/step instead of 4, faster)')
    parser.add_argument('--nn-model', type=str, default='v3',
                        help='NN model version directory (default: v3, fast and accurate)')
    
    # Terrain bumpiness options
    parser.add_argument('--bump', type=float, default=0.0,
                        help='Terrain bump amplitude in meters (0 = flat, try 0.1-0.3)')
    parser.add_argument('--bump-wavelength', type=float, default=20.0,
                        help='Distance between bump peaks in meters (larger = gentler hills, default: 20)')
    parser.add_argument('--bump-octaves', type=int, default=4,
                        help='Noise octaves (more = more detail, default: 4)')
    parser.add_argument('--bump-seed', type=int, default=12345,
                        help='Random seed for bump terrain (default: 12345)')
    parser.add_argument('--bump-max-slope', type=float, default=0.3,
                        help='Max terrain slope (0.3 = 30%% grade, prevents too-steep terrain, default: 0.3)')
    
    # Sinusoidal path options
    parser.add_argument('--sine-amplitude', type=float, default=2.0,
                        help='Sinusoidal path amplitude in meters (default: 2.0)')
    parser.add_argument('--sine-wavelength', type=float, default=30.0,
                        help='Sinusoidal path wavelength in meters (default: 30.0, lower = tighter turns)')
    parser.add_argument('--no-path-reindex', action='store_true',
                        help='Disable closest-point path re-indexing (use original x-projection method)')
    
    # Manual control option
    parser.add_argument('--manual', action='store_true',
                        help='Manual control with G29 steering wheel (no MPC)')
    
    args = parser.parse_args()
    
    visualize = not args.no_vis
    
    terrain_config = None
    if args.terrain_n is not None:
        # Use specified n with sand base params from centralized presets
        from param_consistency import get_terrain_preset
        terrain_config = get_terrain_preset('sand')
        terrain_config['n'] = args.terrain_n
        print(f"Custom terrain: n={args.terrain_n:.2f} (sand base params)")
    elif args.random_terrain:
        import random
        from param_consistency import TRAINING_RANGES_V6 as TR
        import math
        terrain_config = {
            'Kphi': random.uniform(*TR['bekker_Kphi']),
            'Kc': random.uniform(*TR['bekker_Kc']),
            'n': random.uniform(*TR['bekker_n']),
            'cohesion': random.uniform(*TR['mohr_cohesion']),
            # mohr_friction in v6 ranges is radians; convert to degrees for terrain config
            'friction_angle': math.degrees(random.uniform(*TR['mohr_friction'])),
            'janosi_shear': random.uniform(*TR['janosi_shear']),
        }
        print(f"Random terrain: Kphi={terrain_config['Kphi']:.2e}, Kc={terrain_config['Kc']:.0f}, "
              f"n={terrain_config['n']:.2f}, c={terrain_config['cohesion']:.0f}, "
              f"phi={terrain_config['friction_angle']:.1f}°, k={terrain_config['janosi_shear']:.3f}")
    elif args.terrain_config:
        terrain_config = load_terrain_config(args.terrain_config)
        print(f"Loaded terrain config from: {args.terrain_config}")
    
    terrain_preset = args.terrain
    
    if args.mpc_only:
        test_mpc_only()
    elif args.sweep:
        # Sweep through scale/sign combinations
        configs = [
            (0.5, 1),
            (1.0, 1),
            (2.0, 1),
            (0.5, -1),
            (1.0, -1),
            (2.0, -1),
        ]
        results = []
        for scale, sign in configs:
            print(f"\n\n{'#'*70}")
            print(f"# Testing: scale={scale}, sign={sign}")
            print(f"{'#'*70}")
            rms = run_simulation('nn', visualize=visualize, sim_time=args.time, 
                                path_type=args.path, debug=False, 
                                nn_scale=scale, nn_sign=sign,
                                terrain_config=terrain_config, terrain_preset=terrain_preset)
            results.append((scale, sign, rms))
        
        print(f"\n\n{'='*70}")
        print("SWEEP RESULTS SUMMARY")
        print(f"{'='*70}")
        for scale, sign, rms in sorted(results, key=lambda x: x[2] if x[2] else 999):
            sign_str = '+' if sign == 1 else '-'
            rms_str = f"{rms:.3f}m" if rms else "FAIL"
            print(f"  scale={scale}, sign={sign_str}1: RMS={rms_str}")
    elif args.both:
        compare_controllers(visualize=visualize, sim_time=args.time, path_type=args.path,
                           terrain_config=terrain_config, terrain_preset=terrain_preset,
                           v_target=args.speed, kappa_mode=args.kappa,
                           bump_amplitude=args.bump, bump_wavelength=args.bump_wavelength,
                           bump_octaves=args.bump_octaves, bump_seed=args.bump_seed,
                           bump_max_slope=args.bump_max_slope,
                           sine_amplitude=args.sine_amplitude, sine_wavelength=args.sine_wavelength,
                           use_closest_point=not args.no_path_reindex)
    elif args.nn:
        run_simulation('nn', visualize=visualize, sim_time=args.time, path_type=args.path, 
                      debug=args.debug, nn_scale=args.nn_scale, nn_sign=args.nn_sign,
                      terrain_config=terrain_config, terrain_preset=terrain_preset,
                      v_target=args.speed, kappa_mode=args.kappa,
                      use_ukf=args.ukf, ukf_n_init=args.ukf_n_init,
                      ukf_version=args.ukf_version,
                      async_mpc=args.async_mpc, multiprocess_mpc=args.multiprocess,
                      nn_model_dir=args.nn_model,
                      bump_amplitude=args.bump, bump_wavelength=args.bump_wavelength,
                      bump_octaves=args.bump_octaves, bump_seed=args.bump_seed,
                      bump_max_slope=args.bump_max_slope,
                      sine_amplitude=args.sine_amplitude, sine_wavelength=args.sine_wavelength,
                      manual_control=args.manual,
                      use_closest_point=not args.no_path_reindex,
                      steer_excite_amp=args.steer_excite,
                      steer_excite_freq=args.steer_excite_freq,
                      steer_excite_ramp=args.steer_excite_ramp,
                      lateral_load_transfer=not args.no_lat_transfer)
    else:
        run_simulation('linear', visualize=visualize, sim_time=args.time, path_type=args.path, 
                      debug=args.debug, terrain_config=terrain_config, terrain_preset=terrain_preset,
                      v_target=args.speed, kappa_mode=args.kappa,
                      async_mpc=args.async_mpc, multiprocess_mpc=args.multiprocess,
                      bump_amplitude=args.bump, bump_wavelength=args.bump_wavelength,
                      bump_octaves=args.bump_octaves, bump_seed=args.bump_seed,
                      bump_max_slope=args.bump_max_slope,
                      sine_amplitude=args.sine_amplitude, sine_wavelength=args.sine_wavelength,
                      manual_control=args.manual,
                      use_closest_point=not args.no_path_reindex,
                      steer_excite_amp=args.steer_excite,
                      steer_excite_freq=args.steer_excite_freq,
                      steer_excite_ramp=args.steer_excite_ramp,
                      lateral_load_transfer=not args.no_lat_transfer)
