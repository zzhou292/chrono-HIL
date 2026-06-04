#!/usr/bin/env python3
"""
Perlin Noise Terrain Heightmap Generator
=========================================

Procedural terrain heightmap generation using Perlin noise.
Port of the C++ implementation from proj_HIL_scm_teleop.cpp.
"""


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
