# Script to generate static reference path CSVs for all path types
import sys
from pathlib import Path

_UTIL_DIR = Path(__file__).parent
_PROJECT_ROOT = _UTIL_DIR.parent
_SIM_DIR = _PROJECT_ROOT / "simulation"
sys.path.insert(0, str(_SIM_DIR))

from path_utils import make_path_function

path_types = ["lane_change", "double_lane_change", "right_left", "sinusoidal"]
v_target = 1.0  # Neutral speed, geometry only
sine_amplitude = 2.0
sine_wavelength = 30.0
lead_in = 0.0

csv_dir = _PROJECT_ROOT / "paths"
csv_dir.mkdir(parents=True, exist_ok=True)

for path_type in path_types:
    print(f"Generating {path_type}...")
    ref_path = make_path_function(
        path_type=path_type,
        v_target=v_target,
        sine_amplitude=sine_amplitude,
        sine_wavelength=sine_wavelength,
        lead_in=lead_in,
        csv_dir=str(csv_dir.resolve()),
    )
    print(f"  Saved to: {csv_dir.resolve()}/reference_path_{path_type}.csv")
print("All reference paths generated.")
import sys; sys.exit(0)
