# Script to generate static reference path CSVs for all path types
from pathlib import Path
from scm_hmmwv_demo import make_path_function

path_types = ["lane_change", "double_lane_change", "sinusoidal"]
v_target = 1.0  # Neutral speed, geometry only
sine_amplitude = 2.0
sine_wavelength = 30.0
lead_in = 0.0

csv_dir = Path(__file__).parent.parent / "paths"
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
