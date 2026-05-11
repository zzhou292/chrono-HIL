#!/usr/bin/env python3
"""Compare rig-trained NN vs vehicle-trained NN predictions for the typical
operating range observed during dirt/sand sims, sweeping n through the preset
manifold."""

import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from terrain_parameter_estimator import TerrainParameterEstimator, _terrain_params_for_n


def evaluate(model_dir, label):
    est = TerrainParameterEstimator(model_dir=str(model_dir))
    print(f"\n=== {label} ===")
    # Representative ops from dirt sinusoidal: u=8.8, alpha small to moderate
    for alpha_f in (0.03, 0.05, 0.08, 0.12, 0.18):
        for Fz in (5500.0, 6500.0, 7500.0):
            print(f"\nalpha_f={alpha_f:.3f}, Fz={Fz:.0f}")
            print(f"{'n':>6}  {'fy(n)':>10}")
            for n in [0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2]:
                _, fy = est._nn_FxFy(0.05, alpha_f, 8.5, Fz, 0.0, n)
                print(f"{n:6.2f}  {fy:10.1f}")


root = Path(__file__).parent.parent / "nn_models"
evaluate(root / "paper_v2_mlp_16_4", "rig-trained paper_v2_mlp_16_4")
evaluate(root / "vehicle_mlp_64_32", "vehicle-trained vehicle_mlp_64_32")
