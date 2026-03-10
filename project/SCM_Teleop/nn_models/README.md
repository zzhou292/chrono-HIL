# Neural Network Models

Trained neural network models for tire force prediction on SCM deformable terrain.

## Model Versions

| Version | Directory | Architecture | Training Data | Notes |
|---------|-----------|--------------|---------------|-------|
| legacy | `legacy/` | [12, 2] | Original | Small, fast |
| fast | `fast/` | [12, 2] | Updated | Optimized |
| v2 | `v2/` | [32, 16] | Expanded | Better accuracy |
| **v3** | `v3/` | [64, 32] | 50k samples | **Recommended** |

## Files in Each Version

| File | Description |
|------|-------------|
| `best_terrain_nn.pt` | PyTorch model checkpoint |
| `scalers.pkl` | Input/output normalization scalers |
| `training_history.json` | Training loss history |
| `test_metrics.json` | Final test set metrics |
| `training_results.png` | Training curves visualization |

## Usage

### In Simulation
```bash
# Use default (v3) model
python simulation/dallas_chrono_demo.py --nn

# Use specific version
python simulation/dallas_chrono_demo.py --nn --nn-model v2
```

### Loading Manually
```python
import torch
import pickle
from pathlib import Path

model_dir = Path("nn_models/v3")
checkpoint = torch.load(model_dir / "best_terrain_nn.pt")

with open(model_dir / "scalers.pkl", "rb") as f:
    scalers = pickle.load(f)
```

## Training New Models

See `nn_training/README.md` for instructions on training new models.

New models should be saved to `nn_models/new/` or a versioned directory.
