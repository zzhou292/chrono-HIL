# Neural Network Training

Scripts for training and validating neural networks that predict tire forces on SCM deformable terrain.

## Scripts

### `train_terrain_nn.py` - Train NN Tire Model

Train a neural network on SCM simulation data to predict lateral/longitudinal tire forces.

**Basic Usage:**
```bash
# Train with default architecture [12, 2] (legacy)
python train_terrain_nn.py --data /path/to/scm_training_data.csv

# Train with larger architecture for better accuracy
python train_terrain_nn.py --data /path/to/data.csv --hidden 64 32 --output_dir ../nn_models/new

# Train on GPU
python train_terrain_nn.py --data /path/to/data.csv --device cuda
```

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--data <path>` | *required* | Path to training data CSV |
| `--output_dir <path>` | `../nn_models/new` | Output directory for model files |
| `--epochs <n>` | 1000 | Maximum training epochs |
| `--batch_size <n>` | 128 | Batch size for training |
| `--lr <rate>` | 0.01 | Learning rate |
| `--patience <n>` | 50 | Early stopping patience |
| `--device <dev>` | cpu | Device: `cpu` or `cuda` |
| `--hidden <sizes>` | [12, 2] | Hidden layer sizes (e.g., `--hidden 64 32`) |

**Output Files:**
- `best_terrain_nn.pt` - Best model checkpoint
- `scalers.pkl` - Input/output scalers for normalization
- `training_history.json` - Loss history
- `test_metrics.json` - Final test metrics
- `training_results.png` - Training curves plot

---

### `validate_terrain_nn.py` - Validate Trained Model

Validate a trained model and generate diagnostic plots.

**Basic Usage:**
```bash
# Validate default v3 model
python validate_terrain_nn.py

# Validate specific model and data
python validate_terrain_nn.py --model ../nn_models/v2/best_terrain_nn.pt \
                              --scaler ../nn_models/v2/scalers.pkl \
                              --data /path/to/test_data.csv
```

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--model <path>` | `../nn_models/v3/best_terrain_nn.pt` | Path to trained model |
| `--scaler <path>` | `../nn_models/v3/scalers.pkl` | Path to scalers |
| `--data <path>` | - | Path to validation data CSV |
| `--output <file>` | `nn_validation.png` | Output plot filename |

---

### `diagnose_nn_accuracy.py` - NN Accuracy Diagnostics

Compare NN predictions against actual Chrono SCM forces to diagnose accuracy issues.

---

## Data Format

Training data CSV should have columns:
- `Fz` - Normal force (N)
- `slip_angle` - Tire slip angle (rad)
- `longitudinal_slip` - Longitudinal slip ratio
- `camber_angle` - Camber angle (rad)
- `velocity` - Wheel velocity (m/s)
- `bekker_Kphi`, `bekker_Kc`, `bekker_n` - Bekker soil parameters
- `mohr_cohesion`, `mohr_friction` - Mohr-Coulomb parameters
- `janosi_shear` - Janosi shear parameter
- `Fy` - Lateral force (target, N)
- `Fx` - Longitudinal force (target, N)

See [data_collection/README.md](../data_collection/README.md) for data generation tools:
- `collect_scm_data_fast` - Recommended (parallel, 5-10x faster)
- `collect_scm_data` - Original (supports visualization)

## NN Architecture

The default architecture uses tanh activation (C² continuous, required for IPOPT):
- Input: 11 features (Fz, slip angles, terrain params)
- Hidden: Configurable (legacy: [12, 2], recommended: [64, 32])
- Output: 2 (Fy, Fx)
