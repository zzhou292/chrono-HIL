# Terrain Configurations

YAML configuration files for SCM deformable terrain parameters.
NOTE: elastic_stiffness and damping are not part of original Bekker-Wong/Janosi parameters. These are Chrono specific and are held constant.

## Available Presets

### Soil-Only Presets (flat terrain)
| File | Description |
|------|-------------|
| `sand.yaml` | Dry sand - low cohesion, high friction, deep sinkage |
| `clay.yaml` | Wet clay - high cohesion, low friction, sticky |
| `dirt.yaml` | Packed dirt/gravel - moderate parameters |
| `training_mean.yaml` | Mean of NN training range - best for NN accuracy |

### Bumpiness + Soil Presets (terrain topology from Perlin noise)
| File | Bump Amp | Wavelength | Octaves | Max Slope | Description |
|------|----------|------------|---------|-----------|-------------|
| `smooth_road.yaml` | 0.0 m | — | 1 | 10% | Hard-packed flat surface |
| `gentle_gravel.yaml` | 0.03 m | 25 m | 2 | 10% | Slight undulations |
| `rolling_dirt.yaml` | 0.08 m | 18 m | 3 | 15% | Maintained dirt trail |
| `bumpy_field.yaml` | 0.15 m | 12 m | 4 | 20% | Agricultural field |
| `rough_trail.yaml` | 0.2 m | 10 m | 4 | 25% | Rough unpaved trail |
| `sandy_dunes.yaml` | 0.25 m | 25 m | 3 | 20% | Soft sand with dunes |
| `muddy_ruts.yaml` | 0.18 m | 8 m | 5 | 30% | Wet clay with ruts |
| `rocky_hillside.yaml` | 0.3 m | 8 m | 6 | 35% | Hard rocky terrain |
| `washboard.yaml` | 0.12 m | 5 m | 2 | 25% | Periodic corrugations |
| `extreme_offroad.yaml` | 0.35 m | 7 m | 6 | 40% | Maximum difficulty |

## Usage

```bash
# Use a terrain config in simulation
python simulation/dallas_chrono_demo.py --nn --terrain-config terrain_configs/clay.yaml
```

## Parameter Reference

Each YAML file specifies Bekker-Wong terramechanics parameters:

| Parameter | Units | Description |
|-----------|-------|-------------|
| `Kphi` | Pa/m^n | Friction modulus (higher = harder) |
| `Kc` | Pa/m^(n-1) | Cohesion modulus |
| `n` | - | Sinkage exponent (1.0-1.4 typical) |
| `cohesion` | Pa | Soil cohesion |
| `friction_angle` | degrees | Internal friction angle |
| `janosi_shear` | m | Shear displacement coefficient |
| `elastic_stiffness` | N/m | Contact stiffness (optional) |
| `damping` | Ns/m | Contact damping (optional) |

## Creating Custom Configs

```yaml
# Example: Custom mud terrain
Kphi: 1.5e6
Kc: 1000
n: 1.3
cohesion: 500
friction_angle: 25
janosi_shear: 0.04
elastic_stiffness: 2.0e8
damping: 3.0e4
```

## NN Training Range

For best NN tire model accuracy, keep parameters within:
- Kphi: 2.0e6 - 4.0e6
- Kc: 0 - 2.0e5
- n: 1.0 - 1.4
- cohesion: 0 - 60,000
- friction_angle: 15 - 45°
- janosi_shear: 0.001 - 0.02
