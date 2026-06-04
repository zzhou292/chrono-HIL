# `benchmarking/` — paper experiment suite

Every script in this folder tests one paper claim. Each run writes a
timestamped folder under `benchmarking/results/<prefix>_<ts>/` with:

- `manifest.csv` — exact command and sweep settings
- `results.csv` — one row per Chrono run
- `summary_*.csv` — aggregated KPIs
- `raw/<idx>_*/` — per-run logs, diagnostic CSVs, collision/shield CSVs
- `figures/` — PNG figures the paper picks up

Sensor noise is enabled by default in every script; do not add a
`--no-noise` for paper runs.

## Environment

```bash
source ~/miniconda3/etc/profile.d/conda.sh && conda activate sim
export ACADOS_SOURCE_DIR=~/Documents/sbel/acados
cd project/SCM_Final
```

## Orchestrator (preferred)

`run.py` is the single entry point that launches each sub-sweep with
the matrix appropriate for the chosen tier and then runs
`publish_paper_figures.py` at the end to refresh
`my_paper/paper_figures/`.

```bash
# Syntax + smoke (every sub-script runs one quick scenario)
python benchmarking/run.py --tier smoke

# Pilot (clay/dirt/sand x sinusoid/lane-change/right-left,
#        5/7/9 m/s, bumpiness 0/4, 2 seeds). ~6 hr on the 24-core box.
python benchmarking/run.py --tier pilot

# Paper (full broad matrix, 5 seeds). Multi-hour.
python benchmarking/run.py --tier paper

# One sweep at a time
python benchmarking/run.py --tier pilot --only safety
python benchmarking/run.py --tier pilot --only dob_cbf_ablation
```

The runner writes
`benchmarking/results/paper_suite_<tier>_<ts>/suite_manifest.csv`
with the exact sub-commands and estimated run counts; use `--dry-run`
to inspect before launching.

Human-in-the-loop rounds are intentionally not part of the
non-human suite — see below.

## Script map

| Script | Tests |
| --- | --- |
| `mpc_tire_model_sweep.py` | Standard NMPC closed-loop tracking vs tire model (Pacejka / TMeasy / Vehicle NN rate-MLP). Source of the master CTE heatmap in §III. |
| `safety_filter_sweep.py` | Planner-blind shield-only validation (none / DOB-CBF / MPPI / NMPC). `--blind-and-aware` repeats the matrix with the in-horizon NMPC barrier re-enabled. Source of §VI.A–§VI.C. |
| `dob_cbf_nn_ablation.py` | DOB-CBF with the neural tire surrogate vs a kinematic/linear fallback. Source of §VI.D. |
| `mppi_seed_ablation.py` | MPPI shield with / without hand-crafted recovery seeds. Source of §VI.E. |
| `autonomous_obstacle_tire_model_sweep.py` | Planner-tire-model sweep under a fixed MPPI shield, no separate filter ablation. Source of §VI.F. |
| `terrain_estimator_benchmark.py` | Live terrain estimator under closed-loop NMPC and Buzhardt-style scripted open-loop on canonical + 100 random LHS terrains. Source of §IV. |
| `open_loop_terrain_estimator_benchmark.py` | Estimator-only diagnostic with no NMPC in the loop. Used to isolate the role of deliberate command excitation (paper Fig. of the open-loop diagnostic). |
| `tire_model_with_estimator_ablation.py` | Live-estimator-conditioned closed-loop tracking comparison; outputs `tire_model_with_estimator_*` and feeds tab:tires_estimator. |
| `throttle_dob_ablation.py` | Asymmetric throttle DOB on vs off, fair-loop ablation. Source of §V. |
| `latency_profile_figure.py` | Generates the synthetic 5G latency trace and figures. Driven by `--profile-json` (default `config/latency_profiles/5g_nhits_youtube_ul_*.json`). |
| `latency_compensation_sweep.py` | Closed-loop obstacle scenario under the synthetic 5G profile with no-filter / DOB-CBF / MPPI. Source of §VII. |
| `collision_warning_test.py` | Forward collision warning lead-time sweep over terrain × one-way latency; vehicle drives blindly at a single rock. Source of `cw_lead_vs_terrain.png` and `cw_timeline.png`. |
| `brake_test.py` | 27 actual Chrono SCM brake stops used to validate the warning module's analytical `a_b(n̂)` table. Source of `cw_brake_validation.png` and the 0.17 m mean-absolute-error claim. |
| `train_5g_nhits.py` | Trains the N-HiTS 5G traffic model from the public YouTube uplink dataset. Re-running is optional; the suite ships with the trained checkpoint cache. |
| `rig_vs_vehicle_*.py` | Family of offline force-prediction and closed-loop comparisons used in §III.D. Most operate post-hoc on diag CSVs and do not launch new sweeps. |
| `human_delay_compensation_rounds.py` | Manual driving rounds across (filter, command-delay, terrain, path, speed, bumpiness, rounds). Operator at a Logitech G29 (or WASD for smoke). Source of paper §VI.A (HIL protocol). |

`publish_paper_figures.py` is not a sweep; it merges any new result
folders against the per-prefix history (deduplicated on the run key)
and overwrites `my_paper/paper_figures/` with the regenerated figures
and CSVs. The orchestrator runs it automatically at the end of each
invocation; you can also run it on its own after a manual sub-sweep.

## Human-in-the-loop rounds

`human_delay_compensation_rounds.py` is the only sweep that requires a
human driver. It does **not** participate in `--tier paper`.

```bash
# Default G29 protocol, symmetric link (camera delay = command delay)
python benchmarking/human_delay_compensation_rounds.py \
    --filters none mppi dob_cbf nmpc \
    --delays 0.0 0.15 0.30 \
    --rounds 3 --manual-mode g29

# Asymmetric 5G-style link (heavier video downlink)
python benchmarking/human_delay_compensation_rounds.py \
    --filters none dob_cbf mppi \
    --delays 0.0 0.15 0.30 \
    --camera-delay-scale 1.6 \
    --rounds 3 --manual-mode g29

# 8 s WASD smoke (no driver input; verifies the wiring only)
python benchmarking/human_delay_compensation_rounds.py --quick --auto-start
```

Each round delays both the operator command path
(`--manual-input-delay`) and the driver POV camera
(`--camera-input-delay = camera_delay_scale * delay`) and propagates
the command delay to the active filter as `--teleop-delay` so its
predictive horizon is delay-aware.

Per-round metrics aggregated per (filter, delay) cell:

* **safety**: `collisions` (unique obstacles hit), `near_misses`,
  `min_clearance_m`
* **intrusiveness**: `intervention_rate_pct`, `mean_abs_dsteer`,
  `mean_abs_dthrottle`
* **tracking**: `rms_cte_m`, `speed_ratio`

The script writes `summary_by_filter_delay.csv` with both mean and
std across the `--rounds` repeats and produces a 6-panel
`figures/human_delay_compensation_summary.png` (safety + intrusiveness
vs delay) plus a collision heatmap.
