# Paper Experiment Scripts

Each script tests one question and writes a timestamped folder under
`paper_scripts/results/` containing:

- `manifest.csv`: exact command and sweep settings
- `results.csv`: one row per Chrono run
- `summary_*.csv`: aggregate paper KPIs
- `raw/`: per-run logs, controller diagnostic CSVs, collision/shield CSVs
- `figures/`: PNG figures and heatmaps for paper/table drafting

Sensor noise is enabled by default in all scripts; do not add `--no-noise`
for paper runs.

Run from `project/SCM_Teleop` inside the `sim` conda environment:

```bash
source /home/ksha/miniconda3/etc/profile.d/conda.sh
conda activate sim
export ACADOS_SOURCE_DIR=/home/ksha/Documents/sbel/acados

python paper_scripts/mpc_tire_model_sweep.py --quick
python paper_scripts/mpcc_vs_mpc_speed_tracking.py --quick
python paper_scripts/safety_filter_sweep.py --quick
python paper_scripts/dob_cbf_nn_ablation.py --quick
python paper_scripts/autonomous_obstacle_tire_model_sweep.py --quick
python paper_scripts/terrain_estimator_benchmark.py --quick
python paper_scripts/human_delay_compensation_rounds.py --dry-run --quick
```

For repeatable non-human paper results, prefer the suite runner instead of
typing matrices by hand:

```bash
# Syntax/health check only
python paper_scripts/run_paper_suite.py --tier smoke --dry-run

# Manageable high-speed pilot: clay/sand, sinusoid/lane-change,
# 5 and 7 m/s, bumpiness 0/4, 2 seeds.
python paper_scripts/run_paper_suite.py --tier pilot

# Final broad matrix: clay/dirt/sand, all reference paths,
# 5/7/9 m/s, bumpiness 0/4/8, 5 seeds.
python paper_scripts/run_paper_suite.py --tier paper

# High-speed rough-terrain stress subset.
python paper_scripts/run_paper_suite.py --tier stress
```

Use `--dry-run` first.  The runner writes
`paper_scripts/results/paper_suite_<tier>_<timestamp>/suite_manifest.csv`
with the exact commands and estimated Chrono run counts.  Human-in-the-loop
rounds are deliberately not included in this non-human suite.

Full paper sweeps are intentionally broad across terrains, paths, 5--9 m/s
speeds, bumpiness, and seeds, so expect them to take a while.

Current script map:

- `mpc_tire_model_sweep.py`: standard MPC tracking versus tire model.
- `mpcc_vs_mpc_speed_tracking.py`: MPCC versus standard MPC, including
  standard-MPC soft-speed and no-speed ablations for turn behavior.
- `safety_filter_sweep.py`: no-filter, DOB-CBF, MPPI, and NMPC safety
  filters with obstacle avoidance.
- `dob_cbf_nn_ablation.py`: DOB-CBF with and without NN tire usage.
- `autonomous_obstacle_tire_model_sweep.py`: obstacle-aware autonomous MPC
  without a downstream safety filter, swept over tire models.
- `terrain_estimator_benchmark.py`: learned terrain estimator on canonical
  and generated out-of-distribution SCM soils.
- `human_delay_compensation_rounds.py`: one-at-a-time human driving rounds
  across delay, path, terrain, bumpiness, and safety-filter settings.
- `run_paper_suite.py`: orchestration only; launches the one-question scripts
  with explicit repeatable matrices.
