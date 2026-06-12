# Paper Figures Index

**Regenerate everything with one command** (from the repo root, in the
`sim` conda env with `ACADOS_SOURCE_DIR` exported):

```bash
python benchmarking/make_paper_figures.py
```

This rebuilds every `\includegraphics` in `paper.tex` into this folder
with the exact filenames the paper uses, reporting `[ok]/SKIP/FAIL` per
figure. It does **not** re-run the Chrono sweeps — it re-plots from the
newest `benchmarking/results/*` folders (the §VI UKF estimator figures
re-run a short estimator pass over already-collected SCM logs; no
Chrono). Run the sweeps/benches first if the data is stale (see the
repo `README.md` "Reproducing the paper" and `AGENTS.md`).

**All figure generators now live in `benchmarking/`** (they were moved
out of `deliverables/` in the 2026-06-04 consolidation). The two
offline engines they call — `deliverables/ukf_paper_validation.py`
(state-augmented UKF) and `deliverables/cte_vehicle_static_vs_rate.py`
— remain in `deliverables/` as libraries.

## Generator per figure

| File | Paper | Generator (all under `benchmarking/`) |
| --- | --- | --- |
| `sys_arch.png` | Fig. 1 | `make_fig_sys_arch.py` (hand-laid diagram) |
| `cte_master_heatmap.png` | Fig. 2 | `make_fig_cte_master_heatmap.py` (newest `mpc_tire_model_sweep_*`) |
| `tire_model_with_estimator_rms_cte_heatmap.png` | Fig. 3 | `make_fig_tire_estimator_box.py` |
| `rig_vs_vehicle_paired_bars_v2.png` | Fig. 4 | `rig_vs_vehicle_tire_sweep.py` (copied to the `_v2` name by `make_paper_figures.py`) |
| `fig1_force_4way_dirt_sinusoidal_v7_b0_s900.png` | Fig. 5 | `make_fig1_4way.py` |
| `terrain_estimator_scatter.png` | Fig. 6 | `make_estimator_diag_figs.py` (closed-loop `terrain_estimator_benchmark_*`) |
| `open_loop_estimator_diagnostic.png` | Fig. 7 | `make_estimator_diag_figs.py` (open-loop benchmark) |
| `lhs100_fair.png` | Fig. 8 | `bench_terrain_estimators_lhs.py --replot-only --out-name lhs100_fair` |
| `estimator_overall.png` | Fig. 9 | `summarize_estimator_benchmark.py` (reads `lhs100_fair.csv` + `lhs100_cl.csv`) |
| `terrain_estimator_comparison.png` | Fig. 10 | `eval_terrain_estimators.py` (UKF replay of the 3 canonical SCM logs) |
| `throttle_dob_ablation_speed_heatmap.png` | Fig. 11 | `run.py --only throttle_dob_ablation` → `publish_paper_figures.py` |
| `safety_filter_collision_heatmap.png` | Fig. 13 | `run.py --only safety` → publish |
| `dob_cbf_nn_ablation_heatmap.png` | Fig. 14 | `run.py --only dob_cbf_ablation` → publish |
| `mppi_seed_ablation_collision_heatmap.png` | Fig. 15 | `run.py --only mppi_seed_ablation` → publish |
| `autonomous_obstacle_collision_heatmap.png` | Fig. 16 | `run.py --only autonomous_obstacle_tire` → publish |
| `latency_profile_histogram.png` | Fig. 17 | `latency_profile_figure.py` → publish |
| `latency_compensation_collision_heatmap.png` | Fig. 18 | `run.py --only latency_compensation` → publish |
| `cw_brake_validation.png` | Fig. 19 | `make_fig_cw_brake_validation.py` (builds the predicted-vs-measured table in-memory from the newest `brake_test_*`) |
| `cw_lead_vs_terrain.png` | Fig. 20 | `make_fig_collision_warning.py` (newest `collision_warning_*`) |
| `cw_timeline.png` | Fig. 21 | `make_fig_collision_warning.py` |

Sweep heatmaps (Fig. 11/13–18) are produced by each sub-sweep's
`plot_figures()` and copied here by `publish_paper_figures.py`, which
`make_paper_figures.py` invokes first.
