# Paper Table Value Provenance

Current as of 2026-06-26. Paths are relative to `project/SCM_Final`.

This sheet is a source map for the numerical tables in
`my_paper/paper.tex`. For a fresh submission pass, regenerate the publish
artifacts first:

```bash
python benchmarking/publish_paper_figures.py
python benchmarking/plot_compact_heatmaps.py
```

If the underlying Chrono data are stale, run the relevant sweep before
publishing. The main autonomous paper tier is driven by:

```bash
python benchmarking/run.py --tier paper
```

## Numbered Tables

| Paper label | Table content | Backing source | Extraction rule / note |
| --- | --- | --- | --- |
| `tab:swap_points` | Six extension points | `simulation/framework/protocols.py`, `simulation/framework/builtins.py`, `simulation/framework/test_conformance.py` | Descriptive/code-backed. Re-run `python simulation/framework/test_conformance.py` after registry edits. |
| `tab:training_data` | Learned-model training provenance | `README.md`, `simulation/param_consistency.py`, `nn_training/`, `data_collection/` | Descriptive/model-metadata table. Verify against retained checkpoints and training metadata, not a KPI CSV. |
| `tab:tires_static` | Static tire-model benchmark | `my_paper/paper_figures/bench_tire_models_summary.csv`; source folder in `paper_figures/publish_manifest.json` under `mpc_tire_model_sweep` | Use mean/std columns for RMS CTE, speed ratio, and solve time. |
| `tab:tires_estimator` | Live-estimator-conditioned tire-model benchmark | `my_paper/paper_figures/tire_model_with_estimator_results.csv`; source folder in manifest under `tire_model_with_estimator_ablation` | Medians of `rms_cte_m` over `status == ok` rows with valid KPI values. Main source excludes the separate `nn_wrong_prior` mini-run. |
| `tab:wrong_prior` | Wrong static-prior ablation | `benchmarking/results/tire_model_with_estimator_ablation_20260614_213058/results.csv` | Mean `rms_cte_m` by variant and terrain, filtered to `terrain in {clay,sand}`, `path == sinusoidal`, `speed_mps in {5,7}`, `bumpiness == 0`, and `status == ok`. |
| `tab:tires_rig_vs_vehicle` | Controlled rig-vs-whole-vehicle sweep | `my_paper/paper_figures/rig_vs_vehicle_summary_paired.csv` | Use the paired summary rows by signature and generation. |
| `tab:feature_headroom` | Per-axle feature headroom | `my_paper/paper_figures/feature_headroom.csv` | Uses `deployed (rate)` and `+d_v,d_yaw (full)` rows rounded to table precision. |
| `tab:estimator_pilot` | Proprioceptive estimator pilot | `my_paper/paper_figures/closed_loop_estimator_summary.csv` | Use `id` and `ood` rows, mean/median tail absolute error. |
| `tab:estimator_lhs100` | 100-soil terrain-estimator head-to-head | `my_paper/paper_figures/lhs100_fair.csv` and `my_paper/paper_figures/estimator_overall.csv` | Use closed-loop/live summary values for median absolute error, median percent error, and within-20-percent rate. |
| `tab:throttledob` | Throttle DOB ablation | `my_paper/paper_figures/throttle_dob_ablation_summary.csv` | Use `dob_off` and `dob_on` rows; report mean/std RMS CTE, mean speed/profile, and mean speed. |
| `tab:ffdrag` | Feedforward sinkage-drag vs DOB | `benchmarking/results/ff_drag_ablation_20260614_132406/results.csv` | Mean `speed_ratio` by terrain and variant, filtered to `status == ok`; table uses `off`, `ffdrag`, and `dob`. |
| `tab:safety_blind` | Safety-filter-only validation | `my_paper/paper_figures/safety_filter_summary.csv` | Use `none_blind` and `dob_cbf_blind` summary rows; published CSV is filtered to active no-filter/DOB-CBF variants. |
| `tab:safety_aware` | Planner-aware vs planner-blind safety sweep | `my_paper/paper_figures/safety_filter_planner_aware_summary.csv` | Use active rows `none_blind`, `none_aware`, `dob_cbf_blind`, and `dob_cbf_aware`. |
| `tab:dobcbf_nn` | DOB-CBF neural-surrogate ablation | `my_paper/paper_figures/dob_cbf_nn_ablation_summary.csv` | Use summary rows copied from `benchmarking/results/dob_cbf_nn_ablation_20260621_044716/summary_by_variant.csv`. |
| `tab:auto_obstacle` | Planner tire model under fixed DOB-CBF | `my_paper/paper_figures/autonomous_obstacle_summary.csv` | Use summary rows copied from `benchmarking/results/autonomous_obstacle_tire_model_sweep_dob_cbf_mpc_blind_20260621_052924/summary_by_model.csv`. |
| `tab:convoy_cf` | Counterfactual convoy replay | `benchmarking/results/convoy_counterfactual_eval_20260621_231532/summary.csv` | Use `none` and `dob_cbf` rows. Note denominator difference: 45 unfiltered baseline cells and 44 completed DOB-CBF replays; prevented count is 31/39 matched baseline collisions. |
| `tab:latency_comp` | Learned 5G latency compensation sweep | `my_paper/paper_figures/latency_compensation_summary.csv` | Published CSV is filtered to active `none` and `dob_cbf` rows only. |
| `tab:cw_lead` | Forward-collision-warning RED lead time | `benchmarking/results/collision_warning_20260604_014259/results.csv` | Use `lead_red_s` by terrain and latency; values are rounded to two decimals in the paper. |

## Inline Tabular Blocks

The small unlabeled force-replay table after `tab:tires_rig_vs_vehicle` is
supporting evidence for the rig-vs-vehicle discussion. It is not currently
published as a standalone summary CSV; before using it as a headline claim,
recompute it from the replay script that generates `fig1_force_4way_*` and
archive the resulting CSV beside the paper figures.

## Known Caveats

- Some result CSVs include a small number of non-`ok` or blank-KPI rows. The
  paper captions now describe scheduled matrix size where appropriate and use
  valid KPI rows for medians.
- Human-in-the-loop rounds are intentionally outside the automated paper tier;
  HIL figures/protocol demonstrate capability, not a human-subjects safety
  comparison.
- Archived MPPI/SLSQP-NMPC rows exist in old `benchmarking/results/*` folders.
  Current published safety and latency artifacts filter to no-filter and
  DOB-CBF variants only.
