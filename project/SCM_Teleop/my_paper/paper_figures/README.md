# Paper figures — index (2026-05-16)

Every figure below is regenerated from `paper_scripts/results/` by
`python paper_scripts/publish_paper_figures.py`. The single exception is
`closed_loop_estimator_learned.png`, which is the per-terrain time-series
overwrite from `paper_scripts/regenerate_estimator_timeseries.py` —
matches the abstract caption that the auto-published bar chart did not.

## Currently referenced by `my_paper/abstract.tex`

| File | Abstract reference | Status |
| --- | --- | --- |
| `Sys Arch v4.drawio.png` | Fig. 1 left | manual diagram, not auto-generated |
| `bench_tire_models.png` | Fig. 1 right (NMPC tracking by tire model) | live, regenerated from latest tire-model sweep |
| `closed_loop_estimator_learned.png` | Fig. 2 (n̂ convergence) | live time-series for clay / sand / OOD terrain1 |

## Headline ablation figures (recommended for the paper body)

| File | Question | Conclusion |
| --- | --- | --- |
| `bench_tire_models.png` + `bench_tire_models_heatmap.png` | RMS CTE / speed / runtime by tire model (static terrain) | v3 NN best (0.083 m), Pacejka worst (0.104 m); not order of magnitude |
| `tire_model_with_estimator_summary.png` + `..._rms_cte_heatmap.png` | Same comparison with live terrain estimator on | v3 NN + estimator 0.208 m vs Pacejka 0.350 m, TMeasy 0.431 m (40-52 % gap) |
| `throttle_dob_ablation_summary.png` | Asymmetric throttle DOB on vs off | DOB closes 10 % of the speed gap with no tracking cost |
| `mppi_seed_ablation_summary.png` + `..._collision_heatmap.png` | MPPI shield seed trajectories on vs off | Seeds are essential — 40x collision rate without |
| `safety_filter_summary.png` + `safety_filter_collision_heatmap.png` | DOB-CBF vs MPPI vs NMPC vs none (planner blind) | DOB-CBF 0.03 coll/run wins; MPPI 0.09; NMPC 0.66; none 1.97 |
| `safety_filter_planner_aware_summary.png` | Same shields with NMPC barriers also on | Every shield improves; full stack 0 collisions |
| `dob_cbf_nn_ablation_summary.png` + `..._heatmap.png` | DOB-CBF with vs without NN tire model | NN-on 0 coll, NN-off 1 coll/run -- NN clearly helps |
| `closed_loop_estimator_learned.png` + `closed_loop_estimator_true_vs_estimated.png` + `closed_loop_estimator_error_heatmap.png` | Online terrain estimator on ID and OOD | clay |err| 0.005, sand 0.079, OOD mean 0.098 (terrain3 extrapolation gap 0.220) |
| `latency_compensation_summary.png` + `..._collision_heatmap.png` | Shield robustness under learned 5G traffic profile | DOB-CBF 0.22 coll/run, MPPI 0.19, none 1.81 -- both shields cut ~85 % |
| `latency_profile_timeseries.png` + `latency_profile_histogram.png` | The learned N-HiTS 5G profile itself | mean control 57 ms, camera 90 ms; bursty up to 660 ms |
| `autonomous_obstacle_tire_summary.png` + heatmaps | Tire model effect on autonomous obstacle avoidance under MPPI shield | Tire choice does not change shield-mediated outcome |

## Counter-evidence figures (use to justify text changes, not the headline claim)

| File | What it disproves | Recommended action |
| --- | --- | --- |
| `mpcc_vs_mpc_summary.png` + `mpcc_vs_mpc_pareto.png` | "MPCC improves the speed-tracking tradeoff" -- standard MPC dominates on CTE | Either de-emphasize MPCC or reframe as a controller-design ablation |
| `sigma_gate_ablation_summary.png` + `..._collision_heatmap.png` | "Ensemble disagreement on phi gates the shield's friction cone" -- forwarding terrain (with or without sigma) to the shield underperforms ignoring it | Drop the ensemble-phi-gating sentence from the abstract |

## Supporting CSVs in this folder

Each `*_results.csv` is the merged multi-folder dataset that drives the
figure of the same prefix. Each `*_summary.csv` is the aggregated
per-variant mean. Both regenerate when the sweeps re-run.

## Older (pre-suite) figures still in this folder

Kept for reference until the abstract no longer cites them. None are
currently referenced by `abstract.tex`:

* `dynamics_gp_revalidation.png`, `gp_paper_figure.png` -- old GP work
* `estimator_comparison.png`, `learned_generalization*.png`,
  `random_terrain_closed_loop_learned.png` -- earlier learned-estimator
  experiments, superseded by `closed_loop_estimator_*` above
* `exp_joint_n_phi_*` -- joint (n, phi) regression figures from
  `utilities/exp_joint_n_phi.py`. The abstract quotes "RMSE ~ 0.07 in
  n, ~ 3.3 deg in phi" from these; the underlying trace data is no
  longer on disk so the numbers come from a prior run. Re-collect via
  `utilities/collect_diverse_terrains.py` if you need to refresh.
* `ukf_*` -- legacy UKF estimator (parked per AGENT.md)

## How to refresh everything

```bash
source /home/ksha/miniconda3/etc/profile.d/conda.sh && conda activate sim
export ACADOS_SOURCE_DIR=/home/ksha/Documents/sbel/acados
cd /home/ksha/Documents/sbel/chrono_hil/chrono-HIL/project/SCM_Teleop

# 1. Re-publish from existing results (no Chrono runs needed)
python paper_scripts/publish_paper_figures.py
python paper_scripts/regenerate_estimator_timeseries.py

# 2. Re-run a single sweep then re-publish
python paper_scripts/run_paper_suite.py --tier pilot \
    --only sigma_gate_ablation     # or any other sweep name

# 3. Full pilot (~6 hr wall)
python paper_scripts/run_paper_suite.py --tier pilot
```
