# MPPI Seed-Trajectory Ablation

Noise policy: sensor noise enabled in every run.
Safety buffer: 0.5 m beyond the hard collision footprint.
Planner-blind MPC: shield is the sole collision avoider.
Variant notes: mppi_with_seeds: Default MPPI shield with all hand-crafted seed trajectories enabled.; mppi_no_seeds: MPPI shield with seed trajectories disabled; pure Gaussian sampling around operator command.

```csv
variant,n_runs,n_ok,collisions_mean,collisions_std,near_misses_mean,near_misses_std,min_clearance_m_mean,min_clearance_m_std,intervention_rate_pct_mean,intervention_rate_pct_std,mean_abs_dsteer_mean,mean_abs_dsteer_std,mean_abs_dthrottle_mean,mean_abs_dthrottle_std,rms_cte_m_mean,rms_cte_m_std
mppi_with_seeds,1,1,0.0,,0.0,,3.246725,,23.5,,0.3113181818181818,,0.10563636363636363,,0.1257742499636793,
mppi_no_seeds,1,1,0.0,,0.0,,5.473635,,14.8,,0.0536,,0.031266666666666665,,0.09561825608664191,
```