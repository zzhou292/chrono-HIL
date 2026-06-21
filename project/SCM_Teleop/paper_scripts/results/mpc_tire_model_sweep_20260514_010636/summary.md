# Standard MPC Tire-Model Sweep

Noise policy: sensor noise enabled in every run.
Raw per-run logs and diagnostic CSV files are under `raw/`.

```csv
variant,n_runs,n_ok,rms_cte_m_mean,rms_cte_m_std,mean_abs_cte_m_mean,mean_abs_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_speed_mps_mean,mean_speed_mps_std,mean_solve_ms_mean,mean_solve_ms_std,progress_m_mean,progress_m_std
closed_loop_v2_rate_mlp,1,1,0.07220837992965913,,0.060421161999999994,,0.5303852800000001,,2.6519264000000002,,5.222008995502248,,17.236910949405036,
```