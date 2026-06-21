# Standard MPC Tire-Model Sweep

Noise policy: sensor noise enabled in every run.
Raw per-run logs and diagnostic CSV files are under `raw/`.

```csv
variant,n_runs,n_ok,rms_cte_m_mean,rms_cte_m_std,mean_abs_cte_m_mean,mean_abs_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_speed_mps_mean,mean_speed_mps_std,mean_solve_ms_mean,mean_solve_ms_std,progress_m_mean,progress_m_std
closed_loop_mlp,1,1,0.07331192060374629,,0.064136634,,0.4834516,,2.417258,,4.3950075872534144,,15.135552827355934,
```