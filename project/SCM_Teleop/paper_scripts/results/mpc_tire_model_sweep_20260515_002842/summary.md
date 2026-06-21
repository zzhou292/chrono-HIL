# Standard MPC Tire-Model Sweep

Noise policy: sensor noise enabled in every run.
Raw per-run logs and diagnostic CSV files are under `raw/`.

```csv
variant,n_runs,n_ok,rms_cte_m_mean,rms_cte_m_std,mean_abs_cte_m_mean,mean_abs_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_speed_mps_mean,mean_speed_mps_std,mean_solve_ms_mean,mean_solve_ms_std,progress_m_mean,progress_m_std
pacejka,1,1,0.08800931113940161,,0.07659889999999998,,0.45099831999999995,,2.2549916,,2.2824437781109443,,14.420685418231594,
tmeasy,1,1,0.0724238622039311,,0.06055813000000001,,0.43074884,,2.1537442,,2.9844764795144156,,13.3828908675712,
closed_loop_mlp,1,1,0.07260934943539984,,0.060526576000000006,,0.53631688,,2.6815843999999998,,5.263163418290854,,17.417463909745397,
```