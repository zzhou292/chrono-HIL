# Standard MPC Tire-Model Sweep

Noise policy: sensor noise enabled in every run.
Raw per-run logs and diagnostic CSV files are under `raw/`.

```csv
variant,n_runs,n_ok,rms_cte_m_mean,rms_cte_m_std,mean_abs_cte_m_mean,mean_abs_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_speed_mps_mean,mean_speed_mps_std,mean_solve_ms_mean,mean_solve_ms_std,progress_m_mean,progress_m_std
pacejka,1,1,0.08564459563224057,,0.07392889400000001,,0.43135976000000004,,2.1567988000000002,,2.9329438543247344,,13.431156345235706,
tmeasy,1,1,0.0797571369389975,,0.067593738,,0.44125820000000004,,2.2062910000000002,,3.133171471927162,,13.731476684330422,
closed_loop_mlp,1,1,0.06850786093465479,,0.058234834000000006,,0.47073980000000004,,2.353699,,4.405339366515837,,14.663208368224428,
```