# Tire model x live terrain estimator

Noise policy: sensor noise enabled in every run.
Estimator-off variants use static terrain params; estimator-on adds --terrain-estimator so n_terrain is re-conditioned online from IMU + wheel-speed signals.
Metric window starts after the estimator has had time to settle.

```csv
variant,n_runs,n_ok,rms_cte_m_mean,rms_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_speed_mps_mean,mean_speed_mps_std,mean_solve_ms_mean,mean_solve_ms_std
nn_v3_estimator,8,8,0.044116270320667554,0.01777225611834073,0.7274096442857142,0.13097982940353045,4.2419466749999994,0.01387595944510263,6.95364352129574,0.11803483709888626
```