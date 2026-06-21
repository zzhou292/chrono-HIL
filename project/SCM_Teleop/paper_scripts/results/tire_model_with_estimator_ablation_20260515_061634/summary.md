# Tire model x live terrain estimator

Noise policy: sensor noise enabled in every run.
Estimator-off variants use static terrain params; estimator-on adds --terrain-estimator so n_terrain is re-conditioned online from IMU + wheel-speed signals.
Metric window starts after the estimator has had time to settle.

```csv
variant,n_runs,n_ok,rms_cte_m_mean,rms_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_speed_mps_mean,mean_speed_mps_std,mean_solve_ms_mean,mean_solve_ms_std
pacejka_static,1,1,0.14140729098075713,,0.7904311111111111,,3.9521555555555556,,2.7171686746987955,
tmeasy_static,1,1,0.11546453943136004,,0.7692423423423425,,3.846211711711712,,3.1411794354838714,
nn_v3_static,1,1,0.05586327604578492,,0.8129799999999999,,4.0649,,7.00539156626506,
nn_v3_estimator,1,1,0.042306614310038786,,0.856871831831832,,4.28435915915916,,6.988300000000001,
```