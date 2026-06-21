# Safety Filter Sweep

Noise policy: sensor noise enabled in every run.
NMPC diagnostic angle: if NMPC loses to DOB-CBF, inspect raw `nmpc_shield_log.csv` and trajectories for local-minimum behavior, short-horizon braking, or insufficient geometric steering commitment. DOB-CBF has an explicit reactive steering layer that can win on head-on rocks even without long-horizon optimization.

```csv
variant,n_runs,n_ok,collisions_mean,collisions_std,near_misses_mean,near_misses_std,min_clearance_m_mean,min_clearance_m_std,intervention_rate_pct_mean,intervention_rate_pct_std,mean_abs_dsteer_mean,mean_abs_dsteer_std,mean_abs_dthrottle_mean,mean_abs_dthrottle_std,rt_factor_mean,rt_factor_std,rms_cte_m_mean,rms_cte_m_std
none_blind,1,1,0.0,,0.0,,,,,,,,,,1.0,,0.13364286179194065,
dob_cbf_blind,1,1,0.0,,0.0,,,,0.0,,,,,,1.0,,0.11351075176414013,
mppi_blind,1,1,0.0,,0.0,,,,0.0,,0.0,,0.0,,1.0,,0.1307148457264943,
nmpc_blind,1,1,0.0,,0.0,,,,0.0,,0.0,,0.0,,1.0,,0.12723331654171727,
```