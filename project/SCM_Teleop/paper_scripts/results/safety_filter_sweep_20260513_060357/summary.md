# Safety Filter Sweep

Noise policy: sensor noise enabled in every run.
Safety buffer: 0.5 m beyond the hard collision footprint.
NMPC diagnostic angle: if NMPC loses to DOB-CBF, inspect raw `nmpc_shield_log.csv` and trajectories for local-minimum behavior, short-horizon braking, or insufficient geometric steering commitment. DOB-CBF has an explicit reactive steering layer that can win on head-on rocks even without long-horizon optimization.

```csv
variant,n_runs,n_ok,collisions_mean,collisions_std,near_misses_mean,near_misses_std,min_clearance_m_mean,min_clearance_m_std,intervention_rate_pct_mean,intervention_rate_pct_std,mean_abs_dsteer_mean,mean_abs_dsteer_std,mean_abs_dthrottle_mean,mean_abs_dthrottle_std,rt_factor_mean,rt_factor_std,rms_cte_m_mean,rms_cte_m_std
none_blind,8,8,1.5,0.5345224838248488,1.75,0.8864052604279183,-1.6884374999999998,0.5266879556991705,,,,,,,1.0,0.0,0.14280463796130224,0.08759382606519096
dob_cbf_blind,8,8,0.0,0.0,0.875,0.6408699444616557,0.9246032500000001,0.4395964525892712,47.55,16.87618778888509,,,,,1.0,0.0,2.449445075118645,0.872461824808024
mppi_blind,8,8,0.0,0.0,1.75,0.7071067811865476,0.11835000000000007,0.13767835186197064,61.512499999999996,14.543180778043606,0.3140092070125251,0.1849912351463128,0.1698118576658159,0.14909108903878912,1.0,0.0,1.696906258356324,1.0461348832267316
```