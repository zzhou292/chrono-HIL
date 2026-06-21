# Safety Filter Sweep

Noise policy: sensor noise enabled in every run.
NMPC diagnostic angle: if NMPC loses to DOB-CBF, inspect raw `nmpc_shield_log.csv` and trajectories for local-minimum behavior, short-horizon braking, or insufficient geometric steering commitment. DOB-CBF has an explicit reactive steering layer that can win on head-on rocks even without long-horizon optimization.

```csv
variant,n_runs,n_ok,collisions_mean,collisions_std,near_misses_mean,near_misses_std,min_clearance_m_mean,min_clearance_m_std,intervention_rate_pct_mean,intervention_rate_pct_std,mean_abs_dsteer_mean,mean_abs_dsteer_std,mean_abs_dthrottle_mean,mean_abs_dthrottle_std,rt_factor_mean,rt_factor_std,rms_cte_m_mean,rms_cte_m_std
none_blind,4,4,1.5,0.5773502691896257,1.5,0.5773502691896257,-1.4265499999999998,0.6007437862072872,,,,,,,1.0,0.0,0.16461178985444355,0.10618547255386625
dob_cbf_blind,4,4,0.0,0.0,1.25,0.9574271077563381,0.80755975,0.6202103868421719,43.875,20.105948539341945,,,,,1.0,0.0,2.44978895867131,0.7898729390886421
mppi_blind,4,4,1.25,0.5,1.5,0.5773502691896257,-0.44482499999999986,0.037588950060711585,57.075,16.25041025123161,0.18567857142857144,0.0802247749529467,0.0890970238095238,0.03779082114222885,0.9975,0.005000000000000023,0.9457868569961929,0.7865847306542805
nmpc_blind,4,4,1.25,0.9574271077563381,1.5,0.5773502691896257,-0.36914999999999987,0.3547512790674615,58.6,12.655170221428605,0.5150070704064453,0.18138090159935208,0.15304413352272728,0.13935974528343992,0.7225,0.19906029237394382,2.007731976070265,1.5984706943497162
```