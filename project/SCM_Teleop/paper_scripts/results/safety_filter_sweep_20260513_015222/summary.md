# Safety Filter Sweep

Noise policy: sensor noise enabled in every run.
Safety buffer: 0.5 m beyond the hard collision footprint.
NMPC diagnostic angle: if NMPC loses to DOB-CBF, inspect raw `nmpc_shield_log.csv` and trajectories for local-minimum behavior, short-horizon braking, or insufficient geometric steering commitment. DOB-CBF has an explicit reactive steering layer that can win on head-on rocks even without long-horizon optimization.

```csv
variant,n_runs,n_ok,collisions_mean,collisions_std,near_misses_mean,near_misses_std,min_clearance_m_mean,min_clearance_m_std,intervention_rate_pct_mean,intervention_rate_pct_std,mean_abs_dsteer_mean,mean_abs_dsteer_std,mean_abs_dthrottle_mean,mean_abs_dthrottle_std,rt_factor_mean,rt_factor_std,rms_cte_m_mean,rms_cte_m_std
none_blind,4,4,1.5,0.5773502691896257,1.5,0.5773502691896257,-1.40455,0.582501138768558,,,,,,,1.0,0.0,0.21475431632668324,0.21313421480307826
dob_cbf_blind,4,4,0.0,0.0,0.5,0.5773502691896257,0.8512087500000001,0.6848065373808747,38.975,8.487785341300757,,,,,1.0,0.0,2.672522403543591,1.2318864074667952
mppi_blind,4,4,0.0,0.0,1.5,0.5773502691896257,0.0481000000000002,0.05438474050687373,61.025,20.484852127039296,0.23349788324420678,0.10259249720037204,0.110275419637552,0.052311088066414844,1.0,0.0,1.3275725556594513,0.8480583716179421
nmpc_blind,4,4,0.5,0.5773502691896257,1.5,0.5773502691896257,0.023050000000000126,0.23237920876590198,58.075,21.442384040337803,0.5832394921437825,0.09880149579472605,0.11397368563673696,0.06484605178021473,0.8525,0.09742518497116985,2.3669345351264375,1.0103336794907445
```