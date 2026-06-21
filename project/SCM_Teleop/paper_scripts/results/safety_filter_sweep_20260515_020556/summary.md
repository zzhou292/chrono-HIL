# Safety Filter Sweep

Noise policy: sensor noise enabled in every run.
Safety buffer: 0.5 m beyond the hard collision footprint.
NMPC diagnostic angle: if NMPC loses to DOB-CBF, inspect raw `nmpc_shield_log.csv` and trajectories for local-minimum behavior, short-horizon braking, or insufficient geometric steering commitment. DOB-CBF has an explicit reactive steering layer that can win on head-on rocks even without long-horizon optimization.

```csv
variant,n_runs,n_ok,collisions_mean,collisions_std,near_misses_mean,near_misses_std,min_clearance_m_mean,min_clearance_m_std,intervention_rate_pct_mean,intervention_rate_pct_std,mean_abs_dsteer_mean,mean_abs_dsteer_std,mean_abs_dthrottle_mean,mean_abs_dthrottle_std,rt_factor_mean,rt_factor_std,rms_cte_m_mean,rms_cte_m_std
none_blind,32,32,1.96875,0.8607714094664382,2.625,1.2636353087613479,-1.668253125,0.4934701533507208,,,,,,,1.0,0.0,0.20367659706603658,0.3420497863279834
dob_cbf_blind,32,32,0.03125,0.17677669529663687,1.28125,0.8125775397243737,1.0115018125000002,0.9725920125952239,56.16875,16.2453653440386,,,,,1.0,0.0,3.105607276528526,1.5700257162073783
mppi_blind,32,32,0.09375,0.296144581080299,2.28125,0.6342063874918669,0.05833750000000004,0.07729755139467812,69.565625,10.498620781152711,0.379910745009679,0.15992647333489016,0.215111919888567,0.13667888368869618,1.0,0.0,1.4658828549558705,0.9167679037003069
nmpc_blind,32,32,0.65625,0.6530017536902797,1.9375,1.162241993567993,0.16510546875000004,0.8946536931319178,60.190625,15.690193349309299,0.7840950755480676,0.3612746399669977,0.16694883428402338,0.13027182812924618,0.8375,0.10364330891992483,3.521348465198372,3.2940022545052594
```