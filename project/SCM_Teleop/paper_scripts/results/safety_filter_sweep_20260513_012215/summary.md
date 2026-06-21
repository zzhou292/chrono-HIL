# Safety Filter Sweep

Noise policy: sensor noise enabled in every run.
NMPC diagnostic angle: if NMPC loses to DOB-CBF, inspect raw `nmpc_shield_log.csv` and trajectories for local-minimum behavior, short-horizon braking, or insufficient geometric steering commitment. DOB-CBF has an explicit reactive steering layer that can win on head-on rocks even without long-horizon optimization.

```csv
variant,n_runs,n_ok,collisions_mean,collisions_std,near_misses_mean,near_misses_std,min_clearance_m_mean,min_clearance_m_std,intervention_rate_pct_mean,intervention_rate_pct_std,mean_abs_dsteer_mean,mean_abs_dsteer_std,mean_abs_dthrottle_mean,mean_abs_dthrottle_std,rt_factor_mean,rt_factor_std,rms_cte_m_mean,rms_cte_m_std
none_blind,8,8,1.5,0.5345224838248488,1.875,0.8345229603962802,-1.6437374999999999,0.46338066097355174,,,,,,,1.0,0.0,0.18749162826990637,0.12159718966303452
dob_cbf_blind,8,8,0.0,0.0,1.125,0.8345229603962803,0.9084585000000002,0.6281030157755061,42.8875,17.621288869676118,,,,,1.0,0.0,2.760509471787466,0.8947553723005202
mppi_blind,8,8,1.25,0.4629100498862757,1.625,0.9161253813129043,-0.5282,0.2216466235120349,34.675,10.941500288873943,0.4065785750081567,0.28965655593046835,0.35485266364070717,0.2541197532543701,1.0,0.0,0.6470252087038321,0.6554753877424616
nmpc_blind,8,8,1.375,0.7440238091428449,1.75,0.8864052604279183,-0.6133875,0.5504486311507318,31.375,15.229271626893857,0.7365500172269275,0.462003355378332,0.30086164088588013,0.2523588328235938,0.99625,0.005175491695067676,1.0748325698606196,1.0796708303852665
```