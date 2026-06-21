# Safety Filter Sweep

Noise policy: sensor noise enabled in every run.
Safety buffer: 0.5 m beyond the hard collision footprint.
NMPC diagnostic angle: if NMPC loses to DOB-CBF, inspect raw `nmpc_shield_log.csv` and trajectories for local-minimum behavior, short-horizon braking, or insufficient geometric steering commitment. DOB-CBF has an explicit reactive steering layer that can win on head-on rocks even without long-horizon optimization.

```csv
variant,n_runs,n_ok,collisions_mean,collisions_std,near_misses_mean,near_misses_std,min_clearance_m_mean,min_clearance_m_std,intervention_rate_pct_mean,intervention_rate_pct_std,mean_abs_dsteer_mean,mean_abs_dsteer_std,mean_abs_dthrottle_mean,mean_abs_dthrottle_std,rt_factor_mean,rt_factor_std,rms_cte_m_mean,rms_cte_m_std
none_blind,12,12,1.5833333333333333,0.5149286505444373,2.1666666666666665,0.8348471099367218,-1.6837666666666664,0.5259296139103397,,,,,,,1.0,0.0,0.13409630380981905,0.17714623081038675
dob_cbf_blind,12,12,0.08333333333333333,0.28867513459481287,1.1666666666666667,0.5773502691896257,0.7109718333333334,0.3973518658139003,55.96666666666667,10.272676320603056,,,,,1.0,0.0,2.810088922183297,0.7272634944730022
mppi_blind,12,12,0.0,0.0,2.0,0.7385489458759964,0.12380833333333335,0.13961436145412265,61.525,9.596791604015849,0.4015670671814126,0.1725246743303241,0.1533539607070839,0.11990584603873354,1.0,0.0,1.5473639124171628,1.2823693298792742
nmpc_blind,12,12,0.25,0.4522670168666455,1.9166666666666667,0.996204919895622,0.20040766666666668,0.652302718275012,53.349999999999994,8.61061712496423,0.7202400507531026,0.3553253920863298,0.14402776091897798,0.07480504700117933,0.7925,0.11505927326224674,2.5119837326797683,1.4741231529284482
```