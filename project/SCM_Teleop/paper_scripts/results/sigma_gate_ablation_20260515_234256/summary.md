# MPPI Shield Sigma-Gate Ablation

Noise policy: sensor noise enabled in every run.
Safety buffer: 0.5 m beyond the hard collision footprint.
Planner-blind MPC: shield is the sole collision avoider.
Variant notes: sigma_gate_on: Default: live terrain estimator forwards (n, phi, sigma_phi) to the shield.; sigma_gate_off: Shield receives live terrain but ignores sigma_phi (forced to 0).; no_live_terrain: Estimator disabled; shield uses static initial terrain (no live updates).

```csv
variant,n_runs,n_ok,collisions_mean,collisions_std,near_misses_mean,near_misses_std,min_clearance_m_mean,min_clearance_m_std,intervention_rate_pct_mean,intervention_rate_pct_std,mean_abs_dsteer_mean,mean_abs_dsteer_std,mean_abs_dthrottle_mean,mean_abs_dthrottle_std,rms_cte_m_mean,rms_cte_m_std
sigma_gate_on,32,32,0.34375,0.48255870443481425,2.0,0.9503819266229829,0.004528125000000001,0.1576119166822165,59.65,17.160043612246255,0.47400009212562655,0.32812717753769344,0.2143517725487728,0.2062328848907845,1.2037497728783355,0.7698851308002482
sigma_gate_off,32,32,0.21875,0.4200134406451545,1.875,0.9069623173877128,0.08472187499999997,0.1835430893144186,59.0625,17.633632637661474,0.4737856410687583,0.29057115439438913,0.2258974008954727,0.20322758759851203,1.0935898700262368,0.71886381286437
no_live_terrain,32,32,0.125,0.3360107525161235,2.09375,0.8560741225068111,0.10040625,0.1871141361834467,61.83125,15.380705078431143,0.37231058089378616,0.15801415523358053,0.2617315067485704,0.1906328290756926,0.8385887370868662,0.5939907987781103
```