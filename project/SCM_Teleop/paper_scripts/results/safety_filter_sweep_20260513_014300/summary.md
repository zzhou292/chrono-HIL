# Safety Filter Sweep

Noise policy: sensor noise enabled in every run.
NMPC diagnostic angle: if NMPC loses to DOB-CBF, inspect raw `nmpc_shield_log.csv` and trajectories for local-minimum behavior, short-horizon braking, or insufficient geometric steering commitment. DOB-CBF has an explicit reactive steering layer that can win on head-on rocks even without long-horizon optimization.

```csv
variant,n_runs,n_ok,collisions_mean,collisions_std,near_misses_mean,near_misses_std,min_clearance_m_mean,min_clearance_m_std,intervention_rate_pct_mean,intervention_rate_pct_std,mean_abs_dsteer_mean,mean_abs_dsteer_std,mean_abs_dthrottle_mean,mean_abs_dthrottle_std,rt_factor_mean,rt_factor_std,rms_cte_m_mean,rms_cte_m_std
none_blind,8,8,1.5,0.5345224838248488,1.875,0.8345229603962802,-1.6618499999999998,0.5060593753983532,,,,,,,1.0,0.0,0.15970247798460135,0.13514916025129525
dob_cbf_blind,8,8,0.0,0.0,1.0,0.9258200997725514,0.7634352500000001,0.438642209619884,41.3,12.11480794022635,,,,,1.0,0.0,2.8607091886312586,0.8095633036981281
mppi_blind,8,8,0.125,0.3535533905932738,2.0,0.7559289460184544,0.10332500000000003,0.14902660117288744,59.425,15.236024040786642,0.27416700752765305,0.1285367300473294,0.13640662461134226,0.09368161260784597,0.99875,0.0035355339059327463,1.725631627303994,1.1513989150560426
nmpc_blind,8,8,0.625,0.7440238091428449,1.5,1.1952286093343936,0.778083625,1.50061782413663,55.2625,14.97444847541495,0.6552643839599912,0.12141104807863039,0.13240888769484851,0.09547532785859392,0.7775,0.15387843068010357,3.1572067942072275,2.565676820624521
```