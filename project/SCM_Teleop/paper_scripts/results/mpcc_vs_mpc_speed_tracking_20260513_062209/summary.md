# MPCC vs Standard MPC Speed/Tracking

Noise policy: sensor noise enabled in every run.
MPC speed-weight interpretation: if `standard_mpc_soft_speed` reduces CTE without a large speed-ratio loss, the paper baseline should use the softer speed cost rather than force the tracker to chase v_ref in turns.
Troubleshooting interpretation: if relaxed MPCC variants do not improve speed ratio without CTE growth, the current MPCC is limited by missing baseline features/model mismatch rather than just conservative vtheta/cap settings.
Variant notes: standard_mpc: Baseline reference-tracking MPC with the closed-loop NN surrogate.; standard_mpc_soft_speed: Same MPC with weaker speed tracking so turns prioritize path tracking over v_ref recovery.; standard_mpc_overspeed_cap: Treats v_ref as a speed cap instead of a command, avoiding acceleration just to erase underspeed.; standard_mpc_no_speed: Ablates speed tracking entirely; useful to bound whether v_ref chasing is the failure mode.; mpcc_default: Current MPCC defaults.

```csv
variant,n_runs,n_ok,rms_cte_m_mean,rms_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_speed_mps_mean,mean_speed_mps_std,mean_solve_ms_mean,mean_solve_ms_std,progress_m_mean,progress_m_std
standard_mpc,4,4,0.09856479619727948,0.03474854296233014,0.7179728635682159,0.145840558667984,3.589864317841079,0.72920279333992,4.202013188329117,0.9656030136881685,30.334463223836316,6.291938562661861
standard_mpc_soft_speed,4,4,0.09016190484804686,0.03314388558028355,0.6399422038980509,0.16122809827017856,3.1997110194902545,0.806140491350893,3.7943345323741005,1.185862961120925,27.47478770437606,6.899128027059555
standard_mpc_overspeed_cap,4,4,0.10044869968837783,0.030019015380474755,0.5868857946026986,0.21120541045762317,2.9344289730134934,1.0560270522881159,4.206438848920863,1.1843516950142423,25.35255492109087,8.961653576617755
standard_mpc_no_speed,4,4,0.09430541546325946,0.030077697743371712,0.5808427586206897,0.21461005321648724,2.904213793103448,1.0730502660824364,3.853251300164687,0.8983192431357033,25.10083466238659,9.045751921405978
mpcc_default,4,4,0.2778043657541642,0.2558115206255803,0.6323658845577211,0.4238805362366183,3.1618294227886055,2.1194026811830917,0.7639936974789916,0.16863608952941397,26.604575,17.843052647716046
```