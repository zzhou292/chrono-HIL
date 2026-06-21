# Autonomous Obstacle Avoidance by Tire Model

Noise policy: sensor noise enabled in every run.
No downstream safety filter is used; the standard MPC's obstacle barrier is the only autonomous obstacle-avoidance mechanism.
Standard-MPC speed-weight is 15 so obstacle avoidance is not confounded by aggressive v_ref chasing in turns.
Standard-MPC obstacle-weight is 10000.

```csv
variant,n_runs,n_ok,collisions_mean,collisions_std,near_misses_mean,near_misses_std,min_clearance_m_mean,min_clearance_m_std,rms_cte_m_mean,rms_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_solve_ms_mean,mean_solve_ms_std,progress_m_mean,progress_m_std
closed_loop_mlp,2,2,1.0,0.0,2.0,1.4142135623730951,-0.8415000000000001,0.6330019905181974,0.34174744955808584,0.12693093318811102,0.7318074662668665,0.15587364352506422,3.555525081145737,1.27838448419855,31.881703068580315,5.954656566311382
```