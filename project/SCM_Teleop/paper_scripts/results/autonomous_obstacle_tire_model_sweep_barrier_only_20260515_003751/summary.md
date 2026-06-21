# Autonomous Obstacle Avoidance by Tire Model

Noise policy: sensor noise enabled in every run.
No downstream safety filter is used; the standard MPC's obstacle barrier is the only autonomous obstacle-avoidance mechanism.
Standard-MPC speed-weight is 15 so obstacle avoidance is not confounded by aggressive v_ref chasing in turns.
Standard-MPC speed-cost-mode is symmetric.
Standard-MPC obstacle-weight is 5000.
MPC blind to obstacles: False.
Safety buffer: 0.5 m.

```csv
variant,n_runs,n_ok,collisions_mean,collisions_std,near_misses_mean,near_misses_std,min_clearance_m_mean,min_clearance_m_std,rms_cte_m_mean,rms_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_solve_ms_mean,mean_solve_ms_std,progress_m_mean,progress_m_std
pacejka,1,1,0.0,,0.0,,18.246181,,0.06062660321827374,,0.39211944,,2.63023988005997,,12.608426809942628,
tmeasy,1,1,0.0,,0.0,,18.496941,,0.06111609985840065,,0.38322596,,2.1693253373313346,,12.350388407261445,
closed_loop_mlp,1,1,0.0,,0.0,,17.919615,,0.05892537075213019,,0.39418404,,4.330864946889226,,12.92154630957228,
```