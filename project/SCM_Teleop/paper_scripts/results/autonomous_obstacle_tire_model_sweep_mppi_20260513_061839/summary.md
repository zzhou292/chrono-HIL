# Autonomous Obstacle Avoidance by Tire Model

Noise policy: sensor noise enabled in every run.
Fixed downstream safety filter: mppi; tire model is the swept variable.
Standard-MPC speed-weight is 0 so obstacle avoidance is not confounded by aggressive v_ref chasing in turns.
Standard-MPC obstacle-weight is 5000.
MPC blind to obstacles: False.
Safety buffer: 0.5 m.

```csv
variant,n_runs,n_ok,collisions_mean,collisions_std,near_misses_mean,near_misses_std,min_clearance_m_mean,min_clearance_m_std,rms_cte_m_mean,rms_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_solve_ms_mean,mean_solve_ms_std,progress_m_mean,progress_m_std
closed_loop_mlp,2,2,0.0,0.0,1.0,0.0,0.23309999999999986,0.3008032247167572,1.318808875394819,0.0373306735116102,0.8431112289379772,0.009889733049120135,5.079663854888851,2.307039968120208,36.19252169839615,0.16336898501200808
```