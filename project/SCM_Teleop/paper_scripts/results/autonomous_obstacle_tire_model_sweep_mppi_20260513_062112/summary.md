# Autonomous Obstacle Avoidance by Tire Model

Noise policy: sensor noise enabled in every run.
Fixed downstream safety filter: mppi; tire model is the swept variable.
Standard-MPC speed-weight is 70 so obstacle avoidance is not confounded by aggressive v_ref chasing in turns.
Standard-MPC speed-cost-mode is overspeed.
Standard-MPC obstacle-weight is 5000.
MPC blind to obstacles: False.
Safety buffer: 0.5 m.

```csv
variant,n_runs,n_ok,collisions_mean,collisions_std,near_misses_mean,near_misses_std,min_clearance_m_mean,min_clearance_m_std,rms_cte_m_mean,rms_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_solve_ms_mean,mean_solve_ms_std,progress_m_mean,progress_m_std
closed_loop_mlp,2,2,0.0,0.0,0.5,0.7071067811865476,1.0092940000000001,0.461590821477204,2.6887900380742904,1.494702806660532,0.8300230592875097,0.008318879909584154,5.798600767653399,1.6068028442992317,35.71842589470647,0.2560359738583486
```