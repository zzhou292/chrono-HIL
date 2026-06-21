# Autonomous Obstacle Avoidance by Tire Model

Noise policy: sensor noise enabled in every run.
Fixed downstream safety filter: mppi; tire model is the swept variable.
Standard-MPC speed-weight is 0 so obstacle avoidance is not confounded by aggressive v_ref chasing in turns.
Standard-MPC speed-cost-mode is symmetric.
Standard-MPC obstacle-weight is 5000.
MPC blind to obstacles: False.
Safety buffer: 0.5 m.

```csv
variant,n_runs,n_ok,collisions_mean,collisions_std,near_misses_mean,near_misses_std,min_clearance_m_mean,min_clearance_m_std,rms_cte_m_mean,rms_cte_m_std,speed_ratio_mean,speed_ratio_std,mean_solve_ms_mean,mean_solve_ms_std,progress_m_mean,progress_m_std
closed_loop_mlp,8,8,0.0,0.0,0.375,0.5175491695067657,6.2155085,6.194251335378081,0.6636619469753269,1.092331600383187,0.5510629703091663,0.210586125375028,4.722029486590583,1.0021408216384455,24.082345421565748,9.17453328981693
```