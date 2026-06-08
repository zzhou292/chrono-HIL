# vehicle_twohead_128 — EXPERIMENTAL (not deployed)

Unified two-head whole-vehicle surrogate (shared trunk -> control head
[Fx_f,Fy_f,Fx_r,Fy_r] + estimation head [Fy_total=m*ay, M_yaw=Iz*dwz]).
Trunk input = [u,v,omega,delta,throttle,6 soil]; slip ratio feeds Head A only
(stop-gradient trunk) so the estimation head is unperturbed. Trained on
data/dallas_scm/lhs_twohead (widened-box). Held-out R2: control Fy 0.97/0.85,
Fx (low-pass controllable) 0.96/0.83; estimation Fy 0.88 / M_yaw 0.93.
Investigation artifact only — NOT wired into runtime. Conclusion: estimation
head is competitive with vehicle_fy, but a single shared model is not worth the
coupling; control and estimation want different training data (rig vs whole-veh).
Trainer: nn_training/train_vehicle_twohead.py.
