# vehicle_rate_twohead_64_32 — EXPERIMENTAL (not deployed)

Per-axle NMPC tire drop-in (rate-MLP 64-32, vehicle_rate format) trained on the
widened-box OPEN-LOOP lhs_twohead data via build_peraxle_csv.py + train_variant.
Offline force R2 (Fx 0.85 / Fy 0.94) beats vehicle_rate, but in CLOSED-LOOP it
tracks WORSE (rms_cte 0.51 vs vehicle_rate 0.30) due to train/deploy distribution
mismatch (open-loop data vs the controller's closed-loop distribution). Kept as
the documented negative result; NOT deployed.
