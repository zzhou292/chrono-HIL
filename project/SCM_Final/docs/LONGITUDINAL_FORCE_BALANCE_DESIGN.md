# Principled longitudinal force-balance NMPC (design contract)

Goal: replace the kinematic longitudinal channel `u̇ = ax` (+ reactive throttle
DOB) with the theoretically-accurate force balance driven by the learned tyre
surrogate, so soil-dependent traction **and** drag come from one physical model
and the DOB demotes to a small residual.

## Why the easy paths are ruled out (evidence)

1. **Scalar drag term** `u̇ = ax − c_drag(n̂)`: wrecks tracking (clay CTE 0.7–1.0
   m). A constant in `u̇` perturbs the coupled solution and chases an infeasible
   speed.
2. **Surrogate resistance at κ=0** as `−R`: κ=0 is a sparse extrapolation for a
   closed-loop-trained surrogate (disagrees with the DOB by 5–10×), and it
   **double-counts** — drag is already inside `Fx(κ)` (the dirt curve crosses
   zero at κ≈0.04).
3. **Naive force balance** `u̇ = Fx_op/M` with `κ ≈ ax/(μg)`: already tried and
   reverted in this repo (`acados_mpc_solver.py` dynamics comment) — it degrades
   lateral AND longitudinal tracking because `ax` plays two inconsistent roles
   (throttle-surrogate via the integrator AND slip-surrogate via κ).

The common failure mode: reusing `ax` as both the actuation command and the slip
proxy. The fix is to give slip its **own** dynamics.

## The theoretically-accurate model

State adds wheel speed (lumped drive axle is enough for the bicycle plant):

    m · u̇      = Σ Fx(κ, Fz_axle, n̂) − Fx_grade          (longitudinal momentum)
    I_w · ω̇_w  = T_drive − Σ Fx(κ,·)·r_w − M_rr(Fz, n̂)     (wheel angular momentum)
    κ          = (ω_w·r_w − u) / max(u, ε)                 (SAE longitudinal slip)

* `Fx(κ, Fz, n̂)` — the learned tyre surrogate (already a CasADi function in the
  solver). The soil-dependence is **entirely here**.
* `T_drive` — drive/brake torque, the new control (replaces jerk `Jx`). Maps to
  throttle/brake through the **engine map, which is terrain-independent**
  (`throttle = T_drive / T_max(ω_w)`). No soil-dependent actuation-map unknown.
* `M_rr(Fz, n̂)` — rolling-resistance **moment** at the wheel. Essential subtlety:
  the surrogate predicts *net* Fx, which is ~0 at flat-ground cruise, so without
  `M_rr` the model would (wrongly) predict zero cruise throttle. `M_rr` is the
  soil term the net-Fx surrogate does not carry; it is calibrated from
  steady-cruise data (the same quantity the DOB currently integrates to).
* `Fx_grade` — grade/aero; ≈0 on the flat SCM patches, kept for generality.

This is the complete, first-principles longitudinal model: traction, slip
dynamics, drag, and rolling resistance are all explicit; the only learned piece
is the physically-meaningful `Fx(κ)` curve.

## Numerical note (stiffness)

Wheel dynamics (`I_w` small) are fast → the DAE is stiff. Options, in order of
preference: (a) acados **implicit (IRK)** integrator on the augmented model;
(b) singular-perturbation / quasi-steady slip where `ω̇_w≈0` ⇒ algebraic
`κ` solve (but κ stays its own variable, NOT `ax`-derived — that is the
distinction from the reverted attempt); (c) treat `κ` as a rate-limited control
(`κ̇` penalised), the lightest form, validated first.

## Implementation plan (gated `--longitudinal-force-balance`, default off)

1. Solver: add the augmented longitudinal channel (κ control/state + surrogate
   `Fx(κ)` in `u̇`), behind a flag; keep the kinematic default intact.
2. Controller `ControlIntegrator`: map the planned `T_drive`/κ to throttle via
   the engine map; keep a *small* residual DOB.
3. Calibrate `M_rr(Fz, n̂)` and `T_max` from data / the vehicle config.
4. Smoke (compile + one run), then A/B vs the kinematic+DOB default on
   tracking + speed + DOB-off feasibility.

## Honest risk

Multi-step acados restructure (new state/control, stiff integrator, throttle
remap, cost re-tune). The reverted prior attempt + the `M_rr` subtlety are real
cautions. The decoupled terrain-aware speed profile (already default, −45% CTE)
is the validated win that stands regardless; this force balance is the
longitudinal-prediction/DOB-elimination follow-on.
