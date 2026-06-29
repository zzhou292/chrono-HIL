# How Chrono SCM actually computes tire forces (and what it means for our models)

Source: `chrono_fork/chrono/src/chrono_vehicle/terrain/SCMTerrain.cpp`,
`SCMLoader::ComputeInternalForces()` (@1074). Read this before trying to
improve any tire-force surrogate or the longitudinal prediction model — several
things we kept fighting are direct consequences of how SCM works.

## The computation
SCM is a **regular grid of independent soil cells**. Each physics step, for
every cell currently under a contact patch it computes a normal and a tangential
stress, builds a cell force, and the tire force is the **sum of cell forces over
the patch**. There is no closed-form `Fx(slip)` — the force *emerges* from the
patch integral.

Per cell with sinkage `z`:

**Normal stress — Bekker pressure–sinkage** (@1431, plastic branch):
```
σ = (Kc/b + Kphi) · z^n      with  b = 2·area/perimeter   (patch width, @1346–1353)
```
plus an elastic try `σ = K_elastic·(z − z_plastic)` (@1399) and velocity-proportional
damping `σ += −Vn·damping_R` (@1443). Sinkage `z` is split elastic/plastic and the
plastic part **accumulates** (@1428–1439).

**Shear stress — Mohr-Coulomb cap × Janosi–Hanamoto** (@1447–1450):
```
τ = (c + σ·tanφ) · (1 − exp(−j / K_janosi))
```
where **`j = ∫ v_slip dt`** is the *accumulated shear displacement* of the cell
(@1426: `kshear += dot(speed, −T)·dt`), **not** instantaneous slip.

**Cell forces** (@1454–1467):
```
Fn = N̂ · area · σ        Ft = T̂ · area · τ
```
`N̂` is the **deformed-surface normal** — the height-field gradient
`(hW−hE, hS−hN, 2δ)` (@858–867), so it **tilts on the walls of the rut**. `T̂`
points opposite the cell's slip velocity. Tire force = `Σ_cells (Fn + Ft)`.

## Two consequences that explain our results

### 1. Longitudinal force = compaction resistance + shear traction (two mechanisms)
- **Compaction/bulldozing resistance** is the horizontal projection of `Σ Fn`.
  Because the rut's leading wall has a *tilted* normal `N̂`, the Bekker normal
  stress has a backward horizontal component. It is driven by **sinkage `z`**
  (→ `Fz`, soil), and is **slip-independent**. This is the **−400 N zero-slip
  longitudinal force** measured in `project/SCM_PIL` — not a bug, real physics.
- **Shear traction/drag** is the horizontal projection of `Σ Ft` (Janosi).
  Driven by slip *history*, opposes slip.

A single learned `Fx(slip, Fz, soil)` map conflates the two. A pure Janosi
shear law (our first PIL attempt) captures only the second and **misses the
resistance entirely** — exactly why it predicted ~0 at zero slip. Onozuka &
Dallas (IV 2025) keep them separate: a Janosi shear `Fx` plus a *distinct*
rolling-resistance term `−Fz·rw·sin(λθ θn)` in the wheel dynamics (their Eq. 9).

### 2. The force is stateful in principle — but the memory is too fast to matter at the control rate
`τ` depends on **accumulated shear displacement `j = ∫v_slip dt`** (relaxing over
`K_janosi ≈ 0.01–0.025 m`) and on **plastic sinkage `z`** — both *states* in the
continuum model. **However**, `j` *resets per cell* as cells transit the contact
patch, so the aggregate tire force fully develops within **one patch transit
≈ patch_length/u ≈ 0.05 s** — shorter than the 0.1 s control step. At the
control / 4 s-prediction timescale the force is therefore **effectively static in
slip**.

This was tested directly: `project/SCM_PIL/zj_prototype.py` adds a shear-
relaxation state (transient slip `κ'`, relaxation length `σ`) to the longitudinal
force and grids `σ`. The best fit is **`σ* = 0` on all terrains (0 % improvement
over the static model)** — confirming the memory is negligible at this timescale.
Sinkage `z` is likewise quasi-static at steady cruise (it moves only with load
transfer / multi-pass, slow effects).

So the closed-loop longitudinal floor is **not** unmodeled fast dynamics — it is
(i) the **two-mechanism structure** of §1 (a kinematic `u̇ = ax` omits the
compaction resistance entirely), (ii) static force-surrogate accuracy, and
(iii) `a_act` measurement noise at the telemetry rate. The actionable lever is a
**clean static `Fx` surrogate that represents both mechanisms** (resistance +
traction), not added state or a physics prior.

## Implications for this project
- A **clean static `Fx` surrogate that captures both mechanisms** is the right
  model and the actionable lever. The `SCM_PIL` closed-loop test bears this out:
  a freshly rig-trained free-MLP `Fx` driving `u̇ = ΣFx/M` cuts the closed-loop
  acceleration residual **2–3× vs the kinematic `u̇ = ax`** (which omits the
  compaction resistance entirely). State (shear/sinkage) and physics structure
  add nothing at this timescale (§2; `zj_prototype.py`, `σ*=0`).
- The deployed `vehicle_rate_64_32_lhs` surrogate has known training-data bias,
  which is why an *earlier* reconstruction with it looked worse; a clean rig
  surrogate is the one that helps. Re-evaluating SCM_Final's gated
  `longitudinal_force_balance` mode with a clean rig `Fx` is the concrete
  follow-up.
- The reactive **throttle DOB + 10 Hz re-planning** remains a sound pragmatic
  choice, but the *reason* is accuracy/robustness of the force surrogate at the
  cruise operating point — not unmodeled fast dynamics.
- `project/SCM_PIL` is the negative-result exploration: a Janosi-only physics
  surrogate did worse than a free MLP (too rigid where clean data is plentiful;
  it also initially omitted the compaction-resistance term). Physics structure
  is a liability at this data scale, not an asset.
