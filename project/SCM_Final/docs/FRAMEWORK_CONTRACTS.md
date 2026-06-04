# Framework Contracts

This project should be presented as a framework for terrain-aware
off-road autonomy and teleoperation research. The framework claim is
grounded in explicit component boundaries: message contracts for
process-level components, factory contracts for in-process components,
and CLI contracts for benchmark/runtime configuration.

## Component Graph

```
                 VehicleState
        +----------------------------+
        |                            v
+---------------+            +---------------+
| Chrono::HIL   |            | Controller    |
| simulation    |            | source        |
| node          |<-----------| acados NMPC / |
+---------------+ ControlCmd | external ctrl |
   ^   |                     +---------------+
   |   |
   |   v   advisory severity
   |  +------------------+
   |  | Collision        |
   |  | warning (CW)     |
   |  | terrain+latency  |
   |  +------------------+
   |
   | filtered driver inputs
   |
+---------------+        +------------------+
| Safety filter |<-------| Command source   |
| DOB-CBF /     |        | NMPC cmd / G29 / |
| MPPI / NMPC   |        | WASD             |
+---------------+        +------------------+
```

## Process-Level Contracts

### Simulation -> Controller: `VehicleState`

Defined in `simulation/hil_messages.py`.

The sim publishes `VehicleState` at the configured state rate. A
controller must treat this as the complete runtime observation, except
for optional configuration data delivered by `SimStatus`.

Contract fields include:

- Global vehicle pose and orientation.
- Body-frame longitudinal/lateral speed and yaw rate.
- IMU-like acceleration channels.
- Wheel encoder angular rates.
- Steering-angle sensor.
- Optional tire-force diagnostics.
- Optional nearby obstacle list, encoded as `[x0, y0, r0, ...]`.

Swap rule: any new controller can replace the current acados controller
if it subscribes to `VehicleState` and publishes `ControlCommand`.

### Controller -> Simulation: `ControlCommand`

Defined in `simulation/hil_messages.py`.

The sim applies the latest `ControlCommand`, optionally through a
sim-side safety filter. The command is intentionally actuator-level:

- normalized steering in `[-1, 1]`
- throttle in `[0, 1]`
- braking in `[0, 1]`
- optional MPC internal diagnostics
- optional terrain-estimator fields piggybacked for the safety layer

Swap rule: the standard acados NMPC, a learned policy, a ROS bridge,
or a remote controller are interchangeable at this boundary as long as
they publish `ControlCommand`. (The MPCC contouring variant explored
earlier in this project was archived in 2026-05-23; only the standard
NMPC ships in this snapshot.)

### Simulation Lifecycle: `SimStatus`

Defined in `simulation/hil_messages.py`.

The sim publishes configuration and lifecycle messages. Controllers use
this to discover terrain parameters, vehicle parameters, target path,
target speed, and timing information.

## In-Process Contracts

Not every component is a separate process. For benchmark speed and
Chrono integration simplicity, the human driver and safety filters are
currently in-process components of `chrono_sim_node.py`. They still have
explicit contracts.

### Command Source Contract

A command source provides normalized driver inputs:

- steering in `[-1, 1]`
- throttle in `[0, 1]`
- braking in `[0, 1]`

Current command sources:

- external controller over `ControlCommand`
- Logitech G29 manual driver
- WASD manual driver

The sim node applies optional command delay to both autonomous and
manual commands. The driver camera can be delayed separately to emulate
teleoperation downlink latency.

Swap rule: a new human-interface device or autonomy source only needs
to produce the same three normalized actuator channels, either through
`ControlCommand` or through a sim-side driver adapter.

### Safety Filter Contract

Safety filters are created through `simulation/safety/make_safety_filter`
and expose a shared runtime API:

```python
result = filter.filter(
    desired_steering,
    desired_throttle,
    desired_brake,
    vehicle_state,
    obstacles,
)
```

The returned `SafetyFilterResult` contains:

- filtered steering/throttle/brake
- whether the command was modified
- active-constraint count
- solve time
- safety margin and diagnostics

Current safety-filter flavors:

- `dob_cbf`: single-step QP safety filter. This is the most natural
  intent-preserving HIL safety mechanism because it solves a
  minimum-deviation problem around the operator command.
- `mppi`: predictive rollout shield using the learned terrain/tire
  surrogate and seeded recovery trajectories.
- `nmpc`: gradient-based finite-horizon shield used as a comparison.

Swap rule: a new safety filter can replace any existing flavor if it is
registered in `make_safety_filter` and returns `SafetyFilterResult`.

### Collision-Warning Contract

Forward collision warning is exposed through the same swappable-factory
pattern as the safety filters:

```python
warning = make_collision_warning_system(flavor='ttc', ...)
result = warning.evaluate(vehicle_state, obstacles, command_age_s)
```

The returned `CollisionWarning` carries the discrete severity
{GREEN, YELLOW, ORANGE, RED}, the running TTC and stopping-distance
estimates, and the running operator reaction-time budget. The warning
**never modifies commands** — it is a parallel advisory channel that
the HMI consumes alongside whatever filter (or no filter) is selected.

Swap rule: a new warning module can replace `flavor='ttc'` if it
implements `evaluate(state, obstacles, command_age_s) ->
CollisionWarning` and is registered in
`make_collision_warning_system`.

## CLI Swap Points

`simulation/launch_decoupled.py` is the main composition layer.

Useful swap flags:

- `--model nn|pacejka|pacejka-oracle|tmeasy` (controller tire model)
- `--nn-model <checkpoint>`
- `--terrain-estimator --terrain-estimator-mode n` (live `n̂`)
- `--manual` for Logitech G29
- `--wasd` for keyboard manual mode
- `--safety-filter --safety-flavor dob_cbf|mppi|nmpc`
- `--no-safety-nn` for DOB-CBF without NN force prediction
- `--mpc-blind-obstacles` to isolate the safety layer as the obstacle avoider
- `--collision-warning --cw-tire-model <model>` for the advisory CW channel
- `--manual-input-delay`, `--camera-input-delay`, `--teleop-delay`, and
  `--latency-profile-json` for teleoperation latency experiments

## Benchmark Contracts

The paper scripts are contract tests for the framework:

- tire-model sweeps (`mpc_tire_model_sweep.py`) swap the controller
  tire model.
- terrain-estimator sweeps (`terrain_estimator_benchmark.py`) swap
  terrain conditioning.
- safety sweeps (`safety_filter_sweep.py`) swap the downstream safety
  filter.
- the DOB-CBF NN ablation (`dob_cbf_nn_ablation.py`) swaps the safety
  filter's force-prediction model.
- the MPPI seed ablation (`mppi_seed_ablation.py`) swaps the
  hand-crafted recovery seeds in or out.
- the latency sweep (`latency_compensation_sweep.py`) swaps the delay
  channel while holding scenarios fixed.
- the collision-warning sweep (`collision_warning_test.py`) and brake
  validator (`brake_test.py`) check the warning module's
  terrain × latency lead time and its analytical `a_b(n̂)` table.
- HIL rounds (`human_delay_compensation_rounds.py`) swap autonomous
  command generation for a human operator with delayed uplink and
  downlink.

Each sweep writes per-run CSVs plus summary CSVs, and
`publish_paper_figures.py` republishes canonical figures into
`my_paper/paper_figures/`.

## Current Boundary Limitations

These are honest implementation facts to keep the framework claim
precise:

- The controller/sim interface is a hard process boundary over ZMQ.
- The safety-filter interface is a clean Python factory/API boundary,
  but the filter runs inside the sim process so it can intercept commands
  immediately before Chrono actuation.
- G29 and WASD manual drivers are sim-side adapters, not separate ZMQ
  command publishers. They still use the same normalized actuator
  command semantics as `ControlCommand`.
- Terrain updates are piggybacked on `ControlCommand` because the
  controller-to-sim socket is latest-only (`ZMQ_CONFLATE`).

Those limitations do not break the framework story, but they should
shape the language: say "swappable components with explicit contracts,"
not "every component is an independent service."
