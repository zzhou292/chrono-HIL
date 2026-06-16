# Human-in-the-Loop Data Collection Protocol (SCM_Final)

How to collect the human-in-the-loop (HIL) data for the paper, and — just
as important — what *not* to try to collect. Written to resolve the core
worry: **the human's commands are an uncontrolled, un-showable variable.**
The short answer is that you never report the human input; you report the
*safety filter's effect* on it, and you design the runs so that effect is
measurable despite the variability.

---

## 1. What the HIL study must establish (and what it must not)

The HIL section makes **one** claim:

> Under teleoperation latency, a swappable safety filter screening the
> operator's commands keeps the vehicle safe, and the two shipped filters
> sit at different points of a **safety-vs-intrusiveness** trade-off
> (DOB-CBF = minimum-deviation/intent-preserving; MPPI = predictive,
> more aggressive).

It is **not** a study of human driving skill, nor a claim about "humans in
general." That reframing is what makes the uncontrolled human tractable:

- The human is the **disturbance / command source**, not the treatment.
- The **filter** is the treatment, and `none` (filter off) is the control.
- The result is a *paired difference* — same operator, same scenario,
  filter on vs off — so the (large) human variability cancels in the
  comparison instead of contaminating it.

You therefore never plot raw human commands. You plot how the *filter*
changed the outcome.

---

## 2. Core design principle: within-subject, fixed scenarios, repeat

Three rules turn the uncontrolled human into clean data:

1. **Within-subject.** Every operator drives **every** (filter × delay)
   condition. Skill, style, and reaction time are then constant *within*
   each operator's comparison, so the filter effect is a within-operator
   paired contrast.
2. **Fixed, repeated scenarios.** Hold the convoy scenario, terrain, and
   latency profile **identical** across the filter conditions a given
   operator drives, so the only thing that changes between `none` and
   `DOB-CBF` is the filter. The convoy scenarios are deterministic, so the
   matrix already pairs them across filters. Repeat each condition (`--rounds`)
   to average within-operator run-to-run noise.
3. **Counterbalance order.** Randomise / Latin-square the order of
   conditions per operator so learning and fatigue don't alias onto a
   particular filter (operators *will* get better at the course over the
   session).

The human variability does not disappear — it becomes the **error bar** on
each (filter, delay) cell. Enough rounds (and, ideally, operators) shrink
it below the filter effect you are trying to show.

---

## 3. Conditions and what is held fixed

| Axis | Levels | Role |
| --- | --- | --- |
| Filter | `none`, `DOB-CBF`, `MPPI` (optionally `NMPC`) | **treatment** |
| Latency | the learned **5G profile** on both channels (command/uplink + camera/downlink, the latter ≈1.45× via the profile's `camera` channel). Constant `--delays {0,0.15,0.30}` is the simpler alternative. | stress axis |
| Scenario (`--convoy`) | dynamic-traffic convoy scenarios on a **straight forward course**: `lead_brake`, `cut_in`, `stalled`, `swerver`, `rear_approach` (all single-vehicle → real-time with the sensor camera). | **held fixed per operator** |
| Terrain | clay (optionally + sand) | secondary; keep small |
| Speed (pace), bumpiness | fixed (`--speeds 4`, `--bumpiness 0`) | held fixed |

Keep the matrix **small**: a human cannot drive 1000 runs. A defensible
core is `{none, DOB-CBF, MPPI} × {5G profile} × {3–5 convoy scenarios} ×
rounds` — the 5G profile collapses the latency axis to one realistic
condition, freeing budget for more scenarios. That is ~9–15 conditions per
operator; with a couple of rounds, ~30–45 manned runs at ~30 s of driving
each (plus ~30–40 s of Chrono start-up per round). The autonomous sweeps
(Sec VII–VIII) cover the large, reproducible matrix; HIL adds the
*human-command* evidence the autonomous runs cannot.

The note on latency: only the **Chrono-Sensor** camera (`--vis-mode sensor`)
applies the 5G profile's per-frame *camera* latency; the faster Irrlicht view
has no lag mechanism, so it models command latency only. Use `sensor` for the
latency-realistic collection (the single-vehicle scenarios above stay
real-time on it); Irrlicht is for high-vehicle-count scenes where camera
latency is not the focus.

---

## 4. Operators

- **Get ≥3 operators if at all possible** (lab colleagues are fine). Three
  is the difference between "the filter helped this one driver" and "the
  filter helped across drivers," and it lets you show the effect survives
  operator variability.
- If it is realistically just you: run it as a **single-expert-operator**
  study, be explicit about that in the text, and lean hard on within-subject
  rounds (5+ per condition). State operator generalisation as a limitation —
  this is standard and accepted for teleoperation/shared-control papers.
- **Familiarisation.** Give each operator a few unrecorded warm-up runs
  (filter `none`, zero delay) until lap times plateau, *then* start the
  counterbalanced data runs. Log familiarisation separately or discard it.
- **Consent / IRB.** If operators are anyone but you, check whether your
  institution needs a (likely exempt/minimal) human-subjects determination
  *before* collecting — you only record vehicle telemetry and optional
  over-the-shoulder video, not biometric data, but get the determination on
  file.

---

## 5. Session protocol

One command starts a session (G29 + sensor camera + live HMI overlay + the 5G
profile, sweeping the convoy scenarios), logging `sim_diag.csv` (with the
operator's raw commands) + shield/collision logs per round:

```bash
./collect_hil.sh                         # default session (15 rounds)
./collect_hil.sh --rounds 3              # or override anything
./collect_hil.sh --convoy lead_brake jam --terrains clay sand
```

It expands to the driver tool `benchmarking/human_delay_compensation_rounds.py`:

```bash
python benchmarking/human_delay_compensation_rounds.py \
    --manual-mode g29 --vis-mode sensor --live-hud \
    --latency-profile-json config/latency_profiles/5g_nhits_youtube_ul_scm_youtube_ul_smoke.json \
    --convoy lead_brake cut_in stalled swerver rear_approach \
    --filters none dob_cbf mppi \
    --terrains clay --paths straight --speeds 4 --bumpiness 0 --rounds 1
```

Each round prints an operator briefing first (scenario, goal, filter, and the
exact latency) and waits for Enter so you can grab the wheel. The task is a
**timed forward drive-and-avoid** on a straight course, scored on reaching the
goal distance (`--goal-distance`, default 50 m) *and* staying collision-free —
sitting still does not count (`reached_goal`/`clean_success` in the results).

- `--vis-mode sensor` gives the operator the latency-affected camera POV
  (what they drive on). Use `--vis-mode both` only for the clips (Sec 7),
  not the bulk — the third-person window is for spectators, and rendering
  it can perturb timing.
- **5G link (optional, more realistic).** Instead of the fixed `--delays`
  sweep, add `--latency-profile-json config/latency_profiles/<profile>.json`
  to drive every round under a *time-varying* 5G-like link: the
  `control`/`manual` channels are the command **uplink** and the `camera`
  channel is the asymmetric video **downlink** (the shipped profiles set the
  camera channel to ≈1.45× the uplink). This supersedes the constant
  delays and collapses the matrix to one "5G condition" per cell, logging
  the per-channel latency to `<run>/latency_profile.csv`. Run it as a
  *separate session* from the constant 0/0.15/0.30 s sweep (don't mix the
  two delay regimes in one paired comparison).
- **Live HMI overlay (optional).** Add `--live-hud` to pop the Tesla-style
  overlay (`simulation/hil_hud.py`) on each round — a virtual steering wheel
  (operator command = dashed ghost, applied = solid) and an accel/brake bar.
  It reads the driver inputs the sim publishes on `vehicle_state`
  (`steering_op/app`, `throttle_op/app`, `braking_app`), so it updates live
  even in manual mode where there is no controller — and when a filter takes
  over, the solid wheel/bar diverges from the ghost. The window is
  borderless, docks into a screen corner (`--corner br`), and is set
  always-on-top (via `wmctrl`) so it sits over the sim instead of being a
  separate window you raise by hand. Subscribe-only, torn down per round.
- **Real-time / resolution.** The driver POV defaults to **1920×1200 (16:10)
  at 30 Hz, fullscreen** (`--cam-width 1920 --cam-height 1200 --cam-rate 30
  --cam-fullscreen`, `--cam-fov 1.05`; `--no-cam-fullscreen` for a window,
  `--cam-height 1080` for 16:9). It renders at that resolution and scales to
  fill the screen. Two things
  broke real-time on the old setup: the 5760×1080 triple-monitor camera, and
  — measured to be the dominant cost — the **deformable SCM terrain mesh
  triangle count**, which the camera ray-traces (BVH rebuild) every frame.
  Profiling on an RTX 5090: 1080p@30Hz is **0.55×** at the fine `0.08` mesh
  but **1.00×** at `0.12` (per-frame render 4.5 ms → 0.9 ms). So the HIL path
  defaults to `--mesh-resolution 0.12` (the autonomous sweeps keep `0.08` for
  force fidelity, but they render headless so it costs them nothing). SCM
  *physics* is mesh-insensitive (~0.7 ms/step either way); the mesh only
  matters because the camera renders it. Watch the sim's `RT=…x` /
  `[TIMING] sensor=…` line: if it still dips, lower `--cam-rate` or
  `--cam-width` before touching the mesh further.
- Drop `--auto-start` so the script pauses between rounds; that gives the
  operator (and you) a reset/breath between runs and is where you read out
  "round k, scenario X, filter Y."
- **Same scenario across the filter conditions a given operator sees** — the
  convoy presets are deterministic and the matrix holds (convoy, terrain,
  latency) fixed across `none`/`DOB-CBF`/`MPPI`, so the comparison is paired by
  construction. (The counterfactual replay, below, makes this exact: the same
  recorded operator trace is re-run through each filter.)
- Take breaks; **fatigue inflates collisions** and aliases onto whatever
  filter you ran last.

---

## 6. What gets logged (this is the dataset — no video needed)

The script already records, per run, the only quantities you report:

| Metric | What it shows | Reported as |
| --- | --- | --- |
| `collisions`, `near_misses` | did the human+filter hit anything | mean ± sd per (filter, delay) |
| `min_clearance_m` | safety margin | mean ± sd |
| `intervention_rate_pct` | how often the filter overrode the human | the *cost* side of the trade-off |
| `mean_abs_dsteer`, `mean_abs_dthrottle` | how *hard* it overrode | intrusiveness magnitude |
| `speed_ratio` | task progress / how much the filter slowed the human | secondary |

The operator is **avoiding obstacles toward a goal, not tracking a reference
path**, so `rms_cte_m` is *not* a reported HIL metric — crosstrack from the
nominal path would penalise the very avoidance manoeuvre the filter is there
to enable. It is still logged for debugging, but the safety/intrusiveness
metrics above, plus the counterfactual harm-prevented comparison
(`convoy_counterfactual_eval.py`: same operator trace replayed filter-off vs
each filter), are what the paper reports.

That CSV **is** the publishable HIL dataset. None of it needs video.

---

## 7. What to record on video vs log-only (your actual question)

Your instinct is right: **the bulk is logged metrics; video is a tiny,
qualitative supplement.** Record exactly:

1. **Setup clip (~20–40 s):** the operator at the G29 + the camera-POV
   screen + the third-person sim, narrated once. Establishes that this is a
   real human teleoperating over a latent link. One take, supplementary
   material / talk, not a paper figure.
2. **One DOB-CBF takeover clip (~5–10 s):** a single run where the operator
   drives at an obstacle and the filter visibly steers/brakes around it.
   Capture with `--vis-mode both` (POV + third-person) and screen-record
   (there is no built-in recorder). Supplementary / talk.

Everything else is **log-only**. You do not (and should not) record the
hours of manned runs — they live as CSV rows that become the trade-off
figure.

**Capture tip:** for the clips, run a *dedicated, non-data* session with
`--vis-mode both`, screen-record (e.g. OBS) the windows, and pick one good
takeover. Do not screen-record the data sessions — it adds load and you'd
never use the footage.

---

## 8. How it appears in the paper

- **Primary figure (quantitative):** the 6-panel `human_delay_compensation_
  summary` — collisions, clearance, intervention rate, |Δsteer|,
  |Δthrottle|, RMS CTE, each vs delay, one line per filter, **error bars =
  across-operator/round spread**. This *is* the safety-vs-intrusiveness
  trade-off argument; the human variability shows up honestly as the error
  bars and is averaged out of the means.
- **One annotated takeover trajectory (quantitative, reproducible):** from a
  single logged run, overlay the operator's *commanded* heading/path, the
  *filtered* path the vehicle actually took, the obstacle, and the clearance.
  This is the in-paper, reproducible version of the "DOB-CBF takeover" — it
  shows a concrete intervention without needing video and is reconstructable
  from `sim_diag.csv` + the shield log. Strongly recommended; it does more
  than the video clip for a reviewer.
- **Video clips:** supplementary material / the talk only.

So: raw human commands → never shown; filter effect → the trade-off figure;
one concrete takeover → an annotated trajectory plot (+ a clip for the talk).

---

## 9. Statistics

- Report **paired, within-operator** comparisons (filter vs `none` at each
  delay). With ≥3 operators, a paired/Wilcoxon test or just mean ± sd with
  the per-operator points overlaid is enough — collisions are low-count, so
  prefer medians/counts and bootstrap or exact intervals over assuming
  normality.
- With a single operator, report mean ± sd over rounds and frame it as a
  pilot; do not run inferential tests across a single subject.
- Always show the per-(operator, round) scatter behind the bars so the
  human variability is visible, not hidden.

---

## 10. Pitfalls

- **Learning/fatigue** → counterbalance order, warm up to plateau, take
  breaks. The #1 way HIL results lie is order effects.
- **Non-identical scenarios across filters** → the convoy presets are
  deterministic so this is handled by construction; the counterfactual replay
  removes any doubt by re-running the identical operator trace per filter.
- **Over-collecting** → a human cannot generate the autonomous matrix's
  statistics. Keep the manned matrix small and let the autonomous sweeps
  (Sec VII–VIII) carry the large-N safety numbers; HIL adds the
  human-command evidence and the intrusiveness side only.
- **Recording the wrong thing** → don't film data runs; do film one setup +
  one takeover, separately.
- **Claiming generality from one driver** → state the operator count and its
  limitation plainly.

---

## 11. Minimal viable dataset (if time is short)

`./collect_hil.sh` as-is — `{none, DOB-CBF, MPPI} × {5G profile} × {5 convoy
scenarios, clay} × 1 round` = 15 manned runs — plus the two video clips and one
annotated takeover trajectory. One operator (you), ~15 min of driving, honestly
framed as a single-operator pilot. Then `convoy_counterfactual_eval.py
--trace-dir <session>` replays those rounds filter-off vs each filter for the
causal harm-prevented result. Add `--rounds`, more `--convoy` scenarios, a
second terrain, and a second/third operator as bandwidth allows — each
strengthens generality, none is required for the core claim.
