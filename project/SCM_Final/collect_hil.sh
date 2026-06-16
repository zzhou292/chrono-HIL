#!/usr/bin/env bash
# Single-command HIL data-collection session with the Logitech G29.
#
#   ./collect_hil.sh                 # sensible default session
#   ./collect_hil.sh --convoy jam --rounds 5     # override anything
#
# Defaults: G29 wheel, Chrono-Sensor driver POV (so the 5G CAMERA latency is
# applied), live HMI overlay, the learned 5G latency profile on BOTH the
# command and camera channels. The course is a ~100 m off-road boulder field
# (rocks the full width, no clean bypass) on moderately bumpy terrain, with a
# goal gate at 100 m that ends the round early, within a 90 s budget. Four
# convoy scenarios -- convoy(3)/platoon(5)/lead_brake/rear_approach -- with
# DOB-CBF as the only safety filter, 1 round each (4 rounds). Each round logs
# sim_diag.csv (with the operator's raw commands), which the counterfactual
# eval can replay
# (benchmarking/convoy_counterfactual_eval.py --trace <run>/sim_diag.csv).
#
# Results land in benchmarking/results/human_delay_compensation_rounds_<ts>/.
set -o pipefail   # NOT -u: conda's (de)activate scripts reference unset vars

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

# --- environment (the sim env + acados; see CLAUDE.md) ---
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate sim 2>/dev/null || true   # already-active env can trip a re-activate
export ACADOS_SOURCE_DIR="$HOME/Documents/sbel/acados"
export DISPLAY="${DISPLAY:-:0}"   # use the desktop's display (G29 + window)

# --- G29 presence hint (non-fatal) ---
if ! ls /dev/input/js* >/dev/null 2>&1; then
  echo "WARNING: no joystick device (/dev/input/js*) detected -- is the G29 plugged in"
  echo "         and powered? Continuing anyway (Chrono will fall back to no input)."
fi

echo "Starting HIL collection session (G29 + Irrlicht + live HUD)."
echo "You'll be prompted to press Enter before each round -- grab the wheel first."
echo

LATENCY_PROFILE="config/latency_profiles/5g_hil_usable.json"

# vis-mode sensor (NOT irrlicht): only the Chrono-Sensor camera applies the
# profile's per-frame camera (downlink) latency. The scenarios below are all
# single-vehicle, so the sensor camera holds real-time. The 5G profile drives
# BOTH the command (uplink) and camera (downlink) channels and supersedes any
# fixed --delays. Scenarios are swept as separate rounds.
# Off-road boulder field: ~35 scattered rocks spread the full width over the
# whole course (no clean side to swerve into), blue-noise spaced (6 m) so it's
# steerable, on moderately bumpy terrain, out to a 100 m goal gate with a
# generous 90 s budget. Every round has other HMMWVs: convoy (3) and platoon
# (5) plus single-lead scenarios. The lead(s) wait for you to move, then pick a
# line through the field (avoiding rocks and each other). DOB-CBF is the only
# safety filter (MPPI dropped).
exec python benchmarking/human_delay_compensation_rounds.py \
    --manual-mode g29 \
    --vis-mode sensor \
    --live-hud \
    --latency-profile-json "$LATENCY_PROFILE" \
    --convoy convoy platoon lead_brake rear_approach \
    --filters dob_cbf \
    --terrains clay \
    --paths straight \
    --goal-distance 100 \
    --time 90 \
    --rocks 35 \
    --rock-min-spacing 6.0 \
    --rock-centerline-clear 3.0 \
    --rock-spawn-clear 8.0 \
    --rock-size 0.5 1.4 \
    --speeds 4 \
    --bumpiness 4 \
    --rounds 1 \
    "$@"
