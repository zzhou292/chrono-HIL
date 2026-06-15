#!/usr/bin/env bash
# Single-command HIL data-collection session with the Logitech G29.
#
#   ./collect_hil.sh                 # sensible default session
#   ./collect_hil.sh --convoy jam --rounds 5     # override anything
#
# Defaults: G29 wheel, Irrlicht fixed driver view, live HMI overlay, the
# lead-brake convoy, paired filters {none, DOB-CBF, MPPI} at a 0.15 s teleop
# delay, 2 rounds each. Each round logs sim_diag.csv (with the operator's raw
# commands), which the counterfactual eval can replay
# (benchmarking/convoy_counterfactual_eval.py --trace <run>/sim_diag.csv).
#
# Results land in benchmarking/results/human_delay_compensation_rounds_<ts>/.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"

# --- environment (the sim env + acados; see CLAUDE.md) ---
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate sim
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

exec python benchmarking/human_delay_compensation_rounds.py \
    --manual-mode g29 \
    --vis-mode irrlicht \
    --live-hud \
    --convoy lead_brake \
    --filters none dob_cbf mppi \
    --delays 0.15 \
    --terrains clay \
    --paths sinusoidal \
    --speeds 4 \
    --bumpiness 0 \
    --rounds 2 \
    "$@"
