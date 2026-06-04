#!/bin/bash
# 3 ID (clay/dirt/sand) + 4 OOD terrain estimator convergence runs.
# Sinusoidal path, v=7 m/s, 30 s sim, terrain estimator ON.
cd /home/ksha/Documents/sbel/chrono_hil/chrono-HIL/project/SCM_Final
source ~/miniconda3/etc/profile.d/conda.sh
conda activate sim
export ACADOS_SOURCE_DIR=$HOME/Documents/sbel/acados
export ACADOS_UNIQUE_BUILD_DIR=1

WORKERS=6
wait_for_slot() {
    while [ "$(pgrep -fc launch_decoupled.py)" -ge "$WORKERS" ]; do
        sleep 4
    done
}

run_one() {
    local TAG=$1; shift; local PORT=$1; shift
    local DIR=deliverables/runs/$TAG
    rm -rf "$DIR" && mkdir -p "$DIR"
    python -u simulation/launch_decoupled.py \
        --terrain clay --path sinusoidal --speed 7.0 \
        --time 30.0 --lead-in 5.0 --bumpiness 0 \
        --sim-port "$PORT" --ctrl-port $((PORT+1)) \
        --plot-dir "$DIR" --no-vis --no-plot \
        --sim-diag-csv "$DIR/sim_diag.csv" \
        --model nn --nn-model vehicle_rate_64_32_lhs --rms-time-start 2.0 \
        --terrain-estimator \
        "$@" > "$DIR/run.log" 2>&1
}

PORT=60000
# ID terrains
for T in clay dirt sand; do
    wait_for_slot
    echo "[$(date +%H:%M:%S)] launching te_id_${T}"
    run_one "te_id_${T}" $PORT --terrain "$T" &
    PORT=$((PORT + 10))
done

# OOD terrains
for I in 1 2 3 4; do
    wait_for_slot
    echo "[$(date +%H:%M:%S)] launching te_ood_t${I}"
    run_one "te_ood_t${I}" $PORT \
        --terrain dirt \
        --terrain-config deliverables/runs/ood_terrains/terrain${I}.yaml &
    PORT=$((PORT + 10))
done

echo "[$(date +%H:%M:%S)] all submitted, waiting..."
wait
echo "[$(date +%H:%M:%S)] done"
