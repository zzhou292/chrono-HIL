#!/bin/bash
# 8 seeds × 4 tire models = 32 closed-loop runs.
# Same scenario as Fig 3 (clay/sinusoidal/v=7/lead-in 5/rocks 5/obs-w 1000/25 s).
# Workers cap at 6 parallel sims so the 24-core box doesn't thrash.
# Don't use `set -u`: conda's activate hooks reference unset vars
cd /home/ksha/Documents/sbel/chrono_hil/chrono-HIL/project/SCM_Final
source ~/miniconda3/etc/profile.d/conda.sh
conda activate sim
set -e
export ACADOS_SOURCE_DIR=$HOME/Documents/sbel/acados
export ACADOS_UNIQUE_BUILD_DIR=1

WORKERS=6
SEEDS=(1 2 3 4 5 6 7 8)
MODELS=(pacejka tmeasy rig vehicle)

run_one() {
    local TAG=$1; local PORT=$2; shift 2
    local DIR=deliverables/runs/$TAG
    rm -rf "$DIR" && mkdir -p "$DIR"
    python -u simulation/launch_decoupled.py \
        --terrain clay --path sinusoidal --speed 7.0 \
        --time 25.0 --lead-in 5.0 --bumpiness 0 \
        --rocks 5 --rock-seed "$ROCK_SEED" \
        --rock-zone-x 12.0 50.0 --rock-zone-y -3.0 3.0 \
        --rock-size 0.8 1.8 \
        --sim-port "$PORT" --ctrl-port $((PORT+1)) \
        --plot-dir "$DIR" --no-vis --no-plot \
        --sim-diag-csv "$DIR/sim_diag.csv" \
        --obstacle-weight 3500 \
        "$@" > "$DIR/run.log" 2>&1
}

wait_for_slot() {
    while [ "$(pgrep -fc launch_decoupled.py)" -ge "$WORKERS" ]; do
        sleep 4
    done
}

PORT_BASE=50000
JOB=0
for SEED in "${SEEDS[@]}"; do
    export ROCK_SEED=$SEED
    for MTYPE in "${MODELS[@]}"; do
        case $MTYPE in
            pacejka) MODEL_ARGS="--model pacejka --nn-model vehicle_rate_64_32_lhs" ;;
            tmeasy)  MODEL_ARGS="--model tmeasy  --nn-model vehicle_rate_64_32_lhs" ;;
            rig)     MODEL_ARGS="--model nn      --nn-model rig_rate_64_32" ;;
            vehicle) MODEL_ARGS="--model nn      --nn-model vehicle_rate_64_32_lhs" ;;
        esac
        TAG="sweep_s${SEED}_${MTYPE}"
        PORT=$((PORT_BASE + JOB*10))
        wait_for_slot
        echo "[$(date +%H:%M:%S)] launching $TAG (port $PORT, slot taken)"
        run_one "$TAG" "$PORT" $MODEL_ARGS --rms-time-start 2.0 &
        JOB=$((JOB+1))
    done
done
echo "[$(date +%H:%M:%S)] all jobs submitted; waiting for completion"
wait
echo "[$(date +%H:%M:%S)] sweep complete"
