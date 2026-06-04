#!/bin/bash
# Train matched-architecture vehicle (whole-vehicle closed-loop) tire surrogates
# on the LHS-terrain dataset (data/whole_vehicle/lhs).
#
# These checkpoints are the LHS-fair counterparts of the rig_static_32_16,
# rig_rate_32_16, and rig_rate_64_32 retrains -- same architecture, but
# trained on closed-loop axle data drawn from the same LHS box as the rig.
# Net effect: the rig-vs-vehicle comparison is now controlled for both
# (a) hidden-size and (b) training-terrain distribution.
#
# Usage:  nn_training/train_vehicle_lhs.sh
#   expects data/whole_vehicle/lhs/training_data_rich_tire_frame.csv to exist.
#
set -e

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DATA="${ROOT}/data/whole_vehicle/lhs/training_data_tire_frame.csv"
DATA_RICH="${ROOT}/data/whole_vehicle/lhs/training_data_rich_tire_frame.csv"
MODELS_OUT="${ROOT}/nn_models"
TRAINER="${ROOT}/nn_training/train_variant.py"

if [ ! -f "$DATA" ]; then
    echo "ERROR: expected $DATA  (collect_closed_loop_data.py output)" >&2
    exit 1
fi
mkdir -p "$MODELS_OUT"

EPOCHS=300
LR=0.01
PATIENCE=50
BATCH=256
SEED=42

run_one() {
    local NAME="$1"; shift
    echo "============================================================"
    echo "== Training $NAME =="
    echo "============================================================"
    python "$TRAINER" \
        --data "$1" \
        --output-dir "${MODELS_OUT}/${NAME}" \
        --arch mlp \
        "${@:2}" \
        --epochs $EPOCHS --lr $LR --patience $PATIENCE \
        --batch-size $BATCH --seed $SEED \
        --split-by-scenario
}

# Static MLP 32-16 (matches closed_loop_v1_mlp_32_16; new LHS data)
run_one "vehicle_static_32_16_lhs" "$DATA" --mode static --hidden 32 16

# Rate MLP 32-16 (matches closed_loop_v2_both_axles_rate_32_16; new LHS data)
run_one "vehicle_rate_32_16_lhs"   "$DATA" --mode rate   --hidden 32 16

# Rate MLP 64-32 (matches rig_rate_64_32 capacity; new LHS data)
run_one "vehicle_rate_64_32_lhs"   "$DATA" --mode rate   --hidden 64 32

echo ""
echo "=== test metrics ==="
for NAME in vehicle_static_32_16_lhs vehicle_rate_32_16_lhs vehicle_rate_64_32_lhs; do
    META="${MODELS_OUT}/${NAME}/test_metrics.json"
    if [ -f "$META" ]; then
        echo "--- $NAME ---"
        python -c "
import json
d = json.load(open('$META'))
t = d.get('test', {})
print(f'  R2_fx={t.get(\"r2_fx\", float(\"nan\")):.4f}  R2_fy={t.get(\"r2_fy\", float(\"nan\")):.4f}  rmse_fx={t.get(\"rmse_fx\", float(\"nan\")):.1f}  rmse_fy={t.get(\"rmse_fy\", float(\"nan\")):.1f}')
"
    fi
done
