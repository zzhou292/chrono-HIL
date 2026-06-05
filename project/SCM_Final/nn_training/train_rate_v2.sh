#!/bin/bash
# Train rate-augmented NN tire-surrogate variants on the rig-style rate_v1_100k.csv.
# This dataset is NOT shipped in the SCM_Final snapshot; re-collect it via
# data_collection/collect_rate_data.cpp (Chrono SCM tire rig with steering rate),
# or set DATA=/path/to/your/csv before running.
set -e

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DATA="${DATA:-${ROOT}/data/tire_rig/rate_v2_100k.csv}"
MODELS="${MODELS:-${ROOT}/nn_models}"
TRAINER="${TRAINER:-${ROOT}/nn_training/train_variant.py}"

if [ ! -f "$DATA" ]; then
    echo "ERROR: dataset not found at $DATA"
    echo "Either re-collect via data_collection/collect_rate_data.cpp"
    echo "or export DATA=/path/to/rate_v1_100k.csv before running."
    exit 1
fi

EPOCHS=300
LR=0.01
PATIENCE=50
BATCH=256
SEED=42

echo "=== Training rate models on rate_v1_100k dataset ==="
echo "Data: $DATA"
echo "Rows: $(wc -l < "$DATA")"
echo ""

# MLP rate variants (skip 16_8 and 24_12 which already exist)
for H in "12 2" "16 4" "32 16"; do
    NAME="paper_v2_mlp_rate_${H// /_}"
    if [ -d "${MODELS}/${NAME}" ]; then
        echo "--- Skipping $NAME (already exists) ---"
        continue
    fi
    echo "--- Training $NAME ---"
    python "$TRAINER" \
        --data "$DATA" \
        --output-dir "${MODELS}/${NAME}" \
        --arch mlp --mode rate \
        --hidden $H \
        --epochs $EPOCHS --lr $LR --patience $PATIENCE --batch-size $BATCH --seed $SEED
    echo ""
done

# ResNet rate variants
for HD_NB in "8 2" "16 2" "16 4" "32 2"; do
    HD=$(echo $HD_NB | cut -d' ' -f1)
    NB=$(echo $HD_NB | cut -d' ' -f2)
    NAME="paper_v2_resnet_rate_h${HD}_b${NB}"
    if [ -d "${MODELS}/${NAME}" ]; then
        echo "--- Skipping $NAME (already exists) ---"
        continue
    fi
    echo "--- Training $NAME ---"
    python "$TRAINER" \
        --data "$DATA" \
        --output-dir "${MODELS}/${NAME}" \
        --arch resnet --mode rate \
        --hidden-dim $HD --n-blocks $NB \
        --epochs $EPOCHS --lr $LR --patience $PATIENCE --batch-size $BATCH --seed $SEED
    echo ""
done

echo "=== All rate models trained ==="
echo ""
echo "=== Test Metrics Summary ==="
for d in ${MODELS}/paper_v2_mlp_rate_* ${MODELS}/paper_v2_resnet_rate_*; do
    if [ -f "$d/test_metrics.json" ]; then
        name=$(basename "$d")
        r2fx=$(python3 -c "import json; m=json.load(open('$d/test_metrics.json')); print(f\"{m['test']['r2_fx']:.4f}\")")
        r2fy=$(python3 -c "import json; m=json.load(open('$d/test_metrics.json')); print(f\"{m['test']['r2_fy']:.4f}\")")
        echo "  $name: R²_Fx=$r2fx  R²_Fy=$r2fy"
    fi
done
