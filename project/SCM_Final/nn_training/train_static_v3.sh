#!/bin/bash
# Train all static NN tire-surrogate variants on the rig-style scm_static_100k_v4.csv.
# This dataset is NOT shipped in the SCM_Final snapshot; re-collect it via
# data_collection/collect_static_data.cpp (Chrono SCM tire rig), or set
# DATA=/path/to/your/csv before running.
set -e

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
DATA="${DATA:-${ROOT}/data/tire_rig/scm_static_100k_v4.csv}"
MODELS="${MODELS:-${ROOT}/nn_models}"
TRAINER="${TRAINER:-${ROOT}/nn_training/train_variant.py}"

if [ ! -f "$DATA" ]; then
    echo "ERROR: dataset not found at $DATA"
    echo "Either re-collect via data_collection/collect_static_data.cpp"
    echo "or export DATA=/path/to/scm_static_100k_v4.csv before running."
    exit 1
fi

EPOCHS=300
LR=0.01
PATIENCE=50
BATCH=256
SEED=42

echo "=== Training static models on v4 dataset ==="
echo "Data: $DATA"
echo "Rows: $(wc -l < "$DATA")"
echo ""

# MLP variants
for H in "12 2" "16 4" "16 8" "24 12" "32 16"; do
    NAME="paper_v2_mlp_${H// /_}"
    echo "--- Training $NAME ---"
    python "$TRAINER" \
        --data "$DATA" \
        --output-dir "${MODELS}/${NAME}" \
        --arch mlp --mode static \
        --hidden $H \
        --epochs $EPOCHS --lr $LR --patience $PATIENCE --batch-size $BATCH --seed $SEED
    echo ""
done

# ResNet variants
for HD_NB in "8 2" "16 2" "16 4" "32 2"; do
    HD=$(echo $HD_NB | cut -d' ' -f1)
    NB=$(echo $HD_NB | cut -d' ' -f2)
    NAME="paper_v2_resnet_h${HD}_b${NB}"
    echo "--- Training $NAME ---"
    python "$TRAINER" \
        --data "$DATA" \
        --output-dir "${MODELS}/${NAME}" \
        --arch resnet --mode static \
        --hidden-dim $HD --n-blocks $NB \
        --epochs $EPOCHS --lr $LR --patience $PATIENCE --batch-size $BATCH --seed $SEED
    echo ""
done

echo "=== All static models trained ==="
echo ""
echo "=== Test Metrics Summary ==="
for d in ${MODELS}/paper_v2_mlp_* ${MODELS}/paper_v2_resnet_*; do
    if [ -f "$d/test_metrics.json" ] && [[ ! "$d" =~ temporal ]] && [[ ! "$d" =~ rate ]] && [[ ! "$d" =~ gru ]]; then
        name=$(basename "$d")
        r2fx=$(python3 -c "import json; m=json.load(open('$d/test_metrics.json')); print(f\"{m['test']['r2_fx']:.4f}\")")
        r2fy=$(python3 -c "import json; m=json.load(open('$d/test_metrics.json')); print(f\"{m['test']['r2_fy']:.4f}\")")
        echo "  $name: R²_Fx=$r2fx  R²_Fy=$r2fy"
    fi
done
