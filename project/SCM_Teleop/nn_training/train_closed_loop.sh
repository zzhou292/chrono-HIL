#!/bin/bash
# Train an NN tire surrogate on a *closed-loop* dataset collected with
# data_collection/collect_closed_loop_data.py.  The schema is identical
# to the rig-collected static CSV (scenario_id, timestep, slip_ratio,
# slip_angle, velocity, vertical_load, steering_rate, bekker_*,
# mohr_*, janosi_shear, mesh_spacing, Fx, Fy), so train_variant.py
# consumes it as-is in either ``--mode static`` or ``--mode temporal``.
#
# The temporal variant is the one that should actually beat the
# paper_v2 static surrogate on closed-loop validation, because per-wheel
# Fy in closed-loop motion is multi-valued in instantaneous (slip, Fz,
# u, kappa) alone — history disambiguates.
#
# Usage:
#   ./train_closed_loop.sh /abs/path/to/training_data.csv [out_dir]

set -e

DATA="${1:?'Usage: train_closed_loop.sh /abs/path/to/training_data.csv [out_dir]'}"
OUT_BASE="${2:-/home/ksha/Documents/sbel/chrono_hil/chrono-HIL/project/SCM_Teleop/nn_models}"
TRAINER="$(dirname "$(readlink -f "$0")")/train_variant.py"

EPOCHS=300
LR=0.01
PATIENCE=50
BATCH=256
SEED=42

if [ ! -f "$DATA" ]; then
    echo "Training CSV not found: $DATA" >&2
    exit 1
fi

echo "=== Training closed-loop tire surrogates ==="
echo "Data: $DATA  ($(wc -l < "$DATA") rows incl. header)"
echo "Output base: $OUT_BASE"
echo ""

# ---- Static MLP: 16,4 (matches the existing paper_v2 size for fair comparison)
NAME="closed_loop_v1_mlp_16_4"
echo "--- Training $NAME (static MLP 16-4) ---"
python "$TRAINER" \
    --data "$DATA" --output-dir "${OUT_BASE}/${NAME}" \
    --arch mlp --mode static --hidden 16 4 \
    --epochs $EPOCHS --lr $LR --patience $PATIENCE \
    --batch-size $BATCH --seed $SEED

# ---- Static MLP: 32,16 (larger, sees if extra capacity helps with the
# more-complex closed-loop distribution)
NAME="closed_loop_v1_mlp_32_16"
echo ""
echo "--- Training $NAME (static MLP 32-16) ---"
python "$TRAINER" \
    --data "$DATA" --output-dir "${OUT_BASE}/${NAME}" \
    --arch mlp --mode static --hidden 32 16 \
    --epochs $EPOCHS --lr $LR --patience $PATIENCE \
    --batch-size $BATCH --seed $SEED

# ---- Temporal MLP K=4: per-wheel history helps the NN model transient
# Fy that single-step slip cannot explain.
NAME="closed_loop_v1_mlp_temporal_K4_16_8"
echo ""
echo "--- Training $NAME (temporal MLP K=4, 16-8) ---"
python "$TRAINER" \
    --data "$DATA" --output-dir "${OUT_BASE}/${NAME}" \
    --arch mlp --mode temporal --K 4 --dt-nn 0.05 --record-dt 0.01 \
    --hidden 16 8 \
    --epochs $EPOCHS --lr $LR --patience $PATIENCE \
    --batch-size $BATCH --seed $SEED

echo ""
echo "=== Training summary ==="
for NAME in closed_loop_v1_mlp_16_4 closed_loop_v1_mlp_32_16 closed_loop_v1_mlp_temporal_K4_16_8; do
    META="${OUT_BASE}/${NAME}/test_metrics.json"
    if [ -f "$META" ]; then
        echo "  ${NAME}:"
        python3 -c "
import json
m = json.load(open('$META'))['test']
print(f'    R²_Fx={m[\"r2_fx\"]:.4f}  R²_Fy={m[\"r2_fy\"]:.4f}  RMSE_Fx={m[\"rmse_fx\"]:.1f}  RMSE_Fy={m[\"rmse_fy\"]:.1f}')
"
    else
        echo "  ${NAME}: (no metrics — training may have failed)"
    fi
done
