#!/bin/bash
# Run v3 benchmark simulations one at a time
# Usage: bash run_v3_sims.sh

set -e
cd "$(dirname "$0")"
SIM_DIR="$(pwd)"

run_sim() {
    local label="$1"
    local model="$2"
    
    echo ""
    echo "=================================================================="
    echo "  Running: $label -> $model"
    echo "=================================================================="
    
    python3 launch_decoupled.py \
        --model nn --nn-model "$model" \
        --path double_lane_change --terrain clay \
        --time 30 --speed 5.0 --lead-in 10.0 \
        --no-vis --plot-dir "plots/v3_study/${label}" 2>&1 | tail -20
    
    echo "  Done: $label"
}

# v1 baselines (already ran v1_mlp_16_4)
run_sim "v1_resnet_h16_b2"    "paper_v1_resnet_h16_b2"
run_sim "v1_mlp_32_16"        "paper_v1_mlp_32_16"

# v3 scaled-up MLPs
run_sim "v3_mlp_64_32"        "paper_v3_mlp_64_32"
run_sim "v3_mlp_128_64"       "paper_v3_mlp_128_64"
run_sim "v3_mlp_128_64_32"    "paper_v3_mlp_128_64_32"

# v3 ResNets
run_sim "v3_resnet_h64_b2"    "paper_v3_resnet_h64_b2"
run_sim "v3_resnet_h64_b4"    "paper_v3_resnet_h64_b4"

# v3 DenseNets
run_sim "v3_dense_d16_l3"     "paper_v3_densenet_d16_l3"
run_sim "v3_dense_d32_l3"     "paper_v3_densenet_d32_l3"
run_sim "v3_dense_d32_l4"     "paper_v3_densenet_d32_l4"

# v3 training tricks
run_sim "v3_mlp_64_32_jac"    "paper_v3_mlp_64_32_jac"
run_sim "v3_mlp_64_32_sl1"    "paper_v3_mlp_64_32_sl1"
run_sim "v3_mlp_16_4_jac"     "paper_v3_mlp_16_4_jac"

echo ""
echo "All simulations complete!"
