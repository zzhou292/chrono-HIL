#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "[verify] compile check"
python -m compileall simulation test_suite utilities my_paper

echo "[verify] offline learned-estimator replay"
python test_suite/eval_learned_estimator.py

echo "[verify] closed-loop learned-estimator check"
python test_suite/validate_closed_loop_estimator.py \
  --terrains clay dirt sand \
  --duration 20

echo "[verify] random-terrain generation"
python utilities/generate_random_terrains.py --n-terrains 2 --seed 42

echo "[verify] random-terrain closed-loop validation"
python test_suite/validate_random_terrains_closed_loop.py \
  --terrains terrain1 terrain2 \
  --duration 20

echo "[verify] compact tire-model benchmark"
python utilities/bench_tire_models.py \
  --workers 2 \
  --repeats 1 \
  --models Pacejka TMeasy MLP

echo "[verify] joint n/phi experiment"
python utilities/exp_joint_n_phi.py \
  --epochs 80 \
  --trace-dir data/terrain_traces data/terrain_traces_rich \
  --yaml-dir data/terrain_yamls data/terrain_yamls_rich

echo "[verify] done"
