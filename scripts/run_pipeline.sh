#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
python3 "$ROOT_DIR/scripts/autolanding_launcher.py" pipeline \
  --workspace-root "$ROOT_DIR" \
  --semantic-input "$ROOT_DIR/data/samples/semantic_input_example.json" \
  --config "$ROOT_DIR/ai/configs/policy_config.yaml"

echo "[done] Outputs written to:"
echo "  $ROOT_DIR/data/processed/landing_trajectory.json"
echo "  $ROOT_DIR/data/processed/landing_trajectory.csv"
