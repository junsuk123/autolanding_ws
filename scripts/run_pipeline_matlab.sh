#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

python3 "$ROOT_DIR/scripts/autolanding_launcher.py" pipeline \
  --workspace-root "$ROOT_DIR" \
  --semantic-input "$ROOT_DIR/data/samples/semantic_input_example.json" \
  --config "$ROOT_DIR/ai/configs/policy_config.yaml"
