#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CFG="$ROOT_DIR/ai/configs/orchestration_config.yaml"

# 이 스크립트 하나로 전체 파이프라인(full 모드)을 실행합니다.
python3 "$ROOT_DIR/scripts/autolanding_launcher.py" full \
  --workspace-root "$ROOT_DIR" \
  --orchestration-config "$CFG"
