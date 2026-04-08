#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

if command -v matlab >/dev/null 2>&1; then
	matlab -batch "run('matlab/scripts/run_autolanding_validation.m')"
else
	echo "[error] matlab command not found."
	exit 1
fi
