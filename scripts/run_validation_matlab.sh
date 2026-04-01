#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

matlab -batch "run('matlab/scripts/run_autolanding_validation.m')"
