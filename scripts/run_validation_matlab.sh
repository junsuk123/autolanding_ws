#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

# Source cleanup utilities
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "$SCRIPT_DIR/cleanup_utils.sh"

# Signal handlers
on_interrupt() {
	echo ""
	echo "[interrupt] Received SIGINT, cleaning up..."
	cleanup_all_processes
	exit 130
}

cleanup_on_exit() {
	local exit_code="$?"
	if [[ $exit_code -ne 130 ]]; then
		cleanup_all_processes
	fi
	exit "$exit_code"
}

# Setup traps
trap on_interrupt INT TERM
trap cleanup_on_exit EXIT

if command -v matlab >/dev/null 2>&1; then
	matlab -batch "run('matlab/scripts/run_autolanding_validation.m')"
else
	echo "[error] matlab command not found."
	exit 1
fi
