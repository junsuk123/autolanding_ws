#!/bin/bash
# run_mission_matlab.sh
# Execute autonomous landing mission via MATLAB

set -e

# Get workspace root (2 levels up from scripts dir)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Source cleanup utilities
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

echo "========================================"
echo "   AUTOLANDING MISSION LAUNCHER"
echo "========================================"
echo ""
echo "[INFO] Workspace: $WS_ROOT"
echo "[INFO] Script: $SCRIPT_DIR"

# Check prerequisites
echo ""
echo "[CHECK] Checking prerequisites..."

# Check MATLAB
if ! command -v matlab &> /dev/null; then
    echo "[ERROR] MATLAB not found in PATH"
    exit 1
fi
echo "[OK] MATLAB found"

# Check Gazebo
if ! command -v gz &> /dev/null; then
    echo "[WARNING] Gazebo not found - simulation may not work"
else
    echo "[OK] Gazebo found"
fi

# Check ArduPilot SITL
if ! command -v ardupilot &> /dev/null; then
    echo "[WARNING] ArduPilot not found"
fi

# Check MAVProxy
if ! command -v mavproxy.py &> /dev/null; then
    echo "[WARNING] MAVProxy not found - vehicle control may not work"
else
    echo "[OK] MAVProxy found"
fi

# Check for MAVProxy connection
echo ""
echo "[CONNECT] Testing MAVProxy connection..."
vehicle_port=""
if timeout 20 mavproxy.py --master udpin:127.0.0.1:14550 --cmd="status" 2>/dev/null | grep -q "Detected vehicle"; then
    vehicle_port="14550"
fi
if [[ -n "$vehicle_port" ]]; then
    echo "[OK] Vehicle connection available at udpin:127.0.0.1:${vehicle_port}"
else
    echo "[WARNING] Could not connect to vehicle at udpin:127.0.0.1:14550"
    echo "[WARNING] Ensure Gazebo + ArduPilot SITL are running:"
    echo "  Terminal 1: gz sim -s iris_runway.sdf"
    echo "  Terminal 2: ./build/sitl/bin/arducopter --model JSON:127.0.0.1 ..."
fi

# Run MATLAB mission launcher
echo ""
echo "[LAUNCH] Starting MATLAB mission launcher..."
cd "$WS_ROOT"

matlab -batch "run('matlab/run_autolanding_mission.m')"

echo ""
echo "========================================"
echo "   MISSION LAUNCHER COMPLETE"
echo "========================================"
