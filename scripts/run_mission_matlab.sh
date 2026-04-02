#!/bin/bash
# run_mission_matlab.sh
# Execute autonomous landing mission via MATLAB

set -e

# Get workspace root (2 levels up from scripts dir)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

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
for PORT in 5762 5760; do
    if timeout 20 mavproxy.py --master tcp:127.0.0.1:${PORT} --cmd="status" 2>/dev/null | grep -q "Detected vehicle"; then
        vehicle_port="${PORT}"
        break
    fi
done
if [[ -n "$vehicle_port" ]]; then
    echo "[OK] Vehicle connection available at tcp:127.0.0.1:${vehicle_port}"
else
    echo "[WARNING] Could not connect to vehicle at tcp:127.0.0.1:5762 or tcp:127.0.0.1:5760"
    echo "[WARNING] Ensure Gazebo + ArduPilot SITL are running:"
    echo "  Terminal 1: gz sim -s iris_runway.sdf"
    echo "  Terminal 2: ./build/sitl/bin/arducopter --model JSON ..."
fi

# Run MATLAB mission
echo ""
echo "[LAUNCH] Starting MATLAB mission..."
cd "$WS_ROOT"

matlab -batch "run('matlab/run_autolanding_mission.m'); exit"

echo ""
echo "========================================"
echo "   MISSION LAUNCHER COMPLETE"
echo "========================================"
