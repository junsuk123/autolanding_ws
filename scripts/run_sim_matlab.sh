#!/bin/bash
# run_sim_matlab.sh
# Execute full simulation (pipeline + autonomous mission)

set -e

# Get workspace root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "========================================"
echo "   AUTOLANDING SIMULATION LAUNCHER"
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
echo "[OK] MATLAB installed"

# Check Gazebo
if ! command -v gz &> /dev/null; then
    echo "[ERROR] Gazebo not found"
    echo "[ERROR] Install Gazebo and ensure iris_runway.sdf scenario is available"
    exit 1
fi
echo "[OK] Gazebo found"

# Check MAVProxy
if ! command -v mavproxy.py &> /dev/null; then
    echo "[ERROR] MAVProxy not found"
    echo "[ERROR] Install MAVProxy: pip install MAVProxy"
    exit 1
fi
echo "[OK] MAVProxy found"

# Diagnostic: Check for simulator processes
echo ""
echo "[DIAGNOSTICS] Checking for simulator processes..."
if pgrep -f "gz sim" > /dev/null; then
    echo "[OK] Gazebo simulator is running"
else
    echo "[WARNING] Gazebo simulator not detected"
    echo "[HINT] Start in separate terminal: gz sim -s iris_runway.sdf"
fi

if pgrep -f "arducopter.*JSON" > /dev/null; then
    echo "[OK] ArduPilot SITL is running"
else
    echo "[WARNING] ArduPilot SITL not detected"
    echo "[HINT] Start in separate terminal: cd ~/ardupilot && ./build/sitl/bin/arducopter --model JSON ..."
fi

# Test MAVProxy connection
echo ""
echo "[CONNECT] Testing MAVProxy connection to vehicle..."
vehicle_port=""
for PORT in 5762 5760; do
    if timeout 20 mavproxy.py --master tcp:127.0.0.1:${PORT} --cmd="status" 2>/dev/null | grep -q "Detected vehicle"; then
        vehicle_port="${PORT}"
        break
    fi
done
if [[ -n "$vehicle_port" ]]; then
    echo "[OK] Vehicle heartbeat confirmed at tcp:127.0.0.1:${vehicle_port}"
else
    echo "[WARNING] Vehicle not responding on tcp:127.0.0.1:5762 or tcp:127.0.0.1:5760"
    echo "[WARNING] Simulation will proceed but vehicle control may fail"
fi

# Run MATLAB simulation
echo ""
echo "[LAUNCH] Starting MATLAB full simulation..."
cd "$WS_ROOT"

matlab -batch "run('matlab/run_autolanding_sim.m'); exit"

echo ""
echo "========================================"
echo "   SIMULATION LAUNCHER COMPLETE"
echo "========================================"
echo ""
echo "[RESULTS] Check output files in: $WS_ROOT/data/processed/"
