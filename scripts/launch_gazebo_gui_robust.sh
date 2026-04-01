#!/bin/bash
# launch_gazebo_gui_robust.sh
# Robust Gazebo GUI launcher with GPU acceleration

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK_DIR="$(dirname "$SCRIPT_DIR")"

# Get proper display
if [ -z "$DISPLAY" ]; then
    DISPLAY=:0
fi

export DISPLAY
export QT_QPA_PLATFORM=xcb
export QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

# GPU acceleration settings
export LIBGL_ALWAYS_INDIRECT=0
export NVIDIA_DRIVER_CAPABILITIES=graphics,compute
export WAYLAND_DISPLAY=

echo "═══════════════════════════════════════════════════════"
echo "  Gazebo GUI Launcher (GPU-Accelerated)"
echo "═══════════════════════════════════════════════════════"
echo ""
echo "[Launch] Display: $DISPLAY"
echo "[Launch] QT Platform: $QT_QPA_PLATFORM"
echo "[Launch] Working Dir: $(pwd)"
echo ""

# Allow X11 local connections
xhost +local: 2>/dev/null || true

# Kill any existing Gazebo processes
echo "[Launch] Cleaning up existing processes..."
pkill -f "gz sim" 2>/dev/null || true
sleep 1

# Check if world file exists
WORLD_FILE="${WORK_DIR}/simulation/worlds/iris_runway.sdf"
if [ ! -f "$WORLD_FILE" ]; then
    echo "[ERROR] World file not found: $WORLD_FILE"
    # Try default location
    if [ -f "${HOME}/.gz/sim/worlds/iris_runway.sdf" ]; then
        WORLD_FILE="${HOME}/.gz/sim/worlds/iris_runway.sdf"
        echo "[Launch] Using default world: $WORLD_FILE"
    else
        echo "[ERROR] Could not find iris_runway.sdf world file"
        exit 1
    fi
fi

echo "[Launch] Loading world: $WORLD_FILE"
echo ""
echo "[Launch] Starting Gazebo GUI..."
echo "[Launch] Command: /usr/bin/gz sim -v2 -r iris_runway.sdf"
echo ""

# Launch Gazebo - detached but with console logging
setsid /usr/bin/gz sim -v2 -r iris_runway.sdf </dev/null &>>$WORK_DIR/data/logs/gazebo_gui.log &

GZ_PID=$!
echo "[Launch] Gazebo PID: $GZ_PID"
echo "[Launch] Log: $WORK_DIR/data/logs/gazebo_gui.log"
echo ""

# Wait a bit for window to appear
sleep 3

# Check if process is still running
if ps -p $GZ_PID > /dev/null 2>&1; then
    echo "[Launch] ✓ Gazebo process running"
    echo "[Launch] ✓ GUI should be visible on display $DISPLAY"
    echo ""
    echo "═══════════════════════════════════════════════════════"
    echo "  To stop Gazebo: kill $GZ_PID"
    echo "  Or run: pkill -f 'gz sim'"
    echo "═══════════════════════════════════════════════════════"
else
    echo "[ERROR] Gazebo process exited unexpectedly"
    echo ""
    echo "[Launch] Recent log output:"
    tail -20 $WORK_DIR/data/logs/gazebo_gui.log
    exit 1
fi
