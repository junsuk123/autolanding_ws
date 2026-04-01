#!/bin/bash
# safe_start_gazebo.sh
# Safely start Gazebo with proper environment and model discovery
# Used by MATLAB when spawning Gazebo processes

set -e

# Get configuration from arguments
MODE="${1:-server}"  # server or gui
HOME_DIR="${HOME:-.}"
WORK_DIR="${WORK_DIR:-.}"

# Verify required files exist
WORLD_FILE="$HOME_DIR/gz_ws/src/ardupilot_gazebo/worlds/iris_runway.sdf"
MODEL_DIR="$HOME_DIR/gz_ws/src/ardupilot_gazebo/models"

if [ ! -f "$WORLD_FILE" ]; then
    echo "ERROR: World file not found: $WORLD_FILE" >&2
    exit 1
fi

if [ ! -d "$MODEL_DIR" ]; then
    echo "ERROR: Model directory not found: $MODEL_DIR" >&2
    exit 1
fi

# Setup environment
export GZ_MODEL_PATH="$MODEL_DIR:$GZ_MODEL_PATH"
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"

echo "[SafeStart] World: $WORLD_FILE"
echo "[SafeStart] Models: $MODEL_DIR"
echo "[SafeStart] Mode: $MODE"

# Start Gazebo with appropriate parameters
if [ "$MODE" = "server" ]; then
    # Server/headless mode
    export DISPLAY=:0
    export QT_QPA_PLATFORM=offscreen
    exec /usr/bin/gz sim -s -v2 -r "$WORLD_FILE"
else
    # GUI mode
    export DISPLAY=:0
    export QT_QPA_PLATFORM=xcb
    export QT_QPA_PLATFORM_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/qt5/plugins
    xhost +local: 2>/dev/null || true
    exec /usr/bin/gz sim -v2 -r "$WORLD_FILE"
fi
