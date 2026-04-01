#!/bin/bash
# launch_gazebo_gui.sh
# Proper Gazebo GUI launcher with environment fixes

DISPLAY=${DISPLAY:-:0}
export DISPLAY
export QT_QPA_PLATFORM=xcb
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
export QT_DEBUG_PLUGINS=1

echo "[Gazebo] Starting with DISPLAY=$DISPLAY"
echo "[Gazebo] QT_QPA_PLATFORM=$QT_QPA_PLATFORM"

# Allow X11 connection
xhost +local: 2>/dev/null || true

# Start Gazebo - keep terminal open for debugging
gz sim -v2 -r iris_runway.sdf
