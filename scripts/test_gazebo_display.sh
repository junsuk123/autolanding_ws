#!/bin/bash
# test_gazebo_display.sh
# Test script to verify Gazebo can launch with GUI

echo "═══════════════════════════════════════════════════════"
echo "  Gazebo GUI Display Test"
echo "═══════════════════════════════════════════════════════"

echo ""
echo "[TEST] Checking display configuration..."
echo "  DISPLAY: $DISPLAY"
echo "  WAYLAND_DISPLAY: $WAYLAND_DISPLAY"

echo ""
echo "[TEST] Checking X11 connection..."
xhost > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "  ✓ X11 display is accessible"
else
    echo "  ✗ X11 display NOT accessible"
    exit 1
fi

echo ""
echo "[TEST] Checking Gazebo installation..."
which gz > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "  ✓ gz command found: $(which gz)"
else
    echo "  ✗ gz command NOT found"
    exit 1
fi

echo ""
echo "[TEST] Checking Qt5 libraries..."
ldd /usr/bin/gz 2>/dev/null | grep -i qt5 | head -3
if [ $? -eq 0 ]; then
    echo "  ✓ Qt5 libraries detected"
else
    echo "  ! Could not verify Qt5 libraries"
fi

echo ""
echo "[TEST] Killing any existing Gazebo processes..."
pkill -f "gz sim" 2>/dev/null
sleep 2

echo ""
echo "[TEST] Attempting to launch Gazebo GUI..."
echo "  Command: gz sim -v2 -r iris_runway.sdf"
echo "  Log: /tmp/gz_test.log"

export DISPLAY=:0
export QT_QPA_PLATFORM=xcb
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH

timeout 10s gz sim -v2 -r iris_runway.sdf > /tmp/gz_test.log 2>&1 &
GZ_PID=$!

echo "  Started PID: $GZ_PID"
sleep 3

if ps -p $GZ_PID > /dev/null; then
    echo "  ✓ Gazebo process is running"
    kill $GZ_PID 2>/dev/null
    echo "  ✓ Gazebo killed successfully"
else
    echo "  ✗ Gazebo process exited immediately"
    echo ""
    echo "[ERROR LOG]"
    cat /tmp/gz_test.log
    exit 1
fi

echo ""
echo "═══════════════════════════════════════════════════════"
echo "  ✓ Gazebo GUI test PASSED"
echo "═══════════════════════════════════════════════════════"
