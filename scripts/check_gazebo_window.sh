#!/bin/bash
# check_gazebo_window.sh
# Check if Gazebo GUI window is running

DISPLAY=:0

echo "═══════════════════════════════════════════════════════"
echo "  Gazebo Window Status Check"
echo "═══════════════════════════════════════════════════════"
echo ""

# Check if Gazebo process is running
if pgrep -f "gz sim" > /dev/null; then
    echo "✓ Gazebo process is RUNNING"
    GZ_PID=$(pgrep -f "gz sim" | head -1)
    echo "  PID: $GZ_PID"
    ps -p $GZ_PID -o cmd=
    echo ""
else
    echo "✗ Gazebo process is NOT running"
    exit 1
fi

# Check X11 windows
echo "[Check] Looking for Gazebo windows on $DISPLAY..."

# List open windows
if command -v wmctrl &> /dev/null; then
    WINDOWS=$(wmctrl -l | grep -i "gazebo\|gz")
    if [ -n "$WINDOWS" ]; then
        echo "✓ Found Gazebo windows:"
        echo "$WINDOWS"
    else
        echo "! No Gazebo windows found with wmctrl"
        echo ""
        echo "[Info] All open windows:"
        wmctrl -l | head -10
    fi
else
    echo "! wmctrl not installed, using xdotool..."
    if command -v xdotool &> /dev/null; then
        GAZEBO_WINDOW=$(xdotool search --name "gui" 2>/dev/null | head -1)
        if [ -n "$GAZEBO_WINDOW" ]; then
            echo "✓ Found Gazebo window ID: $GAZEBO_WINDOW"
            xdotool windowactivate $GAZEBO_WINDOW 2>/dev/null || true
            echo "✓ Attempted to bring window to focus"
        else
            echo "! Gazebo window not found in xdotool search"
        fi
    else
        echo "! xdotool not installed"
    fi
fi

echo ""
echo "[Debug] Gazebo log output:"
tail -10 /tmp/gz_sim.log 2>/dev/null | head -5

echo ""
echo "═══════════════════════════════════════════════════════"
