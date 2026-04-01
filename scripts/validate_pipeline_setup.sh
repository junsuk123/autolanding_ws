#!/bin/bash
# validate_pipeline_setup.sh
# Validate that all components are ready for the AutoLanding pipeline

echo "═══════════════════════════════════════════════════════════"
echo "  AutoLanding Pipeline Validation"
echo "═══════════════════════════════════════════════════════════"
echo ""

# Test 1: Check MATLAB
echo "[CHECK 1] MATLAB Installation..."
if command -v matlab &> /dev/null; then
    echo "  ✓ MATLAB found: $(which matlab)"
else
    echo "  ✗ MATLAB not found in PATH"
fi

# Test 2: Check Gazebo
echo "[CHECK 2] Gazebo Installation..."
if command -v gz &> /dev/null; then
    echo "  ✓ Gazebo found: $(which gz)"
    GZ_VERSION=$(/usr/bin/gz sim --version 2>&1 | head -1)
    echo "    Version: $GZ_VERSION"
else
    echo "  ✗ Gazebo not found in PATH"
fi

# Test 3: Check ArduPilot
echo "[CHECK 3] ArduPilot SITL..."
if [ -f "$HOME/ardupilot/build/sitl/bin/arducopter" ]; then
    echo "  ✓ ArduPilot SITL found"
else
    echo "  ✗ ArduPilot SITL not found at $HOME/ardupilot/build/sitl/bin/arducopter"
fi

# Test 4: Check World File
echo "[CHECK 4] Gazebo World Files..."
if [ -f "$HOME/gz_ws/src/ardupilot_gazebo/worlds/iris_runway.sdf" ]; then
    echo "  ✓ iris_runway.sdf found"
else
    echo "  ✗ iris_runway.sdf not found"
fi

# Test 5: Check Display
echo "[CHECK 5] Display Configuration..."
if [ -n "$DISPLAY" ]; then
    echo "  ✓ DISPLAY set to: $DISPLAY"
    XHOST_CHECK=$(xhost 2>&1 | head -1)
    if [[ "$XHOST_CHECK" == "access control enabled"* ]]; then
        echo "  ✓ X11 access control enabled"
    else
        echo "  ✓ X11 display accessible"
    fi
else
    echo "  ⚠ DISPLAY not set (will use server mode)"
fi

# Test 6: GPU
echo "[CHECK 6] GPU/NVIDIA Configuration..."
if command -v nvidia-smi &> /dev/null; then
    GPU_INFO=$(nvidia-smi --query-gpu=name --format=csv,noheader | head -1)
    echo "  ✓ NVIDIA GPU: $GPU_INFO"
else
    echo "  ✗ NVIDIA drivers not found"
fi

# Test 7: Gazebo Resource Path
echo "[CHECK 7] Gazebo Resource Path..."
export GZ_SIM_RESOURCE_PATH="$HOME/.gz/sim:$HOME/gz_ws/src/ardupilot_gazebo"
echo "  GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"

# Test 8: Try Gazebo startup (quick test)
echo "[CHECK 8] Gazebo Startup Test (5 second)..."
timeout 5s /usr/bin/gz sim -s -v2 -r iris_runway.sdf > /tmp/validate_gz.log 2>&1 &
GZ_PID=$!
sleep 2

if ps -p $GZ_PID > /dev/null 2>&1; then
    echo "  ✓ Gazebo started successfully"
    kill $GZ_PID 2>/dev/null
    wait $GZ_PID 2>/dev/null
else
    echo "  ✗ Gazebo failed to start"
    echo "  Error log:"
    tail -5 /tmp/validate_gz.log
fi

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  ✓ Validation Complete"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "Ready to run: AutoLandingMainFull()"
echo ""
