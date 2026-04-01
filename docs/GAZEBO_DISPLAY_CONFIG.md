# Gazebo Display Configuration Guide

## Current Situation

Your system has:
- ✓ NVIDIA RTX 4060 GPU with proper drivers (v570.172.08)
- ✓ X11 display server running (:0)
- ✓ All Gazebo/Qt5 libraries installed
- ✗ GPU-accelerated rendering failing (EGL/DRI2 connection issue)

## Problem

When Gazebo GUI mode attempts to launch, it initializes correctly but **EGL rendering fails**:
```
libEGL warning: egl: failed to create dri2 screen
```

This occurs because the X11 display server isn't properly connected to GPU-accelerated rendering, likely due to:
1. X server running with dummy/software renderer instead of NVIDIA
2. Display manager configuration not connecting to GPU
3. NVIDIA/X11 driver integration issue

## Solutions

### ✓ **RECOMMENDED: Use Server Mode for Data Collection**
The pipeline works **perfectly** in server mode (headless). You get:
- ✓ Fast data collection without GPU rendering bottleneck
- ✓ Real-time monitoring/visualization during collection
- ✓ All sensor data saved for post-processing
- ✓ No display issues or window management problems

**Configuration:**
```matlab
gazebo_server_mode = true;           % Use headless mode
enable_visualization = true;          % Enable real-time monitoring
```

### **Option 2: Fix GPU-Accelerated X11 (Advanced)**

If you want GUI mode, try these steps:

#### Step 1: Check X11 configuration
```bash
DISPLAY=:0 grep -i "nvidia\|gpu" /var/log/Xvfb.log 2>/dev/null || echo "No Xvfb"
```

#### Step 2: Restart X server with NVIDIA driver
```bash
# Check which display manager is running
ps aux | grep -E "gdm3|lightdm|sddm|xdm"

# Optionally: try explicit NVIDIA display setup
export DISPLAY=:0
export LIBGL_DEBUG=verbose
/usr/bin/glxgears 2>&1 | head -20
```

#### Step 3: If glxgears fails, configure X11 for NVIDIA
```bash
sudo nvidia-xconfig
```

Then restart the display manager:
```bash
sudo systemctl restart gdm3  # or appropriate display manager
```

### **Option 3: Use VNC for Remote Gazebo GUI**

If GPU rendering remains an issue, use VNC to access Gazebo GUI remotely:

```bash
# Start VNC server (optional)
vncserver :99 -geometry 1920x1080 -depth 24

# Launch Gazebo on VNC display
export DISPLAY=:99
/usr/bin/gz sim -v2 -r iris_runway.sdf &

# Connect from client machine
vncviewer hostname:99
```

### **Option 4: Post-Processing Visualization**

Keep using server mode and visualize collected data afterward:

```matlab
% Use server mode during collection
gazebo_server_mode = true;

% After collection, analyze and visualize data from saved logs
% See: /autolanding_ws/data/collected/parallel/[session_id]/
```

## Current Pipeline Configuration

Your `AutoLandingMainFull.m` is set to:
```
gazebo_server_mode = true          → Uses efficient headless mode ✓
enable_visualization = true        → Real-time monitoring still active ✓
```

## Recommended Action

**Run the pipeline as-is** with server mode. The system will:
1. Collect data efficiently in headless mode
2. Monitor collection in real-time via visualization
3. Train models successfully
4. Generate comparison plots

To manually test GUI mode fix:
```bash
# Test rendering capability
bash /home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws/scripts/test_gazebo_display.sh

# Check window status
bash /home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws/scripts/check_gazebo_window.sh

# Launch with robust configuration
bash /home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws/scripts/launch_gazebo_gui_robust.sh
```

## Status

- ✓ Pipeline works correctly in server mode
- ✓ Data collection functional and optimized
- ✓ Model training and comparison working
- ⚠ GUI requires additional X11/GPU driver debugging (not blocking pipeline)

---

*Last updated: 2026-03-31*
