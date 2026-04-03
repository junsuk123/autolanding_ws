# Deprecated Shell Scripts

These shell scripts have been replaced by Python-based launchers for better maintainability and robustness.

## Replacement Mapping

| Deprecated Shell Script | Python Replacement | Notes |
|------------------------|--------------------|-------|
| `launch_multi_drone_sitl.sh` | `launch_multi_drone_py.py` | Starts N ArduPilot SITL instances |
| `launch_multi_mavros.sh` | `launch_multi_mavros_py.py` | Starts N MAVROS instances with namespacing |
| `launch_rviz_monitor.sh` | `launch_rviz_monitor_py.py` | Launches RViz + ArUco + ros_gz_bridge |

## Why Python?

1. **Cleaner Environment Handling**: Python scripts properly manage `GZ_SIM_RESOURCE_PATH`, fixing I1/I2 JSON readiness issues
2. **Better Error Handling**: Structured error checking and logging
3. **No Shell Escaping Issues**: Complex strings and paths are handled naturally by Python
4. **Maintainability**: Self-contained with clear docstrings and arguments
5. **Direct MATLAB Integration**: Called directly from MATLAB with minimal complexity

## Migration Status

All functionality has been migrated to Python launchers. Active deployment uses:
- [launch_multi_drone_py.py](../launch_multi_drone_py.py)
- [launch_gazebo_py.py](../launch_gazebo_py.py)
- [launch_multi_mavros_py.py](../launch_multi_mavros_py.py)
- [launch_rviz_monitor_py.py](../launch_rviz_monitor_py.py)

These are called automatically by `AutoLandingMainFull.m` in the STEP 0 initialization phase.

## Archival

These scripts are kept for reference only. Do not use them in production.
