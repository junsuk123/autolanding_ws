# Local ROS2 ArduPilot Workspace (Vendored)

This directory vendors the `~/ardu_ws/src` tree into the main repository so it can be versioned together with `autolanding_ws`.

## Included
- `src/` (ArduPilot ROS2/Gazebo source trees)

## Excluded
- `build/`
- `install/`
- `log/`
- `.venv/`

## Build
```bash
cd ardu_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to ardupilot_gz_bringup --cmake-args -DARDUPILOT_ENABLE_DDS=OFF
```

## Verify
```bash
source /opt/ros/humble/setup.bash
source ardu_ws/install/setup.bash
ros2 pkg prefix ardupilot_gz_bringup
```
