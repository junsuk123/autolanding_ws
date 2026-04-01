# ArUco Marker Landing Pad Setup (ROS2 Humble + Gazebo Garden)

This workspace supports an ArUco-based fixed landing pad for trajectory generation and data collection.

## 1. External detector workspace setup

Use Ubuntu 22.04, ROS2 Humble, Gazebo Garden, Python 3.10.

```bash
mkdir -p ~/gz_ros2_aruco_ws/src
cd ~/gz_ros2_aruco_ws/src
git clone https://github.com/SaxionMechatronics/ros2-gazebo-aruco.git
git clone https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco.git
```

Update detector parameters:
- File: `~/gz_ros2_aruco_ws/src/ros2_aruco/ros2_aruco/config/aruco_parameters.yaml`
- Set `image_topic: /camera`
- Set `marker_size: 0.4`

Note:
- This repository now injects the ArUco model from `ros2-gazebo-aruco/gz-world/aruco_box` into the generated world.
- Keep detector marker size aligned with that model's tutorial default (`0.4`) to avoid unstable pose estimates.

Build and dependencies:

```bash
cd ~/gz_ros2_aruco_ws
colcon build
sudo apt-get update
sudo apt-get install -y ros-humble-ros-gzgarden-bridge ros-humble-tf-transformations
pip3 install transforms3d
```

Optional OpenCV pin if marker generation fails:

```bash
pip3 uninstall -y opencv-contrib-python
pip3 install opencv-contrib-python==4.6.0.66
```

Add workspace sourcing:

```bash
echo 'source ~/gz_ros2_aruco_ws/install/setup.bash' >> ~/.bashrc
```

## 2. Run bridge + detector

Terminal A:

```bash
source ~/gz_ros2_aruco_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge /camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo
```

Terminal B:

```bash
source ~/gz_ros2_aruco_ws/install/setup.bash
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@ignition.msgs.Image
```

Terminal C:

```bash
source ~/gz_ros2_aruco_ws/install/setup.bash
ros2 launch ros2_aruco aruco_recognition.launch.py
```

Terminal D:

```bash
source ~/gz_ros2_aruco_ws/install/setup.bash
ros2 topic echo /aruco_markers
```

## 3. This repository integration points

1. Main pipeline generates a temporary world containing a fixed ArUco landing pad model.
2. Drone spawn XY is fixed within 1 m from marker center (default radius 0.8 m).
3. Marker size follows `drone_size * (2^2)` (default `0.5 * 4 = 2.0 m`).
4. Collection publishes pad spec to `/autolanding/landing_pad` and stores `landing_pad_spec.json`.
5. Hover-to-land trajectory is generated to `data/processed/landing_trajectory_aruco_hover.csv`.

## 4. Run in MATLAB

```matlab
cd('/home/j/SynologyDrive/INCSL/devel/INCSL/autolanding_ws')
clear functions
AutoLandingMainFull('gui')
```

Adjust fixed marker/spawn values in:
- `matlab/AutoLandingMainFull.m`
