# Multi-UAV System Design (Gazebo + ArduPilot SITL + MAVROS + MATLAB)

## 1) Isolation Model

Each UAV has a fully isolated path:
- Gazebo model instance: uavX model in one shared world
- SITL instance: one arducopter process per UAV
- MAVROS instance: one bridge per UAV
- ROS namespace: /uavX/mavros/*

No shared control/state topics are used across UAVs.

## 2) Port and SYSID Rules

For UAV index X (1-based):
- SYSID_THISMAV = X
- SITL MAVLink UDP out = 14539 + X
- MAVROS bind UDP port = 14639 + X
- MAVROS FCU URL = udp://127.0.0.1:(14539+X)@(14639+X)

Example:
- UAV1: SYSID=1, FCU udp://127.0.0.1:14540@14640
- UAV2: SYSID=2, FCU udp://127.0.0.1:14541@14641
- UAV3: SYSID=3, FCU udp://127.0.0.1:14542@14642

## 3) Required ROS Interfaces per UAV

Topics:
- /uavX/mavros/state
- /uavX/mavros/local_position/pose
- /uavX/mavros/local_position/velocity_local
- /uavX/mavros/imu/data
- /uavX/mavros/setpoint_position/local
- /uavX/mavros/setpoint_velocity/cmd_vel

Services:
- /uavX/mavros/cmd/arming
- /uavX/mavros/set_mode

## 4) MATLAB Integration Pattern

- Subscribe:
  - /uavX/mavros/local_position/pose
  - /uavX/mavros/imu/data
- Publish:
  - /uavX/mavros/setpoint_position/local
- Loop:
  - for i = 1:N

See MATLAB example:
- matlab/scripts/run_multi_uav_mavros_demo.m

## 5) Communication Flow

1. Gazebo publishes sensor and dynamics interactions.
2. SITL computes control/vehicle dynamics for each SYSID.
3. MAVLink sends telemetry and receives command streams per UAV UDP channel.
4. MAVROS converts MAVLink <-> ROS topics/services in /uavX/mavros namespace.
5. MATLAB reads multi-UAV state, runs global coordination + local references.
6. Commands return through MAVROS -> MAVLink -> SITL.

## 6) Data Collection and Time Sync

- use_sim_time=true for deterministic replay.
- Record with rosbag2 regex:
  - /uav[0-9]+/mavros/.*
  - /clock

## 7) Launch Artifacts

- SITL launcher:
  - scripts/start_multi_uav_sitl.py
- MAVROS launcher:
  - scripts/start_multi_uav_mavros.py
- ROS launch file:
  - simulation/launch/multi_uav_mavros.launch.py
- End-to-end stack launcher:
  - scripts/start_multi_uav_stack.sh

## 8) Scalability Guardrails

- Supported by scripts: N in [1, 10]
- Unique namespace per UAV
- Unique SYSID_THISMAV per UAV
- Unique UDP port per UAV
- Recommended rates:
  - State: 10 Hz
  - Control setpoint: 20 Hz

## 9) Text Network Diagram

[Gazebo World]
    | (sensor + dynamics)
    +--> [SITL UAV1 SYSID=1 UDP14540] <--> [MAVROS /uav1/mavros FCU udp://127.0.0.1:14540@14640]
    +--> [SITL UAV2 SYSID=2 UDP14541] <--> [MAVROS /uav2/mavros FCU udp://127.0.0.1:14541@14641]
    +--> [SITL UAV3 SYSID=3 UDP14542] <--> [MAVROS /uav3/mavros FCU udp://127.0.0.1:14542@14642]
    ...
    +--> [SITL UAVN SYSID=N UDP1454N] <--> [MAVROS /uavN/mavros]

[MATLAB ROS Toolbox]
    <--> /uav*/mavros/local_position/pose
    <--> /uav*/mavros/imu/data
    -->  /uav*/mavros/setpoint_position/local

[rosbag2]
    record /uav*/mavros/* + /clock
