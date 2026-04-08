#!/usr/bin/env python3
"""ROS2 launch file for N isolated MAVROS bridges.

Namespace model:
- /uav1/mavros
- /uav2/mavros
- ...
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def _spawn(context):
    count = int(LaunchConfiguration("count").perform(context))
    start_id = int(LaunchConfiguration("start_id").perform(context))
    ns_prefix = LaunchConfiguration("ns_prefix").perform(context)
    sitl_base = int(LaunchConfiguration("sitl_port_base").perform(context))
    bind_base = int(LaunchConfiguration("mavros_port_base").perform(context))

    actions = []
    for i in range(count):
        uav_id = start_id + i
        sitl_port = sitl_base + i
        bind_port = bind_base + i
        ns = f"{ns_prefix}{uav_id}/mavros".lstrip("/")
        fcu_url = f"udp://127.0.0.1:{sitl_port}@{bind_port}"

        actions.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "launch",
                    "mavros",
                    "apm.launch",
                    f"namespace:={ns}",
                    f"fcu_url:={fcu_url}",
                    f"tgt_system:={uav_id}",
                    "tgt_component:=1",
                    "fcu_protocol:=v2.0",
                ],
                output="screen",
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("count", default_value="3"),
            DeclareLaunchArgument("start_id", default_value="1"),
            DeclareLaunchArgument("ns_prefix", default_value="/uav"),
            DeclareLaunchArgument("sitl_port_base", default_value="14540"),
            DeclareLaunchArgument("mavros_port_base", default_value="14640"),
            OpaqueFunction(function=_spawn),
        ]
    )
