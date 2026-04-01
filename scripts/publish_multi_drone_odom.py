#!/usr/bin/env python3
import argparse
import math
from dataclasses import dataclass
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from pymavlink import mavutil


@dataclass
class DroneState:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    connected: bool = False


class MultiDroneOdomPublisher(Node):
    def __init__(self, workers: int, base_port: int, step: int):
        super().__init__("autolanding_multi_drone_odom")
        self.workers = workers
        self.base_port = base_port
        self.step = step

        self.states = {}
        self.links = {}
        self.odom_pubs = {}
        self.path_pubs = {}
        self.paths = {}
        self.tf_broadcaster = TransformBroadcaster(self)

        for i in range(1, self.workers + 1):
            self.states[i] = DroneState()
            self.odom_pubs[i] = self.create_publisher(Odometry, f"/drone{i}/odom", 10)
            self.path_pubs[i] = self.create_publisher(Path, f"/drone{i}/path", 10)
            self.paths[i] = deque(maxlen=400)
            self.links[i] = self._connect_mav(i)

        self.timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(f"multi-drone odom publisher started (workers={workers})")

    def _connect_mav(self, idx: int):
        # Try master first, then serial1 telemetry port fallback.
        ports = [self.base_port + self.step * (idx - 1), self.base_port + 2 + self.step * (idx - 1)]
        for p in ports:
            endpoint = f"tcp:127.0.0.1:{p}"
            try:
                link = mavutil.mavlink_connection(endpoint, source_system=245 + idx)
                hb = link.wait_heartbeat(timeout=1.0)
                if hb is not None:
                    self.get_logger().info(f"drone{idx} connected via {endpoint}")
                    self.states[idx].connected = True
                    return link
            except Exception:
                pass
        self.get_logger().warning(f"drone{idx} not connected yet")
        return None

    @staticmethod
    def _quat_from_rpy(roll: float, pitch: float, yaw: float):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw

    def _update_state_from_mav(self, idx: int):
        link = self.links.get(idx)
        if link is None:
            self.links[idx] = self._connect_mav(idx)
            return

        updated = False
        for _ in range(20):
            msg = link.recv_match(blocking=False)
            if msg is None:
                break

            mtype = msg.get_type()
            if mtype == "LOCAL_POSITION_NED":
                self.states[idx].x = float(msg.x)
                self.states[idx].y = float(msg.y)
                self.states[idx].z = float(msg.z)
                self.states[idx].vx = float(msg.vx)
                self.states[idx].vy = float(msg.vy)
                self.states[idx].vz = float(msg.vz)
                updated = True
            elif mtype == "ATTITUDE":
                self.states[idx].roll = float(msg.roll)
                self.states[idx].pitch = float(msg.pitch)
                self.states[idx].yaw = float(msg.yaw)
                updated = True

        if updated:
            self.states[idx].connected = True

    def _publish_odom_tf(self, idx: int):
        st = self.states[idx]
        now = self.get_clock().now().to_msg()

        qx, qy, qz, qw = self._quat_from_rpy(st.roll, st.pitch, st.yaw)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "map"
        odom.child_frame_id = f"drone{idx}/base_link"
        odom.pose.pose.position.x = st.x
        odom.pose.pose.position.y = st.y
        odom.pose.pose.position.z = st.z
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = st.vx
        odom.twist.twist.linear.y = st.vy
        odom.twist.twist.linear.z = st.vz
        self.odom_pubs[idx].publish(odom)

        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = "map"
        pose.pose.position.x = st.x
        pose.pose.position.y = st.y
        pose.pose.position.z = st.z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.paths[idx].append(pose)

        path_msg = Path()
        path_msg.header.stamp = now
        path_msg.header.frame_id = "map"
        path_msg.poses = list(self.paths[idx])
        self.path_pubs[idx].publish(path_msg)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = "map"
        tf_msg.child_frame_id = f"drone{idx}/base_link"
        tf_msg.transform.translation.x = st.x
        tf_msg.transform.translation.y = st.y
        tf_msg.transform.translation.z = st.z
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)

    def _tick(self):
        for i in range(1, self.workers + 1):
            self._update_state_from_mav(i)
            self._publish_odom_tf(i)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--base-master-port", type=int, default=5760)
    parser.add_argument("--step", type=int, default=10)
    args = parser.parse_args()

    rclpy.init()
    node = MultiDroneOdomPublisher(max(1, args.workers), args.base_master_port, args.step)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
