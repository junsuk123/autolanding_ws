#!/usr/bin/env python3
"""
RViz monitor with ros_gz_bridge and ros2_aruco integration (Python-based)
Replaces launch_rviz_monitor.sh

Usage:
  python3 launch_rviz_monitor_py.py [--verbose]
"""

import os
import subprocess
import sys
import time
from pathlib import Path


def source_ros_setup():
    """Get ROS2 setup command to source properly."""
    setup_scripts = [
        "/opt/ros/humble/setup.bash",
        os.path.expanduser("~/IICC26_ws/install/setup.bash"),
        os.path.expanduser("~/gz_ros2_aruco_ws/install/setup.bash"),
    ]

    source_cmd = "set +e; "
    for script in setup_scripts:
        if os.path.isfile(script):
            source_cmd += f"source '{script}' 2>/dev/null; "
    source_cmd += "set -e;"

    return source_cmd


def launch_ros_gz_bridge():
    """Launch ros_gz_bridge for Gazebo↔ROS2 communication."""
    # Clean environment before ROS2 operations
    env_clean = "unset LD_LIBRARY_PATH QT_PLUGIN_PATH QML2_IMPORT_PATH QT_QPA_PLATFORMTHEME PYTHONUSERBASE; "
    cmd = f"""
    bash -lc '
    {env_clean}{source_ros_setup()}
    export ROS_DOMAIN_ID=${{ROS_DOMAIN_ID:-0}}
    ros2 run ros_gz_bridge parameter_bridge \\
      --ros-args \\
      -p config_file:="{os.path.expanduser("~/gz_ros2_aruco_ws/src/ros2-gazebo-aruco/config/bridge.yaml")}" \\
      2>&1
    '
    """
    return cmd.strip()


def launch_ros2_aruco():
    """Launch ros2_aruco for ArUco marker detection."""
    env_clean = "unset LD_LIBRARY_PATH QT_PLUGIN_PATH QML2_IMPORT_PATH QT_QPA_PLATFORMTHEME PYTHONUSERBASE; "
    cmd = f"""
    bash -lc '
    {env_clean}{source_ros_setup()}
    export ROS_DOMAIN_ID=${{ROS_DOMAIN_ID:-0}}
    ros2 run ros2_aruco aruco_node \\
      --ros-args \\
      -p marker_size:=0.05 \\
      -p aruco_dictionary_id:="DICT_ARUCO_ORIGINAL" \\
      2>&1
    '
    """
    return cmd.strip()


def launch_rviz():
    """Launch RViz with monitoring config."""
    config_file = os.path.expanduser("~/SynologyDrive/INCSL/devel/INCSL/autolanding_ws/config/autolanding_monitor.rviz")
    
    # Fallback config if custom one not found
    if not os.path.isfile(config_file):
        config_file = ""

    config_arg = f"-d {config_file}" if config_file and os.path.isfile(config_file) else ""
    env_clean = "unset LD_LIBRARY_PATH QT_PLUGIN_PATH QML2_IMPORT_PATH QT_QPA_PLATFORMTHEME PYTHONUSERBASE; "

    cmd = f"""
    bash -lc '
    {env_clean}{source_ros_setup()}
    export ROS_DOMAIN_ID=${{ROS_DOMAIN_ID:-0}}
    export DISPLAY=${{DISPLAY:-:0}}
    rviz2 {config_arg} 2>&1
    '
    """
    return cmd.strip()


def launch_publish_multi_drone_odom():
    """Launch multi-drone odometry publisher."""
    script = os.path.expanduser("~/SynologyDrive/INCSL/devel/INCSL/autolanding_ws/scripts/publish_multi_drone_odom.py")
    
    if not os.path.isfile(script):
        print(f"[WARNING] Odometry publisher script not found: {script}")
        return None

    env_clean = "unset LD_LIBRARY_PATH QT_PLUGIN_PATH QML2_IMPORT_PATH QT_QPA_PLATFORMTHEME PYTHONUSERBASE; "
    cmd = f"""
    bash -lc '
    {env_clean}{source_ros_setup()}
    export ROS_DOMAIN_ID=${{ROS_DOMAIN_ID:-0}}
    python3 "{script}" 2>&1
    '
    """
    return cmd.strip()


def launch_component(name, cmd, log_file):
    """Launch a component as a background process."""
    try:
        with open(log_file, "w") as log_f:
            proc = subprocess.Popen(
                cmd,
                shell=True,
                stdout=log_f,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                preexec_fn=os.setsid,  # Create new process group
            )
            print(f"[INFO] {name} launched (PID={proc.pid}, log={log_file})")
            return proc
    except Exception as e:
        print(f"[ERROR] Failed to launch {name}: {e}", file=sys.stderr)
        return None


class RvizMonitor:
    """Manager for RViz monitoring stack."""

    def __init__(self, verbose=False):
        self.verbose = verbose
        self.processes = {}
        self.log_dir = Path("/tmp")

    def start_all(self):
        """Start all monitoring components."""
        print("[INFO] Starting RViz monitoring stack...")
        
        components = [
            ("ros_gz_bridge", launch_ros_gz_bridge(), self.log_dir / "autolanding_ros_gz_bridge.log"),
            ("ros2_aruco", launch_ros2_aruco(), self.log_dir / "autolanding_ros2_aruco.log"),
            ("multi_drone_odom", launch_publish_multi_drone_odom(), self.log_dir / "autolanding_multi_drone_odom.log"),
            ("rviz2", launch_rviz(), self.log_dir / "autolanding_rviz.log"),
        ]

        for name, cmd, log_file in components:
            if cmd is None:
                print(f"[SKIP] {name} (no command available)")
                continue
            
            proc = launch_component(name, cmd, str(log_file))
            if proc:
                self.processes[name] = proc
                time.sleep(0.5)
            else:
                print(f"[WARNING] Failed to launch {name}")

        return len(self.processes)

    def wait_all(self, timeout=None):
        """Wait for all processes."""
        try:
            for name, proc in self.processes.items():
                try:
                    proc.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    pass
        except KeyboardInterrupt:
            print("\n[INFO] Terminating RViz stack...")
            self.terminate_all()

    def terminate_all(self):
        """Terminate all processes."""
        for name, proc in self.processes.items():
            try:
                os.killpg(os.getpgid(proc.pid), 15)
                print(f"[INFO] {name} terminated")
            except Exception:
                pass
            time.sleep(0.2)


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Launch RViz monitoring stack")
    parser.add_argument("--verbose", action="store_true", help="Verbose logging")
    args = parser.parse_args()

    monitor = RvizMonitor(verbose=args.verbose)
    started = monitor.start_all()

    if started == 0:
        print("[WARNING] No components started", file=sys.stderr)
        return 1

    print(f"[INFO] {started} components launched successfully")
    print("[INFO] Press Ctrl+C to stop")

    monitor.wait_all()
    return 0


if __name__ == "__main__":
    sys.exit(main())
