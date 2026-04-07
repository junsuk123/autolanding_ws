#!/usr/bin/env python3
"""
RViz monitor with ros_gz_bridge and ros2_aruco integration (Python-based).

Usage:
  python3 launch_rviz_monitor_py.py [--verbose]
"""

import os
import subprocess
import sys
import time
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parent.parent
DEFAULT_DRONE_NS_PREFIX = "/autolanding/drone"
DEFAULT_PRIMARY_DRONE_INDEX = 1


def _normalize_topic_namespace(namespace: str) -> str:
    value = str(namespace).strip()
    if not value:
        return "/"
    if not value.startswith("/"):
        value = "/" + value
    if len(value) > 1 and value.endswith("/"):
        value = value.rstrip("/")
    return value


def load_topic_settings() -> dict[str, str]:
    defaults = {
        "drone_ns_prefix": DEFAULT_DRONE_NS_PREFIX,
        "primary_drone_index": str(DEFAULT_PRIMARY_DRONE_INDEX),
        "drone_namespace": f"{DEFAULT_DRONE_NS_PREFIX}{DEFAULT_PRIMARY_DRONE_INDEX}",
        "camera_image_topic": f"{DEFAULT_DRONE_NS_PREFIX}{DEFAULT_PRIMARY_DRONE_INDEX}/camera",
        "camera_info_topic": f"{DEFAULT_DRONE_NS_PREFIX}{DEFAULT_PRIMARY_DRONE_INDEX}/camera_info",
        "aruco_markers_topic": f"{DEFAULT_DRONE_NS_PREFIX}{DEFAULT_PRIMARY_DRONE_INDEX}/aruco_markers",
        "aruco_poses_topic": f"{DEFAULT_DRONE_NS_PREFIX}{DEFAULT_PRIMARY_DRONE_INDEX}/aruco_poses",
    }
    cfg_path = ROOT / "ai" / "configs" / "orchestration_config.yaml"
    try:
        payload = yaml.safe_load(cfg_path.read_text(encoding="utf-8")) if cfg_path.exists() else {}
        if not isinstance(payload, dict):
            return defaults
        out = defaults.copy()
        ns_prefix = _normalize_topic_namespace(str(payload.get("drone_ns_prefix", out["drone_ns_prefix"])))
        try:
            drone_idx = max(1, int(payload.get("primary_drone_index", DEFAULT_PRIMARY_DRONE_INDEX)))
        except Exception:
            drone_idx = DEFAULT_PRIMARY_DRONE_INDEX

        default_drone_ns = f"{ns_prefix}{drone_idx}"
        drone_ns = _normalize_topic_namespace(str(payload.get("drone_namespace", default_drone_ns)))

        def resolve_topic(key: str, suffix: str) -> str:
            val = str(payload.get(key, "")).strip()
            return _normalize_topic_namespace(val) if val else f"{drone_ns}/{suffix}"

        out["drone_ns_prefix"] = ns_prefix
        out["primary_drone_index"] = str(drone_idx)
        out["drone_namespace"] = drone_ns
        out["camera_image_topic"] = resolve_topic("camera_image_topic", "camera")
        out["camera_info_topic"] = resolve_topic("camera_info_topic", "camera_info")
        out["aruco_markers_topic"] = resolve_topic("aruco_markers_topic", "aruco_markers")
        out["aruco_poses_topic"] = resolve_topic("aruco_poses_topic", "aruco_poses")
        return out
    except Exception:
        return defaults


def source_ros_setup() -> str:
    """Return shell snippet that sources available ROS setup files."""
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


def launch_ros_gz_bridge() -> str:
    """Launch ros_gz_bridge for Gazebo↔ROS2 communication."""
    topics = load_topic_settings()
    image_topic = topics["camera_image_topic"]
    info_topic = topics["camera_info_topic"]

    env_clean = "unset LD_LIBRARY_PATH QT_PLUGIN_PATH QML2_IMPORT_PATH QT_QPA_PLATFORMTHEME PYTHONUSERBASE; "
    cmd = f"""
    bash -lc '
    {env_clean}{source_ros_setup()}
    export ROS_DOMAIN_ID=${{ROS_DOMAIN_ID:-0}}
        ros2 run ros_gz_bridge parameter_bridge \
            /camera@sensor_msgs/msg/Image[gz.msgs.Image \
            /camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
            --ros-args -r /camera:={image_topic} -r /camera_info:={info_topic} \
      > /tmp/autolanding_ros_gz_bridge.log 2>&1
    '
    """
    return cmd.strip()


def launch_ros2_aruco() -> str:
    """Launch ros2_aruco for ArUco marker detection."""
    topics = load_topic_settings()
    image_topic = topics["camera_image_topic"]
    info_topic = topics["camera_info_topic"]
    markers_topic = topics["aruco_markers_topic"]
    poses_topic = topics["aruco_poses_topic"]

    env_clean = "unset LD_LIBRARY_PATH QT_PLUGIN_PATH QML2_IMPORT_PATH QT_QPA_PLATFORMTHEME PYTHONUSERBASE; "
    params_file = str(ROOT / "simulation" / "configs" / "aruco_parameters.yaml")
    compat_path = str(ROOT / "scripts")
    cmd = f"""
    bash -lc '
    {env_clean}{source_ros_setup()}
      export ROS_DOMAIN_ID=${{ROS_DOMAIN_ID:-0}}
      export PYTHONPATH="{compat_path}:${{PYTHONPATH:-}}"
    ros2 run ros2_aruco aruco_node \
      --ros-args \
      --params-file "{params_file}" \
            -p image_topic:={image_topic} \
            -p camera_info_topic:={info_topic} \
            -r /aruco_markers:={markers_topic} \
            -r /aruco_poses:={poses_topic} \
      > /tmp/autolanding_ros2_aruco.log 2>&1
    '
    """
    return cmd.strip()


def launch_rviz() -> str:
    """Launch RViz with monitoring config."""
    config_file = str(ROOT / "simulation" / "configs" / "autolanding_monitor.rviz")
    if not os.path.isfile(config_file):
        config_file = ""

    config_arg = f"-d {config_file}" if config_file else ""
    env_clean = (
        "unset LD_LIBRARY_PATH LD_PRELOAD QT_PLUGIN_PATH QML2_IMPORT_PATH QT_QPA_PLATFORMTHEME "
        "PYTHONUSERBASE GTK_PATH LOCPATH GSETTINGS_SCHEMA_DIR GIO_MODULE_DIR GTK_IM_MODULE_FILE "
        "GTK_EXE_PREFIX GDK_BACKEND WAYLAND_DISPLAY XDG_RUNTIME_DIR XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG "
        "GDK_BACKEND_VSCODE_SNAP_ORIG GIO_MODULE_DIR_VSCODE_SNAP_ORIG GSETTINGS_SCHEMA_DIR_VSCODE_SNAP_ORIG "
        "GTK_IM_MODULE_FILE_VSCODE_SNAP_ORIG GTK_EXE_PREFIX_VSCODE_SNAP_ORIG LOCPATH_VSCODE_SNAP_ORIG "
        "XDG_DATA_HOME_VSCODE_SNAP_ORIG XDG_DATA_DIRS_VSCODE_SNAP_ORIG SNAP SNAP_NAME SNAP_INSTANCE_NAME "
        "SNAP_ARCH SNAP_REVISION SNAP_VERSION SNAP_LIBRARY_PATH; "
    )

    cmd = f"""
    bash -lc '
    {env_clean}{source_ros_setup()}
    export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$PATH
    export LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu
    export ROS_DOMAIN_ID=${{ROS_DOMAIN_ID:-0}}
    export DISPLAY=${{DISPLAY:-:0}}
    export QT_QPA_PLATFORM=xcb
    rviz2 {config_arg} > /tmp/autolanding_rviz.log 2>&1
    '
    """
    return cmd.strip()


def launch_publish_multi_drone_odom(workers: int = 1, topic_prefix: str = DEFAULT_DRONE_NS_PREFIX) -> str | None:
    """Launch multi-drone odometry publisher."""
    script = str(ROOT / "scripts" / "publish_multi_drone_odom.py")
    if not os.path.isfile(script):
        print(f"[WARNING] Odometry publisher script not found: {script}")
        return None

    env_clean = "unset LD_LIBRARY_PATH QT_PLUGIN_PATH QML2_IMPORT_PATH QT_QPA_PLATFORMTHEME PYTHONUSERBASE; "
    cmd = f"""
    bash -lc '
    {env_clean}{source_ros_setup()}
    export ROS_DOMAIN_ID=${{ROS_DOMAIN_ID:-0}}
    python3 "{script}" --workers {workers} --topic-prefix "{topic_prefix}" > /tmp/autolanding_multi_drone_odom.log 2>&1
    '
    """
    return cmd.strip()


def launch_component(name: str, cmd: str, log_file: str):
    """Launch a component as background process."""
    try:
        with open(log_file, "w", encoding="utf-8") as log_f:
            proc = subprocess.Popen(
                cmd,
                shell=True,
                stdout=log_f,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            print(f"[INFO] {name} launched (PID={proc.pid}, log={log_file})")
            return proc
    except Exception as exc:
        print(f"[ERROR] Failed to launch {name}: {exc}", file=sys.stderr)
        return None


class RvizMonitor:
    """Manager for RViz monitoring stack."""

    def __init__(self, verbose: bool = False):
        self.verbose = verbose
        self.processes = {}
        self.log_dir = Path("/tmp")

    def cleanup_stale(self):
        """Kill stale monitor processes from previous runs."""
        patterns = [
            "parameter_bridge",
            "aruco_node",
            "publish_multi_drone_odom.py",
            "rviz2",
        ]
        for pat in patterns:
            try:
                subprocess.run(["pkill", "-f", pat], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except Exception:
                pass
        time.sleep(0.5)

    def start_all(self) -> int:
        print("[INFO] Starting RViz monitoring stack...")
        self.cleanup_stale()
        topics = load_topic_settings()
        workers_env = os.environ.get("AUTOLANDING_NUM_WORKERS", "1")
        try:
            workers = max(1, int(workers_env))
        except Exception:
            workers = 1

        components = [
            ("ros_gz_bridge", launch_ros_gz_bridge(), self.log_dir / "autolanding_ros_gz_bridge_supervisor.log"),
            ("ros2_aruco", launch_ros2_aruco(), self.log_dir / "autolanding_ros2_aruco_supervisor.log"),
            (
                "multi_drone_odom",
                launch_publish_multi_drone_odom(workers, topics.get("drone_ns_prefix", DEFAULT_DRONE_NS_PREFIX)),
                self.log_dir / "autolanding_multi_drone_odom_supervisor.log",
            ),
            ("rviz2", launch_rviz(), self.log_dir / "autolanding_rviz_supervisor.log"),
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
        try:
            for _, proc in self.processes.items():
                try:
                    proc.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    pass
        except KeyboardInterrupt:
            print("\n[INFO] Terminating RViz stack...")
            self.terminate_all()

    def terminate_all(self):
        for name, proc in self.processes.items():
            try:
                os.killpg(os.getpgid(proc.pid), 15)
                print(f"[INFO] {name} terminated")
            except Exception:
                pass
            time.sleep(0.2)


def main() -> int:
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
