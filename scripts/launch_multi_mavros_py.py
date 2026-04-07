#!/usr/bin/env python3
"""
Multi-drone MAVROS launcher (Python-based, replaces launch_multi_mavros.sh)

Launches independent MAVROS nodes for each drone with proper namespacing and FCU endpoint configuration.

Usage:
    python3 launch_multi_mavros_py.py [--count 3] [--start-index 1] [--ns-prefix /autolanding/mavros_w] [--verbose]
"""

import argparse
import os
import socket
import subprocess
import sys
import tempfile
import time
from pathlib import Path


def source_ros_setup():
    """Get ROS2 setup command to source properly."""
    setup_scripts = [
        "/opt/ros/humble/setup.bash",
        os.path.expanduser("~/IICC26_ws/install/setup.bash"),
        os.path.expanduser("~/gz_ros2_aruco_ws/install/setup.bash"),
    ]

    # Multi-command to source all scripts, handling unset variables
    source_cmd = "set +e; "
    for script in setup_scripts:
        if os.path.isfile(script):
            source_cmd += f"source '{script}' 2>/dev/null; "
    source_cmd += "set -e;"

    return source_cmd


def wait_for_tcp_listener(host, port, timeout_s=30.0, poll_s=0.5):
    """Wait until host:port accepts a TCP connection."""
    deadline = time.time() + max(0.0, timeout_s)
    while time.time() < deadline:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(0.6)
        try:
            sock.connect((host, int(port)))
            sock.close()
            return True
        except Exception:
            sock.close()
            time.sleep(max(0.1, poll_s))
    return False


def launch_mavros_instance(instance_id, ns_prefix, verbose=False):
    """Launch a single MAVROS instance."""
    namespace = f"{ns_prefix}{instance_id}".lstrip("/")
    serial0_port = 5760 + 10 * (instance_id - 1)
    serial1_port = 5762 + 10 * (instance_id - 1)

    # MAVROS launch command
    cmd_list = [
        "ros2",
        "launch",
        "mavros",
        "apm.launch",
        f"namespace:={namespace}",
        f"fcu_url:=tcp://127.0.0.1:{serial0_port}",
        f"tgt_system:=1",
        f"tgt_component:=1",
        f"fcu_protocol:=v2.0",
        f"respawn_mavros:=false",
        f"log_level:={'DEBUG' if verbose else 'INFO'}",
    ]

    # Wrap with namespace and ROS sourcing, with cleaned environment
    # Remove MATLAB-polluted variables before sourcing ROS2
    env_clean_cmd = "unset LD_LIBRARY_PATH QT_PLUGIN_PATH QML2_IMPORT_PATH QT_QPA_PLATFORMTHEME PYTHONUSERBASE; "
    full_cmd = f"bash -lc '{env_clean_cmd}{source_ros_setup()} export ROS_DOMAIN_ID=${{ROS_DOMAIN_ID:-0}}; {' '.join(cmd_list)}'"

    print(f"[INFO] MAVROS W{instance_id} launcher command:")
    print(f"        namespace={namespace}")
    print(f"        FCU endpoint=tcp:127.0.0.1:{serial0_port}")
    print(f"        fallback=tcp:127.0.0.1:{serial1_port}")

    if not wait_for_tcp_listener("127.0.0.1", serial0_port, timeout_s=35.0, poll_s=0.5):
        print(
            f"[WARNING] W{instance_id} FCU endpoint tcp:127.0.0.1:{serial0_port} did not open within timeout; launching MAVROS anyway",
            file=sys.stderr,
        )
    else:
        print(f"[INFO] W{instance_id} FCU endpoint is reachable, launching MAVROS")

    try:
        log_file = f"/tmp/autolanding_mavros_w{instance_id}.log"
        with open(log_file, "w") as log_f:
            proc = subprocess.Popen(
                full_cmd,
                shell=True,
                stdout=log_f,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                preexec_fn=os.setsid,  # Create new process group
            )
            print(f"[INFO] W{instance_id} launched (PID={proc.pid}, namespace={namespace}, log={log_file})")
            return proc

    except Exception as e:
        print(f"[ERROR] Failed to launch MAVROS W{instance_id}: {e}", file=sys.stderr)
        return None


def verify_mavros_packages():
    """Verify that mavros packages are available."""
    try:
        result = subprocess.run(
            [
                "bash",
                "-lc",
                f"{source_ros_setup()} ros2 pkg prefix mavros >/dev/null 2>&1 && echo OK || echo FAIL",
            ],
            capture_output=True,
            text=True,
            timeout=5,
        )
        if "OK" in result.stdout:
            return True
    except Exception:
        pass
    return False


class MavrosLauncher:
    """Manager for multi-MAVROS processes."""

    def __init__(self, count=3, ns_prefix="/autolanding/mavros_w", verbose=False, start_index=1):
        self.count = count
        self.ns_prefix = ns_prefix
        self.verbose = verbose
        self.start_index = start_index
        self.processes = {}

    def start_all(self):
        """Start all MAVROS instances."""
        print(f"[INFO] Launching {self.count} MAVROS instances")
        print(f"[INFO] Namespace prefix: {self.ns_prefix}")

        for i in range(1, self.count + 1):
            instance_id = self.start_index + (i - 1)
            proc = launch_mavros_instance(instance_id, self.ns_prefix, self.verbose)
            if proc:
                self.processes[i] = proc
                time.sleep(0.5)
            else:
                print(f"[WARNING] Failed to start MAVROS W{i}")

        return len(self.processes)

    def wait_all(self, timeout=None):
        """Wait for all processes to complete."""
        try:
            for worker_id, proc in self.processes.items():
                try:
                    proc.wait(timeout=timeout)
                except subprocess.TimeoutExpired:
                    pass
        except KeyboardInterrupt:
            print("\n[INFO] Terminating all MAVROS instances...")
            self.terminate_all()

    def terminate_all(self):
        """Terminate all processes."""
        for worker_id, proc in self.processes.items():
            try:
                os.killpg(os.getpgid(proc.pid), 15)  # SIGTERM to process group
                print(f"[INFO] W{worker_id} terminated")
            except Exception:
                pass
            time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser(description="Launch multi-drone MAVROS instances")
    parser.add_argument(
        "--count",
        type=int,
        default=3,
        help="Number of MAVROS instances (default: 3)",
    )
    parser.add_argument(
        "--start-index",
        type=int,
        default=1,
        help="Starting instance index (default: 1)",
    )
    parser.add_argument(
        "--ns-prefix",
        type=str,
        default="/autolanding/mavros_w",
        help="Namespace prefix (default: /autolanding/mavros_w)",
    )
    parser.add_argument("--verbose", action="store_true", help="Verbose MAVROS logging")

    args = parser.parse_args()

    if args.count < 1 or args.count > 10:
        print("[ERROR] count must be 1-10", file=sys.stderr)
        return 1
    if args.start_index < 1 or args.start_index > 99:
        print("[ERROR] start-index must be 1-99", file=sys.stderr)
        return 1

    # Verify packages
    if not verify_mavros_packages():
        print("[ERROR] MAVROS packages not found. Check ROS2 installation.", file=sys.stderr)
        return 1

    launcher = MavrosLauncher(count=args.count, ns_prefix=args.ns_prefix, verbose=args.verbose, start_index=args.start_index)
    started = launcher.start_all()

    if started == 0:
        print("[ERROR] Failed to start any MAVROS instances", file=sys.stderr)
        return 1

    print(f"[INFO] {started}/{args.count} MAVROS instances launched successfully")
    print("[INFO] Press Ctrl+C to stop")

    launcher.wait_all()

    return 0


if __name__ == "__main__":
    sys.exit(main())
