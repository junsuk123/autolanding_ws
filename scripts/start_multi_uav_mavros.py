#!/usr/bin/env python3
"""Start N MAVROS instances with strict /uavX/mavros isolation."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path


def source_ros_snippet() -> str:
    scripts = [
        "/opt/ros/humble/setup.bash",
        os.path.expanduser("~/IICC26_ws/install/setup.bash"),
    ]
    out = "set +e; "
    for s in scripts:
        if Path(s).is_file():
            out += f"source '{s}' >/dev/null 2>&1; "
    out += "set -e; "
    return out


def launch_one(uav_id: int, ns_prefix: str, sitl_base: int, mavros_base: int, log_dir: Path, verbose: bool) -> subprocess.Popen:
    sitl_port = sitl_base + (uav_id - 1)
    mavros_bind = mavros_base + (uav_id - 1)
    ns = f"{ns_prefix}{uav_id}/mavros"
    ns_launch = ns.lstrip("/")

    fcu_url = f"udp://127.0.0.1:{sitl_port}@{mavros_bind}"
    log_file = log_dir / f"mavros_uav{uav_id}.log"
    level = "DEBUG" if verbose else "INFO"

    cmd = (
        "bash -lc '"
        + source_ros_snippet()
        + "unset LD_LIBRARY_PATH QT_PLUGIN_PATH QML2_IMPORT_PATH QT_QPA_PLATFORMTHEME PYTHONUSERBASE; "
        + "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}; "
        + "ros2 launch mavros apm.launch "
        + f"namespace:={ns_launch} "
        + f"fcu_url:={fcu_url} "
        + f"tgt_system:={uav_id} "
        + "tgt_component:=1 fcu_protocol:=v2.0 respawn_mavros:=false "
        + f"log_level:={level}'"
    )

    proc = subprocess.Popen(
        cmd,
        shell=True,
        stdout=open(log_file, "w", encoding="utf-8"),
        stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )

    print(
        f"[MAVROS] UAV{uav_id} pid={proc.pid} ns={ns} "
        f"fcu_url={fcu_url} tgt_system={uav_id} log={log_file}"
    )
    return proc


def main() -> int:
    parser = argparse.ArgumentParser(description="Start multi-UAV MAVROS bridge")
    parser.add_argument("--count", type=int, default=3, help="Number of UAVs (1-10)")
    parser.add_argument("--start-id", type=int, default=1, help="Starting UAV id")
    parser.add_argument("--ns-prefix", type=str, default="/uav", help="Namespace prefix, e.g. /uav")
    parser.add_argument("--sitl-port-base", type=int, default=14540, help="SITL UDP base port")
    parser.add_argument("--mavros-port-base", type=int, default=14640, help="MAVROS bind UDP base port")
    parser.add_argument("--verbose", action="store_true", help="Verbose MAVROS logs")
    args = parser.parse_args()

    if args.count < 1 or args.count > 10:
        print("[ERROR] count must be 1..10", file=sys.stderr)
        return 2

    log_dir = Path("/tmp/autolanding_multi_uav").resolve()
    log_dir.mkdir(parents=True, exist_ok=True)

    for i in range(args.count):
        uav_id = args.start_id + i
        launch_one(uav_id, args.ns_prefix, args.sitl_port_base, args.mavros_port_base, log_dir, args.verbose)
        time.sleep(0.7)

    print("[MAVROS] Started all instances. Stop with: pkill -f 'ros2 launch mavros apm.launch'")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
