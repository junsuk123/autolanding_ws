#!/usr/bin/env python3
"""
Multi-drone ArduPilot SITL launcher (Python-based, no shell scripts)
Replaces launch_multi_drone_sitl.sh

Usage:
  python3 launch_multi_drone_py.py [--count 3] [--speedup 1] [--ardupilot-dir ~/ardupilot]
"""

import argparse
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path


def get_ardupilot_dir():
    """Find ArduPilot directory."""
    ardupilot_dir = os.environ.get("ARDUPILOT_DIR", os.path.expanduser("~/ardupilot"))
    return ardupilot_dir


def get_sitl_binary(ardupilot_dir):
    """Get path to SITL binary."""
    bin_path = os.path.join(ardupilot_dir, "build/sitl/bin/arducopter")
    if not os.path.isfile(bin_path):
        raise FileNotFoundError(f"[ERROR] SITL binary not found: {bin_path}")
    return bin_path


def prepare_log_dir():
    """Create log directory."""
    log_dir = Path("/tmp/autolanding_sitl")
    log_dir.mkdir(parents=True, exist_ok=True)
    pid_file = log_dir / "pids.txt"
    pid_file.write_text("")  # Clear previous pids
    return log_dir


def cleanup_instance(instance_id):
    """Kill existing SITL instance."""
    try:
        subprocess.run(
            ["pkill", "-f", f"arducopter.*--model JSON.*-I{instance_id}( |$)"],
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        pass
    time.sleep(0.2)


def start_instance(instance_id, ardupilot_dir, speedup, log_dir):
    """Start a single SITL instance."""
    cleanup_instance(instance_id)

    bin_path = get_sitl_binary(ardupilot_dir)
    # Use per-instance loopback address to avoid JSON endpoint collisions.
    sim_addr = f"127.0.0.{instance_id + 1}"
    log_file = log_dir / f"arducopter_I{instance_id}.log"

    # FDM ports: 9002+10*i for input, 9003+10*i for output
    # SERIAL0: 5760+10*i, SERIAL1: 5762+10*i
    sysid = instance_id + 1
    sysid_param = Path(tempfile.gettempdir()) / f"autolanding_sysid_I{instance_id}.parm"
    sysid_param.write_text(f"SYSID_THISMAV {sysid}\n", encoding="utf-8")
    cmd = [
        bin_path,
        "--model",
        f"JSON:{sim_addr}",
        "--speedup",
        str(speedup),
        "--slave",
        "0",
        "--defaults",
        f"{ardupilot_dir}/Tools/autotest/default_params/copter.parm,"
        f"{ardupilot_dir}/Tools/autotest/default_params/gazebo-iris.parm,"
        f"{sysid_param}",
        f"-I{instance_id}",
    ]

    # Start in background with setsid (new process group)
    proc = subprocess.Popen(
        cmd,
        stdout=open(log_file, "w"),
        stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL,
        preexec_fn=os.setsid,  # Create new session
    )

    pid = proc.pid
    pid_file = log_dir / "pids.txt"
    with open(pid_file, "a") as f:
        f.write(f"{pid},{instance_id},{log_file}\n")

    # Compute serial port numbers
    serial0_port = 5760 + 10 * instance_id
    serial1_port = 5762 + 10 * instance_id
    fdm_in_port = 9002 + 10 * instance_id
    fdm_out_port = 9003 + 10 * instance_id

    print(
        f"[INFO] I{instance_id} started (PID={pid})"
        f"\n        sim_addr={sim_addr}"
        f"\n        master=tcp:127.0.0.1:{serial0_port}"
        f"\n        serial1=tcp:127.0.0.1:{serial1_port}"
        f"\n        SYSID_THISMAV={sysid}"
        f"\n        FDM in:out={fdm_in_port}:{fdm_out_port}"
        f"\n        log={log_file}"
    )

    return pid


def main():
    parser = argparse.ArgumentParser(
        description="Launch multi-drone ArduPilot SITL instances"
    )
    parser.add_argument("--count", type=int, default=3, help="Number of instances (default: 3)")
    parser.add_argument("--start-index", type=int, default=0, help="Starting instance index (default: 0)")
    parser.add_argument("--speedup", type=float, default=1.0, help="Simulation speedup (default: 1.0)")
    parser.add_argument(
        "--ardupilot-dir",
        type=str,
        default=None,
        help="Path to ArduPilot (default: $ARDUPILOT_DIR or ~/ardupilot)",
    )

    args = parser.parse_args()

    if args.count < 1 or args.count > 10:
        print("[ERROR] count must be 1-10")
        sys.exit(1)
    if args.start_index < 0 or args.start_index > 99:
        print("[ERROR] start-index must be 0-99")
        sys.exit(1)

    if args.ardupilot_dir:
        ardupilot_dir = os.path.expanduser(args.ardupilot_dir)
    else:
        ardupilot_dir = get_ardupilot_dir()

    print(f"[INFO] Launching {args.count:d} SITL instances")
    print(f"[INFO] ArduPilot dir: {ardupilot_dir}")
    print(f"[INFO] Simulation speedup: {args.speedup}")

    try:
        log_dir = prepare_log_dir()
        pids = []

        for i in range(args.count):
            instance_id = args.start_index + i
            pid = start_instance(instance_id, ardupilot_dir, args.speedup, log_dir)
            pids.append(pid)
            time.sleep(1.0)  # Stagger startup

        print(f"\n[INFO] All {args.count:d} SITL instances launched successfully")
        print(f"[INFO] Stop with: pkill -f 'arducopter --model JSON'")
        print(f"[INFO] Or kill by PGID: kill -TERM -$(ps -o pgid= {pids[0]})")

        return 0

    except Exception as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
