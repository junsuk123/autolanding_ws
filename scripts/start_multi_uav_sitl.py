#!/usr/bin/env python3
"""Start N isolated ArduPilot SITL instances for multi-UAV simulation.

Design goals:
- One SITL per UAV with unique SYSID_THISMAV
- MAVLink UDP out ports: UAV1->14540, UAV2->14541, ...
- Scale tested by design up to 10 UAVs
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path


def get_ardupilot_dir(cli_value: str | None) -> Path:
    raw = cli_value or os.environ.get("ARDUPILOT_DIR", "~/ardupilot")
    return Path(raw).expanduser().resolve()


def get_binary(ardupilot_dir: Path) -> Path:
    binary = ardupilot_dir / "build" / "sitl" / "bin" / "arducopter"
    if not binary.is_file():
        raise FileNotFoundError(f"SITL binary not found: {binary}")
    return binary


def build_sysid_param_file(uav_id: int, temp_dir: Path) -> Path:
    path = temp_dir / f"uav{uav_id}_sysid.parm"
    path.write_text(f"SYSID_THISMAV {uav_id}\n", encoding="utf-8")
    return path


def launch_one(binary: Path, ardupilot_dir: Path, uav_id: int, speedup: float, log_dir: Path, temp_dir: Path) -> subprocess.Popen:
    instance = uav_id - 1
    sim_addr = f"127.0.0.{uav_id}"
    mavlink_udp_port = 14539 + uav_id
    log_file = log_dir / f"sitl_uav{uav_id}.log"

    defaults = [
        ardupilot_dir / "Tools" / "autotest" / "default_params" / "copter.parm",
        ardupilot_dir / "Tools" / "autotest" / "default_params" / "gazebo-iris.parm",
        build_sysid_param_file(uav_id, temp_dir),
    ]

    cmd = [
        str(binary),
        "--model",
        f"JSON:{sim_addr}",
        "--speedup",
        str(speedup),
        "--slave",
        "0",
        "--serial0",
        f"udpclient:127.0.0.1:{mavlink_udp_port}",
        "--defaults",
        ",".join(str(p) for p in defaults),
        f"-I{instance}",
    ]

    proc = subprocess.Popen(
        cmd,
        stdout=open(log_file, "w", encoding="utf-8"),
        stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )

    print(
        f"[SITL] UAV{uav_id} pid={proc.pid} SYSID={uav_id} "
        f"udp_out=127.0.0.1:{mavlink_udp_port} instance={instance} log={log_file}"
    )
    return proc


def main() -> int:
    parser = argparse.ArgumentParser(description="Start multi-UAV ArduPilot SITL (UDP MAVLink)")
    parser.add_argument("--count", type=int, default=3, help="Number of UAVs (1-10)")
    parser.add_argument("--start-id", type=int, default=1, help="Starting UAV id (default: 1)")
    parser.add_argument("--speedup", type=float, default=1.0, help="SITL speedup")
    parser.add_argument("--ardupilot-dir", type=str, default=None, help="ArduPilot root")
    args = parser.parse_args()

    if args.count < 1 or args.count > 10:
        print("[ERROR] count must be 1..10", file=sys.stderr)
        return 2
    if args.start_id < 1:
        print("[ERROR] start-id must be >= 1", file=sys.stderr)
        return 2

    ardupilot_dir = get_ardupilot_dir(args.ardupilot_dir)
    binary = get_binary(ardupilot_dir)

    log_dir = Path("/tmp/autolanding_multi_uav").resolve()
    log_dir.mkdir(parents=True, exist_ok=True)
    temp_dir = Path(tempfile.mkdtemp(prefix="autolanding_uav_sysid_"))

    print(f"[SITL] ardupilot_dir={ardupilot_dir}")
    print(f"[SITL] binary={binary}")

    procs: list[subprocess.Popen] = []
    for i in range(args.count):
        uav_id = args.start_id + i
        procs.append(launch_one(binary, ardupilot_dir, uav_id, args.speedup, log_dir, temp_dir))
        time.sleep(0.8)

    print("[SITL] Started all instances. Stop with: pkill -f 'arducopter.*JSON:'")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
