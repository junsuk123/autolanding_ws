#!/usr/bin/env python3
"""Run and validate single-Gazebo multi-drone + multi-pad setup."""

from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import subprocess
import time
from pathlib import Path

from pymavlink import mavutil

ROOT = Path(__file__).resolve().parent.parent


def _launch(cmd: list[str], log_path: Path) -> subprocess.Popen:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    handle = open(log_path, 'w', encoding='utf-8')
    return subprocess.Popen(
        cmd,
        stdout=handle,
        stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )


def _terminate(proc: subprocess.Popen | None) -> None:
    if proc is None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except Exception:
        pass


def _topic_list(timeout_s: int = 10) -> tuple[list[str], int, str]:
    proc = subprocess.run(
        ['gz', 'topic', '-l'],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=timeout_s,
        check=False,
    )
    topics = [line.strip() for line in proc.stdout.splitlines() if line.strip()]
    return topics, proc.returncode, proc.stdout


def _port_open(port: int, timeout_s: float = 0.8) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout_s)
        try:
            sock.connect(('127.0.0.1', int(port)))
            return True
        except OSError:
            return False


def _check_topics(drone_count: int) -> dict:
    last_output = ''
    topics: list[str] = []
    for _ in range(25):
        topics, rc, output = _topic_list(timeout_s=10)
        last_output = output[-2000:]
        if rc == 0 and '/world/iris_runway/state' in topics and len(topics) > 5:
            break
        time.sleep(1.5)

    missing = []
    for idx in range(1, drone_count + 1):
        expected = f'/world/iris_runway/model/iris_with_gimbal_w{idx}/joint_state'
        if expected not in topics:
            missing.append(expected)
    return {
        'ok': len(missing) == 0,
        'missing': missing,
        'topic_count': len(topics),
        'sample_topics': topics[:25],
        'last_gz_topic_output': last_output,
    }


def _check_heartbeats(drone_count: int, timeout_each_s: int = 20) -> dict:
    results = []
    ok = True
    for i in range(drone_count):
        port = 5760 + (10 * i)
        item = {'port': port, 'ok': False}
        conn = None
        try:
            wait_deadline = time.time() + 20.0
            while time.time() < wait_deadline and not _port_open(port):
                time.sleep(0.5)
            if not _port_open(port):
                item['error'] = 'port not open'
                ok = False
                results.append(item)
                continue

            conn = mavutil.mavlink_connection(f'tcp:127.0.0.1:{port}', source_system=250 + i)
            hb = conn.wait_heartbeat(timeout=timeout_each_s)
            if hb is not None:
                item['ok'] = True
                item['system'] = int(conn.target_system)
                item['component'] = int(conn.target_component)
            else:
                item['error'] = 'heartbeat timeout'
                ok = False
        except Exception as exc:
            item['error'] = str(exc)
            ok = False
        finally:
            try:
                if conn is not None:
                    conn.close()
            except Exception:
                pass
        results.append(item)
    return {'ok': ok, 'ports': results}


def main() -> int:
    parser = argparse.ArgumentParser(description='Single Gazebo multi-spawn validator')
    parser.add_argument('--drone-count', type=int, default=3)
    parser.add_argument('--pad-count', type=int, default=3)
    parser.add_argument('--spacing-m', type=float, default=6.0)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--keep-running', action='store_true')
    parser.add_argument('--strict-heartbeat', action='store_true', help='Fail unless all MAVLink heartbeat ports are healthy')
    parser.add_argument('--world-out', type=str, default='/tmp/iris_runway_multi_spawn.sdf')
    args = parser.parse_args()

    drone_count = max(1, int(args.drone_count))
    pad_count = max(1, int(args.pad_count))

    # Build world
    build_cmd = [
        'python3', str(ROOT / 'scripts' / 'build_multi_spawn_world.py'),
        '--drone-count', str(drone_count),
        '--pad-count', str(pad_count),
        '--spacing-m', str(float(args.spacing_m)),
        '--out', str(args.world_out),
    ]
    subprocess.run(build_cmd, check=True)

    subprocess.run(['pkill', '-f', 'gz sim|arducopter|launch_gazebo_py.py|launch_multi_drone_py.py'], check=False)
    time.sleep(1.0)

    gz_cmd = ['python3', str(ROOT / 'scripts' / 'launch_gazebo_py.py'), '--world', str(args.world_out)]
    if args.gui and not args.headless:
        gz_cmd.append('--gui')
    else:
        gz_cmd.append('--headless')

    sitl_cmd = ['python3', str(ROOT / 'scripts' / 'launch_multi_drone_py.py'), '--count', str(drone_count)]

    gz_proc = _launch(gz_cmd, Path('/tmp/autolanding_multi_gazebo.log'))
    sitl_proc = _launch(sitl_cmd, Path('/tmp/autolanding_multi_sitl.log'))

    report: dict = {
        'world': str(args.world_out),
        'drone_count': drone_count,
        'pad_count': pad_count,
        'single_gazebo': True,
        'gazebo_pid': gz_proc.pid,
        'sitl_pid': sitl_proc.pid,
    }

    try:
        time.sleep(16.0)
        topic_check = _check_topics(drone_count)
        hb_check = _check_heartbeats(drone_count, timeout_each_s=28)
        report['topic_check'] = topic_check
        report['heartbeat_check'] = hb_check
        report['ok'] = bool(topic_check.get('ok'))
        if args.strict_heartbeat:
            report['ok'] = bool(report['ok'] and hb_check.get('ok'))
        report['strict_heartbeat'] = bool(args.strict_heartbeat)

        out = ROOT / 'data' / 'processed' / 'single_gazebo_multi_spawn_validation.json'
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(json.dumps(report, indent=2))
        print(f'[multi-spawn] report={out}')

        if args.keep_running:
            print('[multi-spawn] keep-running enabled; press Ctrl+C to stop')
            while True:
                time.sleep(1.0)

        return 0 if report['ok'] else 2
    finally:
        if not args.keep_running:
            _terminate(sitl_proc)
            _terminate(gz_proc)
            subprocess.run(
                ['pkill', '-f', 'gz sim|arducopter|launch_gazebo_py.py|launch_multi_drone_py.py'],
                check=False,
            )


if __name__ == '__main__':
    raise SystemExit(main())
