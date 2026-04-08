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


def _run_algorithm(timeout_s: int) -> dict:
    cmd = [
        'matlab',
        '-batch',
        "run('matlab/scripts/run_autolanding_pipeline.m')",
    ]
    try:
        proc = subprocess.run(
            cmd,
            cwd=str(ROOT),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=max(30, int(timeout_s)),
            check=False,
        )
        output = proc.stdout or ''
        return {
            'ok': proc.returncode == 0,
            'returncode': int(proc.returncode),
            'timeout_s': max(30, int(timeout_s)),
            'output_tail': output[-4000:],
        }
    except subprocess.TimeoutExpired as exc:
        output = exc.stdout or ''
        return {
            'ok': False,
            'returncode': 124,
            'timeout_s': max(30, int(timeout_s)),
            'error': 'algorithm timeout',
            'output_tail': output[-4000:],
        }
    except Exception as exc:
        return {
            'ok': False,
            'returncode': 1,
            'timeout_s': max(30, int(timeout_s)),
            'error': str(exc),
        }


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


def _run_state_change_check(port: int, target_alt_m: float = 3.0) -> dict:
    item: dict = {'port': port, 'ok': False}
    conn = None
    try:
        conn = mavutil.mavlink_connection(f'tcp:127.0.0.1:{port}', source_system=240)
        hb = conn.wait_heartbeat(timeout=20)
        if hb is None:
            item['error'] = 'heartbeat timeout'
            return item

        try:
            conn.mav.request_data_stream_send(
                conn.target_system,
                conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                5,
                1,
            )
        except Exception:
            pass

        pre_alt = 0.0
        start_deadline = time.time() + 4.0
        while time.time() < start_deadline:
            msg = conn.recv_match(blocking=True, timeout=1)
            if msg is not None:
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    pre_alt = float(getattr(msg, 'relative_alt', 0.0)) / 1000.0
                    break
                if msg.get_type() == 'VFR_HUD':
                    pre_alt = float(getattr(msg, 'alt', 0.0))
                    break

        try:
            conn.set_mode_apm('GUIDED')
        except Exception:
            pass

        try:
            conn.mav.command_long_send(
                conn.target_system,
                conn.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,
                21196,
                0,
                0,
                0,
                0,
                0,
            )
        except Exception as exc:
            item['error'] = f'arm send failed: {exc}'
            return item

        armed = False
        arm_deadline = time.time() + 18.0
        while time.time() < arm_deadline:
            msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if msg is None:
                continue
            armed = bool(getattr(msg, 'base_mode', 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed:
                break

        if not armed:
            item['error'] = 'armed state not confirmed'
            item['pre_alt_m'] = pre_alt
            return item

        try:
            conn.mav.command_long_send(
                conn.target_system,
                conn.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                float(target_alt_m),
            )
        except Exception as exc:
            item['error'] = f'takeoff send failed: {exc}'
            return item

        post_alt = pre_alt
        mode_seen = None
        telemetry_samples: list[dict[str, object]] = []
        fallback_used = False
        deadline = time.time() + 35.0
        while time.time() < deadline:
            msg = conn.recv_match(blocking=True, timeout=2)
            if msg is None:
                continue
            msg_type = msg.get_type()
            if msg_type == 'GLOBAL_POSITION_INT':
                post_alt = max(post_alt, float(getattr(msg, 'relative_alt', 0.0)) / 1000.0)
                telemetry_samples.append({'type': msg_type, 'relative_alt_m': post_alt})
            elif msg_type == 'VFR_HUD':
                post_alt = max(post_alt, float(getattr(msg, 'alt', 0.0)))
                telemetry_samples.append({'type': msg_type, 'alt_m': post_alt})
            elif msg_type == 'HEARTBEAT':
                mode_seen = mavutil.mode_string_v10(msg)
                telemetry_samples.append({
                    'type': msg_type,
                    'armed': bool(getattr(msg, 'base_mode', 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                    'mode': mode_seen,
                })
            if armed and post_alt >= pre_alt + 0.25:
                break

        if post_alt < pre_alt + 0.25:
            fallback_used = True
            climb_deadline = time.time() + 8.0
            while time.time() < climb_deadline:
                try:
                    conn.mav.set_position_target_local_ned_send(
                        0,
                        conn.target_system,
                        conn.target_component,
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        0x0DC7,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        -0.8,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                        0.0,
                    )
                except Exception:
                    pass
                time.sleep(0.2)

            verify_deadline = time.time() + 15.0
            while time.time() < verify_deadline:
                msg = conn.recv_match(blocking=True, timeout=2)
                if msg is None:
                    continue
                msg_type = msg.get_type()
                if msg_type == 'GLOBAL_POSITION_INT':
                    post_alt = max(post_alt, float(getattr(msg, 'relative_alt', 0.0)) / 1000.0)
                    telemetry_samples.append({'type': msg_type, 'relative_alt_m': post_alt})
                elif msg_type == 'VFR_HUD':
                    post_alt = max(post_alt, float(getattr(msg, 'alt', 0.0)))
                    telemetry_samples.append({'type': msg_type, 'alt_m': post_alt})
                elif msg_type == 'HEARTBEAT':
                    armed = bool(getattr(msg, 'base_mode', 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    mode_seen = mavutil.mode_string_v10(msg)
                    telemetry_samples.append({'type': msg_type, 'armed': armed, 'mode': mode_seen})
                if post_alt >= pre_alt + 0.25:
                    break

        item.update({
            'ok': bool(armed and post_alt >= pre_alt + 0.25),
            'pre_alt_m': pre_alt,
            'post_alt_m': post_alt,
            'armed': armed,
            'mode_seen': mode_seen,
            'telemetry_samples': telemetry_samples[-10:],
            'fallback_used': fallback_used,
        })
        if not item['ok']:
            item.setdefault('error', 'altitude did not rise enough after takeoff')
        return item

    except Exception as exc:
        item['error'] = str(exc)
        return item
    finally:
        try:
            if conn is not None:
                conn.close()
        except Exception:
            pass


def main() -> int:
    parser = argparse.ArgumentParser(description='Single Gazebo multi-spawn validator')
    parser.add_argument('--drone-count', type=int, default=3)
    parser.add_argument('--pad-count', type=int, default=3)
    parser.add_argument('--spacing-m', type=float, default=6.0)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--keep-running', action='store_true')
    parser.add_argument('--strict-heartbeat', action='store_true', help='Fail unless all MAVLink heartbeat ports are healthy')
    parser.add_argument('--run-algorithm', action=argparse.BooleanOptionalAction, default=True, help='Run pipeline algorithm after validation succeeds (default: enabled)')
    parser.add_argument('--algorithm-timeout-s', type=int, default=240, help='Timeout for post-validation pipeline algorithm')
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
        state_change_check = _run_state_change_check(5760, target_alt_m=3.0)
        report['topic_check'] = topic_check
        report['heartbeat_check'] = hb_check
        report['state_change_check'] = state_change_check
        report['ok'] = bool(topic_check.get('ok') and state_change_check.get('ok'))
        if args.strict_heartbeat:
            report['ok'] = bool(report['ok'] and hb_check.get('ok'))
        report['strict_heartbeat'] = bool(args.strict_heartbeat)

        if report['ok'] and bool(args.run_algorithm):
            report['algorithm'] = _run_algorithm(args.algorithm_timeout_s)
            report['ok'] = bool(report['ok'] and report['algorithm'].get('ok', False))
        else:
            report['algorithm'] = {
                'ok': False,
                'skipped': True,
                'reason': 'validation failed or run-algorithm disabled',
            }

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
