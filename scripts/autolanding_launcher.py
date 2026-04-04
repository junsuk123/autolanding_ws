#!/usr/bin/env python3
"""Python launcher for the AutoLanding workspace."""

from __future__ import annotations

import argparse
import json
import os
import time
import subprocess
import sys
import socket
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from pymavlink import mavutil


ROOT = Path(__file__).resolve().parent.parent


def _ensure_root_on_path() -> None:
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))


_ensure_root_on_path()

from src.pipeline import run_pipeline  # noqa: E402
from src.paper_plots import (  # noqa: E402
    generate_model_comparison_artifacts,
    generate_paper_plots,
    write_validation_summary,
)


@dataclass(frozen=True)
class Args:
    mode: str
    orchestration_config: Path
    workspace_root: Path
    semantic_input: Path
    config: Path
    plots_dir: Path
    world: Path
    gui: bool
    headless: bool
    verbose: bool
    no_plots: bool
    workers: int
    scenarios_per_worker: int
    duration: int
    auto_flight_demo: bool
    demo_takeoff_alt_m: float
    demo_master_port: int


def _default_semantic_input() -> Path:
    return ROOT / 'data' / 'samples' / 'semantic_input_example.json'


def _default_config() -> Path:
    return ROOT / 'ai' / 'configs' / 'policy_config.yaml'


def _default_orchestration_config() -> Path:
    return ROOT / 'ai' / 'configs' / 'orchestration_config.yaml'


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding='utf-8'))


def _normalize_mode(value: str) -> str:
    value = value.strip().lower()
    if value in {'pipeline', 'plot', 'validation', 'sim', 'mission', 'stack', 'collect', 'collect_parallel', 'full'}:
        return value
    raise SystemExit(f'[launcher] unsupported mode: {value}')


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    payload = yaml.safe_load(path.read_text(encoding='utf-8'))
    if payload is None:
        return {}
    if not isinstance(payload, dict):
        raise SystemExit(f'[launcher] orchestration config must be a mapping: {path}')
    return payload


def _flag_provided(*flags: str) -> bool:
    argv = set(sys.argv[1:])
    return any(flag in argv for flag in flags)


def _positional_mode_provided() -> bool:
    for token in sys.argv[1:]:
        if token.startswith('-'):
            continue
        return True
    return False


def _script_path(name: str) -> Path:
    return ROOT / 'scripts' / name


def _launch_detached(command: list[str], log_file: Path) -> subprocess.Popen:
    log_file.parent.mkdir(parents=True, exist_ok=True)
    handle = open(log_file, 'w', encoding='utf-8')
    return subprocess.Popen(
        command,
        stdout=handle,
        stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL,
        start_new_session=True,
    )


def _port_open(port: int, timeout_s: float = 0.6) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(timeout_s)
        try:
            sock.connect(('127.0.0.1', int(port)))
            return True
        except OSError:
            return False


def _pick_master_port(preferred_port: int, workers: int) -> int | None:
    candidates: list[int] = []
    if preferred_port > 0:
        candidates.append(preferred_port)
    for i in range(max(1, workers)):
        candidates.append(5760 + (10 * i))
    for port in candidates:
        if _port_open(port):
            return port
    return None


def _run_pymavlink_takeoff(master_port: int, takeoff_alt_m: float) -> dict[str, Any]:
    conn = None
    try:
        conn = mavutil.mavlink_connection(f'tcp:127.0.0.1:{master_port}', source_system=255)
        hb = conn.wait_heartbeat(timeout=25)
        if hb is None:
            return {'ok': False, 'error': 'heartbeat timeout'}

        pre_heartbeat = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        pre_alt_msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        pre_alt = float(pre_alt_msg.relative_alt) / 1000.0 if pre_alt_msg is not None else 0.0

        try:
            conn.set_mode_apm('GUIDED')
        except Exception:
            pass

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
        arm_ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=8)
        armed_wait_deadline = time.time() + 12.0
        armed_confirmed = False
        while time.time() < armed_wait_deadline:
            hb_msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
            if hb_msg is None:
                continue
            armed_confirmed = bool(getattr(hb_msg, 'base_mode', 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed_confirmed:
                break
        if not armed_confirmed:
            return {
                'ok': False,
                'error': 'arm state not confirmed',
                'arm_ack': str(arm_ack) if arm_ack is not None else None,
            }
        time.sleep(0.8)
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
            float(max(1.0, takeoff_alt_m)),
        )

        deadline = time.time() + 20.0
        armed_seen = False
        mode_seen = None
        altitude_seen = pre_alt
        telemetry_samples: list[dict[str, Any]] = []
        while time.time() < deadline:
            msg = conn.recv_match(blocking=True, timeout=2)
            if msg is None:
                continue

            msg_type = msg.get_type()
            if msg_type == 'HEARTBEAT':
                armed_seen = bool(getattr(msg, 'base_mode', 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                mode_seen = mavutil.mode_string_v10(msg)
                telemetry_samples.append({'type': 'HEARTBEAT', 'armed': armed_seen, 'mode': mode_seen})
            elif msg_type == 'GLOBAL_POSITION_INT':
                altitude_seen = max(altitude_seen, float(msg.relative_alt) / 1000.0)
                telemetry_samples.append({'type': 'GLOBAL_POSITION_INT', 'relative_alt_m': altitude_seen})
            elif msg_type == 'VFR_HUD':
                altitude_seen = max(altitude_seen, float(getattr(msg, 'alt', 0.0)))
                telemetry_samples.append({'type': 'VFR_HUD', 'alt_m': altitude_seen})

            if armed_seen and altitude_seen >= pre_alt + 0.5:
                break

        if altitude_seen < pre_alt + 0.25:
            climb_end = time.time() + 6.0
            while time.time() < climb_end:
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
                time.sleep(0.2)

            verify_deadline = time.time() + 12.0
            while time.time() < verify_deadline:
                msg = conn.recv_match(blocking=True, timeout=2)
                if msg is None:
                    continue
                msg_type = msg.get_type()
                if msg_type == 'GLOBAL_POSITION_INT':
                    altitude_seen = max(altitude_seen, float(msg.relative_alt) / 1000.0)
                    telemetry_samples.append({'type': 'GLOBAL_POSITION_INT', 'relative_alt_m': altitude_seen})
                elif msg_type == 'VFR_HUD':
                    altitude_seen = max(altitude_seen, float(getattr(msg, 'alt', 0.0)))
                    telemetry_samples.append({'type': 'VFR_HUD', 'alt_m': altitude_seen})
                elif msg_type == 'HEARTBEAT':
                    armed_seen = bool(getattr(msg, 'base_mode', 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    mode_seen = mavutil.mode_string_v10(msg)
                    telemetry_samples.append({'type': 'HEARTBEAT', 'armed': armed_seen, 'mode': mode_seen})
                if altitude_seen >= pre_alt + 0.25:
                    break

        return {
            'ok': True,
            'heartbeat_system': int(conn.target_system),
            'heartbeat_component': int(conn.target_component),
            'takeoff_alt_m': float(max(1.0, takeoff_alt_m)),
            'pre_alt_m': pre_alt,
            'post_alt_m': altitude_seen,
            'armed_seen': armed_seen,
            'mode_seen': mode_seen,
            'telemetry_samples': telemetry_samples[-8:],
            'heartbeat_seen': bool(pre_heartbeat is not None),
            'arm_ack': str(arm_ack) if arm_ack is not None else None,
        }
    except Exception as exc:
        return {'ok': False, 'error': str(exc)}
    finally:
        try:
            if conn is not None:
                conn.close()
        except Exception:
            pass


def _run_flight_demo(args: Args) -> dict[str, Any]:
    if not args.auto_flight_demo:
        return {'enabled': False, 'reason': 'auto_flight_demo disabled'}

    candidates: list[int] = []
    if args.demo_master_port > 0:
        candidates.append(args.demo_master_port)
    for i in range(max(1, args.workers)):
        candidates.append(5760 + (10 * i))

    # Preserve order while deduplicating.
    seen: set[int] = set()
    ordered_candidates: list[int] = []
    for port in candidates:
        if port in seen:
            continue
        seen.add(port)
        ordered_candidates.append(port)

    attempts = []
    for port in ordered_candidates:
        if not _port_open(port):
            attempts.append({'port': port, 'ok': False, 'error': 'port closed'})
            continue
        result = _run_pymavlink_takeoff(port, args.demo_takeoff_alt_m)
        attempts.append({'port': port, **result})
        if bool(result.get('ok', False)):
            return {
                'enabled': True,
                'ok': True,
                'master_port': port,
                'pymavlink': result,
                'attempts': attempts,
            }

    return {
        'enabled': True,
        'ok': False,
        'error': 'No MAVLink master produced a heartbeat',
        'attempts': attempts,
    }


def _summarize_trajectory(result: dict[str, Any], target: dict[str, Any]) -> dict[str, Any]:
    trajectory = result.get('trajectory', [])
    if not trajectory:
        return {
            'trajectory_points': 0,
            'path_length_m': 0.0,
            'duration_s': 0.0,
            'touchdown_error_xy': 0.0,
            'touchdown_error_z': 0.0,
        }

    path_length = 0.0
    for prev, cur in zip(trajectory[:-1], trajectory[1:]):
        dx = float(cur['x']) - float(prev['x'])
        dy = float(cur['y']) - float(prev['y'])
        dz = float(cur['z']) - float(prev['z'])
        path_length += (dx * dx + dy * dy + dz * dz) ** 0.5

    last = trajectory[-1]
    target_x = float(target.get('x', 0.0))
    target_y = float(target.get('y', 0.0))
    target_z = float(target.get('z', 0.0))
    touchdown_error_xy = ((float(last['x']) - target_x) ** 2 + (float(last['y']) - target_y) ** 2) ** 0.5
    touchdown_error_z = abs(float(last['z']) - target_z)

    return {
        'trajectory_points': len(trajectory),
        'path_length_m': path_length,
        'duration_s': float(last['t']),
        'touchdown_error_xy': touchdown_error_xy,
        'touchdown_error_z': touchdown_error_z,
    }


def _run_pipeline(args: Args) -> dict[str, Any]:
    summary = run_pipeline(str(args.workspace_root), str(args.semantic_input), str(args.config))
    payload = _load_json(Path(summary['out_json']))
    input_payload = _load_json(args.semantic_input)
    payload.setdefault('meta', {})['target'] = input_payload.get('target', {})

    validation = _summarize_trajectory(payload, input_payload.get('target', {}))
    validation['semantic_risk'] = float(summary.get('semantic_risk', 0.0))
    validation['fused_confidence'] = float(payload.get('trajectory', [{}])[-1].get('fused_confidence', 0.0)) if payload.get('trajectory') else 0.0

    validation_path = args.workspace_root / 'data' / 'processed' / 'landing_trajectory_model_validation.json'
    write_validation_summary(validation_path, validation)

    if not args.no_plots:
        plot_paths = generate_paper_plots(payload, args.plots_dir)
        summary['paper_plots'] = plot_paths

        comparison_payloads = {
            'ontology_ai': payload,
            'pure_ai_baseline': payload,
        }
        comparison_paths = generate_model_comparison_artifacts(comparison_payloads, args.plots_dir)
        summary['paper_model_comparison'] = comparison_paths

    summary['validation_json'] = str(validation_path)
    summary['validation'] = validation
    return summary


def _run_stack(args: Args) -> dict[str, Any]:
    gazebo_launcher = _script_path('launch_gazebo_py.py')
    sitl_launcher = _script_path('launch_multi_drone_py.py')
    rviz_launcher = _script_path('launch_rviz_monitor_py.py')

    gazebo_cmd = [sys.executable, str(gazebo_launcher), '--world', str(args.world)]
    gazebo_cmd.append('--gui' if args.gui and not args.headless else '--headless')
    if args.verbose:
        gazebo_cmd.append('--verbose')

    sitl_cmd = [sys.executable, str(sitl_launcher), '--count', str(max(1, args.workers))]
    rviz_cmd = [sys.executable, str(rviz_launcher)]

    gazebo = _launch_detached(gazebo_cmd, Path('/tmp/autolanding_gazebo.log'))
    sitl = _launch_detached(sitl_cmd, Path('/tmp/autolanding_sitl_launcher.log'))
    rviz = _launch_detached(rviz_cmd, Path('/tmp/autolanding_rviz_launcher.log'))

    # Allow simulators/SITL links to initialize before control commands.
    time.sleep(10.0)
    demo_result = _run_flight_demo(args)

    summary = _run_pipeline(args)
    summary['launcher_processes'] = {
        'gazebo_pid': gazebo.pid,
        'sitl_pid': sitl.pid,
        'rviz_pid': rviz.pid,
    }
    summary['flight_demo'] = demo_result
    return summary


def _run_collect(args: Args) -> dict[str, Any]:
    gazebo_launcher = _script_path('launch_gazebo_py.py')
    sitl_launcher = _script_path('launch_multi_drone_py.py')

    gazebo_cmd = [sys.executable, str(gazebo_launcher), '--world', str(args.world)]
    gazebo_cmd.append('--gui' if args.gui and not args.headless else '--headless')
    if args.verbose:
        gazebo_cmd.append('--verbose')

    sitl_cmd = [sys.executable, str(sitl_launcher), '--count', str(max(1, args.workers))]

    timestamp = int(time.time())
    gz_log = Path(f'/tmp/autolanding_collect_gazebo_{timestamp}.log')
    sitl_log = Path(f'/tmp/autolanding_collect_sitl_{timestamp}.log')

    gazebo = _launch_detached(gazebo_cmd, gz_log)
    sitl = _launch_detached(sitl_cmd, sitl_log)

    # Allow stack startup before pipeline/plot generation.
    time.sleep(2.0)
    summary = _run_pipeline(args)

    collection_manifest = {
        'mode': args.mode,
        'workers': max(1, args.workers),
        'scenarios_per_worker': max(1, args.scenarios_per_worker),
        'requested_duration_s': max(10, args.duration),
        'gazebo_pid': gazebo.pid,
        'sitl_pid': sitl.pid,
        'gazebo_log': str(gz_log),
        'sitl_log': str(sitl_log),
        'started_at_epoch_s': timestamp,
    }

    manifest_path = args.workspace_root / 'data' / 'processed' / 'collection_orchestration_summary.json'
    write_validation_summary(manifest_path, collection_manifest)

    summary['collection_orchestration'] = collection_manifest
    summary['collection_orchestration_json'] = str(manifest_path)
    return summary


def parse_args() -> Args:
    parser = argparse.ArgumentParser(description='AutoLanding Python launcher')
    parser.add_argument('mode', nargs='?', default='pipeline')
    parser.add_argument('--orchestration-config', default=str(_default_orchestration_config()))
    parser.add_argument('--workspace-root', default=str(ROOT))
    parser.add_argument('--semantic-input', default=str(_default_semantic_input()))
    parser.add_argument('--config', default=str(_default_config()))
    parser.add_argument('--plots-dir', default=str(ROOT / 'data' / 'plots' / 'paper'))
    parser.add_argument('--world', default='/tmp/iris_runway_aruco_landing.sdf')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--verbose', action='store_true')
    parser.add_argument('--no-plots', action='store_true')
    parser.add_argument('--workers', type=int, default=3)
    parser.add_argument('--scenarios-per-worker', type=int, default=2)
    parser.add_argument('--duration', type=int, default=120)
    parser.add_argument('--auto-flight-demo', action='store_true')
    parser.add_argument('--demo-takeoff-alt-m', type=float, default=3.0)
    parser.add_argument('--demo-master-port', type=int, default=5770)

    ns = parser.parse_args()

    orch_cfg_path = Path(ns.orchestration_config).expanduser().resolve()
    orch_cfg = _load_yaml(orch_cfg_path)

    if not _flag_provided('--workspace-root'):
        ns.workspace_root = orch_cfg.get('workspace_root', ns.workspace_root)
    if not _flag_provided('--semantic-input'):
        ns.semantic_input = orch_cfg.get('semantic_input', ns.semantic_input)
    if not _flag_provided('--config'):
        ns.config = orch_cfg.get('config', ns.config)
    if not _flag_provided('--plots-dir'):
        ns.plots_dir = orch_cfg.get('plots_dir', ns.plots_dir)
    if not _flag_provided('--world'):
        ns.world = orch_cfg.get('world', ns.world)
    if not _flag_provided('--workers'):
        ns.workers = orch_cfg.get('workers', ns.workers)
    if not _flag_provided('--scenarios-per-worker'):
        ns.scenarios_per_worker = orch_cfg.get('scenarios_per_worker', ns.scenarios_per_worker)
    if not _flag_provided('--duration'):
        ns.duration = orch_cfg.get('duration', ns.duration)

    if not _flag_provided('--gui', '--headless'):
        if bool(orch_cfg.get('gui', False)):
            ns.gui = True
            ns.headless = False
        if bool(orch_cfg.get('headless', False)):
            ns.headless = True
            ns.gui = False

    if not _flag_provided('--verbose'):
        ns.verbose = bool(orch_cfg.get('verbose', ns.verbose))
    if not _flag_provided('--no-plots'):
        ns.no_plots = bool(orch_cfg.get('no_plots', ns.no_plots))
    if not _flag_provided('--auto-flight-demo'):
        ns.auto_flight_demo = bool(orch_cfg.get('auto_flight_demo', ns.auto_flight_demo))
    if not _flag_provided('--demo-takeoff-alt-m'):
        ns.demo_takeoff_alt_m = float(orch_cfg.get('demo_takeoff_alt_m', ns.demo_takeoff_alt_m))
    if not _flag_provided('--demo-master-port'):
        ns.demo_master_port = int(orch_cfg.get('demo_master_port', ns.demo_master_port))

    mode_from_cfg = str(orch_cfg.get('mode', ns.mode))
    mode = _normalize_mode(ns.mode if _positional_mode_provided() else mode_from_cfg)

    return Args(
        mode=mode,
        orchestration_config=orch_cfg_path,
        workspace_root=Path(ns.workspace_root).expanduser().resolve(),
        semantic_input=Path(ns.semantic_input).expanduser().resolve(),
        config=Path(ns.config).expanduser().resolve(),
        plots_dir=Path(ns.plots_dir).expanduser().resolve(),
        world=Path(ns.world).expanduser().resolve(),
        gui=bool(ns.gui and not ns.headless),
        headless=bool(ns.headless),
        verbose=bool(ns.verbose),
        no_plots=bool(ns.no_plots),
        workers=max(1, int(ns.workers)),
        scenarios_per_worker=max(1, int(ns.scenarios_per_worker)),
        duration=max(10, int(ns.duration)),
        auto_flight_demo=bool(ns.auto_flight_demo),
        demo_takeoff_alt_m=max(1.0, float(ns.demo_takeoff_alt_m)),
        demo_master_port=max(0, int(ns.demo_master_port)),
    )


def main() -> int:
    args = parse_args()

    if args.mode in {'pipeline', 'validation', 'plot'}:
        summary = _run_pipeline(args)
        print(json.dumps(summary, indent=2))
        return 0

    if args.mode in {'sim', 'mission', 'stack', 'full'}:
        summary = _run_stack(args)
        summary['orchestration_config'] = str(args.orchestration_config)
        print(json.dumps(summary, indent=2))
        return 0

    if args.mode in {'collect', 'collect_parallel'}:
        summary = _run_collect(args)
        print(json.dumps(summary, indent=2))
        return 0

    raise SystemExit(f'[launcher] unsupported mode: {args.mode}')


if __name__ == '__main__':
    raise SystemExit(main())