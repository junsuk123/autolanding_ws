#!/usr/bin/env python3
"""Python launcher for the AutoLanding workspace."""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import socket
import time
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parent.parent
DEFAULT_DRONE_NS_PREFIX = '/autolanding/drone'
DEFAULT_PRIMARY_DRONE_INDEX = 1


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
from src.research_benchmark import (  # noqa: E402
    collect_synthetic_dataset,
    run_train_validate,
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
    auto_landing_execute: bool
    landing_master_port: int
    landing_timeout_s: float


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
    if value in {
        'pipeline',
        'plot',
        'validation',
        'sim',
        'mission',
        'stack',
        'collect',
        'collect_parallel',
        'full',
        'research',
    }:
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


def _normalize_topic_namespace(namespace: str) -> str:
    value = str(namespace).strip()
    if not value:
        return '/'
    if not value.startswith('/'):
        value = '/' + value
    if len(value) > 1 and value.endswith('/'):
        value = value.rstrip('/')
    return value


def _resolve_drone_topic_settings(orch_cfg: dict[str, Any]) -> dict[str, str]:
    raw_idx = orch_cfg.get('primary_drone_index', DEFAULT_PRIMARY_DRONE_INDEX)
    try:
        drone_idx = max(1, int(raw_idx))
    except Exception:
        drone_idx = DEFAULT_PRIMARY_DRONE_INDEX

    ns_prefix = _normalize_topic_namespace(str(orch_cfg.get('drone_ns_prefix', DEFAULT_DRONE_NS_PREFIX)))
    default_drone_ns = f'{ns_prefix}{drone_idx}'
    drone_ns = _normalize_topic_namespace(str(orch_cfg.get('drone_namespace', default_drone_ns)))

    def _topic(key: str, suffix: str) -> str:
        configured = str(orch_cfg.get(key, '')).strip()
        return _normalize_topic_namespace(configured) if configured else f'{drone_ns}/{suffix}'

    return {
        'drone_namespace': drone_ns,
        'camera_image_topic': _topic('camera_image_topic', 'camera'),
        'camera_info_topic': _topic('camera_info_topic', 'camera_info'),
        'aruco_markers_topic': _topic('aruco_markers_topic', 'aruco_markers'),
        'aruco_poses_topic': _topic('aruco_poses_topic', 'aruco_poses'),
    }


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


def _read_aruco_target_from_ros(args: Args, timeout_s: float = 2.5) -> tuple[dict[str, float] | None, str | None]:
    """Read first visible ArUco marker pose from configured ROS2 markers topic.

    Returns (target_dict, error_msg). target_dict is None when unavailable.
    """
    try:
        orch_cfg = _load_yaml(args.orchestration_config)
        topic_settings = _resolve_drone_topic_settings(orch_cfg)
        aruco_markers_topic = topic_settings['aruco_markers_topic']

        common_env = _script_path('common_ros_env.sh')
        setup = ""
        if common_env.exists():
            setup += f"source '{common_env}' >/dev/null 2>&1; "
            setup += f"autl_source_ros_stacks '{args.workspace_root}' >/dev/null 2>&1; "
            setup += "autl_export_comm_defaults >/dev/null 2>&1; "

        cmd = (
            "bash -lc '"
            "set +u; "
            f"{setup}"
            "set -u; "
            f"ros2 topic echo {aruco_markers_topic} ros2_aruco_interfaces/msg/ArucoMarkers --once 2>/dev/null'"
        )
        result = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            timeout=max(1.0, float(timeout_s)),
        )
        raw = (result.stdout or '').strip()
        if not raw:
            return None, 'aruco topic has no message'

        marker_id = None
        m_id = re.search(r"marker_ids:\s*\n\s*-\s*(-?\d+)", raw)
        if m_id:
            marker_id = int(m_id.group(1))

        # Parse first pose position x/y/z from ros2 topic echo output.
        m_xyz = re.search(
            r"poses:\s*\n\s*-\s*position:\s*\n\s*x:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"
            r"\n\s*y:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)"
            r"\n\s*z:\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)",
            raw,
        )
        if not m_xyz:
            return None, 'aruco topic message parse failed'

        target = {
            'x': float(m_xyz.group(1)),
            'y': float(m_xyz.group(2)),
            'z': float(m_xyz.group(3)),
            'source': f'{aruco_markers_topic}.pose.position',
        }
        if marker_id is not None:
            target['marker_id'] = float(marker_id)
        return target, None
    except subprocess.TimeoutExpired:
        return None, 'aruco topic read timeout'
    except Exception as exc:
        return None, str(exc)


def _gazebo_running() -> bool:
    """Check if Gazebo is already running."""
    try:
        result = subprocess.run(
            ['pgrep', '-f', 'gz sim'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=2.0,
        )
        return result.returncode == 0
    except Exception:
        return False


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


def _wait_for_ports(ports: list[int], timeout_s: float = 45.0) -> bool:
    deadline = time.time() + max(5.0, float(timeout_s))
    unique_ports: list[int] = []
    seen: set[int] = set()
    for port in ports:
        p = int(port)
        if p in seen:
            continue
        seen.add(p)
        unique_ports.append(p)

    while time.time() < deadline:
        if all(_port_open(port, timeout_s=0.4) for port in unique_ports):
            return True
        time.sleep(0.5)
    return False


def _run_pymavlink_takeoff(master_port: int, takeoff_alt_m: float) -> dict[str, Any]:
    conn = None
    try:
        try:
            from pymavlink import mavutil
        except Exception as exc:
            return {'ok': False, 'error': f'pymavlink unavailable: {exc}'}

        conn = mavutil.mavlink_connection(f'tcp:127.0.0.1:{master_port}', source_system=255)
        hb = None
        prime_deadline = time.time() + 8.0
        while time.time() < prime_deadline and hb is None:
            try:
                conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
            except Exception:
                pass
            hb = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)

        if hb is None:
            hb = conn.wait_heartbeat(timeout=16)
        if hb is None:
            return {'ok': False, 'error': 'heartbeat timeout'}

        pre_alt = 0.0
        pre_deadline = time.time() + 5.0
        while time.time() < pre_deadline:
            alt_msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if alt_msg is not None:
                pre_alt = float(alt_msg.relative_alt) / 1000.0
                break

        try:
            conn.set_mode_apm('GUIDED')
        except Exception:
            pass

        guided = False
        guided_deadline = time.time() + 10.0
        while time.time() < guided_deadline:
            hb_msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if hb_msg is None:
                continue
            if mavutil.mode_string_v10(hb_msg) == 'GUIDED':
                guided = True
                break
        if not guided:
            return {'ok': False, 'error': 'GUIDED mode not confirmed'}

        try:
            conn.mav.request_data_stream_send(
                conn.target_system,
                conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                8,
                1,
            )
        except Exception:
            pass

        arm_ack = None
        armed = False
        for arm_try in range(3):
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
            arm_ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=6)
            arm_deadline = time.time() + 8.0
            while time.time() < arm_deadline:
                hb_msg = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
                if hb_msg is None:
                    continue
                armed = bool(getattr(hb_msg, 'base_mode', 0) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if armed:
                    break
            if armed:
                break
            if arm_try < 2:
                time.sleep(0.8)
        if not armed:
            return {'ok': False, 'error': 'arm state not confirmed', 'arm_ack': str(arm_ack) if arm_ack is not None else None}

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

        target_alt_m = float(max(1.0, takeoff_alt_m))
        success_alt_m = max(pre_alt + 0.8, target_alt_m * 0.8)
        climb_rate_m_s = 0.8

        try:
            conn.mav.request_data_stream_send(
                conn.target_system,
                conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                10,
                1,
            )
        except Exception:
            pass

        post_alt = pre_alt
        sample_tail: list[dict[str, Any]] = []
        check_deadline = time.time() + 45.0
        last_reinforce = 0.0
        while time.time() < check_deadline:
            msg = conn.recv_match(blocking=True, timeout=1)
            if msg is None:
                if time.time() - last_reinforce >= 2.0:
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
                            target_alt_m,
                        )
                    except Exception:
                        pass

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
                            -climb_rate_m_s,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                        )
                    except Exception:
                        pass

                    last_reinforce = time.time()
                continue
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                post_alt = max(post_alt, float(msg.relative_alt) / 1000.0)
                sample_tail.append({'type': 'GLOBAL_POSITION_INT', 'relative_alt_m': post_alt})
                if len(sample_tail) > 8:
                    sample_tail = sample_tail[-8:]
            if post_alt >= success_alt_m:
                break

        if post_alt < success_alt_m:
            climb_deadline = time.time() + 15.0
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

                msg = conn.recv_match(blocking=True, timeout=1)
                if msg is not None and msg.get_type() == 'GLOBAL_POSITION_INT':
                    post_alt = max(post_alt, float(msg.relative_alt) / 1000.0)
                    sample_tail.append({'type': 'GLOBAL_POSITION_INT', 'relative_alt_m': post_alt})
                    if len(sample_tail) > 8:
                        sample_tail = sample_tail[-8:]
                if post_alt >= success_alt_m:
                    break

        ok = bool(post_alt >= success_alt_m)
        return {
            'ok': ok,
            'pre_alt_m': pre_alt,
            'post_alt_m': post_alt,
            'takeoff_alt_m': target_alt_m,
            'success_alt_m': success_alt_m,
            'arm_ack': str(arm_ack) if arm_ack is not None else None,
            'telemetry_samples': sample_tail,
            'error': None if ok else f'altitude did not rise enough after takeoff (target {success_alt_m:.2f} m)',
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
    for i in range(max(1, args.workers)):
        candidates.append(5762 + (10 * i))

    seen: set[int] = set()
    ordered_candidates: list[int] = []
    for p in candidates:
        if p in seen:
            continue
        seen.add(p)
        ordered_candidates.append(p)

    attempts: list[dict[str, Any]] = []
    rounds = 3
    for round_idx in range(rounds):
        for port in ordered_candidates:
            if not _port_open(port):
                attempts.append({'round': round_idx + 1, 'port': port, 'ok': False, 'error': 'port closed'})
                continue
            result = _run_pymavlink_takeoff(port, args.demo_takeoff_alt_m)
            attempts.append({'round': round_idx + 1, 'port': port, **result})
            if bool(result.get('ok', False)):
                return {
                    'enabled': True,
                    'ok': True,
                    'master_port': port,
                    'pymavlink': result,
                    'attempts': attempts,
                }
        if round_idx < rounds - 1:
            time.sleep(2.0)

    return {
        'enabled': True,
        'ok': False,
        'error': 'No MAVLink master produced successful takeoff',
        'attempts': attempts,
    }


def _resolve_landing_target(args: Args) -> dict[str, Any]:
    orch_cfg = _load_yaml(args.orchestration_config)
    prefer_ros_aruco = bool(orch_cfg.get('prefer_ros_aruco_target', True))
    aruco_timeout_s = float(orch_cfg.get('aruco_ros_target_timeout_s', 8.0))

    if prefer_ros_aruco:
        ros_target, ros_err = _read_aruco_target_from_ros(args, timeout_s=aruco_timeout_s)
        if ros_target is not None:
            return ros_target

    semantic_input = _load_json(args.semantic_input)
    target = semantic_input.get('target', {'x': 0.0, 'y': 0.0, 'z': 0.0})
    out = {
        'x': float(target.get('x', 0.0)),
        'y': float(target.get('y', 0.0)),
        'z': float(target.get('z', 0.0)),
        'source': 'semantic_input.target',
    }

    aruco = semantic_input.get('aruco', {})
    marker_center = aruco.get('marker_center')
    if isinstance(marker_center, list) and len(marker_center) >= 3:
        out = {
            'x': float(marker_center[0]),
            'y': float(marker_center[1]),
            'z': float(marker_center[2]),
            'source': 'semantic_input.aruco.marker_center',
        }

    if prefer_ros_aruco:
        out['aruco_ros_fallback_reason'] = ros_err if 'ros_err' in locals() else 'unknown'
    return out


def _load_controller_bundle(args: Args) -> dict[str, Any] | None:
    model_path = args.workspace_root / 'data' / 'models' / 'research_linear_models.json'
    if not model_path.exists():
        return None
    try:
        payload = _load_json(model_path)
    except Exception:
        return None
    if not isinstance(payload, dict):
        return None
    if 'proposed_ontology_ai' not in payload:
        return None
    return payload


def _predict_model_velocity(bundle: dict[str, Any], state: dict[str, float], sem: dict[str, float]) -> tuple[float, float, float]:
    model = bundle.get('proposed_ontology_ai', {})
    names = model.get('feature_names', [])
    weights = model.get('weights', [])
    if not names or not weights:
        raise ValueError('invalid model bundle')

    # Construct feature vector in the same order as training.
    fmap = {
        'x': float(state.get('x', 0.0)),
        'y': float(state.get('y', 0.0)),
        'z': float(state.get('z', 0.0)),
        'vx': float(state.get('vx', 0.0)),
        'vy': float(state.get('vy', 0.0)),
        'vz': float(state.get('vz', 0.0)),
        'dx': float(state.get('dx', 0.0)),
        'dy': float(state.get('dy', 0.0)),
        'dz': float(state.get('dz', 0.0)),
        'wind_risk': float(sem.get('wind_risk', 0.0)),
        'vision_risk': float(sem.get('vision_risk', 0.0)),
        'alignment_risk': float(sem.get('alignment_risk', 0.0)),
        'semantic_risk': float(sem.get('semantic_risk', 0.0)),
        'semantic_safety': float(sem.get('semantic_safety', 0.0)),
    }

    x = [float(fmap.get(name, 0.0)) for name in names] + [1.0]
    # weights shape: (n_features + 1, n_targets)
    w = [[float(v) for v in row] for row in weights]
    n_targets = len(w[0]) if w and w[0] else 0
    if n_targets < 3:
        raise ValueError('invalid target dimension in model')

    pred = [0.0, 0.0, 0.0]
    for i, xi in enumerate(x):
        for j in range(3):
            pred[j] += xi * float(w[i][j])

    return float(pred[0]), float(pred[1]), float(pred[2])


def _write_landing_rollout(args: Args, rows: list[dict[str, Any]]) -> str:
    out_dir = args.workspace_root / 'data' / 'collected'
    out_dir.mkdir(parents=True, exist_ok=True)
    ts = int(time.time())
    out_csv = out_dir / f'landing_controller_rollout_{ts}.csv'
    if rows:
        with open(out_csv, 'w', encoding='utf-8', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
    return str(out_csv)


def _run_landing_execution(args: Args, trajectory: list[dict[str, Any]], preferred_port: int = 0) -> dict[str, Any]:
    if not args.auto_landing_execute:
        return {'enabled': False, 'reason': 'auto_landing_execute disabled'}

    if not trajectory:
        return {'enabled': True, 'ok': False, 'error': 'trajectory is empty'}

    try:
        from pymavlink import mavutil
    except Exception as exc:
        return {'enabled': True, 'ok': False, 'error': f'pymavlink unavailable: {exc}'}

    candidates: list[int] = []
    if preferred_port > 0:
        candidates.append(preferred_port)
    if args.landing_master_port > 0:
        candidates.append(args.landing_master_port)
    for i in range(max(1, args.workers)):
        candidates.append(5760 + (10 * i))
        candidates.append(5762 + (10 * i))

    seen: set[int] = set()
    ordered_candidates: list[int] = []
    for p in candidates:
        if p in seen:
            continue
        seen.add(p)
        ordered_candidates.append(p)

    conn = None
    selected_port = 0
    for port in ordered_candidates:
        if not _port_open(port):
            continue
        try:
            trial = mavutil.mavlink_connection(f'tcp:127.0.0.1:{port}', source_system=254)
            hb = trial.wait_heartbeat(timeout=8)
            if hb is None:
                trial.close()
                continue
            conn = trial
            selected_port = port
            break
        except Exception:
            try:
                trial.close()
            except Exception:
                pass
            continue

    if conn is None:
        return {'enabled': True, 'ok': False, 'error': 'no reachable MAVLink master port for landing execution'}

    try:
        orch_cfg = _load_yaml(args.orchestration_config)
        requested_controller = str(orch_cfg.get('landing_controller', 'auto')).strip().lower()
        if requested_controller not in {'auto', 'model', 'pid', 'png'}:
            requested_controller = 'auto'
        topic_settings = _resolve_drone_topic_settings(orch_cfg)
        aruco_markers_topic = topic_settings['aruco_markers_topic']
        aruco_source = f'{aruco_markers_topic}.pose.position'

        target = _resolve_landing_target(args)
        prefer_ros_aruco = bool(orch_cfg.get('prefer_ros_aruco_target', True))
        require_aruco_for_landing = bool(orch_cfg.get('require_ros_aruco_for_landing', True))
        aruco_reacquire_window_s = float(orch_cfg.get('aruco_ros_reacquire_window_s', 20.0))
        aruco_reacquire_interval_s = float(orch_cfg.get('aruco_ros_reacquire_interval_s', 1.0))
        aruco_reacquire_read_timeout_s = float(orch_cfg.get('aruco_ros_reacquire_read_timeout_s', 2.0))
        aruco_reacquired_after_takeoff = False
        aruco_reacquire_attempts = 0

        # If marker wasn't visible at first read, keep trying briefly to account for
        # takeoff/attitude changes after which the marker can enter camera FOV.
        if (
            prefer_ros_aruco
            and str(target.get('source', '')) != aruco_source
            and aruco_reacquire_window_s > 0.0
        ):
            reacquire_deadline = time.time() + max(0.0, aruco_reacquire_window_s)
            last_ros_err = target.get('aruco_ros_fallback_reason')
            while time.time() < reacquire_deadline:
                ros_target, ros_err = _read_aruco_target_from_ros(
                    args,
                    timeout_s=max(0.5, aruco_reacquire_read_timeout_s),
                )
                aruco_reacquire_attempts += 1
                if ros_target is not None:
                    target = ros_target
                    aruco_reacquired_after_takeoff = True
                    break
                last_ros_err = ros_err
                time.sleep(max(0.1, aruco_reacquire_interval_s))

            if not aruco_reacquired_after_takeoff and last_ros_err:
                target['aruco_ros_fallback_reason'] = str(last_ros_err)

        aruco_locked = str(target.get('source', '')) == aruco_source
        if prefer_ros_aruco and require_aruco_for_landing and not aruco_locked:
            return {
                'enabled': True,
                'ok': False,
                'master_port': selected_port,
                'target_source': str(target.get('source', 'unknown')),
                'aruco_required': True,
                'aruco_detected': False,
                'aruco_reacquired_after_takeoff': bool(aruco_reacquired_after_takeoff),
                'aruco_reacquire_attempts': int(aruco_reacquire_attempts),
                'target': {
                    'x': float(target.get('x', 0.0)),
                    'y': float(target.get('y', 0.0)),
                    'z': float(target.get('z', 0.0)),
                    'marker_id': target.get('marker_id'),
                    'aruco_ros_fallback_reason': target.get('aruco_ros_fallback_reason'),
                },
                'error': 'landing blocked: require_ros_aruco_for_landing is true but ArUco target unavailable',
            }

        semantic_input = _load_json(args.semantic_input)
        context_rel = semantic_input.get('context_file')
        sem = {}
        if isinstance(context_rel, str):
            context_path = (args.workspace_root / context_rel).resolve()
            if context_path.exists():
                try:
                    from src.semantic_feature_extractor import extract_semantic_features

                    sem = extract_semantic_features(_load_json(context_path))
                except Exception:
                    sem = {}

        model_bundle = _load_controller_bundle(args)
        if requested_controller == 'auto':
            controller = 'model' if model_bundle is not None else 'pid'
        elif requested_controller == 'model' and model_bundle is None:
            controller = 'pid'
        else:
            controller = requested_controller

        try:
            conn.set_mode_apm('GUIDED')
        except Exception:
            pass

        sent_points = 0
        z_min = None
        rollout_rows: list[dict[str, Any]] = []

        # Velocity control mask (vx,vy,vz enabled)
        type_mask_vel_only = 0x0DC7

        kp = float(orch_cfg.get('landing_pid_kp', 0.8))
        kd = float(orch_cfg.get('landing_pid_kd', 0.15))
        max_xy = float(orch_cfg.get('landing_max_xy_speed', 0.8))
        max_vz_down = float(orch_cfg.get('landing_max_descent_speed', 0.7))
        min_vz_down = float(orch_cfg.get('landing_min_descent_speed', 0.2))
        nav_const = float(orch_cfg.get('landing_png_gain', 2.0))

        prev_ex = 0.0
        prev_ey = 0.0
        prev_t = time.time()

        ctrl_deadline = time.time() + float(max(5.0, args.landing_timeout_s))
        current_alt = None
        while time.time() < ctrl_deadline:
            now = time.time()
            dt = max(1e-3, now - prev_t)
            prev_t = now

            lp = conn.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.25)
            gp = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.25)

            if gp is not None:
                current_alt = float(gp.relative_alt) / 1000.0

            if lp is None:
                # If local position is unavailable, fallback to trajectory point commanding.
                if trajectory:
                    p = trajectory[min(sent_points, len(trajectory) - 1)]
                    vx_cmd = float(p.get('vx_cmd', 0.0))
                    vy_cmd = float(p.get('vy_cmd', 0.0))
                    vz_cmd = max(min_vz_down, min(max_vz_down, abs(float(p.get('vz_cmd', -0.3)))))
                else:
                    vx_cmd = 0.0
                    vy_cmd = 0.0
                    vz_cmd = min_vz_down
                x = 0.0
                y = 0.0
                z_up = float(current_alt if current_alt is not None else 0.0)
            else:
                x = float(getattr(lp, 'x', 0.0))
                y = float(getattr(lp, 'y', 0.0))
                z_ned = float(getattr(lp, 'z', 0.0))
                z_up = -z_ned

                tx = float(target['x'])
                ty = float(target['y'])
                tz = float(target['z'])
                ex = tx - x
                ey = ty - y
                de_x = (ex - prev_ex) / dt
                de_y = (ey - prev_ey) / dt
                prev_ex = ex
                prev_ey = ey

                if controller == 'model' and model_bundle is not None:
                    state = {
                        'x': x,
                        'y': y,
                        'z': z_up,
                        'vx': float(getattr(lp, 'vx', 0.0)),
                        'vy': float(getattr(lp, 'vy', 0.0)),
                        'vz': float(getattr(lp, 'vz', 0.0)),
                        'dx': ex,
                        'dy': ey,
                        'dz': max(0.0, z_up - tz),
                    }
                    try:
                        vx_cmd, vy_cmd, vz_model = _predict_model_velocity(model_bundle, state, sem)
                        vz_cmd = max(min_vz_down, min(max_vz_down, abs(vz_model)))
                    except Exception:
                        controller = 'pid'

                if controller == 'pid':
                    vx_cmd = max(-max_xy, min(max_xy, kp * ex + kd * de_x))
                    vy_cmd = max(-max_xy, min(max_xy, kp * ey + kd * de_y))
                    alt_err = max(0.0, z_up - tz)
                    vz_cmd = min(max_vz_down, max(min_vz_down, 0.25 + 0.2 * alt_err))
                elif controller == 'png':
                    r = max(1e-3, (ex * ex + ey * ey) ** 0.5)
                    vx_cmd = max(-max_xy, min(max_xy, nav_const * ex / r * min(max_xy, r)))
                    vy_cmd = max(-max_xy, min(max_xy, nav_const * ey / r * min(max_xy, r)))
                    alt_err = max(0.0, z_up - tz)
                    vz_cmd = min(max_vz_down, max(min_vz_down, 0.2 + 0.18 * alt_err))

            # NED velocity: positive vz means descending.
            conn.mav.set_position_target_local_ned_send(
                0,
                conn.target_system,
                conn.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask_vel_only,
                0.0,
                0.0,
                0.0,
                float(vx_cmd),
                float(vy_cmd),
                float(vz_cmd),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )

            sent_points += 1
            if z_min is None or z_up < z_min:
                z_min = z_up

            rollout_rows.append(
                {
                    't_epoch': now,
                    'controller': controller,
                    'target_x': float(target['x']),
                    'target_y': float(target['y']),
                    'target_z': float(target['z']),
                    'x': float(x),
                    'y': float(y),
                    'z': float(z_up),
                    'vx_cmd': float(vx_cmd),
                    'vy_cmd': float(vy_cmd),
                    'vz_cmd_down': float(vz_cmd),
                    'current_alt_m': float(current_alt if current_alt is not None else z_up),
                }
            )

            if current_alt is not None and current_alt <= 0.25:
                break

            time.sleep(0.1)

        # Finalize with LAND command.
        conn.mav.command_long_send(
            conn.target_system,
            conn.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )

        touchdown_ok = False
        final_alt_m = None
        land_deadline = time.time() + 30.0
        while time.time() < land_deadline:
            msg = conn.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg is None:
                continue
            final_alt_m = float(msg.relative_alt) / 1000.0
            if final_alt_m <= 0.15:
                touchdown_ok = True
                break

        rollout_csv = _write_landing_rollout(args, rollout_rows)

        return {
            'enabled': True,
            'ok': bool(touchdown_ok),
            'master_port': selected_port,
            'target_source': str(target.get('source', 'unknown')),
            'aruco_required': bool(prefer_ros_aruco and require_aruco_for_landing),
            'aruco_detected': bool(str(target.get('source', '')) == aruco_source),
            'aruco_reacquired_after_takeoff': bool(aruco_reacquired_after_takeoff),
            'aruco_reacquire_attempts': int(aruco_reacquire_attempts),
            'target': {
                'x': float(target.get('x', 0.0)),
                'y': float(target.get('y', 0.0)),
                'z': float(target.get('z', 0.0)),
                'marker_id': target.get('marker_id'),
                'aruco_ros_fallback_reason': target.get('aruco_ros_fallback_reason'),
            },
            'controller': controller,
            'sent_points': sent_points,
            'z_min_cmd_m': z_min,
            'final_relative_alt_m': final_alt_m,
            'rollout_csv': rollout_csv,
            'error': None if touchdown_ok else 'touchdown altitude threshold not reached',
        }
    except Exception as exc:
        return {'enabled': True, 'ok': False, 'master_port': selected_port, 'error': str(exc)}
    finally:
        try:
            conn.close()
        except Exception:
            pass


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

    # Check if Gazebo is already running
    if _gazebo_running():
        print('[launcher] Gazebo is already running; skipping launch (will use existing instance)')
        gazebo = None
        gazebo_pid = None
    else:
        gazebo_cmd = [sys.executable, str(gazebo_launcher), '--world', str(args.world)]
        gazebo_cmd.append('--gui' if args.gui and not args.headless else '--headless')
        if args.verbose:
            gazebo_cmd.append('--verbose')

        gazebo = _launch_detached(gazebo_cmd, Path('/tmp/autolanding_gazebo.log'))
        gazebo_pid = gazebo.pid

    sitl_cmd = [sys.executable, str(sitl_launcher), '--count', str(max(1, args.workers))]
    rviz_cmd = [sys.executable, str(rviz_launcher)]

    if gazebo is not None:
        sitl = _launch_detached(sitl_cmd, Path('/tmp/autolanding_sitl_launcher.log'))
    else:
        # If Gazebo was already running, still launch SITL but skip initialization wait
        sitl = _launch_detached(sitl_cmd, Path('/tmp/autolanding_sitl_launcher.log'))

    expected_ports = [5760 + (10 * i) for i in range(max(1, args.workers))]
    _wait_for_ports(expected_ports, timeout_s=50.0)
    time.sleep(6.0)
    # Start monitor stack early so bridge + aruco topics are ready before landing target resolution.
    rviz = _launch_detached(rviz_cmd, Path('/tmp/autolanding_rviz_launcher.log'))
    time.sleep(3.0)
    demo_result = _run_flight_demo(args)

    summary = _run_pipeline(args)
    payload = _load_json(Path(summary['out_json']))
    trajectory = payload.get('trajectory', []) if isinstance(payload, dict) else []

    preferred_port = int(demo_result.get('master_port', 0)) if isinstance(demo_result, dict) else 0
    landing_result = _run_landing_execution(args, trajectory, preferred_port=preferred_port)

    summary['launcher_processes'] = {
        'gazebo_pid': gazebo_pid,
        'sitl_pid': sitl.pid,
        'rviz_pid': rviz.pid,
    }
    summary['flight_demo'] = demo_result
    summary['landing_execution'] = landing_result
    return summary


def _run_research(args: Args) -> dict[str, Any]:
    dataset_csv = args.workspace_root / 'data' / 'processed' / 'research_dataset.csv'
    dataset_meta = args.workspace_root / 'data' / 'processed' / 'research_dataset_meta.json'
    summary_json = args.workspace_root / 'data' / 'processed' / 'research_train_validate_summary.json'
    models_json = args.workspace_root / 'data' / 'models' / 'research_linear_models.json'

    collect_info = collect_synthetic_dataset(
        config_file=args.config,
        out_csv=dataset_csv,
        out_json=dataset_meta,
        num_samples=max(1000, args.duration * 30),
        seed=42,
    )

    train_summary = run_train_validate(
        dataset_csv=dataset_csv,
        out_json=summary_json,
        out_models_json=models_json,
        seed=42,
    )

    return {
        'collection': collect_info,
        'summary_json': str(summary_json),
        'models_json': str(models_json),
        'claim_check': train_summary.get('claim_check', {}),
        'improvement_percent': train_summary.get('improvement_percent', {}),
    }


def _run_collect(args: Args) -> dict[str, Any]:
    gazebo_launcher = _script_path('launch_gazebo_py.py')
    sitl_launcher = _script_path('launch_multi_drone_py.py')

    # Check if Gazebo is already running
    if _gazebo_running():
        print('[launcher] Gazebo is already running; skipping launch (will use existing instance)')
        gazebo = None
        gazebo_pid = None
        gz_log = None
    else:
        gazebo_cmd = [sys.executable, str(gazebo_launcher), '--world', str(args.world)]
        gazebo_cmd.append('--gui' if args.gui and not args.headless else '--headless')
        if args.verbose:
            gazebo_cmd.append('--verbose')

        timestamp = int(time.time())
        gz_log = Path(f'/tmp/autolanding_collect_gazebo_{timestamp}.log')
        gazebo = _launch_detached(gazebo_cmd, gz_log)
        gazebo_pid = gazebo.pid

    sitl_cmd = [sys.executable, str(sitl_launcher), '--count', str(max(1, args.workers))]

    timestamp = int(time.time())
    sitl_log = Path(f'/tmp/autolanding_collect_sitl_{timestamp}.log')

    sitl = _launch_detached(sitl_cmd, sitl_log)

    # Allow stack startup before pipeline/plot generation.
    time.sleep(2.0)
    summary = _run_pipeline(args)

    collection_manifest = {
        'mode': args.mode,
        'workers': max(1, args.workers),
        'scenarios_per_worker': max(1, args.scenarios_per_worker),
        'requested_duration_s': max(10, args.duration),
        'gazebo_pid': gazebo_pid,
        'gazebo_log': str(gz_log) if gz_log else '<using existing instance>',
        'sitl_pid': sitl.pid,
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
    parser.add_argument('--scenarios-per-worker', type=int, default=10)
    parser.add_argument('--duration', type=int, default=120)
    parser.add_argument('--auto-flight-demo', action='store_true')
    parser.add_argument('--demo-takeoff-alt-m', type=float, default=3.0)
    parser.add_argument('--demo-master-port', type=int, default=0)
    parser.add_argument('--auto-landing-execute', action='store_true')
    parser.add_argument('--landing-master-port', type=int, default=0)
    parser.add_argument('--landing-timeout-s', type=float, default=45.0)

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
    if not _flag_provided('--auto-landing-execute'):
        ns.auto_landing_execute = bool(orch_cfg.get('auto_landing_execute', ns.auto_landing_execute))
    if not _flag_provided('--landing-master-port'):
        ns.landing_master_port = int(orch_cfg.get('landing_master_port', ns.landing_master_port))
    if not _flag_provided('--landing-timeout-s'):
        ns.landing_timeout_s = float(orch_cfg.get('landing_timeout_s', ns.landing_timeout_s))

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
        auto_landing_execute=bool(ns.auto_landing_execute),
        landing_master_port=max(0, int(ns.landing_master_port)),
        landing_timeout_s=max(10.0, float(ns.landing_timeout_s)),
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

    if args.mode == 'research':
        summary = _run_research(args)
        summary['orchestration_config'] = str(args.orchestration_config)
        print(json.dumps(summary, indent=2))
        return 0

    raise SystemExit(f'[launcher] unsupported mode: {args.mode}')


if __name__ == '__main__':
    raise SystemExit(main())