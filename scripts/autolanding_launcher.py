#!/usr/bin/env python3
"""Python launcher for the AutoLanding workspace."""

from __future__ import annotations

import argparse
import json
import os
import time
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


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

    summary = _run_pipeline(args)
    summary['launcher_processes'] = {
        'gazebo_pid': gazebo.pid,
        'sitl_pid': sitl.pid,
        'rviz_pid': rviz.pid,
    }
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