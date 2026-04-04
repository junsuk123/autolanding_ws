#!/usr/bin/env python3
"""Launch isolated AutoLanding worker stacks in separate process groups.

Each worker starts its own MATLAB-controlled collection pipeline with:
- collection-only mode
- unique worker index offset
- unique Gazebo partition
- shared ROS domain for central RViz unification

The goal is to multiply the collection system instead of running workers only inside
one MATLAB session.
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path
from datetime import datetime


def workspace_root() -> Path:
    return Path(__file__).resolve().parent.parent


def matlab_batch_cmd(matlab_dir: Path, mode: str) -> str:
    return f"cd('{matlab_dir.as_posix()}'); AutoLandingMainFull('{mode}');"


def launch_process(command: str, env: dict[str, str], log_path: Path) -> subprocess.Popen:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_f = open(log_path, 'w')
    return subprocess.Popen(
        command,
        shell=True,
        env=env,
        stdout=log_f,
        stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )


def start_rviz_monitor(root: Path, env: dict[str, str]) -> subprocess.Popen | None:
    launcher = root / "scripts" / "launch_rviz_monitor_py.py"
    if not launcher.is_file():
        print(f"[swarm] RViz launcher not found: {launcher}", file=sys.stderr)
        return None

    cmd = f"python3 '{launcher.as_posix()}'"
    log_path = Path('/tmp') / 'autolanding_rviz_swarm.log'
    proc = launch_process(cmd, env, log_path)
    print(f"[swarm] RViz started pid={proc.pid} log={log_path}")
    return proc


def start_worker(worker_index: int, args: argparse.Namespace, root: Path, matlab_dir: Path) -> subprocess.Popen:
    env = os.environ.copy()
    env['AUTOLANDING_COLLECTION_ROOT'] = str(args.swarm_root / f'worker_{worker_index}')
    env['AUTOLANDING_COLLECTION_ONLY'] = '1'
    env['AUTOLANDING_ISOLATED_WORKER_MODE'] = '1'
    env['AUTOLANDING_NUM_WORKERS'] = '1'
    env['AUTOLANDING_SCENARIOS_PER_WORKER'] = str(args.scenarios_per_worker)
    env['AUTOLANDING_ENABLE_RVIZ'] = '0'
    env['AUTOLANDING_ENABLE_VISUALIZATION'] = '0'
    env['AUTOLANDING_WORKER_INDEX_OFFSET'] = str(worker_index)
    env['GZ_PARTITION'] = f"autolanding_w{worker_index}"
    env.setdefault('ROS_DOMAIN_ID', str(args.ros_domain_id))
    env['AUTOLANDING_GAZEBO_MODE'] = 'gui' if args.gui else 'server'

    matlab_cmd = args.matlab_cmd
    batch = matlab_batch_cmd(matlab_dir, 'gui' if args.gui else 'server')
    command = f"{matlab_cmd} -batch \"{batch}\""

    log_path = Path('/tmp') / f'autolanding_worker_{worker_index}.log'
    proc = launch_process(command, env, log_path)
    print(
        f"[swarm] worker={worker_index} pid={proc.pid} partition={env['GZ_PARTITION']} "
        f"domain={env['ROS_DOMAIN_ID']} log={log_path}"
    )
    return proc


def terminate_group(proc: subprocess.Popen) -> None:
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except Exception:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description='Launch isolated AutoLanding worker stacks')
    parser.add_argument('--workers', type=int, default=3, help='Number of isolated workers')
    parser.add_argument('--scenarios-per-worker', type=int, default=1, help='Scenarios per worker')
    parser.add_argument('--gui', action='store_true', help='Run worker stacks in GUI mode')
    parser.add_argument('--server', action='store_true', help='Run worker stacks in server/headless mode')
    parser.add_argument('--ros-domain-id', type=int, default=0, help='Shared ROS domain for RViz unification')
    parser.add_argument('--matlab-cmd', type=str, default=os.environ.get('MATLAB_CMD', 'matlab'), help='MATLAB command')
    parser.add_argument('--no-rviz', action='store_true', help='Do not start central RViz monitor')
    args = parser.parse_args()

    if args.workers < 1:
        print('[swarm] workers must be >= 1', file=sys.stderr)
        return 1
    if args.server:
        args.gui = False

    root = workspace_root()
    matlab_dir = root / 'matlab'
    if not matlab_dir.is_dir():
        print(f'[swarm] MATLAB directory not found: {matlab_dir}', file=sys.stderr)
        return 1

    swarm_run_id = datetime.now().strftime('%Y%m%d_%H%M%S')
    args.swarm_root = root / 'data' / 'collected' / 'isolated_swarm' / swarm_run_id
    args.swarm_root.mkdir(parents=True, exist_ok=True)
    print(f'[swarm] run_id={swarm_run_id} root={args.swarm_root}')

    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(args.ros_domain_id)

    rviz_proc = None
    if not args.no_rviz:
        rviz_proc = start_rviz_monitor(root, env)
        time.sleep(2.0)

    workers: list[subprocess.Popen] = []
    try:
        for worker_index in range(1, args.workers + 1):
            workers.append(start_worker(worker_index, args, root, matlab_dir))
            time.sleep(1.5)

        exit_code = 0
        for worker_index, proc in enumerate(workers, start=1):
            rc = proc.wait()
            if rc != 0:
                print(f'[swarm] worker={worker_index} exited with code {rc}', file=sys.stderr)
                exit_code = rc if exit_code == 0 else exit_code
            else:
                print(f'[swarm] worker={worker_index} completed successfully')

        merge_script = root / 'scripts' / 'merge_autolanding_swarm_outputs.py'
        if merge_script.is_file():
            merge_env = os.environ.copy()
            merge_env['AUTOLANDING_SWARM_ROOT'] = str(args.swarm_root)
            merge_proc = subprocess.run(
                [sys.executable, str(merge_script), '--source-root', str(args.swarm_root)],
                env=merge_env,
                check=False,
            )
            if merge_proc.returncode == 0:
                merged_root = args.swarm_root / 'merged'
                print(f'[swarm] merged outputs under {merged_root}')
                print(
                    '[swarm] MATLAB unified consumer: '
                    f'AUTOLANDING_USE_EXISTING_DATASET=1 AUTOLANDING_COLLECTION_ROOT={merged_root}'
                )
            else:
                print(f'[swarm] merge script exited with {merge_proc.returncode}', file=sys.stderr)

        return exit_code
    except KeyboardInterrupt:
        print('[swarm] interrupted, stopping workers...')
        for proc in workers:
            terminate_group(proc)
        if rviz_proc is not None:
            terminate_group(rviz_proc)
        return 130
    finally:
        if rviz_proc is not None:
            try:
                terminate_group(rviz_proc)
            except Exception:
                pass


if __name__ == '__main__':
    raise SystemExit(main())
