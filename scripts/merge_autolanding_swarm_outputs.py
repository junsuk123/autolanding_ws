#!/usr/bin/env python3
"""Merge isolated AutoLanding worker outputs into one unified dataset tree.

The script preserves each worker/scenario directory under a merged root and writes
an index manifest for downstream MATLAB and RViz workflows.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
from datetime import datetime
from pathlib import Path


def find_scenario_dirs(source_root: Path) -> list[Path]:
    scenario_dirs: list[Path] = []
    seen: set[Path] = set()
    for raw_path in source_root.rglob('raw_data.mat'):
        scenario_dir = raw_path.parent
        if scenario_dir in seen:
            continue
        seen.add(scenario_dir)
        scenario_dirs.append(scenario_dir)
    return sorted(scenario_dirs)


def merge_scenarios(source_root: Path, output_root: Path) -> dict:
    scenario_dirs = find_scenario_dirs(source_root)
    output_root.mkdir(parents=True, exist_ok=True)

    manifest = {
        'created_at': datetime.now().isoformat(timespec='seconds'),
        'source_root': str(source_root),
        'output_root': str(output_root),
        'scenario_count': len(scenario_dirs),
        'scenarios': [],
    }

    for scenario_dir in scenario_dirs:
        rel = scenario_dir.relative_to(source_root)
        dest_dir = output_root / rel
        dest_dir.parent.mkdir(parents=True, exist_ok=True)
        shutil.copytree(scenario_dir, dest_dir, dirs_exist_ok=True)
        manifest['scenarios'].append({
            'source': str(scenario_dir),
            'destination': str(dest_dir),
            'worker': next((part for part in rel.parts if part.startswith('worker_')), ''),
            'session': next((part for part in rel.parts if part.startswith('scenario_') or part.startswith('20')), ''),
        })

    manifest_path = output_root / 'unified_manifest.json'
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description='Merge isolated AutoLanding worker outputs')
    parser.add_argument('--source-root', required=True, help='Root directory containing isolated worker runs')
    parser.add_argument('--output-root', default='', help='Merged output directory (default: <source-root>/merged)')
    args = parser.parse_args()

    source_root = Path(args.source_root).expanduser().resolve()
    if not source_root.is_dir():
        raise SystemExit(f'[merge] source root not found: {source_root}')

    output_root = Path(args.output_root).expanduser().resolve() if args.output_root else source_root / 'merged'
    manifest = merge_scenarios(source_root, output_root)

    print(f"[merge] source_root={manifest['source_root']}")
    print(f"[merge] output_root={manifest['output_root']}")
    print(f"[merge] scenario_count={manifest['scenario_count']}")
    print(f"[merge] manifest={output_root / 'unified_manifest.json'}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
