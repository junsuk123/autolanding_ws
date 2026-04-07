from __future__ import annotations

import json
import math
import csv
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt


def write_validation_summary(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding='utf-8')


def _series(trajectory: list[dict[str, Any]], key: str) -> list[float]:
    return [float(point.get(key, 0.0)) for point in trajectory]


def _path_length(trajectory: list[dict[str, Any]]) -> float:
    if len(trajectory) < 2:
        return 0.0
    total = 0.0
    for prev, cur in zip(trajectory[:-1], trajectory[1:]):
        dx = float(cur['x']) - float(prev['x'])
        dy = float(cur['y']) - float(prev['y'])
        dz = float(cur['z']) - float(prev['z'])
        total += math.sqrt(dx * dx + dy * dy + dz * dz)
    return total


def generate_paper_plots(result: dict[str, Any], output_dir: Path) -> dict[str, str]:
    trajectory = result.get('trajectory', [])
    if not trajectory:
        raise ValueError('trajectory is empty; cannot generate plots')

    output_dir.mkdir(parents=True, exist_ok=True)

    t = _series(trajectory, 't')
    x = _series(trajectory, 'x')
    y = _series(trajectory, 'y')
    z = _series(trajectory, 'z')
    fused = _series(trajectory, 'fused_confidence')
    speed = [math.sqrt(float(point.get('vx_cmd', 0.0)) ** 2 + float(point.get('vy_cmd', 0.0)) ** 2 + float(point.get('vz_cmd', 0.0)) ** 2) for point in trajectory]

    target = result.get('meta', {}).get('target', {'x': 0.0, 'y': 0.0, 'z': 0.0})
    path_length = _path_length(trajectory)
    touchdown_error_xy = math.sqrt((x[-1] - float(target.get('x', 0.0))) ** 2 + (y[-1] - float(target.get('y', 0.0))) ** 2)
    touchdown_error_z = abs(z[-1] - float(target.get('z', 0.0)))

    plt.style.use('seaborn-v0_8-whitegrid')
    fig = plt.figure(figsize=(14, 9), constrained_layout=True)
    grid = fig.add_gridspec(2, 3)

    try:
        ax3d = fig.add_subplot(grid[:, 0], projection='3d')
        ax3d.plot(x, y, z, color='#0f766e', linewidth=2.5)
        ax3d.scatter([x[0]], [y[0]], [z[0]], color='#2563eb', s=60, label='start')
        ax3d.scatter([x[-1]], [y[-1]], [z[-1]], color='#dc2626', s=60, label='touchdown')
        ax3d.scatter([float(target.get('x', 0.0))], [float(target.get('y', 0.0))], [float(target.get('z', 0.0))], color='#111827', s=50, marker='x', label='target')
        ax3d.set_title('Landing Trajectory')
        ax3d.set_xlabel('X (m)')
        ax3d.set_ylabel('Y (m)')
        ax3d.set_zlabel('Z (m)')
        ax3d.legend(loc='upper right')
    except Exception:
        # Fallback for environments where mplot3d is unavailable.
        ax3d = fig.add_subplot(grid[:, 0])
        ax3d.plot(x, z, color='#0f766e', linewidth=2.5)
        ax3d.scatter([x[0]], [z[0]], color='#2563eb', s=60, label='start')
        ax3d.scatter([x[-1]], [z[-1]], color='#dc2626', s=60, label='touchdown')
        ax3d.set_title('Landing Trajectory (X-Z fallback)')
        ax3d.set_xlabel('X (m)')
        ax3d.set_ylabel('Z (m)')
        ax3d.legend(loc='upper right')

    ax_xy = fig.add_subplot(grid[0, 1])
    ax_xy.plot(x, y, color='#7c3aed', linewidth=2.0)
    ax_xy.scatter([float(target.get('x', 0.0))], [float(target.get('y', 0.0))], color='#111827', s=35, marker='x')
    ax_xy.set_title('XY Footprint')
    ax_xy.set_xlabel('X (m)')
    ax_xy.set_ylabel('Y (m)')
    ax_xy.axis('equal')

    ax_alt = fig.add_subplot(grid[0, 2])
    ax_alt.plot(t, z, color='#16a34a', linewidth=2.0)
    ax_alt.set_title('Altitude Profile')
    ax_alt.set_xlabel('Time (s)')
    ax_alt.set_ylabel('Z (m)')

    ax_speed = fig.add_subplot(grid[1, 1])
    ax_speed.plot(t, speed, color='#d97706', linewidth=2.0)
    ax_speed.set_title('Commanded Speed')
    ax_speed.set_xlabel('Time (s)')
    ax_speed.set_ylabel('Speed (m/s)')

    ax_sem = fig.add_subplot(grid[1, 2])
    ax_sem.plot(t, fused, color='#0ea5e9', linewidth=2.0)
    ax_sem.set_title('Fused Confidence')
    ax_sem.set_xlabel('Time (s)')
    ax_sem.set_ylabel('Score')
    ax_sem.set_ylim(0.0, 1.0)

    fig.suptitle(
        f'AutoLanding Paper Figure | points={len(trajectory)} | path={path_length:.2f} m | xy={touchdown_error_xy:.3f} m | z={touchdown_error_z:.3f} m',
        fontsize=13,
        fontweight='bold',
    )

    png_path = output_dir / 'paper_trajectory_summary.png'
    pdf_path = output_dir / 'paper_trajectory_summary.pdf'
    fig.savefig(png_path, dpi=220, bbox_inches='tight')
    fig.savefig(pdf_path, bbox_inches='tight')
    plt.close(fig)

    return {'png': str(png_path), 'pdf': str(pdf_path)}


def generate_model_comparison_artifacts(
    models: dict[str, dict[str, Any]],
    output_dir: Path,
) -> dict[str, str]:
    if not models:
        raise ValueError('models is empty; cannot generate comparison artifacts')

    output_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, Any]] = []

    for model_name, payload in models.items():
        trajectory = payload.get('trajectory', [])
        target = payload.get('meta', {}).get('target', {'x': 0.0, 'y': 0.0, 'z': 0.0})

        if not trajectory:
            rows.append(
                {
                    'model': model_name,
                    'trajectory_points': 0,
                    'path_length_m': 0.0,
                    'touchdown_error_xy_m': 0.0,
                    'touchdown_error_z_m': 0.0,
                    'duration_s': 0.0,
                    'semantic_risk': 0.0,
                }
            )
            continue

        x = _series(trajectory, 'x')
        y = _series(trajectory, 'y')
        z = _series(trajectory, 'z')
        t = _series(trajectory, 't')
        path_length = _path_length(trajectory)
        touchdown_error_xy = math.sqrt((x[-1] - float(target.get('x', 0.0))) ** 2 + (y[-1] - float(target.get('y', 0.0))) ** 2)
        touchdown_error_z = abs(z[-1] - float(target.get('z', 0.0)))
        semantic_risk = float(payload.get('semantic_features', {}).get('semantic_risk', 0.0))

        rows.append(
            {
                'model': model_name,
                'trajectory_points': len(trajectory),
                'path_length_m': path_length,
                'touchdown_error_xy_m': touchdown_error_xy,
                'touchdown_error_z_m': touchdown_error_z,
                'duration_s': float(t[-1]) if t else 0.0,
                'semantic_risk': semantic_risk,
            }
        )

    csv_path = output_dir / 'paper_model_comparison_metrics.csv'
    with open(csv_path, 'w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    json_path = output_dir / 'paper_model_comparison_metrics.json'
    json_path.write_text(json.dumps(rows, indent=2), encoding='utf-8')

    labels = [str(row['model']) for row in rows]
    xy_error = [float(row['touchdown_error_xy_m']) for row in rows]
    path_length = [float(row['path_length_m']) for row in rows]
    duration = [float(row['duration_s']) for row in rows]

    plt.style.use('seaborn-v0_8-whitegrid')
    fig, axes = plt.subplots(1, 3, figsize=(14, 4.5), constrained_layout=True)

    axes[0].bar(labels, xy_error, color='#dc2626')
    axes[0].set_title('Touchdown XY Error')
    axes[0].set_ylabel('Error (m)')

    axes[1].bar(labels, path_length, color='#0f766e')
    axes[1].set_title('Path Length')
    axes[1].set_ylabel('Length (m)')

    axes[2].bar(labels, duration, color='#2563eb')
    axes[2].set_title('Flight Duration')
    axes[2].set_ylabel('Time (s)')

    fig.suptitle('AutoLanding Model Comparison (Paper Metrics)', fontsize=12, fontweight='bold')
    png_path = output_dir / 'paper_model_comparison.png'
    pdf_path = output_dir / 'paper_model_comparison.pdf'
    fig.savefig(png_path, dpi=220, bbox_inches='tight')
    fig.savefig(pdf_path, bbox_inches='tight')
    plt.close(fig)

    return {
        'csv': str(csv_path),
        'json': str(json_path),
        'png': str(png_path),
        'pdf': str(pdf_path),
    }


def generate_benchmark_comparison_artifacts(summary: dict[str, Any], output_dir: Path) -> dict[str, str]:
    baseline = summary.get('baseline', {}).get('val', {})
    proposed = summary.get('proposed_ontology_ai', {}).get('val', {})

    if not baseline or not proposed:
        raise ValueError('benchmark summary missing baseline/proposed validation metrics')

    rows = [
        {
            'model': 'pure_ai_baseline',
            'rmse': float(baseline.get('rmse', 0.0)),
            'mae': float(baseline.get('mae', 0.0)),
        },
        {
            'model': 'ontology_ai_proposed',
            'rmse': float(proposed.get('rmse', 0.0)),
            'mae': float(proposed.get('mae', 0.0)),
        },
    ]

    output_dir.mkdir(parents=True, exist_ok=True)

    csv_path = output_dir / 'paper_benchmark_model_comparison_metrics.csv'
    with open(csv_path, 'w', encoding='utf-8', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    json_path = output_dir / 'paper_benchmark_model_comparison_metrics.json'
    json_path.write_text(json.dumps(rows, indent=2), encoding='utf-8')

    labels = [str(row['model']) for row in rows]
    rmse = [float(row['rmse']) for row in rows]
    mae = [float(row['mae']) for row in rows]

    plt.style.use('seaborn-v0_8-whitegrid')
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.5), constrained_layout=True)

    axes[0].bar(labels, rmse, color=['#dc2626', '#0f766e'])
    axes[0].set_title('Validation RMSE')
    axes[0].set_ylabel('RMSE')

    axes[1].bar(labels, mae, color=['#dc2626', '#0f766e'])
    axes[1].set_title('Validation MAE')
    axes[1].set_ylabel('MAE')

    fig.suptitle('AutoLanding Benchmark: Ontology+AI vs Pure AI (8:2 Validation)', fontsize=12, fontweight='bold')
    png_path = output_dir / 'paper_benchmark_model_comparison.png'
    pdf_path = output_dir / 'paper_benchmark_model_comparison.pdf'
    fig.savefig(png_path, dpi=220, bbox_inches='tight')
    fig.savefig(pdf_path, bbox_inches='tight')
    plt.close(fig)

    return {
        'csv': str(csv_path),
        'json': str(json_path),
        'png': str(png_path),
        'pdf': str(pdf_path),
    }