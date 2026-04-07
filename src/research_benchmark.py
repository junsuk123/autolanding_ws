from __future__ import annotations

import csv
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .semantic_feature_extractor import extract_semantic_features


@dataclass
class LinearModel:
    feature_names: list[str]
    weights: np.ndarray  # shape: (n_features + 1, n_targets), includes bias as last row


def _clip(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _oracle_control(
    state: dict[str, float],
    target: dict[str, float],
    semantic_features: dict[str, float],
    cfg: dict[str, Any],
) -> dict[str, float]:
    policy_cfg = cfg["policy"]
    fusion_cfg = cfg["fusion"]

    max_xy_speed = float(policy_cfg["max_xy_speed"])
    max_z_descent = float(policy_cfg["max_z_descent_speed"])
    min_z_descent = float(policy_cfg["min_z_descent_speed"])

    semantic_weight = float(fusion_cfg["semantic_weight"])
    ai_weight = float(fusion_cfg["ai_weight"])

    x = float(state["x"])
    y = float(state["y"])
    z = float(state["z"])

    tx = float(target["x"])
    ty = float(target["y"])
    tz = float(target["z"])

    semantic_safety = float(semantic_features.get("semantic_safety", 0.5))
    semantic_risk = float(semantic_features.get("semantic_risk", 0.5))
    ai_confidence = _clip(1.0 - 0.8 * semantic_risk, 0.0, 1.0)

    fused_confidence = _clip(
        semantic_weight * semantic_safety + ai_weight * ai_confidence,
        0.0,
        1.0,
    )

    dx = tx - x
    dy = ty - y
    dz = z - tz

    vx_cmd = _clip(0.8 * dx, -max_xy_speed, max_xy_speed)
    vy_cmd = _clip(0.8 * dy, -max_xy_speed, max_xy_speed)

    base_descent = min_z_descent + (max_z_descent - min_z_descent) * fused_confidence
    vz_cmd = -_clip(base_descent, min_z_descent, max_z_descent)
    if dz < 0.2:
        vz_cmd = -min_z_descent * 0.4

    return {
        "vx_cmd": float(vx_cmd),
        "vy_cmd": float(vy_cmd),
        "vz_cmd": float(vz_cmd),
        "fused_confidence": float(fused_confidence),
        "semantic_risk": float(semantic_risk),
    }


def _build_random_context(rng: np.random.Generator, z: float) -> dict[str, Any]:
    wind_speed = float(rng.uniform(0.0, 10.0))
    gust_risk = float(rng.uniform(0.0, 1.0))
    tag_confidence = float(rng.uniform(0.35, 0.98))
    tag_error = float(rng.uniform(0.0, 0.35))
    yaw_error = float(rng.uniform(-0.35, 0.35))

    return {
        "objects": [
            {
                "id": "drone_1",
                "type": "Drone",
                "attributes": {
                    "z": z,
                    "vx": float(rng.uniform(-0.6, 0.6)),
                    "vy": float(rng.uniform(-0.6, 0.6)),
                    "yaw_error": yaw_error,
                },
            },
            {"id": "pad_1", "type": "LandingPad", "attributes": {"x": 0.0, "y": 0.0, "z": 0.0}},
            {"id": "wind_1", "type": "WindField", "attributes": {"wind_speed": wind_speed, "gust_risk": gust_risk}},
            {
                "id": "marker_1",
                "type": "VisualMarker",
                "attributes": {"tag_confidence": tag_confidence, "tag_error": tag_error},
            },
        ],
        "relations": [
            {"subject": "drone_1", "predicate": "near", "object": "pad_1"},
            {"subject": "drone_1", "predicate": "visibleTo", "object": "marker_1"},
            {"subject": "drone_1", "predicate": "affectedBy", "object": "wind_1"},
            {"subject": "drone_1", "predicate": "alignedWith", "object": "pad_1"},
        ],
    }


def collect_synthetic_dataset(
    config_file: Path,
    out_csv: Path,
    out_json: Path,
    num_samples: int = 3000,
    seed: int = 42,
) -> dict[str, Any]:
    rng = np.random.default_rng(seed)
    cfg = yaml.safe_load(config_file.read_text(encoding="utf-8"))

    rows: list[dict[str, float]] = []

    for _ in range(int(num_samples)):
        state = {
            "x": float(rng.uniform(-1.2, 1.2)),
            "y": float(rng.uniform(-1.2, 1.2)),
            "z": float(rng.uniform(0.6, 5.0)),
            "vx": float(rng.uniform(-0.8, 0.8)),
            "vy": float(rng.uniform(-0.8, 0.8)),
            "vz": float(rng.uniform(-0.8, 0.3)),
        }
        target = {"x": 0.0, "y": 0.0, "z": 0.0}

        context = _build_random_context(rng, state["z"])
        sem = extract_semantic_features(context)
        ctrl = _oracle_control(state, target, sem, cfg)

        dx = target["x"] - state["x"]
        dy = target["y"] - state["y"]
        dz = state["z"] - target["z"]

        rows.append(
            {
                "x": state["x"],
                "y": state["y"],
                "z": state["z"],
                "vx": state["vx"],
                "vy": state["vy"],
                "vz": state["vz"],
                "dx": dx,
                "dy": dy,
                "dz": dz,
                "wind_risk": float(sem.get("wind_risk", 0.0)),
                "vision_risk": float(sem.get("vision_risk", 0.0)),
                "alignment_risk": float(sem.get("alignment_risk", 0.0)),
                "semantic_risk": float(sem.get("semantic_risk", 0.0)),
                "semantic_safety": float(sem.get("semantic_safety", 0.0)),
                "vx_cmd": ctrl["vx_cmd"],
                "vy_cmd": ctrl["vy_cmd"],
                "vz_cmd": ctrl["vz_cmd"],
            }
        )

    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with open(out_csv, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    meta = {
        "num_samples": len(rows),
        "seed": int(seed),
        "config_file": str(config_file),
        "out_csv": str(out_csv),
    }
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(meta, indent=2), encoding="utf-8")
    return meta


def collect_rollout_dataset(
    rollout_csv_files: list[Path],
    out_csv: Path,
    out_json: Path,
    min_samples: int = 50,
) -> dict[str, Any]:
    rows: list[dict[str, float]] = []

    for csv_path in rollout_csv_files:
        if not csv_path.exists():
            continue
        with open(csv_path, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            prev: dict[str, float] | None = None
            for row in reader:
                try:
                    t_epoch = float(row.get("t_epoch", 0.0))
                    x = float(row.get("x", 0.0))
                    y = float(row.get("y", 0.0))
                    z = float(row.get("z", 0.0))
                    tx = float(row.get("target_x", 0.0))
                    ty = float(row.get("target_y", 0.0))
                    tz = float(row.get("target_z", 0.0))
                    vx_cmd = float(row.get("vx_cmd", 0.0))
                    vy_cmd = float(row.get("vy_cmd", 0.0))
                    vz_cmd_down = float(row.get("vz_cmd_down", 0.0))
                except Exception:
                    continue

                vx = 0.0
                vy = 0.0
                vz = 0.0
                if prev is not None:
                    dt = max(1e-3, t_epoch - float(prev.get("t_epoch", t_epoch - 0.1)))
                    vx = (x - float(prev.get("x", x))) / dt
                    vy = (y - float(prev.get("y", y))) / dt
                    vz = (z - float(prev.get("z", z))) / dt

                rows.append(
                    {
                        "x": x,
                        "y": y,
                        "z": z,
                        "vx": float(vx),
                        "vy": float(vy),
                        "vz": float(vz),
                        "dx": tx - x,
                        "dy": ty - y,
                        "dz": z - tz,
                        "wind_risk": 0.0,
                        "vision_risk": 0.0,
                        "alignment_risk": 0.0,
                        "semantic_risk": 0.0,
                        "semantic_safety": 0.5,
                        "vx_cmd": vx_cmd,
                        "vy_cmd": vy_cmd,
                        # training label convention expects downward as negative vz_cmd
                        "vz_cmd": -abs(vz_cmd_down),
                    }
                )

                prev = {
                    "t_epoch": t_epoch,
                    "x": x,
                    "y": y,
                    "z": z,
                }

    if len(rows) < int(min_samples):
        raise ValueError(
            f"rollout dataset too small: {len(rows)} samples (minimum required: {int(min_samples)})"
        )

    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with open(out_csv, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    meta = {
        "source": "rollout_csv",
        "num_samples": len(rows),
        "num_files": len([p for p in rollout_csv_files if p.exists()]),
        "min_samples": int(min_samples),
        "out_csv": str(out_csv),
        "files": [str(p) for p in rollout_csv_files if p.exists()],
    }
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(meta, indent=2), encoding="utf-8")
    return meta


def _prepare_xy(rows: list[dict[str, float]], feature_names: list[str], target_names: list[str]) -> tuple[np.ndarray, np.ndarray]:
    x = np.array([[float(r[name]) for name in feature_names] for r in rows], dtype=np.float64)
    y = np.array([[float(r[name]) for name in target_names] for r in rows], dtype=np.float64)
    return x, y


def _train_ridge(x: np.ndarray, y: np.ndarray, l2: float = 1e-3) -> np.ndarray:
    # Add bias column
    xb = np.hstack([x, np.ones((x.shape[0], 1), dtype=np.float64)])
    xtx = xb.T @ xb
    reg = l2 * np.eye(xtx.shape[0], dtype=np.float64)
    reg[-1, -1] = 0.0  # no regularization on bias
    w = np.linalg.solve(xtx + reg, xb.T @ y)
    return w


def _predict(x: np.ndarray, w: np.ndarray) -> np.ndarray:
    xb = np.hstack([x, np.ones((x.shape[0], 1), dtype=np.float64)])
    return xb @ w


def _metrics(y_true: np.ndarray, y_pred: np.ndarray) -> dict[str, float]:
    err = y_pred - y_true
    mae = float(np.mean(np.abs(err)))
    rmse = float(np.sqrt(np.mean(err ** 2)))
    return {
        "mae": mae,
        "rmse": rmse,
        "mae_vx": float(np.mean(np.abs(err[:, 0]))),
        "mae_vy": float(np.mean(np.abs(err[:, 1]))),
        "mae_vz": float(np.mean(np.abs(err[:, 2]))),
        "rmse_vx": float(np.sqrt(np.mean(err[:, 0] ** 2))),
        "rmse_vy": float(np.sqrt(np.mean(err[:, 1] ** 2))),
        "rmse_vz": float(np.sqrt(np.mean(err[:, 2] ** 2))),
    }


def _split_rows(
    rows: list[dict[str, float]],
    seed: int,
    train_ratio: float = 0.8,
    val_ratio: float = 0.2,
) -> tuple[list[dict[str, float]], list[dict[str, float]]]:
    rng = np.random.default_rng(seed)
    idx = np.arange(len(rows))
    rng.shuffle(idx)

    n = len(rows)
    n_train = int(float(train_ratio) * n)
    n_val = int(float(val_ratio) * n)

    if n_train <= 0 or n_val <= 0:
        raise ValueError("invalid split; train and val sizes must be > 0")

    if n_train + n_val > n:
        n_val = n - n_train
        if n_val <= 0:
            raise ValueError("invalid split; val size became <= 0")

    train_idx = idx[:n_train]
    val_idx = idx[n_train : n_train + n_val]
    train_rows = [rows[int(i)] for i in train_idx]
    val_rows = [rows[int(i)] for i in val_idx]
    return train_rows, val_rows


def run_train_validate(
    dataset_csv: Path,
    out_json: Path,
    out_models_json: Path,
    seed: int = 42,
    train_ratio: float = 0.8,
    val_ratio: float = 0.2,
) -> dict[str, Any]:
    with open(dataset_csv, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = [{k: float(v) for k, v in row.items()} for row in reader]

    if len(rows) < 50:
        raise ValueError("dataset too small; collect at least 50 samples")

    train_rows, val_rows = _split_rows(rows, seed=seed, train_ratio=train_ratio, val_ratio=val_ratio)

    baseline_features = ["x", "y", "z", "vx", "vy", "vz", "dx", "dy", "dz"]
    proposed_features = baseline_features + [
        "wind_risk",
        "vision_risk",
        "alignment_risk",
        "semantic_risk",
        "semantic_safety",
    ]
    targets = ["vx_cmd", "vy_cmd", "vz_cmd"]

    x_train_b, y_train = _prepare_xy(train_rows, baseline_features, targets)
    x_val_b, y_val = _prepare_xy(val_rows, baseline_features, targets)

    x_train_p, _ = _prepare_xy(train_rows, proposed_features, targets)
    x_val_p, _ = _prepare_xy(val_rows, proposed_features, targets)

    w_b = _train_ridge(x_train_b, y_train)
    w_p = _train_ridge(x_train_p, y_train)

    val_pred_b = _predict(x_val_b, w_b)
    val_pred_p = _predict(x_val_p, w_p)

    val_b = _metrics(y_val, val_pred_b)
    val_p = _metrics(y_val, val_pred_p)
    improvement_rmse = 100.0 * (val_b["rmse"] - val_p["rmse"]) / max(val_b["rmse"], 1e-9)
    improvement_mae = 100.0 * (val_b["mae"] - val_p["mae"]) / max(val_b["mae"], 1e-9)

    summary = {
        "dataset_csv": str(dataset_csv),
        "num_samples": len(rows),
        "split": {
            "train": len(train_rows),
            "val": len(val_rows),
            "train_ratio": float(train_ratio),
            "val_ratio": float(val_ratio),
        },
        "baseline": {"val": val_b},
        "proposed_ontology_ai": {"val": val_p},
        "improvement_percent": {
            "rmse": float(improvement_rmse),
            "mae": float(improvement_mae),
        },
        "claim_check": {
            "similar_or_better": bool(val_p["rmse"] <= val_b["rmse"] * 1.02),
            "better": bool(val_p["rmse"] < val_b["rmse"]),
        },
    }

    model_bundle = {
        "baseline": {
            "feature_names": baseline_features,
            "target_names": targets,
            "weights": w_b.tolist(),
        },
        "proposed_ontology_ai": {
            "feature_names": proposed_features,
            "target_names": targets,
            "weights": w_p.tolist(),
        },
    }

    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    out_models_json.parent.mkdir(parents=True, exist_ok=True)
    out_models_json.write_text(json.dumps(model_bundle, indent=2), encoding="utf-8")

    return summary
