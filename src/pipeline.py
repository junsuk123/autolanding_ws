import argparse
import csv
import json
import os
from typing import Any, Dict

import yaml

try:
    from .semantic_feature_extractor import extract_semantic_features
    from .trajectory_policy import generate_trajectory
except ImportError:
    from semantic_feature_extractor import extract_semantic_features
    from trajectory_policy import generate_trajectory


def _load_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _write_json(path: str, payload: Dict[str, Any]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)


def _write_csv(path: str, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if not rows:
        return
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def run_pipeline(workspace_root: str, semantic_input_file: str, config_file: str) -> Dict[str, Any]:
    semantic_input = _load_json(semantic_input_file)
    context_rel = semantic_input["context_file"]
    context_file = os.path.join(workspace_root, context_rel)

    context = _load_json(context_file)
    with open(config_file, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    semantic_features = extract_semantic_features(context)

    trajectory = generate_trajectory(
        initial_state=semantic_input["initial_state"],
        target=semantic_input["target"],
        semantic_features=semantic_features,
        config=cfg,
    )

    result = {
        "semantic_features": semantic_features,
        "trajectory": trajectory,
        "meta": {
            "semantic_input_file": semantic_input_file,
            "context_file": context_file,
            "config_file": config_file,
        },
    }

    out_json = os.path.join(workspace_root, "data/processed/landing_trajectory.json")
    out_csv = os.path.join(workspace_root, "data/processed/landing_trajectory.csv")
    _write_json(out_json, result)
    _write_csv(out_csv, trajectory)

    return {
        "out_json": out_json,
        "out_csv": out_csv,
        "trajectory_points": len(trajectory),
        "semantic_risk": semantic_features.get("semantic_risk", 0.0),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Ontology+AI landing trajectory pipeline")
    parser.add_argument("--workspace-root", required=True)
    parser.add_argument("--semantic-input", required=True)
    parser.add_argument("--config", required=True)

    args = parser.parse_args()

    summary = run_pipeline(
        workspace_root=args.workspace_root,
        semantic_input_file=args.semantic_input,
        config_file=args.config,
    )

    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
