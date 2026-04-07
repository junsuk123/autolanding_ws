#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import glob
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.research_benchmark import collect_synthetic_dataset, collect_rollout_dataset, run_train_validate
from src.paper_plots import generate_benchmark_comparison_artifacts


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run ontology+AI vs AI-only training/validation benchmark with data collection"
    )
    parser.add_argument("--workspace-root", default=".")
    parser.add_argument("--samples", type=int, default=4000)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--config", default="ai/configs/policy_config.yaml")
    parser.add_argument("--dataset-csv", default="data/processed/research_dataset.csv")
    parser.add_argument("--dataset-meta", default="data/processed/research_dataset_meta.json")
    parser.add_argument("--summary-json", default="data/processed/research_train_validate_summary.json")
    parser.add_argument("--models-json", default="data/models/research_linear_models.json")
    parser.add_argument("--plots-dir", default="data/plots/paper")
    parser.add_argument("--use-rollouts", action="store_true")
    parser.add_argument("--rollout-glob", default="data/collected/landing_controller_rollout_*.csv")
    parser.add_argument("--min-samples", type=int, default=50)
    parser.add_argument("--fallback-to-synthetic-on-rollout-failure", action="store_true", default=True)
    parser.add_argument("--no-fallback-to-synthetic-on-rollout-failure", action="store_false", dest="fallback_to_synthetic_on_rollout_failure")
    parser.add_argument("--train-ratio", type=float, default=0.8)
    parser.add_argument("--val-ratio", type=float, default=0.2)

    args = parser.parse_args()
    root = Path(args.workspace_root).resolve()

    cfg = (root / args.config).resolve()
    dataset_csv = (root / args.dataset_csv).resolve()
    dataset_meta = (root / args.dataset_meta).resolve()
    summary_json = (root / args.summary_json).resolve()
    models_json = (root / args.models_json).resolve()
    plots_dir = (root / args.plots_dir).resolve()

    collection_mode = "synthetic"
    fallback_reason = None

    if bool(args.use_rollouts):
        pattern = str((root / args.rollout_glob).resolve())
        rollout_files = [Path(p) for p in sorted(glob.glob(pattern))]
        if rollout_files:
            try:
                collect_info = collect_rollout_dataset(
                    rollout_csv_files=rollout_files,
                    out_csv=dataset_csv,
                    out_json=dataset_meta,
                    min_samples=max(10, int(args.min_samples)),
                )
                collection_mode = "rollout"
            except Exception as exc:
                if not bool(args.fallback_to_synthetic_on_rollout_failure):
                    raise
                fallback_reason = f"rollout collection failed: {exc}"
                collect_info = collect_synthetic_dataset(
                    config_file=cfg,
                    out_csv=dataset_csv,
                    out_json=dataset_meta,
                    num_samples=int(args.samples),
                    seed=int(args.seed),
                )
        else:
            if not bool(args.fallback_to_synthetic_on_rollout_failure):
                raise SystemExit(f"No rollout CSV files found by pattern: {pattern}")
            fallback_reason = f"no rollout CSV files found by pattern: {pattern}"
            collect_info = collect_synthetic_dataset(
                config_file=cfg,
                out_csv=dataset_csv,
                out_json=dataset_meta,
                num_samples=int(args.samples),
                seed=int(args.seed),
            )
    else:
        collect_info = collect_synthetic_dataset(
            config_file=cfg,
            out_csv=dataset_csv,
            out_json=dataset_meta,
            num_samples=int(args.samples),
            seed=int(args.seed),
        )

    summary = run_train_validate(
        dataset_csv=dataset_csv,
        out_json=summary_json,
        out_models_json=models_json,
        seed=int(args.seed),
        train_ratio=float(args.train_ratio),
        val_ratio=float(args.val_ratio),
    )
    benchmark_artifacts = generate_benchmark_comparison_artifacts(summary, plots_dir)

    result = {
        "collection_mode": collection_mode,
        "fallback_reason": fallback_reason,
        "collection": collect_info,
        "summary_json": str(summary_json),
        "models_json": str(models_json),
        "benchmark_artifacts": benchmark_artifacts,
        "claim_check": summary.get("claim_check", {}),
        "improvement_percent": summary.get("improvement_percent", {}),
    }
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
