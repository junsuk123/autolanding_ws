#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CFG="$ROOT_DIR/ai/configs/orchestration_config.yaml"
KEEP_SIM_RUNNING=0
RUN_BENCHMARK=1
MIN_SAMPLES=50

while [[ $# -gt 0 ]]; do
	case "$1" in
		--keep-sim)
			KEEP_SIM_RUNNING=1
			shift
			;;
		--skip-benchmark)
			RUN_BENCHMARK=0
			shift
			;;
		--min-samples)
			MIN_SAMPLES="${2:-50}"
			shift 2
			;;
		-h|--help)
			echo "Usage: run_full_pipeline.sh [--keep-sim] [--skip-benchmark] [--min-samples N]"
			echo "  --keep-sim   Keep Gazebo/SITL running after successful pipeline completion"
			echo "  --skip-benchmark   Skip rollout-based model training/validation/plot stage"
			echo "  --min-samples N    Minimum rollout samples required for training (default: 50)"
			exit 0
			;;
		*)
			echo "[error] Unknown argument: $1" >&2
			exit 2
			;;
	esac
done

# Gazebo process management
GZ_PID=""

# Function to kill Gazebo on exit
cleanup_gazebo() {
	echo "[cleanup] Terminating Gazebo processes..."
	pkill -TERM -f "gz sim" 2>/dev/null || true
	pkill -TERM -f "parameter_bridge" 2>/dev/null || true
	pkill -TERM -f "aruco_node" 2>/dev/null || true
	pkill -TERM -f "publish_multi_drone_odom.py" 2>/dev/null || true
	pkill -TERM -f "rviz2" 2>/dev/null || true
	sleep 2
	
	# Force kill remaining processes if necessary
	if pgrep -f "gz sim" >/dev/null 2>&1; then
		echo "[cleanup] Force killing remaining Gazebo processes..."
		pkill -9 -f "gz sim" 2>/dev/null || true
	fi
	pkill -9 -f "parameter_bridge" 2>/dev/null || true
	pkill -9 -f "aruco_node" 2>/dev/null || true
	pkill -9 -f "publish_multi_drone_odom.py" 2>/dev/null || true
	pkill -9 -f "rviz2" 2>/dev/null || true
	
	echo "[cleanup] Gazebo cleanup complete"
}

on_interrupt() {
	cleanup_gazebo
	exit 130
}

cleanup_on_failure() {
	local exit_code="$1"
	cleanup_gazebo
	exit "$exit_code"
}

# Cleanup only on interrupt/signals or launcher failure.
trap on_interrupt INT TERM

echo "[pipeline] Starting full pipeline with Gazebo..."
echo "[pipeline] Root directory: $ROOT_DIR"
echo "[pipeline] Configuration: $CFG"

# 이 스크립트 하나로 전체 파이프라인(full 모드)을 실행합니다.
echo "[pipeline] Running autolanding pipeline..."
python3 "$ROOT_DIR/scripts/autolanding_launcher.py" full \
  --workspace-root "$ROOT_DIR" \
  --orchestration-config "$CFG"

status=$?
if [[ $status -ne 0 ]]; then
	cleanup_on_failure "$status"
fi

echo "[pipeline] Pipeline execution complete"

if [[ $RUN_BENCHMARK -eq 1 ]]; then
	echo "[pipeline] Running rollout-based benchmark training/validation (8:2 split)..."
	python3 "$ROOT_DIR/scripts/run_research_benchmark.py" \
	  --workspace-root "$ROOT_DIR" \
	  --use-rollouts \
	  --rollout-glob "data/collected/landing_controller_rollout_*.csv" \
	  --min-samples "$MIN_SAMPLES" \
	  --fallback-to-synthetic-on-rollout-failure \
	  --train-ratio 0.8 \
	  --val-ratio 0.2

	bench_status=$?
	if [[ $bench_status -ne 0 ]]; then
		echo "[error] benchmark stage failed with exit code: $bench_status"
		cleanup_on_failure "$bench_status"
	fi
	echo "[pipeline] Benchmark stage complete (models + validation + paper plots)."
fi

if [[ $KEEP_SIM_RUNNING -eq 1 ]]; then
	echo "[pipeline] Gazebo/SITL are left running by request (--keep-sim)."
	echo "[pipeline] Press Ctrl+C when you want to stop and clean up the simulation."
	while true; do
		sleep 60
	done
fi

echo "[pipeline] Cleaning up Gazebo/SITL and exiting."
cleanup_gazebo
echo "[pipeline] Done."
