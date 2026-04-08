#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CFG="$ROOT_DIR/ai/configs/orchestration_config.yaml"
KEEP_SIM_RUNNING=0

# Prefer workspace Python tools (MAVProxy/pymavlink) when available.
if [[ -d "$ROOT_DIR/.venv/bin" ]]; then
	export PATH="$ROOT_DIR/.venv/bin:$PATH"
fi

while [[ $# -gt 0 ]]; do
	case "$1" in
		--keep-sim)
			KEEP_SIM_RUNNING=1
			shift
			;;
		-h|--help)
			echo "Usage: run_full_pipeline.sh [--keep-sim]"
			echo "  --keep-sim   Keep Gazebo/SITL running after successful pipeline completion"
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

SIM_LAUNCHER="$ROOT_DIR/simulation/launch/start_gz_ardupilot.sh"
if [[ ! -x "$SIM_LAUNCHER" ]]; then
	echo "[error] Missing simulation launcher: $SIM_LAUNCHER" >&2
	exit 1
fi

if ! command -v matlab >/dev/null 2>&1; then
	echo "[error] matlab command not found" >&2
	exit 1
fi

echo "[pipeline] Launching Gazebo/SITL stack..."
bash "$SIM_LAUNCHER" --kill-existing > "$ROOT_DIR/data/logs/full_pipeline_gazebo.log" 2>&1 &
SIM_PID=$!

ready=0
for _ in $(seq 1 30); do
	if pgrep -f "gz sim" >/dev/null 2>&1; then
		ready=1
		break
	fi
	sleep 2
done

if [[ $ready -ne 1 ]]; then
	echo "[warning] Gazebo did not report readiness before MATLAB launch. Continuing anyway."
fi

wait_for_mavlink_heartbeat() {
	local mavproxy_bin="mavproxy.py"
	if [[ -x "$ROOT_DIR/.venv/bin/mavproxy.py" ]]; then
		mavproxy_bin="$ROOT_DIR/.venv/bin/mavproxy.py"
	elif ! command -v mavproxy.py >/dev/null 2>&1; then
		echo "[warning] mavproxy.py not found; skipping heartbeat readiness check."
		return 0
	fi

	echo "[pipeline] Waiting for MAVLink heartbeat on tcp:127.0.0.1:5760..."
	local attempts=20
	for _ in $(seq 1 "$attempts"); do
		if timeout 10 "$mavproxy_bin" --master tcp:127.0.0.1:5760 --cmd="status" >/tmp/autolanding_mavproxy_ready.log 2>&1; then
			if grep -q "Detected vehicle" /tmp/autolanding_mavproxy_ready.log; then
				echo "[pipeline] MAVLink heartbeat detected."
				return 0
			fi
		fi
		sleep 2
	done

	echo "[error] MAVLink heartbeat was not detected in time."
	echo "[hint] Check logs: $ROOT_DIR/data/logs/full_pipeline_gazebo.log and /tmp/autolanding_mavproxy_ready.log"
	return 1
}

if ! wait_for_mavlink_heartbeat; then
	cleanup_on_failure 1
fi

echo "[pipeline] Running autolanding pipeline in MATLAB..."
matlab -batch "run('matlab/scripts/run_autolanding_pipeline.m')"

status=$?
if [[ $status -ne 0 ]]; then
	cleanup_on_failure "$status"
fi

echo "[pipeline] Pipeline execution complete"

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
