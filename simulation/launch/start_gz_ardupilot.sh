#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
LOCK_DIR="${XDG_RUNTIME_DIR:-/tmp}/gazebo_ardupilot_locks"
LOCK_FILE="$LOCK_DIR/gazebo_launch.lock"

EXAMPLE="iris_runway"
DRY_RUN=0
USE_LOCAL_FALLBACK=0
KILL_EXISTING=0
NO_DUPLICATE_CHECK=0

# Helper functions for duplicate prevention
create_lock_dir() {
	mkdir -p "$LOCK_DIR"
	chmod 700 "$LOCK_DIR"
}

check_existing_gazebo() {
	# Check for running gazebo processes
	if pgrep -f "gz sim" >/dev/null 2>&1; then
		return 0  # Gazebo is running
	fi
	return 1  # Gazebo is not running
}

get_existing_gazebo_pids() {
	pgrep -f "gz sim" || echo ""
}

wait_for_gazebo_shutdown() {
	local max_wait=30
	local elapsed=0
	echo "[info] Waiting for existing Gazebo to shutdown..."
	while [[ $elapsed -lt $max_wait ]]; do
		if ! check_existing_gazebo; then
			echo "[info] Existing Gazebo process has terminated"
			sleep 2  # Give system time to clean up resources
			return 0
		fi
		sleep 1
		((elapsed++))
	done
	echo "[warn] Timeout waiting for Gazebo shutdown after ${max_wait}s"
	return 1
}

kill_existing_gazebo() {
	local pids
	local max_attempts=3
	local attempt=0
	
	pids=$(get_existing_gazebo_pids)
	if [[ -z "$pids" ]]; then
		echo "[info] No existing Gazebo processes to kill"
		return 0
	fi
	
	echo "[info] Killing existing Gazebo processes: $pids"
	
	# Multiple attempts to ensure clean shutdown
	while [[ $attempt -lt $max_attempts ]] && check_existing_gazebo; do
		# Attempt 1: graceful SIGTERM
		if [[ $attempt -eq 0 ]]; then
			echo "[info] Attempt $((attempt+1))/$max_attempts: Sending SIGTERM to Gazebo processes"
			pkill -TERM -f "gz sim" 2>/dev/null || true
			sleep 3
		# Attempt 2: stronger SIGKILL to main process
		elif [[ $attempt -eq 1 ]]; then
			echo "[info] Attempt $((attempt+1))/$max_attempts: Sending SIGKILL to main Gazebo processes"
			pkill -9 -f "gz sim" 2>/dev/null || true
			sleep 2
		# Attempt 3: kill all gz-related processes
		else
			echo "[info] Attempt $((attempt+1))/$max_attempts: Killing all gz-related processes"
			pkill -9 -f "gz" 2>/dev/null || true
			sleep 2
		fi
		
		((attempt++))
	done
	
	# Final check
	if check_existing_gazebo; then
		echo "[warn] Failed to kill all Gazebo processes after $max_attempts attempts"
		echo "[hint] Manual cleanup: pkill -9 -f 'gz sim'"
		return 1
	else
		echo "[info] All Gazebo processes terminated successfully"
		return 0
	fi
}

acquire_exclusive_lock() {
	create_lock_dir
	
	if [[ -f "$LOCK_FILE" ]]; then
		local lock_pid
		lock_pid=$(cat "$LOCK_FILE" 2>/dev/null || echo "")
		
		if [[ -n "$lock_pid" ]] && kill -0 "$lock_pid" 2>/dev/null; then
			return 1  # Lock is active
		else
			# Stale lock, remove it
			rm -f "$LOCK_FILE"
		fi
	fi
	
	echo $$ > "$LOCK_FILE"
	return 0
}

release_lock() {
	rm -f "$LOCK_FILE"
}

# Cleanup on exit
cleanup_on_exit() {
	release_lock
}
trap cleanup_on_exit EXIT

while [[ $# -gt 0 ]]; do
	case "$1" in
		--example)
			EXAMPLE="${2:-iris_runway}"
			shift 2
			;;
		--dry-run)
			DRY_RUN=1
			shift
			;;
		--local-fallback)
			USE_LOCAL_FALLBACK=1
			shift
			;;
		--kill-existing)
			KILL_EXISTING=1
			shift
			;;
		--no-duplicate-check)
			NO_DUPLICATE_CHECK=1
			shift
			;;
		-h|--help)
			cat <<'EOF'
Usage: start_gz_ardupilot.sh [options]

Options:
	--example <name>           ardupilot_gz_bringup example (default: iris_runway)
	                           examples: iris_runway, iris_maze, multiagent
	--dry-run                  Print commands only
	--local-fallback           If official ROS2 package is missing, run local launcher
	--kill-existing            Kill any existing Gazebo processes before launching
	--no-duplicate-check       Skip duplicate prevention checks (not recommended)
	-h, --help                 Show this help

Features:
	- Automatic duplicate prevention: prevents multiple Gazebo instances
	- Lock file mechanism: ensures only one launch at a time
	- Use --kill-existing to terminate running instances
	
This script follows ArduPilot ROS2+Gazebo docs:
	https://ardupilot.org/dev/docs/ros2-gazebo.html
EOF
			exit 0
			;;
		*)
			echo "[error] unknown argument: $1" >&2
			exit 2
			;;
	esac
done

if [[ -f /opt/ros/humble/setup.bash ]]; then
	# shellcheck disable=SC1091
	set +u
	source /opt/ros/humble/setup.bash
	set -u
else
	echo "[error] /opt/ros/humble/setup.bash not found" >&2
	exit 1
fi

# ArduPilot docs recommend Gazebo Harmonic for ROS2 Humble.
export GZ_VERSION="${GZ_VERSION:-harmonic}"

add_path_var() {
	local var_name="$1"
	local candidate="$2"
	if [[ -d "$candidate" ]]; then
		local current="${!var_name:-}"
		if [[ -z "$current" ]]; then
			printf -v "$var_name" '%s' "$candidate"
		elif [[ ":$current:" != *":$candidate:"* ]]; then
			printf -v "$var_name" '%s:%s' "$candidate" "$current"
		fi
		export "$var_name"
	fi
}

dedupe_path_var() {
	local var_name="$1"
	local current="${!var_name:-}"
	local out=""
	IFS=':' read -r -a parts <<< "$current"
	for p in "${parts[@]}"; do
		[[ -z "$p" ]] && continue
		[[ ! -d "$p" ]] && continue
		if [[ ":$out:" != *":$p:"* ]]; then
			if [[ -z "$out" ]]; then
				out="$p"
			else
				out="$out:$p"
			fi
		fi
	done
	printf -v "$var_name" '%s' "$out"
	export "$var_name"
}

for ws_setup in "$HOME/ardu_ws/install/setup.bash" "$HOME/gz_ws/install/setup.bash"; do
	if [[ -f "$ws_setup" ]]; then
		# shellcheck disable=SC1090
		set +u
		source "$ws_setup"
		set -u
	fi
done

# Align with ros2-gazebo guidance: ensure Gazebo can find ArduPilot plugins/resources.
add_path_var GZ_SIM_SYSTEM_PLUGIN_PATH "$HOME/ardu_ws/install/ardupilot_gazebo/lib"
add_path_var GZ_SIM_SYSTEM_PLUGIN_PATH "$HOME/gz_ws/src/ardupilot_gazebo/build"
add_path_var GZ_SIM_RESOURCE_PATH "$HOME/ardu_ws/src/ardupilot_gazebo"
add_path_var GZ_SIM_RESOURCE_PATH "$HOME/ardu_ws/src/ardupilot_gazebo/models"
add_path_var GZ_SIM_RESOURCE_PATH "$HOME/ardu_ws/src/ardupilot_gazebo/worlds"
add_path_var GZ_SIM_RESOURCE_PATH "$HOME/gz_ws/src/ardupilot_gazebo"
add_path_var GZ_SIM_RESOURCE_PATH "$HOME/gz_ws/src/ardupilot_gazebo/models"
add_path_var GZ_SIM_RESOURCE_PATH "$HOME/gz_ws/src/ardupilot_gazebo/worlds"

dedupe_path_var GZ_SIM_SYSTEM_PLUGIN_PATH
dedupe_path_var GZ_SIM_RESOURCE_PATH

# ============================================================================
# Duplicate Prevention & Environment Setup
# ============================================================================
if [[ $NO_DUPLICATE_CHECK -eq 0 ]]; then
	if check_existing_gazebo; then
		if [[ $KILL_EXISTING -eq 1 ]]; then
			echo "[info] --kill-existing specified, terminating existing Gazebo processes"
			kill_existing_gazebo
		else
			existing_pids=$(get_existing_gazebo_pids)
			echo "[error] Gazebo is already running. Choose one:"
			echo "  1. Wait for existing process: kill $existing_pids"
			echo "  2. Re-run with --kill-existing to terminate and restart"
			exit 1
		fi
	fi
	
	if ! acquire_exclusive_lock; then
		echo "[error] Another Gazebo launch is in progress (lock file: $LOCK_FILE)"
		echo "[hint] Use --kill-existing to force shutdown existing instances"
		exit 1
	fi
	echo "[info] Acquired exclusive launch lock: $LOCK_FILE"
fi

# ============================================================================
# Select and Execute Launcher (Only Once)
# ============================================================================
LAUNCH_FILE="${EXAMPLE}.launch.py"
OFFICIAL_CMD=(ros2 launch ardupilot_gz_bringup "$LAUNCH_FILE")
LOCAL_CMD=(bash "$ROOT_DIR/scripts/verify_gz_ardupilot_stack.sh")

echo "[info] GZ_VERSION=$GZ_VERSION"
echo "[info] GZ_SIM_SYSTEM_PLUGIN_PATH=${GZ_SIM_SYSTEM_PLUGIN_PATH:-<unset>}"
echo "[info] GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH:-<unset>}"

if [[ $DRY_RUN -eq 1 ]]; then
	echo "[info] DRY_RUN: commands that would be executed:"
	if ros2 pkg prefix ardupilot_gz_bringup >/dev/null 2>&1; then
		echo "  ${OFFICIAL_CMD[*]}"
	else
		echo "  ${LOCAL_CMD[*]}"
	fi
	exit 0
fi

# Try official package first
if ros2 pkg prefix ardupilot_gz_bringup >/dev/null 2>&1; then
	echo "[info] using official ArduPilot launch: ${OFFICIAL_CMD[*]}"
	exec "${OFFICIAL_CMD[@]}"
fi

# Fall back to local launcher
if [[ -f "$ROOT_DIR/scripts/verify_gz_ardupilot_stack.sh" ]]; then
	if [[ "${AUTOLANDING_SKIP_LOCAL_FALLBACK:-0}" == "1" ]]; then
		echo "[error] Local fallback disabled by AUTOLANDING_SKIP_LOCAL_FALLBACK=1, but official ROS2 launcher is unavailable." >&2
		exit 1
	fi
	echo "[info] ROS2 package 'ardupilot_gz_bringup' not found; using local Gazebo+SITL fallback launcher:"
	echo "[info] ${LOCAL_CMD[*]}"
	cd "$ROOT_DIR"
	exec "${LOCAL_CMD[@]}"
fi

echo "[error] Neither official package nor local launcher found!"
echo "[error] Expected: $ROOT_DIR/scripts/verify_gz_ardupilot_stack.sh"
exit 1
