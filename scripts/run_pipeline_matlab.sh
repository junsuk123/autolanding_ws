#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT_DIR"

# Kill process tree recursively
kill_tree() {
	local parent="$1"
	local children=""
	children="$(pgrep -P "$parent" 2>/dev/null || true)"

	if [[ -n "$children" ]]; then
		while read -r child; do
			[[ -n "$child" ]] && kill_tree "$child"
		done <<< "$children"
	fi

	if kill -0 "$parent" 2>/dev/null; then
		kill "$parent" 2>/dev/null || true
		sleep 0.2
		kill -9 "$parent" 2>/dev/null || true
	fi
}

# Kill all processes matching a pattern (graceful then forceful)
kill_pattern_graceful() {
	local pattern="$1"
	local label="$2"
	local pids=""
	local remain=""

	pids="$(pgrep -f "$pattern" 2>/dev/null || true)"
	if [[ -z "$pids" ]]; then
		return 0
	fi

	echo "[cleanup] Stopping $label..."
	while read -r pid; do
		[[ -n "$pid" ]] && kill "$pid" 2>/dev/null || true
	done <<< "$pids"

	# Wait for graceful termination
	for _ in {1..10}; do
		sleep 0.2
		remain="$(pgrep -f "$pattern" 2>/dev/null || true)"
		if [[ -z "$remain" ]]; then
			return 0
		fi
	done

	# Force kill remaining
	while read -r pid; do
		[[ -n "$pid" ]] && kill -9 "$pid" 2>/dev/null || true
	done <<< "$remain"
}

# Function to cleanup all related processes
cleanup_all_processes() {
	echo "[cleanup] Terminating all pipeline processes..."

	# Kill ROS 2 nodes and domains
	kill_pattern_graceful "ros2 run" "ROS 2 nodes"
	kill_pattern_graceful "ros2 launch" "ROS 2 launches"
	kill_pattern_graceful "ros2 node" "ROS 2 node commands"
	
	# Kill simulation processes
	kill_pattern_graceful "gz sim" "Gazebo simulation"
	kill_pattern_graceful "gzserver" "Gazebo server"
	kill_pattern_graceful "gzclient" "Gazebo client"
	
	# Kill SITL/ArduPilot processes
	kill_pattern_graceful "sitl_copter" "ArduPilot SITL"
	kill_pattern_graceful "sim_vehicle" "ArduPilot sim_vehicle"
	
	# Kill MAVROS
	kill_pattern_graceful "mavros" "MAVROS"
	kill_pattern_graceful "mavros_node" "MAVROS nodes"
	
	# Kill related Python scripts and nodes
	kill_pattern_graceful "launch_gazebo" "Gazebo launch scripts"
	kill_pattern_graceful "start_multi_uav" "Multi-UAV scripts"
	kill_pattern_graceful "publish_multi_drone_odom" "Odometry publisher"
	kill_pattern_graceful "aruco_node" "ArUco detection nodes"
	kill_pattern_graceful "parameter_bridge" "Parameter bridge"
	kill_pattern_graceful "rviz2" "RViz2"
	
	# Additional cleanup
	ros2 daemon stop 2>/dev/null || true
	
	echo "[cleanup] All pipeline processes terminated"
}

# Signal handlers
on_interrupt() {
	echo "[interrupt] Received SIGINT, cleaning up..."
	cleanup_all_processes
	exit 130
}

cleanup_on_failure() {
	local exit_code="$1"
	cleanup_all_processes
	exit "$exit_code"
}

cleanup_on_exit() {
	local exit_code="$?"
	if [[ $exit_code -ne 130 ]]; then
		cleanup_all_processes
	fi
	exit "$exit_code"
}

# Setup traps
trap on_interrupt INT TERM
trap cleanup_on_exit EXIT

if command -v matlab >/dev/null 2>&1; then
  matlab -batch "run('matlab/scripts/run_autolanding_pipeline.m')"
else
  echo "[error] matlab command not found."
  echo "Run inside MATLAB: run('matlab/scripts/run_autolanding_pipeline.m')"
  exit 1
fi
