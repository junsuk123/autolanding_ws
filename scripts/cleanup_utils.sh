#!/usr/bin/env bash
# cleanup_utils.sh
# Shared cleanup utilities for autolanding pipeline scripts
# Source this file in other scripts: source "$(dirname "$0")/cleanup_utils.sh"

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

cleanup_pid_is_excluded() {
	local pid="$1"
	local extra_pids="${AUTOLANDING_CLEANUP_EXTRA_EXCLUDE_PIDS:-}"
	local exclude_pid=""

	for exclude_pid in "$$" "$PPID" $extra_pids; do
		if [[ -n "$exclude_pid" && "$pid" == "$exclude_pid" ]]; then
			return 0
		fi
	done

	return 1
}

cleanup_filter_pid_list() {
	local pid_list="$1"
	local filtered=""
	local pid=""

	while read -r pid; do
		[[ -n "$pid" ]] || continue
		if ! cleanup_pid_is_excluded "$pid"; then
			filtered+="$pid"$'\n'
		fi
	done <<< "$pid_list"

	printf '%s' "$filtered"
}

# Kill all processes matching a pattern (graceful then forceful)
kill_pattern_graceful() {
	local pattern="$1"
	local label="${2:-processes matching: $pattern}"
	local pids=""
	local remain=""

	pids="$(cleanup_filter_pid_list "$(pgrep -f "$pattern" 2>/dev/null || true)")"
	if [[ -z "$pids" ]]; then
		return 0
	fi

	local count=$(echo "$pids" | wc -l)
	echo "[cleanup] Stopping $label ($count process(es))..."
	while read -r pid; do
		[[ -n "$pid" ]] && kill "$pid" 2>/dev/null || true
	done <<< "$pids"

	# Wait for graceful termination
	for _ in {1..10}; do
		sleep 0.2
		remain="$(cleanup_filter_pid_list "$(pgrep -f "$pattern" 2>/dev/null || true)")"
		if [[ -z "$remain" ]]; then
			return 0
		fi
	done

	# Force kill remaining
	echo "[cleanup]   (Force killing remaining $label...)"
	while read -r pid; do
		[[ -n "$pid" ]] && kill -9 "$pid" 2>/dev/null || true
	done <<< "$remain"
}

# Main cleanup function - kills all pipeline-related processes
cleanup_all_processes() {
	echo ""
	echo "[cleanup] ============================================"
	echo "[cleanup] Terminating all pipeline processes..."
	echo "[cleanup] ============================================"

	# Kill ROS 2 nodes and domains
	kill_pattern_graceful "ros2 run" "ROS 2 nodes"
	kill_pattern_graceful "ros2 launch" "ROS 2 launches"
	kill_pattern_graceful "ros2 node" "ROS 2 node commands"
	kill_pattern_graceful "ros2 service" "ROS 2 services"
	kill_pattern_graceful "ros2 daemon" "ROS 2 daemon"
	
	# Kill simulation processes
	kill_pattern_graceful "gz sim" "Gazebo simulation"
	kill_pattern_graceful "gzserver" "Gazebo server"
	kill_pattern_graceful "gzclient" "Gazebo client"
	kill_pattern_graceful "gazebo" "Gazebo (legacy)"
	
	# Kill SITL/ArduPilot processes
	kill_pattern_graceful "sitl_copter" "ArduPilot SITL"
	kill_pattern_graceful "sim_vehicle" "ArduPilot sim_vehicle"
	kill_pattern_graceful "arducopter" "ArduCopter SITL"
	kill_pattern_graceful "ardusub" "ArduSub SITL"
	kill_pattern_graceful "arduplane" "ArduPlane SITL"
	
	# Kill MAVROS and related
	kill_pattern_graceful "mavros" "MAVROS"
	kill_pattern_graceful "mavros_node" "MAVROS nodes"
	kill_pattern_graceful "mavproxy" "MAVProxy"
	
	# Kill related Python scripts and nodes
	kill_pattern_graceful "launch_gazebo" "Gazebo launch scripts"
	kill_pattern_graceful "start_multi_uav" "Multi-UAV scripts"
	kill_pattern_graceful "publish_multi_drone_odom" "Odometry publisher"
	kill_pattern_graceful "aruco_node" "ArUco detection nodes"
	kill_pattern_graceful "parameter_bridge" "Parameter bridge"
	kill_pattern_graceful "rviz2" "RViz2"
	kill_pattern_graceful "domain_bridge" "Domain bridge"
	
	# Kill additional pipeline components
	kill_pattern_graceful "launch_isolated_worker" "Isolated worker"
	kill_pattern_graceful "semantic_feature_extractor" "Feature extractor"
	kill_pattern_graceful "trajectory_policy" "Trajectory policy"
	kill_pattern_graceful "autolanding" "Autolanding processes"
	kill_pattern_graceful "orchestrator" "Orchestrator"
	
	# ROS 2 daemon final cleanup
	ros2 daemon stop 2>/dev/null || true
	
	echo "[cleanup] ============================================"
	echo "[cleanup] All pipeline processes terminated"
	echo "[cleanup] ============================================"
	echo ""
}

# Get all process info for debugging
list_all_pipeline_processes() {
	echo ""
	echo "[debug] ============================================"
	echo "[debug] Active pipeline-related processes:"
	echo "[debug] ============================================"
	
	local pids
	pids=$(pgrep -af "gz sim|gazebo|mavros|ardupilot|sitl_copter|ros2|launch_gazebo|start_multi_uav|aruco_node|parameter_bridge|rviz2|autolanding" 2>/dev/null || true)
	
	if [[ -z "$pids" ]]; then
		echo "[debug] No pipeline processes found"
	else
		echo "$pids" | while read -r line; do
			echo "[debug]   $line"
		done
	fi
	
	echo "[debug] ============================================"
	echo ""
}

# Kill all remaining descendant processes for a parent (deep clean)
kill_all_descendants() {
	local parent_pid="$1"
	local level="${2:-0}"
	local indent=$(printf '%*s' "$((level * 2))" '')
	
	local children=""
	children="$(pgrep -P "$parent_pid" 2>/dev/null || true)"
	
	if [[ -n "$children" ]]; then
		while read -r child; do
			if [[ -n "$child" ]]; then
				echo "[cleanup] ${indent}└─ Killing process $child"
				kill_all_descendants "$child" "$((level + 1))"
				kill "$child" 2>/dev/null || true
				sleep 0.1
				kill -9 "$child" 2>/dev/null || true
			fi
		done <<< "$children"
	fi
}

# Export functions so they can be used after sourcing
export -f kill_tree
export -f kill_pattern_graceful
export -f cleanup_all_processes
export -f list_all_pipeline_processes
export -f kill_all_descendants
