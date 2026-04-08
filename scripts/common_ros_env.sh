#!/usr/bin/env bash

# Shared ROS2/Gazebo environment loader for AutoLanding scripts.
# Keep this file source-only; do not execute directly.

autl_source_if_exists() {
  local setup_path="$1"
  if [[ -f "$setup_path" ]]; then
    # shellcheck disable=SC1090
    set +u
    source "$setup_path"
    set -u
    echo "[INFO] sourced: $setup_path"
    return 0
  fi
  return 1
}

autl_source_ros_stacks() {
  local root_dir="$1"
  local setup_candidates=()
  local p

  setup_candidates+=("/opt/ros/${ROS_DISTRO:-humble}/setup.bash")
  setup_candidates+=("$root_dir/ardu_ws/install/setup.bash")
  setup_candidates+=("${AUTOLANDING_ROS_WS:-}/install/setup.bash")
  setup_candidates+=("${ARDUPILOT_ROS_WS:-}/install/setup.bash")
  setup_candidates+=("$HOME/ardu_ws/install/setup.bash")
  setup_candidates+=("$HOME/gz_ros2_aruco_ws/install/setup.bash")
  setup_candidates+=("$root_dir/../IICC26_ws/install/setup.bash")

  for p in "${setup_candidates[@]}"; do
    if [[ -n "$p" ]]; then
      autl_source_if_exists "$p" || true
    fi
  done
}

autl_export_comm_defaults() {
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  export PYTHONNOUSERSITE=1
  export GZ_VERSION="${GZ_VERSION:-harmonic}"
}