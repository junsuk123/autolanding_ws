#!/usr/bin/env bash
set -euo pipefail

# End-to-end scalable stack launcher:
# Gazebo -> N x SITL -> N x MAVROS -> rosbag2 record

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
COUNT="${1:-3}"

if [[ "$COUNT" -lt 1 || "$COUNT" -gt 10 ]]; then
  echo "[ERROR] COUNT must be 1..10" >&2
  exit 2
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export USE_SIM_TIME="true"

# Optional GUI forcing for users who need a visible Gazebo window.
export AUTOLANDING_FORCE_HEADLESS="${AUTOLANDING_FORCE_HEADLESS:-0}"

mkdir -p /tmp/autolanding_multi_uav

echo "[STACK] Launching Gazebo/SITL baseline..."
"$ROOT_DIR/simulation/launch/start_gz_ardupilot.sh" --kill-existing >/tmp/autolanding_multi_uav/gazebo_stack.log 2>&1 &
sleep 8

echo "[STACK] Launching ${COUNT} SITL instances (UDP 14540+)"
python3 "$ROOT_DIR/scripts/start_multi_uav_sitl.py" --count "$COUNT" >/tmp/autolanding_multi_uav/sitl_supervisor.log 2>&1

echo "[STACK] Launching ${COUNT} MAVROS bridges (/uavX/mavros)"
python3 "$ROOT_DIR/scripts/start_multi_uav_mavros.py" --count "$COUNT" >/tmp/autolanding_multi_uav/mavros_supervisor.log 2>&1

STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_DIR="$ROOT_DIR/data/logs/multi_uav_bag_${STAMP}"
mkdir -p "$BAG_DIR"

echo "[STACK] Starting rosbag2 recording for /uav*/mavros/* and /clock"
set +u
source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true
set -u
nohup bash -lc "set +u; source /opt/ros/humble/setup.bash >/dev/null 2>&1; set -u; ros2 bag record --use-sim-time --regex '/uav[0-9]+/mavros/.*|/clock' -o '$BAG_DIR'" \
  >/tmp/autolanding_multi_uav/rosbag.log 2>&1 < /dev/null &

echo "[STACK] Ready"
echo "[STACK] rosbag: $BAG_DIR"
echo "[STACK] Stop: pkill -f 'ros2 bag record|ros2 launch mavros apm.launch|arducopter.*JSON:|gz sim'"
