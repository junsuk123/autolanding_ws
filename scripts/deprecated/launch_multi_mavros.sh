#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKER_COUNT=1
NS_PREFIX="/mavros_w"
BASE_SERIAL1_PORT=5762
CONNECT_TIMEOUT=90
MAX_RESTARTS=2

if [[ -f "$ROOT_DIR/scripts/common_ros_env.sh" ]]; then
  # shellcheck disable=SC1090
  source "$ROOT_DIR/scripts/common_ros_env.sh"
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --workers)
      WORKER_COUNT="${2:-1}"
      shift 2
      ;;
    --namespace-prefix)
      NS_PREFIX="${2:-/mavros_w}"
      shift 2
      ;;
    --base-serial1-port)
      BASE_SERIAL1_PORT="${2:-5762}"
      shift 2
      ;;
    --connect-timeout)
      CONNECT_TIMEOUT="${2:-45}"
      shift 2
      ;;
    --max-restarts)
      MAX_RESTARTS="${2:-1}"
      shift 2
      ;;
    *)
      echo "[WARN] Unknown argument: $1"
      shift
      ;;
  esac
done

if ! [[ "$WORKER_COUNT" =~ ^[0-9]+$ ]] || [[ "$WORKER_COUNT" -lt 1 ]]; then
  echo "[ERROR] invalid --workers value: $WORKER_COUNT"
  exit 1
fi

if ! [[ "$BASE_SERIAL1_PORT" =~ ^[0-9]+$ ]] || [[ "$BASE_SERIAL1_PORT" -lt 1 ]]; then
  echo "[ERROR] invalid --base-serial1-port value: $BASE_SERIAL1_PORT"
  exit 1
fi

if ! [[ "$CONNECT_TIMEOUT" =~ ^[0-9]+$ ]] || [[ "$CONNECT_TIMEOUT" -lt 1 ]]; then
  echo "[ERROR] invalid --connect-timeout value: $CONNECT_TIMEOUT"
  exit 1
fi

if ! [[ "$MAX_RESTARTS" =~ ^[0-9]+$ ]] || [[ "$MAX_RESTARTS" -lt 0 ]]; then
  echo "[ERROR] invalid --max-restarts value: $MAX_RESTARTS"
  exit 1
fi

if declare -F autl_source_ros_stacks >/dev/null 2>&1; then
  autl_source_ros_stacks "$ROOT_DIR"
fi
if declare -F autl_export_comm_defaults >/dev/null 2>&1; then
  autl_export_comm_defaults
else
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  export PYTHONNOUSERSITE=1
  export GZ_VERSION="${GZ_VERSION:-harmonic}"
fi

if ! ros2 pkg prefix mavros >/dev/null 2>&1; then
  echo "[WARN] mavros package not found; skipping MAVROS launcher"
  exit 0
fi
if ! ros2 pkg prefix mavros_msgs >/dev/null 2>&1; then
  echo "[WARN] mavros_msgs package not found; skipping MAVROS launcher"
  exit 0
fi

echo "[INFO] ROOT_DIR=$ROOT_DIR"
echo "[INFO] ROS_DOMAIN_ID: environment default (no override)"
echo "[INFO] GZ_VERSION=$GZ_VERSION"
echo "[INFO] WORKER_COUNT=$WORKER_COUNT"
echo "[INFO] NS_PREFIX=$NS_PREFIX"

touch /tmp/autolanding_mavros_nodes.txt
: > /tmp/autolanding_mavros_nodes.txt

cleanup_mavros_ns() {
  local ns="$1"
  # Kill only MAVROS nodes already bound to this namespace.
  pkill -f "mavros_node.*__ns:=${ns}( |$)" 2>/dev/null || true
}

launch_mavros_worker() {
  local ns="$1"
  local fcu_url="$2"
  local log_file="$3"

  nohup setsid ros2 run mavros mavros_node --ros-args \
    -r __ns:="${ns}" \
    -p fcu_url:="${fcu_url}" \
    > "${log_file}" 2>&1 < /dev/null &

  echo "$!"
}

wait_for_connected() {
  local ns="$1"
  local timeout_s="$2"
  local elapsed=0
  local topic="${ns}/state"

  while [[ "$elapsed" -lt "$timeout_s" ]]; do
    local out=""
    out="$(timeout 2 ros2 topic echo --once "${topic}" 2>/dev/null || true)"
    if echo "$out" | grep -q "connected: true"; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done
  return 1
}

for ((i=1; i<=WORKER_COUNT; i++)); do
  serial0_port=$((BASE_SERIAL1_PORT - 2 + 10 * (i - 1)))
  serial1_port=$((serial0_port + 2))
  fcu_url="tcp://127.0.0.1:${serial0_port}@${serial1_port}"
  ns="${NS_PREFIX}${i}"
  log_file="/tmp/autolanding_mavros_w${i}.log"

  cleanup_mavros_ns "$ns"
  sleep 0.2
  echo "[INFO] launching MAVROS worker ${i}: ns=${ns}, fcu_url=${fcu_url}"

  connected=0
  launch_pid=""
  for ((attempt=0; attempt<=MAX_RESTARTS; attempt++)); do
    launch_pid="$(launch_mavros_worker "$ns" "$fcu_url" "$log_file")"
    if wait_for_connected "$ns" "$CONNECT_TIMEOUT"; then
      connected=1
      echo "[INFO] MAVROS worker ${i} connected: ${ns}/state connected=true"
      break
    fi

    echo "[WARN] MAVROS worker ${i} did not reach connected=true within ${CONNECT_TIMEOUT}s (attempt $((attempt+1))/$((MAX_RESTARTS+1)))"
    cleanup_mavros_ns "$ns"
    sleep 1
  done

  if [[ "$connected" -ne 1 ]]; then
    echo "[WARN] MAVROS worker ${i} launch completed but state connection is not ready"
  fi

  echo "${i},${ns},${fcu_url},${log_file},${launch_pid},connected=${connected}" >> /tmp/autolanding_mavros_nodes.txt
  sleep 0.4
done

echo "[INFO] MAVROS nodes launch requested"
echo "[INFO] details: /tmp/autolanding_mavros_nodes.txt"