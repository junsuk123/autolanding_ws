#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOMAIN_ID="0"
WORKER_COUNT=1
NS_PREFIX="/mavros_w"
BASE_SERIAL1_PORT=5762

while [[ $# -gt 0 ]]; do
  case "$1" in
    --domain)
      DOMAIN_ID="${2:-0}"
      shift 2
      ;;
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
    *)
      echo "[WARN] Unknown argument: $1"
      shift
      ;;
  esac
done

source_if_exists() {
  local p="$1"
  if [[ -f "$p" ]]; then
    # shellcheck disable=SC1090
    set +u
    source "$p"
    set -u
    echo "[INFO] sourced: $p"
  fi
}

if [[ "$DOMAIN_ID" == "auto" ]]; then
  DOMAIN_ID="0"
fi

if ! [[ "$WORKER_COUNT" =~ ^[0-9]+$ ]] || [[ "$WORKER_COUNT" -lt 1 ]]; then
  echo "[ERROR] invalid --workers value: $WORKER_COUNT"
  exit 1
fi

if ! [[ "$BASE_SERIAL1_PORT" =~ ^[0-9]+$ ]] || [[ "$BASE_SERIAL1_PORT" -lt 1 ]]; then
  echo "[ERROR] invalid --base-serial1-port value: $BASE_SERIAL1_PORT"
  exit 1
fi

source_if_exists "/opt/ros/humble/setup.bash"
source_if_exists "$HOME/SynologyDrive/INCSL/devel/INCSL/IICC26_ws/install/setup.bash"

export ROS_DOMAIN_ID="$DOMAIN_ID"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export PYTHONNOUSERSITE=1

if ! ros2 pkg prefix mavros >/dev/null 2>&1; then
  echo "[WARN] mavros package not found; skipping MAVROS launcher"
  exit 0
fi
if ! ros2 pkg prefix mavros_msgs >/dev/null 2>&1; then
  echo "[WARN] mavros_msgs package not found; skipping MAVROS launcher"
  exit 0
fi

echo "[INFO] ROOT_DIR=$ROOT_DIR"
echo "[INFO] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[INFO] WORKER_COUNT=$WORKER_COUNT"
echo "[INFO] NS_PREFIX=$NS_PREFIX"

touch /tmp/autolanding_mavros_nodes.txt
: > /tmp/autolanding_mavros_nodes.txt

for ((i=1; i<=WORKER_COUNT; i++)); do
  serial1_port=$((BASE_SERIAL1_PORT + 10 * (i - 1)))
  fcu_url="tcp://127.0.0.1:${serial1_port}"
  ns="${NS_PREFIX}${i}"
  log_file="/tmp/autolanding_mavros_w${i}.log"

  echo "[INFO] launching MAVROS worker ${i}: ns=${ns}, fcu_url=${fcu_url}"
  nohup ros2 run mavros mavros_node --ros-args \
    -r __ns:="${ns}" \
    -p fcu_url:="${fcu_url}" \
    -p gcs_url:="" \
    > "${log_file}" 2>&1 &

  echo "${i},${ns},${fcu_url},${log_file}" >> /tmp/autolanding_mavros_nodes.txt
  sleep 0.4
done

echo "[INFO] MAVROS nodes launch requested"
echo "[INFO] details: /tmp/autolanding_mavros_nodes.txt"