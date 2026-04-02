#!/usr/bin/env bash
set -euo pipefail

# Verify ROS2 MAVROS environment for optional MATLAB control backend.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

source_if_exists() {
  local p="$1"
  if [[ -f "$p" ]]; then
    # shellcheck disable=SC1090
    set +u
    source "$p"
    set -u
  fi
}

source_if_exists "/opt/ros/humble/setup.bash"
source_if_exists "$WS_ROOT/../IICC26_ws/install/setup.bash"

echo "[check] ros2 command"
if ! command -v ros2 >/dev/null 2>&1; then
  echo "[fail] ros2 command not found"
  exit 1
fi
echo "[ok] ros2 found"

echo "[check] required ROS2 packages"
MISSING=0
for PKG in mavros mavros_msgs; do
  if ros2 pkg prefix "$PKG" >/dev/null 2>&1; then
    echo "[ok] $PKG"
  else
    echo "[fail] $PKG not found"
    MISSING=1
  fi
done

if [[ "$MISSING" -ne 0 ]]; then
  echo "[hint] Install with: bash scripts/install_mavros2.sh"
  exit 2
fi

echo "[check] MAVROS launch files"
if ros2 launch mavros px4.launch fcu_url:=tcp://127.0.0.1:5760@5762 >/tmp/mavros_verify_launch.log 2>&1 & then
  PID=$!
  sleep 4
  if ps -p "$PID" >/dev/null 2>&1; then
    echo "[ok] mavros launch started (PID=$PID)"
    kill "$PID" >/dev/null 2>&1 || true
    wait "$PID" 2>/dev/null || true
  else
    echo "[warn] mavros launch did not stay up. See /tmp/mavros_verify_launch.log"
  fi
else
  echo "[warn] failed to invoke mavros launch. See /tmp/mavros_verify_launch.log"
fi

echo "[check] launch_multi_mavros.sh syntax"
bash -n "$WS_ROOT/scripts/launch_multi_mavros.sh"
echo "[ok] launch_multi_mavros.sh syntax"

echo "[done] verification finished"
