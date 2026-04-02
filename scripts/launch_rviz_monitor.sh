#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DOMAIN_ID="0"
START_BRIDGE=1
START_ARUCO=1
WORKER_COUNT=1
PIDFILE="${AUTOLANDING_RVIZ_PIDFILE:-/tmp/autolanding_rviz_monitor.pid}"
BRIDGE_PIDS=()
ARUCO_PID=""
ODOM_PID=""
RVIZ_PID=""
STOP_ONLY=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --domain)
      DOMAIN_ID="${2:-auto}"
      shift 2
      ;;
    --no-bridge)
      START_BRIDGE=0
      shift
      ;;
    --no-aruco)
      START_ARUCO=0
      shift
      ;;
    --workers)
      WORKER_COUNT="${2:-1}"
      shift 2
      ;;
    --stop)
      STOP_ONLY=1
      shift
      ;;
    *)
      echo "[WARN] Unknown argument: $1"
      shift
      ;;
  esac
done

stop_monitor() {
  if [[ -f "$PIDFILE" ]]; then
    local launcher_pid
    launcher_pid="$(cat "$PIDFILE" 2>/dev/null || true)"
    if [[ -n "$launcher_pid" ]] && kill -0 "$launcher_pid" 2>/dev/null; then
      echo "[INFO] stopping rviz monitor launcher pid $launcher_pid"
      kill -- "-$launcher_pid" 2>/dev/null || kill "$launcher_pid" 2>/dev/null || true
      for _ in 1 2 3 4 5; do
        sleep 1
        if ! kill -0 "$launcher_pid" 2>/dev/null; then
          break
        fi
      done
      if kill -0 "$launcher_pid" 2>/dev/null; then
        echo "[WARN] launcher did not exit cleanly; sending SIGKILL"
        kill -9 -- "-$launcher_pid" 2>/dev/null || kill -9 "$launcher_pid" 2>/dev/null || true
      fi
    else
      echo "[WARN] removing stale pidfile: $PIDFILE"
    fi
    rm -f "$PIDFILE"
  else
    echo "[INFO] no rviz monitor pidfile found: $PIDFILE"
  fi
}

if [[ "$STOP_ONLY" -eq 1 ]]; then
  stop_monitor
  exit 0
fi

cleanup() {
  local exit_code=$?
  local pid

  if [[ -n "$RVIZ_PID" ]] && kill -0 "$RVIZ_PID" 2>/dev/null; then
    kill -- "-$RVIZ_PID" 2>/dev/null || kill "$RVIZ_PID" 2>/dev/null || true
  fi

  for pid in "$ARUCO_PID" "$ODOM_PID"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done

  for pid in "${BRIDGE_PIDS[@]}"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
    fi
  done

  rm -f "$PIDFILE"
  exit "$exit_code"
}

trap cleanup EXIT
trap 'exit 130' INT TERM

if [[ -f "$PIDFILE" ]]; then
  existing_pid="$(cat "$PIDFILE" 2>/dev/null || true)"
  if [[ -n "$existing_pid" ]] && kill -0 "$existing_pid" 2>/dev/null; then
    echo "[ERROR] RViz monitor is already running (pid $existing_pid). Use --stop first."
    exit 1
  fi
  rm -f "$PIDFILE"
fi

echo $$ > "$PIDFILE"

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

pick_domain_id() {
  echo "0"
}

source_if_exists "/opt/ros/humble/setup.bash"
source_if_exists "$HOME/gz_ros2_aruco_ws/install/setup.bash"
source_if_exists "$HOME/SynologyDrive/INCSL/devel/INCSL/IICC26_ws/install/setup.bash"

export ROS_DOMAIN_ID="$(pick_domain_id)"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# Avoid MATLAB-local Python packages and Qt GUI theme vars from polluting ROS/RViz tools.
export PYTHONNOUSERSITE=1
if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
  SANITIZED_LD_LIBRARY_PATH="$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -vE '(/usr/local/MATLAB|/snap/core20|/snap/core22|/snap/)' | paste -sd ':' -)"
  export LD_LIBRARY_PATH="$SANITIZED_LD_LIBRARY_PATH"
fi
unset LD_PRELOAD || true
export QT_PLUGIN_PATH="${QT_PLUGIN_PATH:-/usr/lib/x86_64-linux-gnu/qt5/plugins}"
export QT_QPA_PLATFORM_PLUGIN_PATH="${QT_QPA_PLATFORM_PLUGIN_PATH:-/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms}"
unset QML2_IMPORT_PATH QT_QPA_PLATFORMTHEME || true
export QT_ACCESSIBILITY=0
export QT_LOGGING_RULES="${QT_LOGGING_RULES:-qt.accessibility.*=false}"
export QT_X11_NO_MITSHM=1
export QT_XCB_GL_INTEGRATION=none
export QT_OPENGL=software
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe

if [[ -z "${XDG_RUNTIME_DIR:-}" ]]; then
  export XDG_RUNTIME_DIR="/tmp/xdg-runtime-${USER:-rviz}"
fi
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR" 2>/dev/null || true

echo "[INFO] ROOT_DIR=$ROOT_DIR"
echo "[INFO] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[INFO] WORKER_COUNT=$WORKER_COUNT"

autolanding_rviz_cfg="$ROOT_DIR/simulation/configs/autolanding_monitor.rviz"
if [[ ! -f "$autolanding_rviz_cfg" ]]; then
  echo "[ERROR] RViz config not found: $autolanding_rviz_cfg"
  exit 1
fi

if [[ "$START_BRIDGE" -eq 1 ]]; then
  echo "[INFO] starting ros_gz_bridge parameter bridge"
  # Camera topics are sensor streams; bridge only GZ->ROS to prevent ROS->GZ conversion errors.
  nohup ros2 run ros_gz_bridge parameter_bridge \
    /camera@sensor_msgs/msg/Image[gz.msgs.Image \
    /camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
    > /tmp/autolanding_ros_gz_bridge.log 2>&1 &
  BRIDGE_PIDS+=("$!")

  for ((i=1; i<=WORKER_COUNT; i++)); do
    echo "[INFO] starting camera bridge for /drone${i}/camera"
    nohup ros2 run ros_gz_bridge parameter_bridge \
      "/drone${i}/camera@sensor_msgs/msg/Image[gz.msgs.Image" \
      "/drone${i}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo" \
      > "/tmp/autolanding_ros_gz_bridge_drone${i}.log" 2>&1 &
    BRIDGE_PIDS+=("$!")
  done
fi

if [[ "$START_ARUCO" -eq 1 ]]; then
  if python3 - <<'PY' >/tmp/autolanding_aruco_depcheck.log 2>&1
import numpy as np
import cv2  # noqa: F401
import transforms3d  # noqa: F401
if not hasattr(np, 'float'):
    raise RuntimeError("numpy has no np.float alias required by installed transforms3d/ros2_aruco stack")
print("aruco python deps look compatible")
PY
  then
    echo "[INFO] starting ros2_aruco detector"
    nohup ros2 launch ros2_aruco aruco_recognition.launch.py \
      > /tmp/autolanding_aruco.log 2>&1 &
    ARUCO_PID="$!"
  else
    echo "[WARN] skipping ros2_aruco launch due to Python dependency incompatibility"
    echo "[WARN] see /tmp/autolanding_aruco_depcheck.log"
  fi
fi

echo "[INFO] starting multi-drone odom namespace publisher"
nohup python3 "$ROOT_DIR/scripts/publish_multi_drone_odom.py" --workers "$WORKER_COUNT" \
  > /tmp/autolanding_multi_drone_odom.log 2>&1 &
ODOM_PID="$!"

echo "[INFO] launching RViz"

restart_count=0
while true; do
  set +e
  setsid env -i HOME="$HOME" USER="${USER:-rviz}" PATH="/usr/bin:/bin" DISPLAY="${DISPLAY:-:0}" XAUTHORITY="${XAUTHORITY:-}" XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
    bash -lc 'source /opt/ros/humble/setup.bash >/dev/null 2>&1; source "$HOME/gz_ros2_aruco_ws/install/setup.bash" >/dev/null 2>&1; source "$HOME/SynologyDrive/INCSL/devel/INCSL/IICC26_ws/install/setup.bash" >/dev/null 2>&1; export ROS_DOMAIN_ID=0; exec env QT_QPA_PLATFORM=xcb QT_ACCESSIBILITY=0 QT_XCB_GL_INTEGRATION=none QT_OPENGL=software LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe rviz2 -d '"$autolanding_rviz_cfg"'' &
  RVIZ_PID=$!
  wait "$RVIZ_PID"
  rviz_exit=$?
  set -e
  RVIZ_PID=""
  echo "[WARN] rviz2 exited with code ${rviz_exit}"
  restart_count=$((restart_count + 1))
  echo "[INFO] restarting rviz2 (attempt ${restart_count})"
  sleep 2
done
