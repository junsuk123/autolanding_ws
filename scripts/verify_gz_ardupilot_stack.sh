#!/usr/bin/env bash
set -euo pipefail

WORKDIR="$(cd "$(dirname "$0")/.." && pwd)"
ARDUPILOT_ROOT="${HOME}/ardupilot"
GZ_WORLD="/tmp/iris_runway_aruco_landing.sdf"
SIM_ADDR="${SIM_ADDR:-127.0.0.1}"
GZ_LOG="${WORKDIR}/data/processed/verify_gz_server.log"
SITL_LOG="${WORKDIR}/data/processed/verify_sitl.log"
MAV_LOG="${WORKDIR}/data/processed/verify_mavproxy.log"

# Keep server/client on the same Gazebo transport partition.
export GZ_PARTITION="${AUTOLANDING_GZ_PARTITION:-autolanding}"

# Prefer workspace Python tools (MAVProxy/pymavlink) when available.
if [[ -d "${WORKDIR}/.venv/bin" ]]; then
  export PATH="${WORKDIR}/.venv/bin:${PATH}"
fi

MAVPROXY_BIN="mavproxy.py"
if [[ -x "${WORKDIR}/.venv/bin/mavproxy.py" ]]; then
  MAVPROXY_BIN="${WORKDIR}/.venv/bin/mavproxy.py"
fi

PY_EXEC="python3"
if [[ -x "${WORKDIR}/.venv/bin/python" ]]; then
  PY_EXEC="${WORKDIR}/.venv/bin/python"
fi

MAV_PORT="${AUTOLANDING_MAVLINK_UDP_PORT:-14550}"
MAVROS_ENABLE="${AUTOLANDING_ENABLE_MAVROS:-1}"
MAVROS_REQUIRE="${AUTOLANDING_REQUIRE_MAVROS:-1}"
AUTOLANDING_SITL_WIPE="${AUTOLANDING_SITL_WIPE:-1}"
# Architecture baseline: SITL <-> MAVROS over TCP 5762 (MAVLink).
MAVROS_FCU_URL="${AUTOLANDING_MAVROS_FCU_URL:-tcp://127.0.0.1:5762}"

if [[ -f "$WORKDIR/scripts/common_ros_env.sh" ]]; then
  # shellcheck disable=SC1090
  source "$WORKDIR/scripts/common_ros_env.sh"
fi
if declare -F autl_source_ros_stacks >/dev/null 2>&1; then
  autl_source_ros_stacks "$WORKDIR"
fi
if declare -F autl_export_comm_defaults >/dev/null 2>&1; then
  autl_export_comm_defaults
fi

# Guard against invalid ROS_DOMAIN_ID propagated as empty string.
if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  export ROS_DOMAIN_ID=0
elif ! [[ "${ROS_DOMAIN_ID}" =~ ^[0-9]+$ ]]; then
  echo "[WARN] Invalid ROS_DOMAIN_ID='${ROS_DOMAIN_ID}', forcing 0"
  export ROS_DOMAIN_ID=0
fi

# MATLAB batch sessions can inject incompatible Qt/OpenGL library paths.
# Always prioritize system libs for Gazebo launcher paths.
unset OSG_LD_LIBRARY_PATH || true
unset QT_PLUGIN_PATH || true
unset QML2_IMPORT_PATH || true
unset QT_QPA_PLATFORMTHEME || true
export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"

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

# Ensure Gazebo can resolve package://ardupilot_gazebo/* URIs and plugins.
add_path_var GZ_SIM_SYSTEM_PLUGIN_PATH "$HOME/ardu_ws/install/ardupilot_gazebo/lib"
add_path_var GZ_SIM_SYSTEM_PLUGIN_PATH "$HOME/ardu_ws/src/ardupilot_gazebo/build"
add_path_var GZ_SIM_RESOURCE_PATH "$HOME/ardu_ws/install/ardupilot_gazebo/share"
add_path_var GZ_SIM_RESOURCE_PATH "$HOME/ardu_ws/install/ardupilot_gazebo/share/ardupilot_gazebo"
add_path_var GZ_SIM_RESOURCE_PATH "$HOME/ardu_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/models"
add_path_var GZ_SIM_RESOURCE_PATH "$HOME/ardu_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/worlds"
add_path_var GZ_SIM_RESOURCE_PATH "$HOME/ardu_ws/src/ardupilot_gazebo"
add_path_var GZ_SIM_RESOURCE_PATH "$HOME/ardu_ws/src/ardupilot_gazebo/models"
add_path_var GZ_SIM_RESOURCE_PATH "$HOME/ardu_ws/src/ardupilot_gazebo/worlds"
add_path_var GZ_SIM_RESOURCE_PATH "$HOME/gz_ros2_aruco_ws/src/ros2-gazebo-aruco/gz-world"
add_path_var GZ_SIM_RESOURCE_PATH "$HOME/gz_ros2_aruco_ws/src/ros2-gazebo-aruco/gz-world/aruco_box"
dedupe_path_var GZ_SIM_SYSTEM_PLUGIN_PATH
dedupe_path_var GZ_SIM_RESOURCE_PATH

mkdir -p "${WORKDIR}/data/processed"

generate_aruco_world() {
  local base_world=""
  if [[ -f "$HOME/ardu_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/worlds/iris_runway.sdf" ]]; then
    base_world="$HOME/ardu_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/worlds/iris_runway.sdf"
  elif [[ -f "$HOME/ardu_ws/src/ardupilot_gazebo/worlds/iris_runway.sdf" ]]; then
    base_world="$HOME/ardu_ws/src/ardupilot_gazebo/worlds/iris_runway.sdf"
  else
    echo "[WARN] Base world iris_runway.sdf not found. Using default world name: iris_runway.sdf"
    GZ_WORLD="iris_runway.sdf"
    return 0
  fi

  python3 - "$base_world" "$GZ_WORLD" <<'PY'
import re
import sys
import os
from pathlib import Path

base = Path(sys.argv[1])
out = Path(sys.argv[2])
text = base.read_text(encoding='utf-8')

aruco_scale_xy = float(os.environ.get("AUTOLANDING_ARUCO_SCALE_XY", "2.4"))
aruco_scale_z = float(os.environ.get("AUTOLANDING_ARUCO_SCALE_Z", "1.0"))
aruco_height = 0.5 * aruco_scale_z
aruco_center_z = 0.5 * aruco_height
drone_spawn_x = float(os.environ.get("AUTOLANDING_INITIAL_SPAWN_X_M", "1.0"))
drone_spawn_y = float(os.environ.get("AUTOLANDING_INITIAL_SPAWN_Y_M", "0.35"))
drone_spawn_z = float(os.environ.get("AUTOLANDING_INITIAL_SPAWN_Z_M", "0.15"))

include_block = """
    <include>
      <name>aruco_landing_box</name>
      <uri>model://aruco_box</uri>
      <scale>{scale_xy:.3f} {scale_xy:.3f} {scale_z:.3f}</scale>
      <pose>0 0 {center_z:.3f} 0 0 0</pose>
    </include>
""".format(scale_xy=aruco_scale_xy, scale_z=aruco_scale_z, center_z=aruco_center_z)

iris_pose_re = re.compile(
  r'(<uri>model://iris_with_gimbal</uri>\s*<pose[^>]*>)([^<]+)(</pose>)',
  re.MULTILINE,
)
text = iris_pose_re.sub(rf'\g<1>{drone_spawn_x:.3f} {drone_spawn_y:.3f} {drone_spawn_z:.3f} 0 0 90\g<3>', text, count=1)

if '<name>aruco_landing_box</name>' not in text:
    marker = '</world>'
    idx = text.rfind(marker)
    if idx != -1:
        text = text[:idx] + include_block + "\n" + text[idx:]

out.write_text(text, encoding='utf-8')
print(f"[INFO] Generated ArUco world: {out}")
print(f"[INFO] Drone initial spawn pose set to x={drone_spawn_x:.3f}, y={drone_spawn_y:.3f}, z={drone_spawn_z:.3f}")
PY
}

generate_aruco_world

kill_all_gazebo_processes() {
  echo "[INFO] Pre-cleanup: terminating all running Gazebo processes..."

  # Broad patterns cover Gazebo Sim/Harmonic, classic Gazebo, and ign aliases.
  local gz_patterns=(
    "gz sim"
    "gzserver"
    "gzclient"
    "ign gazebo"
    "gazebo --"
    "gazebo "
  )

  local pat=""
  for pat in "${gz_patterns[@]}"; do
    pkill -TERM -f "$pat" >/dev/null 2>&1 || true
  done
  pkill -TERM -f "rviz2|rviz" >/dev/null 2>&1 || true
  sleep 2

  for pat in "${gz_patterns[@]}"; do
    pkill -KILL -f "$pat" >/dev/null 2>&1 || true
  done
  pkill -KILL -f "rviz2|rviz" >/dev/null 2>&1 || true
  sleep 1

  # Final safety net for any leftover gz-related processes.
  pkill -KILL -f "(^|/)gz($| )" >/dev/null 2>&1 || true
  pkill -KILL -f "(^|/)gazebo($| )" >/dev/null 2>&1 || true

  # PID-level final sweep for stubborn children (gz sim server/gui, rviz).
  local leftover_pids=""
  leftover_pids="$(pgrep -f 'gz sim|gzserver|gzclient|ign gazebo|rviz2|rviz' 2>/dev/null || true)"
  if [[ -n "${leftover_pids}" ]]; then
    echo "[WARN] Pre-cleanup: force killing leftover Gazebo/RViz PIDs: ${leftover_pids//$'\n'/ }"
    while IFS= read -r pid; do
      [[ -n "$pid" ]] || continue
      kill -9 "$pid" >/dev/null 2>&1 || true
    done <<< "$leftover_pids"
  fi

  if pgrep -f 'gz sim|gzserver|gzclient|ign gazebo|rviz2|rviz' >/dev/null 2>&1; then
    echo "[WARN] Pre-cleanup: some Gazebo/RViz processes still detected after force kill."
  else
    echo "[INFO] Pre-cleanup: Gazebo/RViz process cleanup complete."
  fi
}

cleanup() {
  pkill -f "autolanding_mav_udp_relay.py" || true
  pkill -f "mavproxy.py" || true
  pkill -f "ros2 launch mavros px4.launch" || true
  pkill -f "mavros_node" || true
  pkill -f "rviz2" || true
  pkill -f "rviz" || true
  pkill -TERM -f "build/sitl/bin/arducopter" || true
  pkill -TERM -f "(^|/)arducopter($| )" || true
  pkill -f "sim_vehicle.py" || true
  # Remove hidden SITL conflicts from other stacks (e.g., ardupilot_sitl launch).
  fuser -k 5760/tcp >/dev/null 2>&1 || true
  fuser -k 5762/tcp >/dev/null 2>&1 || true
  fuser -k 14550/udp >/dev/null 2>&1 || true
  sleep 1
  pkill -KILL -f "(^|/)arducopter($| )" || true
  kill_all_gazebo_processes
}

cleanup
sleep 1

cd "${WORKDIR}"
GZ_BIN=(/usr/bin/gz)
GZ_FLAGS=(-v4 -r)

# Prefer GUI when explicitly requested, even from MATLAB shells where DISPLAY may be empty.
if [[ "${AUTOLANDING_FORCE_HEADLESS:-0}" != "1" ]]; then
  if [[ -z "${DISPLAY:-}" ]]; then
    if [[ -n "${AUTOLANDING_DISPLAY:-}" ]]; then
      export DISPLAY="${AUTOLANDING_DISPLAY}"
    else
      active_disp="$(who 2>/dev/null | sed -n 's/.*(\(:[0-9][0-9]*\)).*/\1/p' | head -n1)"
      if [[ -n "${active_disp}" ]]; then
        export DISPLAY="${active_disp}"
      elif compgen -G "/tmp/.X11-unix/X*" >/dev/null 2>&1; then
        first_xsock="$(ls /tmp/.X11-unix/X* 2>/dev/null | head -n1)"
        if [[ -n "${first_xsock}" ]]; then
          export DISPLAY=":$(basename "${first_xsock}" | sed 's/^X//')"
        else
          export DISPLAY=:0
        fi
      else
        export DISPLAY=:0
      fi
    fi
  fi
  if [[ -z "${XAUTHORITY:-}" ]]; then
    if [[ -f "/run/user/$(id -u)/gdm/Xauthority" ]]; then
      export XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority"
    elif [[ -f "$HOME/.Xauthority" ]]; then
      export XAUTHORITY="$HOME/.Xauthority"
    fi
  fi
  # MATLAB batch sessions can inherit Wayland-heavy env that hides GUI windows.
  export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
  export QT_QPA_PLATFORM_PLUGIN_PATH="${QT_QPA_PLATFORM_PLUGIN_PATH:-/usr/lib/x86_64-linux-gnu/qt5/plugins}"
  export LD_LIBRARY_PATH="/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
  export LIBGL_ALWAYS_INDIRECT="${LIBGL_ALWAYS_INDIRECT:-0}"
  export NVIDIA_DRIVER_CAPABILITIES="${NVIDIA_DRIVER_CAPABILITIES:-graphics,compute}"
  # Default to hardware rendering. Force software mode only when explicitly requested.
  if [[ "${AUTOLANDING_GUI_SOFTWARE:-0}" == "1" ]]; then
    if [[ -f "/usr/lib/x86_64-linux-gnu/dri/llvmpipe_dri.so" || -f "/usr/lib/dri/llvmpipe_dri.so" ]]; then
      export LIBGL_ALWAYS_SOFTWARE=1
      export MESA_LOADER_DRIVER_OVERRIDE="${MESA_LOADER_DRIVER_OVERRIDE:-llvmpipe}"
      echo "[INFO] GUI rendering mode: software (llvmpipe)"
    else
      echo "[WARN] AUTOLANDING_GUI_SOFTWARE=1 but llvmpipe is not installed. Falling back to hardware rendering."
      unset LIBGL_ALWAYS_SOFTWARE || true
      unset MESA_LOADER_DRIVER_OVERRIDE || true
    fi
  else
    unset LIBGL_ALWAYS_SOFTWARE || true
    unset MESA_LOADER_DRIVER_OVERRIDE || true
    echo "[INFO] GUI rendering mode: hardware"
  fi
  export GDK_BACKEND="${GDK_BACKEND:-x11}"
  export NO_AT_BRIDGE=1
  unset WAYLAND_DISPLAY
  echo "[INFO] GUI env: DISPLAY=${DISPLAY:-<unset>} XAUTHORITY=${XAUTHORITY:-<unset>}"
fi

if [[ "${AUTOLANDING_FORCE_HEADLESS:-0}" == "1" ]]; then
  GZ_FLAGS=(-s -v4 -r)
  echo "[INFO] Gazebo mode: headless"
else
  echo "[INFO] Gazebo mode: GUI"
fi

nohup setsid "${GZ_BIN[@]}" sim "${GZ_FLAGS[@]}" "${GZ_WORLD}" >"${GZ_LOG}" 2>&1 < /dev/null &
GZ_PID=$!
sleep 5

if ! ps -p "${GZ_PID}" >/dev/null 2>&1; then
  if [[ "${GZ_FLAGS[*]}" != *"-s"* && "${AUTOLANDING_FORCE_GUI:-0}" != "1" ]]; then
    echo "[WARN] Gazebo GUI failed to start. Falling back to headless mode." | tee -a "${GZ_LOG}"
    nohup setsid "${GZ_BIN[@]}" sim -s -v4 -r "${GZ_WORLD}" >>"${GZ_LOG}" 2>&1 < /dev/null &
    GZ_PID=$!
    sleep 4
  elif [[ "${AUTOLANDING_FORCE_GUI:-0}" == "1" ]]; then
    echo "[FAIL] Gazebo GUI failed to start and AUTOLANDING_FORCE_GUI=1 is set." | tee -a "${GZ_LOG}"
  fi
fi

if ! ps -p "${GZ_PID}" >/dev/null 2>&1; then
  echo "[FAIL] Gazebo failed to start (GUI/headless both failed)."
  echo "GZ log: ${GZ_LOG}"
  tail -n 80 "${GZ_LOG}" || true
  exit 1
fi

SITL_DEFAULTS="${ARDUPILOT_ROOT}/Tools/autotest/default_params/copter.parm,${ARDUPILOT_ROOT}/Tools/autotest/default_params/gazebo-iris.parm"
if [[ -f "${WORKDIR}/mav.parm" ]]; then
  SITL_DEFAULTS="${SITL_DEFAULTS},${WORKDIR}/mav.parm"
fi

SITL_WIPE_ARG=""
if [[ "${AUTOLANDING_SITL_WIPE}" == "1" ]]; then
  SITL_WIPE_ARG="-w"
fi

cd "${WORKDIR}"

nohup setsid "${ARDUPILOT_ROOT}/build/sitl/bin/arducopter" \
  --model "JSON:${SIM_ADDR}" --speedup 1 --slave 0 --serial0=udpclient:127.0.0.1:${MAV_PORT} \
  ${SITL_WIPE_ARG} \
  --defaults "${SITL_DEFAULTS}" \
  -I0 >"${SITL_LOG}" 2>&1 < /dev/null &
SITL_PID=$!
sleep 8

MAVROS_LOG="${WORKDIR}/data/processed/verify_mavros.log"
if [[ "${MAVROS_ENABLE}" == "1" ]]; then
  set +u
  source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true
  [[ -f "$WORKDIR/../IICC26_ws/install/setup.bash" ]] && source "$WORKDIR/../IICC26_ws/install/setup.bash" >/dev/null 2>&1 || true
  set -u
  nohup setsid bash -lc "source /opt/ros/humble/setup.bash >/dev/null 2>&1; [[ -f '$WORKDIR/../IICC26_ws/install/setup.bash' ]] && source '$WORKDIR/../IICC26_ws/install/setup.bash' >/dev/null 2>&1; ros2 launch mavros px4.launch fcu_url:=${MAVROS_FCU_URL}" >"${MAVROS_LOG}" 2>&1 < /dev/null &
  MAVROS_PID=$!
  sleep 6

  MAVROS_OK=0
  MAVROS_TOPIC_OK=0
  for _ in {1..20}; do
    if timeout 2 bash -lc "source /opt/ros/humble/setup.bash >/dev/null 2>&1; [[ -f '$WORKDIR/../IICC26_ws/install/setup.bash' ]] && source '$WORKDIR/../IICC26_ws/install/setup.bash' >/dev/null 2>&1; ros2 topic list 2>/dev/null | grep -q '^/mavros/state$'"; then
      MAVROS_TOPIC_OK=1
    fi
    if timeout 2 bash -lc "source /opt/ros/humble/setup.bash >/dev/null 2>&1; [[ -f '$WORKDIR/../IICC26_ws/install/setup.bash' ]] && source '$WORKDIR/../IICC26_ws/install/setup.bash' >/dev/null 2>&1; ros2 topic echo --once /mavros/state 2>/dev/null | grep -q 'connected: true'"; then
      MAVROS_OK=1
      break
    fi
    sleep 1
  done

  if [[ ${MAVROS_TOPIC_OK} -ne 1 ]]; then
    if [[ "${MAVROS_REQUIRE}" == "1" ]]; then
      echo "[FAIL] MAVROS state topic /mavros/state not available"
      echo "MAVROS log: ${MAVROS_LOG}"
      tail -n 120 "${MAVROS_LOG}" || true
      exit 1
    else
      echo "[WARN] MAVROS state topic unavailable; continuing because AUTOLANDING_REQUIRE_MAVROS=0"
    fi
  elif [[ ${MAVROS_OK} -ne 1 ]]; then
    if [[ "${MAVROS_REQUIRE}" == "1" ]]; then
      echo "[WARN] MAVROS state topic exists but connected=true was not observed within readiness window"
    else
      echo "[WARN] MAVROS connected=true not observed; continuing because AUTOLANDING_REQUIRE_MAVROS=0"
    fi
  fi
fi

# Recovery mode: return after processes are spawned; caller will probe MAVLink readiness.
if [[ "${AUTOLANDING_RECOVERY_QUICK:-0}" == "1" ]]; then
  if ps -p "${GZ_PID}" >/dev/null 2>&1 && ps -p "${SITL_PID}" >/dev/null 2>&1; then
    echo "[PASS] Recovery quick mode: Gazebo/SITL processes started."
    echo "GZ log: ${GZ_LOG}"
    echo "SITL log: ${SITL_LOG}"
    exit 0
  fi
  echo "[FAIL] Recovery quick mode: process spawn check failed."
  echo "GZ log: ${GZ_LOG}"
  echo "SITL log: ${SITL_LOG}"
  exit 1
fi

set +e
timeout -k 2 45 "${PY_EXEC}" - <<PY >"${MAV_LOG}" 2>&1
from pymavlink import mavutil

m = mavutil.mavlink_connection('udpin:127.0.0.1:${MAV_PORT}', source_system=255, autoreconnect=False)
hb = m.wait_heartbeat(timeout=40)
if hb is None:
    raise SystemExit(2)
print('Detected vehicle heartbeat on udpin:127.0.0.1:${MAV_PORT}')
PY
MAV_RC=$?
set -e

if [[ ${MAV_RC} -eq 0 ]] && ps -p "${SITL_PID}" >/dev/null 2>&1; then
  sleep 2
fi

if [[ ${MAV_RC} -eq 0 ]] && ps -p "${SITL_PID}" >/dev/null 2>&1; then
  echo "[PASS] Heartbeat detected. gzsim + ardupilot stack is alive."
  echo "GZ log: ${GZ_LOG}"
  echo "SITL log: ${SITL_LOG}"
  echo "MAV log: ${MAV_LOG}"
  if [[ "${MAVROS_ENABLE}" == "1" ]]; then
    echo "MAVROS log: ${MAVROS_LOG}"
  fi
  exit 0
fi

if [[ ${MAV_RC} -eq 0 ]]; then
  echo "[FAIL] Heartbeat probe passed but SITL process is not alive (false positive readiness)."
  echo "GZ log: ${GZ_LOG}"
  echo "SITL log: ${SITL_LOG}"
  echo "MAV log: ${MAV_LOG}"
  exit 1
fi

echo "[FAIL] No vehicle heartbeat detected."
echo "MAVProxy exit code: ${MAV_RC}"
echo "GZ log: ${GZ_LOG}"
echo "SITL log: ${SITL_LOG}"
echo "MAV log: ${MAV_LOG}"
if [[ "${MAVROS_ENABLE}" == "1" ]]; then
  echo "MAVROS log: ${MAVROS_LOG}"
fi
exit 1
