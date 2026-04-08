#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
RVIZ_CFG="$ROOT_DIR/simulation/configs/autolanding_monitor.rviz"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Source cleanup utilities
source "$SCRIPT_DIR/cleanup_utils.sh"

# Signal handlers
on_interrupt() {
	echo ""
	echo "[rviz] Received SIGINT, cleaning up..."
	cleanup_all_processes
	exit 130
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

# Remove VS Code snap / desktop injected vars that can break rviz2 dynamic linking.
unset LD_LIBRARY_PATH LD_PRELOAD QT_PLUGIN_PATH QML2_IMPORT_PATH QT_QPA_PLATFORMTHEME PYTHONUSERBASE
unset GTK_PATH LOCPATH GSETTINGS_SCHEMA_DIR GIO_MODULE_DIR GTK_IM_MODULE_FILE GTK_EXE_PREFIX GDK_BACKEND WAYLAND_DISPLAY
unset XDG_RUNTIME_DIR XDG_CONFIG_DIRS_VSCODE_SNAP_ORIG GDK_BACKEND_VSCODE_SNAP_ORIG GIO_MODULE_DIR_VSCODE_SNAP_ORIG
unset GSETTINGS_SCHEMA_DIR_VSCODE_SNAP_ORIG GTK_IM_MODULE_FILE_VSCODE_SNAP_ORIG GTK_EXE_PREFIX_VSCODE_SNAP_ORIG
unset LOCPATH_VSCODE_SNAP_ORIG XDG_DATA_HOME_VSCODE_SNAP_ORIG XDG_DATA_DIRS_VSCODE_SNAP_ORIG
unset SNAP SNAP_NAME SNAP_INSTANCE_NAME SNAP_ARCH SNAP_REVISION SNAP_VERSION SNAP_LIBRARY_PATH

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi
if [[ -f "$HOME/IICC26_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  set +u
  source "$HOME/IICC26_ws/install/setup.bash"
  set -u
fi
if [[ -f "$HOME/gz_ros2_aruco_ws/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  set +u
  source "$HOME/gz_ros2_aruco_ws/install/setup.bash"
  set -u
fi

export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:$PATH
export LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu
export QT_QPA_PLATFORM=xcb
export DISPLAY="${DISPLAY:-:0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

if [[ -f "$RVIZ_CFG" ]]; then
  rviz2 -d "$RVIZ_CFG"
else
  rviz2
fi
