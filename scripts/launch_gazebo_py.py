#!/usr/bin/env python3
"""
Gazebo simulation launcher with multi-drone support (Python-based)
Replaces scripts that call 'gz sim' directly

Ensures GZ_SIM_RESOURCE_PATH includes /tmp for dynamically generated model variants.

Usage:
  python3 launch_gazebo_py.py [--world /path/to/world.sdf] [--gui] [--verbose]
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path


def get_gazebo_env():
    """Prepare Gazebo environment variables with clean library paths."""
    env = os.environ.copy()

    # **CRITICAL**: Remove MATLAB-polluted LD_LIBRARY_PATH to avoid Qt conflicts
    # MATLAB injects its own Qt libraries which conflict with system Gazebo Qt
    env.pop("LD_LIBRARY_PATH", None)
    
    # Clean up other MATLAB-injected variables that interfere with rendering
    # OSG_LD_LIBRARY_PATH: MATLAB's OpenSceneGraph libs conflict with Gazebo rendering
    # QT_*: MATLAB's Qt variables conflict with system Gazebo Qt
    for var in ["OSG_LD_LIBRARY_PATH", "QT_PLUGIN_PATH", "QML2_IMPORT_PATH", "QT_QPA_PLATFORMTHEME"]:
        env.pop(var, None)

    # Keep Gazebo GUI rendering on the same stable path as the working shell launchers.
    env["QT_QPA_PLATFORM"] = "xcb"
    env["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/usr/lib/x86_64-linux-gnu/qt5/plugins"
    env["QT_XCB_GL_INTEGRATION"] = "none"
    env["QT_OPENGL"] = "software"
    env["LIBGL_ALWAYS_SOFTWARE"] = "1"
    env["MESA_LOADER_DRIVER_OVERRIDE"] = "llvmpipe"
    env["LIBGL_ALWAYS_INDIRECT"] = "0"

    # Ensure /tmp is in model search path so dynamically generated iris_with_gimbal_w* variants are found
    existing_path = env.get("GZ_SIM_RESOURCE_PATH", "")
    if existing_path:
        path_list = existing_path.split(":")
    else:
        path_list = []

    # Add /tmp if not already there
    if "/tmp" not in path_list:
        path_list.insert(0, "/tmp")

    # Add ardupilot_gazebo resources
    gz_ws_path = os.path.expanduser("~/gz_ws/src/ardupilot_gazebo")
    if os.path.isdir(gz_ws_path) and gz_ws_path not in path_list:
        path_list.insert(1, gz_ws_path)
        path_list.insert(2, os.path.join(gz_ws_path, "models"))
        path_list.insert(3, os.path.join(gz_ws_path, "worlds"))

    # Add ros2_aruco resources
    aruco_ws_path = os.path.expanduser("~/gz_ros2_aruco_ws/src/ros2-gazebo-aruco")
    if os.path.isdir(aruco_ws_path):
        gz_world_path = os.path.join(aruco_ws_path, "gz-world")
        if os.path.isdir(gz_world_path) and gz_world_path not in path_list:
            path_list.insert(4, gz_world_path)

    env["GZ_SIM_RESOURCE_PATH"] = ":".join(path_list)

    # Ensure GZ_SIM_SYSTEM_PLUGIN_PATH includes ardupilot plugin
    gz_plugin_path = os.path.expanduser("~/gz_ws/src/ardupilot_gazebo/build")
    if os.path.isdir(gz_plugin_path):
        existing_plugin_path = env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
        if gz_plugin_path not in existing_plugin_path:
            env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = f"{gz_plugin_path}:{existing_plugin_path}".rstrip(":")

    # Set GZ_VERSION
    env["GZ_VERSION"] = "harmonic"
    
    # Ensure clean Python path (no MATLAB site-packages)
    env["PYTHONNOUSERSITE"] = "1"

    return env


def launch_gazebo(world_file, gui=True, verbose=False):
    """Launch Gazebo with the given world file."""
    if not os.path.isfile(world_file):
        print(f"[ERROR] World file not found: {world_file}", file=sys.stderr)
        return False

    # Prepare command
    cmd = ["gz", "sim"]

    if verbose:
        cmd.append("-v4")
    else:
        cmd.append("-v1")

    if gui:
        cmd.append("-g")  # GUI mode
    else:
        cmd.append("-s")  # Server (headless) mode

    cmd.append("-r")  # Use physics server
    cmd.append(world_file)

    # Prepare environment
    env = get_gazebo_env()

    # Set DISPLAY if GUI requested
    if gui:
        display = os.environ.get("DISPLAY", ":0")
        env["DISPLAY"] = display
        # Find valid XAUTHORITY file
        xauth_candidates = [
            os.environ.get("XAUTHORITY", ""),
            f"/run/user/{os.getuid()}/gdm/Xauthority",
            os.path.expanduser("~/.Xauthority"),
        ]
        for xauth_path in xauth_candidates:
            if xauth_path and os.path.isfile(xauth_path):
                env["XAUTHORITY"] = xauth_path
                break

    print(f"[INFO] Launching Gazebo {'(GUI)' if gui else '(Server)'}")
    print(f"[INFO] World: {world_file}")
    print(f"[INFO] GZ_SIM_RESOURCE_PATH: {env['GZ_SIM_RESOURCE_PATH']}")

    try:
        log_file = Path("/tmp/gz_sim.log")
        with open(log_file, "w") as log_f:
            proc = subprocess.Popen(
                cmd,
                env=env,
                stdout=log_f,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
            )
            print(f"[INFO] Gazebo started (PID={proc.pid}, log=/tmp/gz_sim.log)")
            return proc

    except Exception as e:
        print(f"[ERROR] Failed to launch Gazebo: {e}", file=sys.stderr)
        return False


def main():
    parser = argparse.ArgumentParser(description="Launch Gazebo with multi-drone world")
    parser.add_argument(
        "--world",
        type=str,
        default="/tmp/iris_runway_aruco_landing.sdf",
        help="Path to world SDF file",
    )
    parser.add_argument("--gui", action="store_true", default=True, help="Enable GUI (default)")
    parser.add_argument("--headless", action="store_true", help="Run in headless mode (server only)")
    parser.add_argument("--verbose", action="store_true", help="Verbose Gazebo logging")

    args = parser.parse_args()

    gui = not args.headless

    proc = launch_gazebo(args.world, gui=gui, verbose=args.verbose)
    if not proc:
        return 1

    # Keep process alive
    try:
        proc.wait()
    except KeyboardInterrupt:
        print("\n[INFO] Gazebo terminated by user")
        proc.terminate()
        proc.wait(timeout=5)

    return 0


if __name__ == "__main__":
    sys.exit(main())
