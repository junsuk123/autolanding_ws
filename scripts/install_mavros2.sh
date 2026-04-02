#!/usr/bin/env bash
set -euo pipefail

# Install ROS2 MAVROS packages and geographiclib datasets for ROS2 Humble.
# This script is optional: default control backend is still MAVProxy.

ROS_DISTRO_DEFAULT="humble"
ROS_DISTRO="${ROS_DISTRO:-$ROS_DISTRO_DEFAULT}"

if ! command -v apt-get >/dev/null 2>&1; then
  echo "[error] apt-get not found. This installer supports Debian/Ubuntu only."
  exit 1
fi

if ! command -v sudo >/dev/null 2>&1; then
  echo "[error] sudo not found. Please install packages manually as root."
  exit 1
fi

echo "[info] Using ROS_DISTRO=$ROS_DISTRO"
echo "[step] apt update"
sudo apt-get update

echo "[step] Install MAVROS packages"
INSTALL_PKGS=(
  "ros-${ROS_DISTRO}-mavros"
  "ros-${ROS_DISTRO}-mavros-extras"
  "ros-${ROS_DISTRO}-mavros-msgs"
  geographiclib-tools
)

# Optional package names vary by distro/repo snapshot.
for OPT in geographiclib-datasets-geoids geographiclib-datasets-gravity geographiclib-datasets-magnetic; do
  if apt-cache show "$OPT" >/dev/null 2>&1; then
    INSTALL_PKGS+=("$OPT")
  else
    echo "[warn] optional package unavailable: $OPT"
  fi
done

sudo apt-get install -y "${INSTALL_PKGS[@]}"

# MAVROS provides a helper installer for GeographicLib datasets in many ROS distros.
# Run it when present to avoid missing geoid/model warnings at runtime.
MAVROS_GEO_SCRIPT="/opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh"
if [[ -x "$MAVROS_GEO_SCRIPT" ]]; then
  echo "[step] Install GeographicLib datasets via MAVROS helper"
  sudo "$MAVROS_GEO_SCRIPT"
else
  echo "[warn] MAVROS geographiclib helper script not found at: $MAVROS_GEO_SCRIPT"
  echo "[warn] Continuing because datasets may already be available from apt packages."
fi

echo "[done] MAVROS installation complete"
echo "[next] Run: bash scripts/verify_mavros2_setup.sh"
