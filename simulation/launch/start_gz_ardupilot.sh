#!/usr/bin/env bash
set -euo pipefail

echo "Use your prepared system environment first (gz-harmonic, ardupilot, MAVProxy, ardupilot_gazebo)."
echo
echo "Terminal 1:"
echo "  gz sim -v4 -r iris_runway.sdf"
echo
echo "Terminal 2:"
echo "  cd ~/ardupilot"
echo "  ./Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console"
echo
echo "MAVProxy quick commands:"
echo "  mode guided"
echo "  arm throttle"
echo "  takeoff 5"
