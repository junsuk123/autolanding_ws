#!/usr/bin/env bash
set -euo pipefail

WORKDIR="$(cd "$(dirname "$0")/.." && pwd)"
ARDUPILOT_ROOT="${HOME}/ardupilot"
GZ_WORLD="iris_runway.sdf"
GZ_LOG="${WORKDIR}/data/processed/verify_gz_server.log"
SITL_LOG="${WORKDIR}/data/processed/verify_sitl.log"
MAV_LOG="${WORKDIR}/data/processed/verify_mavproxy.log"

mkdir -p "${WORKDIR}/data/processed"

cleanup() {
  pkill -f "mavproxy.py --master tcp:127.0.0.1:5760" || true
  pkill -f "build/sitl/bin/arducopter --model JSON" || true
  pkill -f "gz sim -s -v4 -r ${GZ_WORLD}" || true
}

cleanup
sleep 1

cd "${WORKDIR}"
nohup gz sim -s -v4 -r "${GZ_WORLD}" >"${GZ_LOG}" 2>&1 &
GZ_PID=$!
sleep 5

nohup "${ARDUPILOT_ROOT}/build/sitl/bin/arducopter" \
  --model JSON --speedup 1 --slave 0 \
  --defaults "${ARDUPILOT_ROOT}/Tools/autotest/default_params/copter.parm,${ARDUPILOT_ROOT}/Tools/autotest/default_params/gazebo-iris.parm" \
  --sim-address=127.0.0.1 -I0 >"${SITL_LOG}" 2>&1 &
SITL_PID=$!
sleep 8

MAV_RC=1
for attempt in 1 2 3 4 5 6; do
  set +e
  timeout 10 mavproxy.py --master tcp:127.0.0.1:5760 --cmd="status" >"${MAV_LOG}" 2>&1
  MAV_RC=$?
  set -e

  if grep -q "Detected vehicle" "${MAV_LOG}"; then
    break
  fi
  sleep 3
done

if grep -q "Detected vehicle" "${MAV_LOG}"; then
  echo "[PASS] Heartbeat detected. gzsim + ardupilot stack is alive."
  echo "GZ log: ${GZ_LOG}"
  echo "SITL log: ${SITL_LOG}"
  echo "MAV log: ${MAV_LOG}"
  exit 0
fi

echo "[FAIL] No vehicle heartbeat detected."
echo "MAVProxy exit code: ${MAV_RC}"
echo "GZ log: ${GZ_LOG}"
echo "SITL log: ${SITL_LOG}"
echo "MAV log: ${MAV_LOG}"
exit 1
