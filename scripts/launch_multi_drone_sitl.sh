#!/usr/bin/env bash
set -euo pipefail

COUNT="${1:-3}"
if ! [[ "$COUNT" =~ ^[0-9]+$ ]] || [[ "$COUNT" -lt 1 ]]; then
  echo "Usage: $0 <count>=1..N"
  exit 1
fi

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
BIN_PATH="${ARDUPILOT_DIR}/build/sitl/bin/arducopter"
if [[ ! -x "$BIN_PATH" ]]; then
  echo "[ERROR] SITL binary not found: $BIN_PATH"
  exit 1
fi

mkdir -p /tmp/autolanding_sitl

echo "[INFO] launching $COUNT SITL instances"
for ((i=0; i<COUNT; i++)); do
  log_file="/tmp/autolanding_sitl/arducopter_I${i}.log"
  cmd=("$BIN_PATH" --model JSON --speedup 1 --slave 0 \
    --defaults "$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm,$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-iris.parm" \
    --sim-address=127.0.0.1 -I"$i")

  nohup "${cmd[@]}" > "$log_file" 2>&1 &
  echo "[INFO] I${i} started (master tcp:127.0.0.1:$((5760 + 10*i)), serial1 tcp:127.0.0.1:$((5762 + 10*i))) log=$log_file"
  sleep 1
done

echo "[INFO] done"
echo "[INFO] stop with: pkill -f 'arducopter --model JSON'"
