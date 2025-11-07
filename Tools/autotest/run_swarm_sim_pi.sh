#!/bin/bash
set -euo pipefail

ARDUPILOT_DIR="$HOME/ardupilot"
BIN="$ARDUPILOT_DIR/build/sitl/bin/arduplane"
DEFAULTS="$ARDUPILOT_DIR/Tools/autotest/models/plane.parm"
HOME_COORDS="40.3117414,44.455211099999985,1294.86,0.0"

mkdir -p "$ARDUPILOT_DIR"/{1,2}

echo "Detected environment: $ENVIRONMENT"

(
  cd "$ARDUPILOT_DIR/1"
  "$BIN" \
      -S --model plane --speedup 5 --slave 0 \
      --defaults "$DEFAULTS" \
      --serial0=tcp:0.0.0.0:5760:nowait \
      --serial1=udpclient:0.0.0.0:5000 \
      --serial2=udpclient:192.168.105.60:15000 \
      --sim-address=127.0.0.1 -I0 \
      --home "$HOME_COORDS" \
      --sysid 1
) &
(
  cd "$ARDUPILOT_DIR/2"
  "$BIN" \
      -S --model plane --speedup 5 --slave 0 \
      --defaults "$DEFAULTS" \
      --serial0=tcp:0.0.0.0:5770:nowait \
      --serial1=udpclient:0.0.0.0:6000 \
      --serial2=udpclient:192.168.105.205:16000 \
      --sim-address=127.0.0.1 -I1 \
      --home "$HOME_COORDS" \
      --sysid 2
) &
(
  mavlink-routerd -v \
      --tcp-port 0 \
      --endpoint "$WIN_IP":14550 \
      --endpoint "$WIN_IP":14500 \
      --endpoint "$WIN_IP":14560 \
      --endpoint "$WIN_IP":14570 \
      --endpoint 192.168.105.60:15010 \
      --endpoint 192.168.105.205:16010 \
      127.0.0.1:5000 \
      127.0.0.1:6000
) &

wait
