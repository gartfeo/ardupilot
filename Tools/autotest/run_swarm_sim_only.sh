#!/bin/bash
set -euo pipefail

ARDUPILOT_DIR="$HOME/ardupilot"
BIN="$ARDUPILOT_DIR/build/sitl/bin/arduplane"
DEFAULTS="$ARDUPILOT_DIR/Tools/autotest/models/plane.parm"
HOME_COORDS="40.3117414,44.455211099999985,1294.86,0.0"

mkdir -p "$ARDUPILOT_DIR"/{1,2}

# --- Detect if running under WSL and compute Windows host IP ---
if grep -qiE "(microsoft|wsl)" /proc/sys/kernel/osrelease 2>/dev/null || \
   grep -qi microsoft /proc/version 2>/dev/null; then
    ENVIRONMENT="WSL"
    # Prefer default gateway (correct Windows host IP in WSL2)
    WIN_IP="${WSL_HOST_IP:-$(ip -4 route show default 2>/dev/null | awk '{print $3; exit}')}"
    # Fallback to resolv.conf if needed
    if [[ -z "${WIN_IP:-}" ]]; then
        WIN_IP="$(awk '/^nameserver /{print $2; exit}' /etc/resolv.conf 2>/dev/null || true)"
    fi
else
    ENVIRONMENT="Linux"
    WIN_IP="127.0.0.1"
fi

echo "Detected environment: $ENVIRONMENT"
echo "Using Windows IP: ${WIN_IP:-unknown}"

(
  cd "$ARDUPILOT_DIR/1"
  "$BIN" \
      -S --model plane --speedup 5 --slave 0 \
      --defaults "$DEFAULTS" \
      --serial0=tcp:127.0.0.1:5760:nowait \
      --serial1=tcp:"$WIN_IP":5762 \
      --serial2=udpclient:0.0.0.0:5000 \
      --sim-address=127.0.0.1 -I0 \
      --home "$HOME_COORDS" \
      --sysid 1
) &

(
  cd "$ARDUPILOT_DIR/2"
  "$BIN" \
      -S --model plane --speedup 5 --slave 0 \
      --defaults "$DEFAULTS" \
      --serial0=tcp:127.0.0.1:5760:nowait \
      --serial1=tcp:"$WIN_IP":5772 \
      --serial2=udpclient:0.0.0.0:6000 \
      --sim-address=127.0.0.1 -I1 \
      --home "$HOME_COORDS" \
      --sysid 2
) &
(
  mavlink-routerd -v \
      --tcp-port 0 \
      --endpoint 0.0.0.0:15000 \
      127.0.0.1:5000 \
      127.0.0.1:6000
) &

wait
