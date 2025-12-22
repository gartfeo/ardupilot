#!/bin/bash
set -euo pipefail

# HARDWARE TEST VERSION (non-configurable):
# - SITL #1 serial2 -> /dev/ttyUSB0 @115200
# - SITL #2 serial2 -> /dev/ttyUSB1 @115200
# - mavlink-routerd reads from /dev/ttyUSB2 @115200
# - Windows IP auto-detected in WSL (or pass as arg)

WIN_IP="${1:-}"

# ---- Detect Windows host IP in WSL if not provided ----
if [[ -z "$WIN_IP" ]]; then
  if grep -qiE "(microsoft|wsl)" /proc/sys/kernel/osrelease 2>/dev/null || \
     grep -qi microsoft /proc/version 2>/dev/null; then
    WIN_IP="$(ip -4 route show default 2>/dev/null | awk '{print $3; exit}' || true)"
    [[ -z "${WIN_IP:-}" ]] && WIN_IP="$(awk '/^nameserver /{print $2; exit}' /etc/resolv.conf 2>/dev/null || true)"
  fi
fi

if [[ -z "${WIN_IP:-}" ]]; then
  echo "ERROR: Windows IP not detected. Pass it explicitly: $0 <WIN_IP>"
  exit 1
fi

echo "Using Windows IP: $WIN_IP"

# ---- ArduPilot paths ----
ARDUPILOT_DIR="$HOME/ardupilot"
BIN="$ARDUPILOT_DIR/build/sitl/bin/arduplane"

if [[ ! -x "$BIN" ]]; then
  echo "ERROR: SITL binary not found/executable: $BIN"
  echo "Build it with:"
  echo "  cd $ARDUPILOT_DIR"
  echo "  ./waf configure --board sitl"
  echo "  ./waf plane"
  exit 1
fi

# ---- Hardware devices (hardcoded) ----
UART_BAUD=115200
UART1=/dev/ttyUSB0
UART2=/dev/ttyUSB1
ROUTER_UART=/dev/ttyUSB2

for dev in "$UART1" "$UART2" "$ROUTER_UART"; do
  if [[ ! -e "$dev" ]]; then
    echo "ERROR: Missing device: $dev"
    echo "Check: ls -l /dev/ttyUSB*"
    exit 1
  fi
done

mkdir -p "$ARDUPILOT_DIR"/{1,2}

# Optional: kill children on exit
cleanup() { pkill -P $$ 2>/dev/null || true; }
trap cleanup EXIT INT TERM

# ---- SITL instance 1 ----
(
  cd "$ARDUPILOT_DIR/1"
  "$BIN" \
      -S --model plane --speedup 5 \
      --serial0=tcp:0.0.0.0:5660:nowait \
      --serial1=tcp:${WIN_IP}:5762 \
      --serial2=uart:${UART1}:${UART_BAUD} \
      --sim-address=127.0.0.1 -I0 \
      --home 40.3117414,44.455211099999985,1294.86,0 \
      --sysid 1
) &

# ---- SITL instance 2 ----
(
  cd "$ARDUPILOT_DIR/2"
  "$BIN" \
      -S --model plane --speedup 5 \
      --serial0=tcp:0.0.0.0:5670:nowait \
      --serial1=tcp:${WIN_IP}:5772 \
      --serial2=uart:${UART2}:${UART_BAUD} \
      --sim-address=127.0.0.1 -I1 \
      --home 40.3117414,44.455211099999985,1294.86,0 \
      --sysid 2
) &

# ---- Router: read from ground radio, forward to Windows ports ----
(
  mavlink-routerd -v -s 255 \
      --tcp-port 0 \
      --endpoint ${WIN_IP}:14550 \
      --endpoint ${WIN_IP}:14500 \
      --endpoint ${WIN_IP}:14560 \
      --endpoint ${WIN_IP}:14570 \
      ${ROUTER_UART}:${UART_BAUD}
) &

wait
