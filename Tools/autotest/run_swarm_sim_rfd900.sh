#!/bin/bash
set -euo pipefail

# SITL Swarm Simulation with RFD900 radios
#
# All settings configurable via environment variables:
#   WIN_IP         - Windows host IP (auto-detected in WSL)
#   NUM_INSTANCES  - Number of SITL instances (default: 2)
#   ARDUPILOT_DIR  - ArduPilot directory (default: $HOME/ardupilot)
#   MODEL          - Vehicle model (default: plane)
#   SPEEDUP        - Simulation speedup (default: 5)
#   HOME_COORDS    - Home coordinates (default: Armenia)
#   UART_BAUD      - UART baud rate (default: 115200)
#   USE_UARTS      - Enable hardware UARTs (default: true)
#   ROUTER_UART    - Router UART device (default: /dev/ttyUSB{NUM_INSTANCES})
#
# Usage: $0 [WIN_IP] [NUM_INSTANCES]
#   or set environment variables before running

WIN_IP="${1:-${WIN_IP:-}}"
NUM_INSTANCES="${2:-${NUM_INSTANCES:-2}}"

# ---- Detect Windows host IP in WSL if not provided ----
if [[ -z "$WIN_IP" ]]; then
  if grep -qiE "(microsoft|wsl)" /proc/sys/kernel/osrelease 2>/dev/null || \
     grep -qi microsoft /proc/version 2>/dev/null; then
    WIN_IP="$(ip -4 route show default 2>/dev/null | awk '{print $3; exit}' || true)"
    [[ -z "${WIN_IP:-}" ]] && WIN_IP="$(awk '/^nameserver /{print $2; exit}' /etc/resolv.conf 2>/dev/null || true)"
  fi
fi

if [[ -z "${WIN_IP:-}" ]]; then
  echo "ERROR: Windows IP not detected. Pass it explicitly: $0 <WIN_IP> [NUM_INSTANCES]"
  exit 1
fi

if ! [[ "$NUM_INSTANCES" =~ ^[0-9]+$ ]] || [[ "$NUM_INSTANCES" -lt 1 ]]; then
  echo "ERROR: NUM_INSTANCES must be a positive integer (got: $NUM_INSTANCES)"
  exit 1
fi

echo "Using Windows IP: $WIN_IP"
echo "Number of instances: $NUM_INSTANCES"

# ---- Configuration (env vars with defaults) ----
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
MODEL="${MODEL:-plane}"
SPEEDUP="${SPEEDUP:-5}"
HOME_COORDS="${HOME_COORDS:-40.3117414,44.455211099999985,1294.86,0.0}"
UART_BAUD="${UART_BAUD:-115200}"
USE_UARTS="${USE_UARTS:-true}"

BIN="$ARDUPILOT_DIR/build/sitl/bin/arduplane"
DEFAULTS="$ARDUPILOT_DIR/Tools/autotest/models/plane.parm"

if [[ ! -x "$BIN" ]]; then
  echo "ERROR: SITL binary not found/executable: $BIN"
  echo "Build it with:"
  echo "  cd $ARDUPILOT_DIR"
  echo "  ./waf configure --board sitl"
  echo "  ./waf plane"
  exit 1
fi

if [[ ! -f "$DEFAULTS" ]]; then
  echo "ERROR: Defaults not found: $DEFAULTS"
  exit 1
fi

echo "Using SITL BIN: $BIN"
echo "Defaults: $DEFAULTS"
echo "Model: $MODEL  Speedup: $SPEEDUP"
echo "Home: $HOME_COORDS"

# ---- Hardware devices (optional) ----
ROUTER_UART="${ROUTER_UART:-/dev/ttyUSB${NUM_INSTANCES}}"

if [[ "$USE_UARTS" == "true" ]]; then
  # Check required UARTs for each instance
  for i in $(seq 0 $((NUM_INSTANCES - 1))); do
    dev="/dev/ttyUSB${i}"
    if [[ ! -e "$dev" ]]; then
      echo "ERROR: Missing device: $dev (needed for instance $((i + 1)))"
      echo "Check: ls -l /dev/ttyUSB*"
      exit 1
    fi
  done
  echo "Using hardware UARTs: /dev/ttyUSB0 - /dev/ttyUSB$((NUM_INSTANCES - 1))"
else
  echo "Hardware UARTs disabled (USE_UARTS=false)"
fi

# Create instance directories
for i in $(seq 1 "$NUM_INSTANCES"); do
  mkdir -p "$ARDUPILOT_DIR/$i"
done

cleanup() { pkill -P $$ 2>/dev/null || true; }
trap cleanup EXIT INT TERM

# ---- Launch SITL instances ----
for i in $(seq 1 "$NUM_INSTANCES"); do
  idx=$((i - 1))
  serial0_port=$((5660 + idx * 10))
  serial1_port=$((5762 + idx * 10))

  # Build serial2 argument (UART or disabled)
  if [[ "$USE_UARTS" == "true" ]]; then
    uart_dev="/dev/ttyUSB${idx}"
    serial2_arg="--serial2=uart:${uart_dev}:${UART_BAUD}"
    echo "Starting SITL instance $i (sysid=$i, serial0=:$serial0_port, serial1=:$serial1_port, uart=$uart_dev)"
  else
    serial2_arg=""
    echo "Starting SITL instance $i (sysid=$i, serial0=:$serial0_port, serial1=:$serial1_port)"
  fi

  (
    cd "$ARDUPILOT_DIR/$i"
    "$BIN" \
        -S --model "$MODEL" --speedup "$SPEEDUP" --slave 0 \
        --defaults "$DEFAULTS" \
        --serial0=tcp:0.0.0.0:${serial0_port}:nowait \
        --serial1=tcp:${WIN_IP}:${serial1_port} \
        $serial2_arg \
        --sim-address=127.0.0.1 -I${idx} \
        --home "$HOME_COORDS" \
        --sysid $i
  ) &
done

# ---- Router: read from ground radio, forward to Windows ports (optional) ----
if [[ -e "$ROUTER_UART" ]]; then
  echo "Starting mavlink-routerd on $ROUTER_UART"
  (
    mavlink-routerd -v -s 255 \
        --tcp-port 0 \
        --endpoint ${WIN_IP}:14550 \
        --endpoint ${WIN_IP}:14500 \
        --endpoint ${WIN_IP}:14560 \
        --endpoint ${WIN_IP}:14570 \
        ${ROUTER_UART}:${UART_BAUD}
  ) &
else
  echo "Router UART ($ROUTER_UART) not found - skipping mavlink-routerd"
fi

wait
