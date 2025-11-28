#!/bin/bash
set -euo pipefail

usage() {
cat <<'EOF'
run_swarm.sh — unified launcher for multi-SITL profiles

USAGE:
  ./run_swarm.sh [PROFILE] [-n N]
  ./run_swarm.sh -h|--help

PROFILES:
  sim       : serial2 -> local UDP (5000, 6000, …). Router -> Windows $WIN_IP.
  sim_only  : like 'sim', but router exports 0.0.0.0:15000 (no Windows endpoints).
  rfd900    : serial2 -> UART (/dev/ttyUSB* per instance). Router reads from RFD device.
  pi        : serial2 -> udpclient:<PI_TARGETS_CSV>. Router also mirrors extra PI endpoints.

FLAGS:
  -n, --instances N    number of SITL instances (default 2)
  -h, --help           show this help

DEFAULT WINDOWS ENDPOINT PORTS (SIM/RFD900/PI):
  Generated from N using: 14500, 14550 + 10*k for k = 0..N  (i.e., N+1 ports)
  Examples: N=1 → 14500,14550,14560;  N=3 → 14500,14550,14560,14570,14580
  Override with: ROUTER_WIN_PORTS="14550,14555,..." (comma-separated)

ENV OVERRIDES (common):
  INSTANCES (2)  MODEL (plane)  SPEEDUP (5)
  ARDUPILOT_DIR ($HOME/ardupilot)
  BIN (…/build/sitl/bin/arduplane)
  DEFAULTS (…/Tools/autotest/models/plane.parm)
  HOME_COORDS ("40.3117414,44.455211099999985,1294.86,0.0")
  RUN_ROUTER (1|0)  ROUTER_BIN (mavlink-routerd)
  ROUTER_SYSID (255)
  WIN_IP (force Windows IP)  WSL_HOST_IP (alt WSL gw)

PROFILE ENVS:
  rfd900:  RFD_USB_LIST="/dev/ttyUSB0,/dev/ttyUSB1"  RFD_USB_ROUTER="/dev/ttyUSB2"  RFD_BAUD=115200
  pi:      PI_TARGETS_CSV="ip1:port1,ip2:port2,..."  PI_ROUTER_EXTRA_CSV="ipX:portX,ipY:portY,..."

PORT RULES (per instance i=1..N):
  serial0 (sim/sim_only): tcp:127.0.0.1:(5760 + 10*(i-1)):nowait
  serial1 (sim/sim_only/rfd900): tcp:$WIN_IP:(5762 + 10*(i-1))
  serial2 (sim/sim_only): udpclient:0.0.0.0:(5000 + 1000*(i-1))
  serial0 (rfd900): tcp:0.0.0.0:(5660 + 10*(i-1)):nowait
  serial2 (rfd900): uart from RFD_USB_LIST[i]
  serial0 (pi): tcp:0.0.0.0:(5760 + 10*(i-1)):nowait
  serial1 (pi): udpclient:0.0.0.0:(5000 + 1000*(i-1))
  serial2 (pi): udpclient:PI_TARGETS_CSV[i] (falls back to last if shorter)

EXAMPLES:
  ./run_swarm.sh sim -n 3
  RUN_ROUTER=0 ./run_swarm.sh sim_only -n 2
  WIN_IP=172.20.112.1 ./run_swarm.sh sim -n 4
  RFD_USB_LIST="/dev/ttyUSB3,/dev/ttyUSB4,/dev/ttyUSB5" ./run_swarm.sh rfd900 -n 3
  PI_TARGETS_CSV="192.168.105.60:15000,192.168.105.205:16000,192.168.105.88:17000" ./run_swarm.sh pi -n 3
EOF
}

# -------- Parse args --------
PROFILE="sim"
INSTANCES="${INSTANCES:-2}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    -n|--instances)
      [[ $# -lt 2 ]] && { echo "Missing value for $1"; exit 1; }
      INSTANCES="$2"; shift 2 ;;
    -d|--dist)
      [[ $# -lt 2 ]] && { echo "Missing value for $1"; exit 1; }
      DIST_M="$2"; shift 2 ;;
    sim|sim_only|rfd900|pi)
      PROFILE="$1"; shift ;;
    *)
      echo "Unknown arg: $1"; echo; usage; exit 1 ;;
  esac
done

# Validate INSTANCES
if ! [[ "$INSTANCES" =~ ^[0-9]+$ ]] || [[ "$INSTANCES" -lt 1 ]]; then
  echo "INSTANCES must be a positive integer (got: $INSTANCES)"; exit 1
fi

# -------- Config & defaults --------
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
BIN="${BIN:-$ARDUPILOT_DIR/build/sitl/bin/arduplane}"
DEFAULTS="${DEFAULTS:-$ARDUPILOT_DIR/Tools/autotest/models/plane.parm}"
HOME_COORDS="${HOME_COORDS:-40.3117414,44.455211099999985,1294.86,0.0}"
MODEL="${MODEL:-plane}"
SPEEDUP="${SPEEDUP:-5}"
RUN_ROUTER="${RUN_ROUTER:-1}"
ROUTER_BIN="${ROUTER_BIN:-mavlink-routerd}"
ROUTER_SYSID="${ROUTER_SYSID:-255}"
ROUTER_WIN_STEP="${ROUTER_WIN_STEP:-10}"

# Generate default Windows ports from N if not overridden:
generate_win_ports() {
  local n="$1" base=14550 step=10
  local -a out=(14500)
  # N+1 ports: k=0..N
  for ((k=0; k<=n; k++)); do out+=("$((base + step*k))"); done
  (IFS=','; echo "${out[*]}")
}
ROUTER_WIN_PORTS="${ROUTER_WIN_PORTS:-$(generate_win_ports "$INSTANCES" "$ROUTER_WIN_STEP")}"

# Generate default Windows ports from N if not overridden:
generate_win_ports() {
  local n="$1" base=14550 step=10
  local -a out=()
  # N+1 ports: k=0..N
  for ((k=0; k<=n; k++)); do out+=("$((base + step*k))"); done
  (IFS=','; echo "${out[*]}")
}
ROUTER_WIN_PORTS="${ROUTER_WIN_PORTS:-$(generate_win_ports "$INSTANCES")}"

# rfd900 specifics
RFD_USB_LIST="${RFD_USB_LIST:-/dev/ttyUSB0,/dev/ttyUSB1}"
RFD_USB_ROUTER="${RFD_USB_ROUTER:-/dev/ttyUSB2}"
RFD_BAUD="${RFD_BAUD:-115200}"

# pi specifics
PI_TARGETS_CSV="${PI_TARGETS_CSV:-192.168.105.60:15000,192.168.105.205:16000}"
PI_ROUTER_EXTRA_CSV="${PI_ROUTER_EXTRA_CSV:-192.168.105.60:15010,192.168.105.205:16010}"

# -------- Detect WSL & Windows host IP --------
detect_windows_ip() {
  local env="Linux" ip="127.0.0.1"
  if grep -qiE "(microsoft|wsl)" /proc/sys/kernel/osrelease 2>/dev/null || \
     grep -qi microsoft /proc/version 2>/dev/null; then
    env="WSL"
    ip="${WSL_HOST_IP:-$(ip -4 route show default 2>/dev/null | awk '{print $3; exit}')}"
    [[ -z "${ip:-}" ]] && ip="$(awk '/^nameserver /{print $2; exit}' /etc/resolv.conf 2>/dev/null || true)"
    [[ -z "${ip:-}" ]] && ip="127.0.0.1"
  fi
  echo "$env;$ip"
}

ENV_AND_IP="$(detect_windows_ip)"
ENVIRONMENT="${ENV_AND_IP%%;*}"
WIN_IP="${WIN_IP:-${ENV_AND_IP#*;}}"

echo "Profile: $PROFILE"
echo "Instances: $INSTANCES"
echo "Detected environment: $ENVIRONMENT"
echo "Using Windows IP: $WIN_IP"
echo "Router Windows ports: $ROUTER_WIN_PORTS"

# -------- Prep --------
IFS=',' read -r -a WIN_OUT_PORTS <<< "$ROUTER_WIN_PORTS"
IFS=',' read -r -a RFD_USB_DEVS <<< "$RFD_USB_LIST"
IFS=',' read -r -a PI_TARGETS <<< "$PI_TARGETS_CSV"
IFS=',' read -r -a PI_ROUTER_EXTRA <<< "$PI_ROUTER_EXTRA_CSV"

mkdir -p "$ARDUPILOT_DIR"
for i in $(seq 1 "$INSTANCES"); do mkdir -p "$ARDUPILOT_DIR/$i"; done

[[ -x "$BIN" ]] || { echo "Binary not found/executable: $BIN"; exit 1; }
[[ -f "$DEFAULTS" ]] || { echo "Defaults not found: $DEFAULTS"; exit 1; }
if [[ "$RUN_ROUTER" == "1" ]] && ! command -v "$ROUTER_BIN" >/dev/null 2>&1; then
  echo "Router not found in PATH: $ROUTER_BIN (set RUN_ROUTER=0 to skip)"; exit 1
fi
if [[ "$PROFILE" == "rfd900" ]] && [[ "${#RFD_USB_DEVS[@]}" -lt "$INSTANCES" ]]; then
  echo "rfd900: need at least $INSTANCES serial devices in RFD_USB_LIST (got ${#RFD_USB_DEVS[@]})."; exit 1
fi

cleanup() { pkill -P $$ || true; }
trap cleanup EXIT INT TERM

# -------- Port helpers --------
serial0_for() {
  local idx="$1"
  case "$PROFILE" in
    sim|sim_only)  echo "tcp:127.0.0.1:$((5760 + 10*(idx-1))):nowait" ;;
    rfd900)        echo "tcp:0.0.0.0:$((5660 + 10*(idx-1))):nowait" ;;
    pi)            echo "tcp:0.0.0.0:$((5760 + 10*(idx-1))):nowait" ;;
    *)             echo "tcp:127.0.0.1:$((5760 + 10*(idx-1))):nowait" ;;
  esac
}

serial1_for() {
  local idx="$1"
  case "$PROFILE" in
    sim|sim_only|rfd900) echo "tcp:$WIN_IP:$((5762 + 10*(idx-1)))" ;;
    pi)                  echo "udpclient:0.0.0.0:$((5000 + 1000*(idx-1)))" ;;
    *)                   echo "tcp:$WIN_IP:$((5762 + 10*(idx-1)))" ;;
  esac
}

serial2_for() {
  local idx="$1"
  case "$PROFILE" in
    sim|sim_only)  echo "udpclient:0.0.0.0:$((5000 + 1000*(idx-1)))" ;;
    rfd900)        echo "uart:${RFD_USB_DEVS[$((idx-1))]}:$RFD_BAUD" ;;
    pi)
      local sel="${PI_TARGETS[$((idx-1))]:-${PI_TARGETS[-1]}}"
      echo "udpclient:${sel}"
      ;;
    *)             echo "udpclient:0.0.0.0:$((5000 + 1000*(idx-1)))" ;;
  esac
}

home_coords() {
  local i="$1"  
  # local lat_base=40.3103 # ScyClub
  # local lon_base=44.4392 # ScyClub
  local lat_base=40.3117414
  local lon_base=44.455211099999985
  local alt_base=1294.86

  # Parameters
  local DIST_M="${DIST_M:-2}"
  local PER_ROW="${PER_ROW:-3}"

  # Convert meters → degrees
  local lat_step lon_step
  lat_step=$(awk -v d="$DIST_M" 'BEGIN {printf "%.8f", d / 111111}')
  lon_step=$(awk -v d="$DIST_M" -v lat="$lat_base" 'BEGIN {printf "%.8f", d / (111111 * cos(lat * 3.14159 / 180))}')

  # Grid positions
  local idx=$((i - 1))
  local row=$(( idx / PER_ROW ))
  local col=$(( idx % PER_ROW ))

  local lat=$(awk -v b="$lat_base" -v s="$lat_step" -v r="$row" 'BEGIN {printf "%.8f", b + r * s}')
  local lon=$(awk -v b="$lon_base" -v s="$lon_step" -v c="$col" 'BEGIN {printf "%.8f", b + c * s}')

  echo "$lat,$lon,$alt_base,0.0"
}

build_router_args() {
  local args=( -v -s "$ROUTER_SYSID" --tcp-port 0 )
  case "$PROFILE" in
    sim)
      for p in "${WIN_OUT_PORTS[@]}"; do args+=( --endpoint "$WIN_IP:$p" ); done
      for i in $(seq 1 "$INSTANCES"); do args+=( "127.0.0.1:$((5000 + 1000*(i-1)))" ); done
      ;;
    sim_only)
      args+=( --endpoint "0.0.0.0:15000" )
      for i in $(seq 1 "$INSTANCES"); do args+=( "127.0.0.1:$((5000 + 1000*(i-1)))" ); done
      ;;
    rfd900)
      for p in "${WIN_OUT_PORTS[@]}"; do args+=( --endpoint "$WIN_IP:$p" ); done
      args+=( "$RFD_USB_ROUTER:$RFD_BAUD" )
      ;;
    pi)
      for p in "${WIN_OUT_PORTS[@]}"; do args+=( --endpoint "$WIN_IP:$p" ); done
      for e in "${PI_ROUTER_EXTRA[@]}"; do [[ -n "$e" ]] && args+=( --endpoint "$e" ); done
      for i in $(seq 1 "$INSTANCES"); do args+=( "127.0.0.1:$((5000 + 1000*(i-1)))" ); done
      ;;
  esac
  printf '%s\0' "${args[@]}"
}

# -------- Launch SITL --------
for i in $(seq 1 "$INSTANCES"); do
  (
    cd "$ARDUPILOT_DIR/$i"
    idx=$(( i - 1 ))
    echo "Starting SITL #$i (sysid=$i, -I$idx)"
  
    "$BIN" \
      -S --model "$MODEL" --speedup "$SPEEDUP" --slave 0 \
      --defaults "$DEFAULTS" \
      --serial0="$(serial0_for "$i")" \
      --serial1="$(serial1_for "$i")" \
      --serial2="$(serial2_for "$i")" \
      --sim-address=127.0.0.1 -I"$idx" \
      --home "$(home_coords "$i")" \
      --sysid "$i"
  ) &
done

# -------- Launch router --------
if [[ "$RUN_ROUTER" == "1" ]]; then
  echo "Starting mavlink-routerd..."
  mapfile -d '' ROUTER_ARGS < <(build_router_args)
  "$ROUTER_BIN" "${ROUTER_ARGS[@]}" &
fi

wait
