#!/bin/bash

WIN_IP="${1:-}"

# Auto-detect Windows host IP if not provided (WSL)
if [[ -z "$WIN_IP" ]]; then
  if grep -qi microsoft /proc/version; then
    WIN_IP=$(awk '/nameserver/ {print $2; exit}' /etc/resolv.conf)
  else
    echo "Usage: $0 <windows_ip>"
    exit 1
  fi
fi

echo "Using Windows IP: $WIN_IP"

mkdir -p ~/ardupilot/{1,2}

(
  cd ~/ardupilot/1
  ~/ardupilot/build/sitl/bin/arduplane \
    -S --model plane --speedup 5 \
    --serial0=tcp:0.0.0.0:5660:nowait \
    --serial1=tcp:${WIN_IP}:5762 \
    --serial2=uart:/dev/ttyUSB0:115200 \
    --sim-address=127.0.0.1 -I0 \
    --home 40.3117414,44.4552111,1294.86,0 \
    --sysid 1
) &

(
  cd ~/ardupilot/2
  ~/ardupilot/build/sitl/bin/arduplane \
    -S --model plane --speedup 5 \
    --serial0=tcp:0.0.0.0:5670:nowait \
    --serial1=tcp:${WIN_IP}:5772 \
    --serial2=uart:/dev/ttyUSB1:115200 \
    --sim-address=127.0.0.1 -I1 \
    --home 40.3117414,44.4552111,1294.86,0 \
    --sysid 2
) &

(
  mavlink-routerd -v -s 255 \
    --tcp-port 0 \
    --endpoint ${WIN_IP}:14550 \
    --endpoint ${WIN_IP}:14500 \
    --endpoint ${WIN_IP}:14560 \
    --endpoint ${WIN_IP}:14570 \
    /dev/ttyUSB2:115200
) &

wait
