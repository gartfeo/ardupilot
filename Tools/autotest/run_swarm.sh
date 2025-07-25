#!/bin/bash
set -e                          # stop on first error
trap "pkill -f mavlink-routerd; pkill -f arduplane" EXIT

mkdir -p ~/ardupilot/{1,2}

(
  cd ~/ardupilot/1
  /home/gart/ardupilot/build/sitl/bin/arduplane \
      -S --model plane --speedup 5 \
      --serial0=udpclient:0.0.0.0:5000 \
      --sim-address=127.0.0.1 -I0 \
      --home 40.3117414,44.455211099999985,1294.86,0 \
      --sysid 1
) &
(
  cd ~/ardupilot/2
  /home/gart/ardupilot/build/sitl/bin/arduplane \
      -S --model plane --speedup 5 \
      --serial0=udpclient:0.0.0.0:6000 \
      --sim-address=127.0.0.1 -I1 \
      --home 40.3117414,44.455211099999985,1294.86,0 \
      --sysid 2
) &
(
  cd ~/mavlink-router
  mavlink-routerd \
      -t 0 \
      -l /tmp/flightstack-logs \
      -v \
      -e 172.21.176.1:14550 \
      -e 172.21.176.1:14500 \
      -e 172.21.176.1:14560 \
      -e 172.21.176.1:14570 \
      127.0.0.1:5000 \
      127.0.0.1:6000 
) &

wait            # keep script alive until everything exits
