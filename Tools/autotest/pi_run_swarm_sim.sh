#!/bin/bash
mkdir -p ~/ardupilot/{1,2}

(
  cd ~/ardupilot/1
  /home/gart/ardupilot/build/sitl/bin/arduplane \
      -S --model plane --speedup 5 \
      --serial0=tcp:0.0.0.0:5760:nowait \
      --serial1=udpclient:0.0.0.0:5000 \
      --serial2=udpclient:192.168.105.60:15000 \
      --sim-address=127.0.0.1 -I0 \
      --home 40.3117414,44.455211099999985,1294.86,0 \
      --sysid 1
) &
(
  cd ~/ardupilot/2
  /home/gart/ardupilot/build/sitl/bin/arduplane \
      -S --model plane --speedup 5 \
      --serial0=tcp:0.0.0.0:5770:nowait \
      --serial1=udpclient:0.0.0.0:6000 \
      --serial2=udpclient:192.168.105.205:16000 \
      --sim-address=127.0.0.1 -I1 \
      --home 40.3117414,44.455211099999985,1294.86,0 \
      --sysid 2
) &
(
  mavlink-routerd -v \
      --tcp-port 0 \
      --endpoint 172.21.176.1:14550 \
      --endpoint 172.21.176.1:14500 \
      --endpoint 172.21.176.1:14560 \
      --endpoint 172.21.176.1:14570 \
      --endpoint 192.168.105.60:15010 \
      --endpoint 192.168.105.205:16010 \
      127.0.0.1:5000 \
      127.0.0.1:6000
) &

wait
