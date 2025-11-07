#!/bin/bash
mkdir -p ~/ardupilot/{1,2}

(
  cd ~/ardupilot/1
  /home/gart/ardupilot/build/sitl/bin/arduplane \
      -S --model plane --speedup 5 \
      --serial0=tcp:0.0.0.0:5660:nowait \
      --serial1=tcp:172.21.176.1:5762 \
      --serial2=uart:/dev/ttyUSB0:115200 \
      --sim-address=127.0.0.1 -I0 \
      --home 40.3117414,44.455211099999985,1294.86,0 \
      --sysid 1
) &
(
  cd ~/ardupilot/2
  /home/gart/ardupilot/build/sitl/bin/arduplane \
      -S --model plane --speedup 5 \
      --serial0=tcp:0.0.0.0:5670:nowait \
      --serial1=tcp:172.21.176.1:5772 \
      --serial2=uart:/dev/ttyUSB1:115200 \
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
      /dev/ttyUSB2:115200
) &

wait
