#!/bin/bash

mkdir -p ~/ardupilot/1 ~/ardupilot/2

# instance 1
(
  cd ~/ardupilot/1
  /home/gart/ardupilot/build/sitl/bin/arduplane \
    -S --model plane --speedup 5 \
    --serial0=udpclient:0.0.0.0:5000 \
    --sim-address=127.0.0.1 -I0 \
    --home 40.3117414,44.455211099999985,1294.86,0 \
    --sysid 1
) &

# instance 2
(
  cd ~/ardupilot/2
  /home/gart/ardupilot/build/sitl/bin/arduplane \
    -S --model plane --speedup 5 \
    --serial0=udpclient:0.0.0.0:6000 \
    --sim-address=127.0.0.1 -I1 \
    --home 40.3117414,44.455211099999985,1294.86,0 \
    --sysid 2
) &

# mavlink-router
(
  cd ~/mavlink-router
  mavlink-routerd -c srouter.conf -v
) &
