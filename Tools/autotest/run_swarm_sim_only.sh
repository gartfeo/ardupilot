#!/bin/bash
mkdir -p ~/ardupilot/{1,2}

(
  cd ~/ardupilot/1
  /home/gart/ardupilot/build/sitl/bin/arduplane \
      -S --model plane --speedup 5 --slave 0 \
      --defaults ../Tools/autotest/models/plane.parm \
      --serial0=tcp:127.0.0.1:5760:nowait \
      --serial1=tcp:172.21.176.1:5762 \
      --serial2=udpclient:0.0.0.0:5000 \
      --sim-address=127.0.0.1 -I0 \
      --home 40.3117414,44.455211099999985,1294.86,0.0 \
      --sysid 1
) &
(
  cd ~/ardupilot/2
  /home/gart/ardupilot/build/sitl/bin/arduplane \
      -S --model plane --speedup 5  --slave 0 \
      --defaults ../Tools/autotest/models/plane.parm \
      --serial0=tcp:127.0.0.1:5760:nowait \
      --serial1=tcp:172.21.176.1:5772 \
      --serial2=udpclient:0.0.0.0:6000 \
      --sim-address=127.0.0.1 -I1 \
      --home 40.3117414,44.455211099999985,1294.86,0 \
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
