# Pixhawk 6X HIL firmware

Hardware-in-the-loop firmware for the Holybro Pixhawk 6X. ArduPlane runs on the
real STM32H7, but its IMU, barometer, compass, GPS and airspeed sensor are
ArduPilot's simulated sensor backends. They are fed from the states a PC
simulator returns over USB. The board sends its servo outputs to the simulator
once per main loop, and the real outputs stay dead.

The link, timing rules, status texts and log fields are described in
`libraries/SITL/examples/HIL/readme.md`. The reference simulator is
`Tools/autotest/pysim/hil_bridge.py`. The HIL link is `SERIALn_PROTOCOL` 100
(SIM HIL).

> **Before you start:** disconnect every ESC, servo and other actuator from
> MAIN, AUX and CAN, and remove the propellers. The firmware gates every output
> (see "Safety"), but HIL is bench work on a bare board.

## Ports

The board enumerates as two USB serial ports (two CDC interfaces on one USB
connection):

| USB interface | ArduPilot port | Use | On Windows |
|---|---|---|---|
| OTG1, first CDC (`MI_00`) | SERIAL0 | MAVLink: Mission Planner or another GCS | the board's first COM port |
| OTG2, second CDC (`MI_02`) | SERIAL8 | HIL link to the simulator (`SERIAL8_PROTOCOL` 100, the hwdef default) | the second COM port; may show as "ArduPilot SLCAN" |

The "SLCAN" label comes from the Windows USB driver installed with Mission
Planner, which names the board's second CDC interface for its usual SLCAN use.
In this firmware that interface carries the HIL link. Do not set
`CAN_SLCAN_SERNUM` to 8.

COM numbers differ between PCs; on the bench PC today OTG1 is COM15 and OTG2 is
COM11. To tell them apart, open Device Manager > Ports (COM & LPT) > the port >
Properties > Details > Hardware Ids. The interface number is `MI_00` for OTG1
and `MI_02` for OTG2 (verify this on your PC once).

## Building

The board is built from this fork's worktree in WSL. From Windows PowerShell:

```powershell
wsl bash -lc "cd ~/fw-worktrees/ardupilot-sim-hil; ./waf configure --board Pixhawk6X-HIL; ./waf plane"
```

The firmware is `build/Pixhawk6X-HIL/bin/arduplane.apj`. Copy it to Windows
(adjust the distribution and user names):

```powershell
Copy-Item \\wsl.localhost\Ubuntu-24.04\home\gart\fw-worktrees\ardupilot-sim-hil\build\Pixhawk6X-HIL\bin\arduplane.apj $env:USERPROFILE\Downloads\arduplane-hil.apj
```

The firmware keeps the board's APJ board id (53), so Mission Planner accepts
it for a Pixhawk 6X. It does not carry or replace the bootloader.

## Flashing

1. **Back up the parameters first**, while the flight firmware is still on the
   board: Mission Planner > CONFIG > Full Parameter List > Save to file.
2. Turn safety **on** (the safety button LED flashes) before flashing, and leave
   it on until step 4.
3. Mission Planner > SETUP > Install Firmware > **Load custom firmware**, and
   pick `arduplane-hil.apj`. Mission Planner disconnects, reboots the board into
   its bootloader and flashes it over OTG1.
4. **Power-cycle the board**: unplug USB and every other power source, wait a
   few seconds, plug it back in. A reflash or reboot resets only the main
   processor (FMU). The IO coprocessor (IOMCU) that drives MAIN 1-8 stays
   powered and keeps what the previous firmware set up: its failsafe PWM, its
   RC mixer, its GPIO outputs and its SBUS output. With safety off, it runs
   them from the moment the FMU stops talking to it until the HIL firmware has
   booted and cleared them (see "Safety"). A power cycle restarts the IOMCU
   with safety on and nothing set up.
5. Reset the parameters, so the HIL defaults apply. Defaults only replace
   parameters that are not saved on the board. Use CONFIG > Full Parameter
   List > Reset to Default, or set `FORMAT_VERSION` to 0 and reboot.
6. Set `SIM_OPOS_LAT`, `SIM_OPOS_LNG`, `SIM_OPOS_ALT` and `SIM_OPOS_HDG` to the
   simulator's origin, then reboot.

## Parameters

The HIL defaults (`hwdef/include/HIL.inc` and `hwdef/include/HIL.parm`) set
the following:

| Parameter | Value | Why |
|---|---|---|
| `SERIAL8_PROTOCOL` | 100 | OTG2 is the HIL link. Set in the hwdef (`HAL_OTG2_PROTOCOL`), not in a defaults file. |
| `SIM_OH_MASK` | 0 | No real output is driven. |
| `GPS1_TYPE` | 100 | Simulated GPS (otherwise it waits for a real one). |
| `ARSPD_TYPE` | 100 | Simulated airspeed sensor. |
| `BATT_MONITOR` | 0 | A battery monitor would read the real ADC. |
| `BRD_HEAT_TARG` | -1 | The heater would chase the simulated IMU temperature. |
| `INS_ACC*OFFS_*`, `INS_ACC*SCAL_*` | 0.001, 1.001 | Mark the simulated accelerometers as calibrated. |

`SIM_OPOS_*` must match the bridge's `--origin`. The bridge sends absolute
positions, while the board takes its home and its position origin from
`SIM_OPOS_*`. Until the first state arrives, the board sits at `SIM_OPOS_*`.
If the two disagree, the vehicle jumps when the first state arrives, and its
local position is offset from home.

Aircraft-specific settings (servo functions, airspeeds, takeoff and mission
parameters) belong to your simulator's parameter file, not to this firmware.
So does the loop rate: ArduPlane's default `SCHED_LOOP_RATE` is 50 Hz, and the
link and the reference bridge are sized for 200 Hz (`SCHED_LOOP_RATE` 200, or
pass the bridge `--loop-rate-hz` with your rate).

## Running a session

1. Bare board, USB connected, flashed and reset as above.
2. Start the simulator bridge on the **OTG2** COM port. The COM port lives on
   Windows, so run the reference bridge with Windows Python (with `pymavlink`
   and `pyserial` installed). The script can run straight from the WSL
   worktree:

   ```powershell
   python \\wsl.localhost\Ubuntu-24.04\home\gart\fw-worktrees\ardupilot-sim-hil\Tools\autotest\pysim\hil_bridge.py --device COM11 --model static --origin -35.363261,149.165230,584,353
   ```

3. Connect Mission Planner to the **OTG1** COM port (see below).
4. Watch the messages. `HIL: waiting for simulator on SERIAL8` when the port
   opens, then `HIL: first state received`. Arming is refused with
   `PreArm: HIL: no simulator state` until the first state arrives, and with
   `PreArm: HIL: no simulator state (link lost)` whenever the link is lost.
   Link loss and recovery are reported once each:
   `HIL: simulator state lost, holding the last one` (CRITICAL) and
   `HIL: simulator state recovered` (WARNING). If no port has protocol 100 the
   board says `HIL: no port, set SERIALn_PROTOCOL 100`.
5. To arm, the IOMCU safety switch must be off. With no actuators connected,
   press the safety button. Do not set `BRD_SAFETY_DEFLT` 0 on a board wired to
   an airframe.

The board does not wait for the simulator. It boots, runs and logs without
one, and its pre-arm check refuses arming. Ways around the pre-arm checks
also get around this one (see "Safety"); a run armed that way is invalid.
Rebooting the board makes both COM ports disappear for a few seconds. The
reference bridge reopens its port and re-initialises the simulation, because
board time goes back.

## Mission Planner

- Connect to the OTG1 COM port only (any baud rate; USB ignores it). Never open
  the OTG2 port in Mission Planner or another tool while the bridge uses it: one
  port has one owner.
- When a HITL runner holds OTG1 for the whole run, for example navpy's runner,
  connect Mission Planner over UDP instead: connection type UDP, Connect, local
  port **9640**. That is the runner's mirror of its board link. It drops
  Mission Planner's RC overrides, manual control, stream-rate requests and
  `SYSID_MYGCS` writes, and passes everything else.
- **Never use Mission Planner's legacy Simulation tab.** It speaks the old HIL
  protocol that ArduPilot removed in 2021. It would send its own HIL messages
  and fight the simulator.

## Safety

With `SIM_OH_MASK` 0, from the moment the HIL firmware has started its IOMCU
thread, the firmware drives no physical actuator output of any kind
(`SITL::OutputGate`, `libraries/SITL/SIM_OutputGate.h`):

- FMU PWM: a channel outside `SIM_OH_MASK` is written as 0 (no pulse).
- IOMCU MAIN outputs: the IOMCU keeps receiving PWM frames of 0, so it never
  enters its FMU-failsafe mixer. Its failsafe PWM is 0, IOMCU mixing is
  refused, and a failsafe page, mixer or GPIO setup left by earlier firmware
  is reset at start-up.
- DShot: a channel outside `SIM_OH_MASK` gets no DShot frame and no DShot
  command (beeps, LEDs, 3D or reverse mode, telemetry), on the FMU and the
  IOMCU.
- GPIO: a servo channel outside `SIM_OH_MASK` is never set up as a GPIO
  (`SERVOn_FUNCTION` -1), on the FMU or the IOMCU, so relays, the advanced
  failsafe pins and Lua cannot drive it, and it cannot be used as a GPIO
  input either. If `SIM_OH_MASK` loses a channel after boot, its next GPIO
  write drives it low; reboot after lowering `SIM_OH_MASK`. Relays also need
  their instance in `SIM_OH_RELAY_MSK` (default 0 on a board).
- Non-PWM drivers: DroneCAN servo and ESC commands are masked by
  `SIM_OH_MASK` and DroneCAN relay (hardpoint) commands by
  `SIM_OH_RELAY_MSK`, the IOMCU SBUS output is refused and cleared, and Volz,
  SBUS servo out, Robotis, FETtec, KDECAN, PiccoloCAN, Torqeedo and the Hirth
  EFI are not built.

Before that point the firmware cannot gate anything. After a reflash or an
FMU-only reboot the IOMCU runs what the previous firmware left (failsafe PWM,
RC mixer, GPIO and SBUS outputs) whenever safety is off. That is why
"Flashing" asks for safety on and a power cycle.

This has been checked by building and reading the code and by SITL tests of
the rules (`test_sim_output_gate`), not yet on this board. Bench-check it once
per firmware build, before trusting it:

1. Power-cycle the board with safety on. Put a scope or servo tester on
   MAIN 1-8 and AUX 1-8.
2. Turn safety off, arm, apply full RC throttle, move the sticks, then stop
   the bridge. There must be no pulses at any point, and the IO amber LED
   must not show IOMCU failsafe.
3. Repeat with one AUX output in DShot mode (`SERVO_BLH_MASK`,
   `SERVO_BLH_OTYPE`) and one `SERVOn_FUNCTION` -1 used as a relay pin
   (`RELAYn_PIN`, with the relay allowed in `SIM_OH_RELAY_MSK` so that only
   the output gate stops it). Toggle the relay and trigger a DShot beep
   (arming beeps): still no activity on either pin. Reset these parameters
   afterwards.
4. With `LOG_DISARMED` 1, check the `PM` log record (`MaxT`, `NLon`) while
   the bridge runs: the HIL link's parsing runs at the end of every main loop
   and must not make loops overrun (see "Reading the HIL log").

Rules that hold whatever the firmware does:

- No actuators, no propellers.
- Never set `SIM_OH_MASK` non-zero on a board wired to an airframe.
- Keep `CAN_SLCAN_SERNUM` away from SERIAL8.

The arming gate is a pre-arm check (an auxiliary authorisation). It decides
whether a run is valid, not whether outputs are safe, and the usual ways
around pre-arm checks get around it: a forced arm (`MAV_CMD_COMPONENT_ARM_DISARM`
with param2 21196), `ARMING_CHECK` 0 or `ARMING_REQUIRE` 0 on Plane, and
re-arming after a watchdog reset that happened while armed. A run armed that
way, or armed while the link was not ok, is invalid; the PC-side run gate must
fail it.

## Going back to flight firmware

Parameters saved under the HIL firmware stay on the board when flight firmware
is flashed. These include `GPS1_TYPE` 100, `ARSPD_TYPE` 100, `BATT_MONITOR` 0,
`SERIAL8_PROTOCOL`, the `SIM_*` values and the fake accelerometer calibration
markers. The flight firmware would then wait for a simulated GPS and trust a
calibration that never happened.

1. Flash the normal Pixhawk 6X ArduPlane firmware (SETUP > Install Firmware).
2. Reset the parameters to default and reboot.
3. Load the backup from "Flashing" step 1 (Full Parameter List > Load from
   file > Write Params) and reboot.
4. Check the accelerometer and compass calibrations and the pre-arm checks
   before any flight.

## Reading the HIL log

- Set `LOG_DISARMED` 1 to log before arming as well.
- Download the log: Mission Planner > DATA > DataFlash Logs > Download
  DataFlash Log Via Mavlink (or take the microSD card). Then use Review a Log
  and graph the `HIL` message.
- On a healthy 200 Hz link:
  - `Age` is about one loop period (5 ms) plus the USB latency, and `AgeMx`
    shows the worst case. The board declares the link lost above 50 ms.
  - `Rej`, `CrcE`, `ParE` and `TxDrop` stay at 0.
  - `Stale` grows by a small fraction of `Tx` (the Plane autotest fails a SITL
    run above 5%).
  - The link state `LS` is 2 (ok); 3 means lost.
  - `Tx` plus `TxDrop` grows exactly as `Lp` (main loops completed): one
    actuator frame per loop.
- The HIL link parses whatever waited on the port, at the end of each main
  loop. The port's receive buffer bounds that work: 1024 bytes (13 states) at
  the default `SERIAL8_BAUD` of 115200; ChibiOS sizes it up with the baud
  rate (2 x `SERIAL8_BAUD` / 400 bytes above 204800 baud), even on USB, which
  ignores the baud rate otherwise. Keep `SERIAL8_BAUD` at its default on USB.
- `Stale` growing with `Tx`, or `AgeMx` far above a few loop periods, means
  the PC answers late. The run should be failed, not analysed.
- HIL records prove the link behaved. They do not prove how a real aircraft
  would fly.
