# Hardware-in-the-loop (HIL) simulator backend

Classic hardware-in-the-loop for ArduPilot. A PC simulator owns the vehicle
physics. The flight controller runs its normal firmware, but its sensors are
ArduPilot's simulated sensor backends (as in simulation on hardware), fed from
the states the simulator sends. Once per main loop the flight controller sends
its servo outputs back.

```
 flight controller (board or SITL)                         PC simulator (bridge)
 ----------------------------------                         ---------------------
 main loop k: outputs u_k at board time t_k
   HIL_ACTUATOR_CONTROLS(t_k, u_k)  ---- serial port ---->  advance physics t_(k-1) -> t_k
                                                            with u_(k-1) held, latch u_k
   simulated IMU, baro, GPS,        <--- serial port -----  HIL_STATE_QUATERNION(t_k, state)
   compass, airspeed read the
   newest accepted state
```

The backend is `libraries/SITL/SIM_HIL.{h,cpp}` (the `hil` model), with the
link rules in `libraries/SITL/SIM_HIL_Link.{h,cpp}`. The reference simulator
bridge is `Tools/autotest/pysim/hil_bridge.py`. The Pixhawk 6X firmware and its
bench guide are in `libraries/AP_HAL_ChibiOS/hwdef/Pixhawk6X-HIL/README.md`.

This is a feature of this fork. Upstream ArduPilot removed its old HIL in 2021
(PR #17721) and does not carry this backend.

The HIL link uses its own serial protocol value: `SERIALn_PROTOCOL` **100**
(`SerialProtocol_SIM_HIL`, "SIM HIL link").

## Contents

- [Quick start: SITL and the reference bridge](#quick-start-sitl-and-the-reference-bridge)
- [Quick start: a real board](#quick-start-a-real-board)
- [Link contract v2](#link-contract-v2)
- [Timing and determinism](#timing-and-determinism)
- [Lockstep and real time](#lockstep-and-real-time)
- [Link health, reboots and gaps](#link-health-reboots-and-gaps)
- [Status texts and the HIL log](#status-texts-and-the-hil-log)
- [Tests](#tests)
- [Safety model](#safety-model)
- [Bandwidth and baud rate](#bandwidth-and-baud-rate)
- [Writing your own bridge](#writing-your-own-bridge)
- [Why MAVLink HIL messages over a serial port](#why-mavlink-hil-messages-over-a-serial-port)
- [Limitations](#limitations)

## Quick start: SITL and the reference bridge

SITL plays the board. Any `SERIALn` that SITL manages (SERIAL0-7) can carry the
link. SITL's SERIAL5 listens on TCP port 5765 + 10 x instance by default; move
it with `--serial5=tcp:<n>` (5760 + 10 x instance + n, or n itself above 1000).

The `hil` and `hil:lockstep` frames of `sim_vehicle.py` set everything up. For
ArduPlane they load `Tools/autotest/models/plane.parm` and
`Tools/autotest/default_params/plane-hil.parm`:

```
SERIAL5_PROTOCOL 100    # the HIL link on SERIAL5
SCHED_LOOP_RATE  200    # the 200 Hz loop the link is sized for (Plane's own default is 50 Hz)
LOG_DISARMED     1      # HIL log records also before arming
```

1. Start SITL with the HIL backend, in real time:

   ```
   Tools/autotest/sim_vehicle.py -v ArduPlane -f hil --console --map
   ```

2. In a second terminal, start the reference bridge:

   ```
   python3 Tools/autotest/pysim/hil_bridge.py --device tcp:127.0.0.1:5765 --model kinematic
   ```

   `--origin lat,lon,alt_m,hdg_deg` must match SITL's home. The default is
   SITL's default home, CMAC (-35.363261,149.165230,584,353).
   `--loop-rate-hz` must match `SCHED_LOOP_RATE` (default 200). Only the GAP
   reports depend on it.

3. Wait for `HIL: first state received`. Then show that the outputs move the
   aircraft. The kinematic model reads Plane's default outputs: SERVO1 aileron,
   SERVO2 elevator, SERVO3 throttle.

   ```
   mode MANUAL
   arm throttle
   rc 3 2000        # throttle: airspeed rises, the toy rolls along the runway
   rc 2 1800        # elevator up above 12 m/s: it climbs
   rc 1 1800        # aileron right: it banks and turns right
   ```

   The kinematic model is a toy. The Plane autotest flies it through an AUTO
   takeoff, a square circuit and a loiter (see [Tests](#tests)), but it has no
   aerodynamics. Use your own simulator (JSBSim or similar) behind the same link
   for real work.

For lockstep (deterministic, faster than real time) use `-f hil:lockstep`.
SITL then waits for the bridge as soon as its HIL port opens, so start the
bridge too. Copter and Rover have the same two frames
(`default_params/copter-hil.parm`, `default_params/rover-hil.parm`); the
reference bridge has no copter or rover model, so use `--model static` there
(and `--loop-rate-hz 400` for Copter, whose loop rate is left at its default).

## Quick start: a real board

1. Build and flash HIL firmware for the board (for the Pixhawk 6X, see
   `libraries/AP_HAL_ChibiOS/hwdef/Pixhawk6X-HIL/README.md`). Disconnect every
   actuator and remove the propellers first.
2. Reset the parameters so the HIL defaults apply. Set `SIM_OPOS_LAT`,
   `SIM_OPOS_LNG`, `SIM_OPOS_ALT` and `SIM_OPOS_HDG` to the simulator's origin.
   Set `SCHED_LOOP_RATE` for your vehicle (200 matches the bridge default).
3. Connect the ground station to the MAVLink port. Connect the bridge to the HIL
   port, for example the second USB CDC port (OTG2):

   ```
   python Tools/autotest/pysim/hil_bridge.py --device COM11 --model static --origin -35.363261,149.165230,584,353
   ```

   On Linux the device is `/dev/ttyACM1` or similar. Serial ports need
   `pyserial`. USB CDC ignores `--baud`. On a real UART it must match
   `SERIALn_BAUD`.

## Link contract v2

### Transport

- A dedicated serial port. The HIL backend owns it through AP_SerialManager with
  the fork-local protocol value `SerialProtocol_SIM_HIL` (`SERIALn_PROTOCOL`
  100). It opens the first port with that protocol, at `SERIALn_BAUD`, once
  the system is initialised, and retries every second while there is none
  (the protocol can be set without a reboot). There is no compile-time port
  index.
  - Pixhawk 6X: SERIAL8 = USB OTG2, the second USB CDC port. The HIL hwdef
    include makes 100 OTG2's default protocol.
  - SITL: a SERIALn that SITL manages (SERIAL0-7), mapped with
    `--serialN=tcp:<n>`, with `SERIALn_PROTOCOL` 100.
- Framing: MAVLink 2 only, common dialect, CRC checked. MAVLink 1 frames are
  ignored and counted. The parser state is private, never a GCS channel.
- The board sends as `SYSID_THISMAV`, component `MAV_COMP_ID_AUTOPILOT1`. It
  ignores frames carrying its own system id (loopback guard).
- The bridge uses system id 251 by convention (not enforced). The reference
  bridge sends as component `MAV_COMP_ID_ONBOARD_COMPUTER` and ignores frames
  carrying its own system id. The board's `SYSID_THISMAV` must therefore
  differ from the bridge's system id, or each side drops the other's frames.

### Board to PC: HIL_ACTUATOR_CONTROLS (#93), once per main loop

| Field | Type | Meaning |
|---|---|---|
| `time_usec` | uint64 | Board `AP_HAL::micros64()` when the outputs were computed. Strictly increasing within one boot. |
| `controls[16]` | float | Servo output PWM in microseconds, channels 1 to 16. 0 means no output. |
| `mode` | uint8 | `MAV_MODE_FLAG_SAFETY_ARMED` (128) while armed. Other bits 0. |
| `flags` | uint64 | Bit 0 (lockstep, value 1) set in SITL lockstep mode, 0 in real-time mode. No frame counter. |

`controls` carries PWM by an ArduPilot convention. The MAVLink text says
-1..1, but PWM is lossless and is what ArduPilot's own SITL physics receives.
A PX4-style HIL simulator that expects normalised controls will misread it.

The lockstep bit is the meaning that `common.xml` gives bit 0 of `flags`
("1: indicate simulation using lockstep"). The fork's MAVLink has no named enum
for it.

A frame is written only when the port's transmit buffer has room for all of
it; otherwise it is dropped before any byte is written and counted
(`TxDrop`). A write that still comes up short (only possible if another user
of the port took the room; the HIL backend is the port's only user) is
counted as dropped too, but the bytes the port took stay queued, and the
bridge sees a broken frame. Only frames written whole can be answered.

### PC to board: HIL_STATE_QUATERNION (#115), one reply per actuator frame

| Field | Type | Unit | Meaning and acceptance rule |
|---|---|---|---|
| `time_usec` | uint64 | us | Must equal **exactly** the `time_usec` of one of the last 32 actuator frames this board wrote in this boot (`HILLink::SENT_RING_SIZE`; 160 ms at 200 Hz), and be newer than the applied state. Rejected and counted: zero, unknown, not newer than the applied state, and a reply from before a reboot, unless its `time_usec` happens to equal one of this boot's recent frames (the message carries no boot identity). |
| `attitude_quaternion[4]` | float | - | w, x, y, z, body to NED. Must be finite with a norm within 0.01 of 1 (`QUAT_NORM_TOLERANCE`); it is normalised on use. |
| `rollspeed`, `pitchspeed`, `yawspeed` | float | rad/s | Body rates. Must be finite and within 100 rad/s (`MAX_BODY_RATE_RADS`). |
| `lat`, `lon` | int32 | degE7 | Must pass `check_latlng`. |
| `alt` | int32 | mm | Above mean sea level. Converted to cm in 64-bit arithmetic, rounded half away from zero, and rejected beyond +-83 km (`LOCATION_ALT_MAX_M`). |
| `vx`, `vy`, `vz` | int16 | cm/s | NED ground velocity. |
| `ind_airspeed` | uint16 | cm/s | The equivalent airspeed (EAS) the airspeed sensor must read. The board sets the model's `airspeed_pitot` to it directly (EAS semantics, as `SIM_Aircraft.h` documents). |
| `true_airspeed` | uint16 | cm/s | True airspeed. The message carries no angle of attack or sideslip, so the board's air-relative velocity is this speed along the body x axis. |
| `xacc`, `yacc`, `zacc` | int16 | mG | Body-frame specific force (what an accelerometer reads). Level at rest is (0, 0, -1000). |

A state that fails any rule is rejected as a whole and counted (`Rej`). The
board reads what is waiting on the port and applies the newest acceptable
state; older acceptable states in the same read are counted as replaced
(`Rpl`).

The `ind_airspeed` semantics depend on two airspeed fixes in this fork. The
first is upstream's fix for the double EAS2TAS division in the simulated pitot
(b99fdd751a, PR #33713). The second is one differential-pressure model,
`SITL::SIM::airspeed_sensor_pressure()`, shared by `AP_HAL_SITL` and simulation
on hardware.

## Timing and determinism

Rule zero: **physics time follows only the board clock.**

- Physics time comes only from the board's `time_usec` values. The simulator
  never reads the PC clock for physics. PC time is allowed only for logs,
  timeouts and diagnostics.
- Integration uses a named fixed step. The plant time is a whole number of
  fixed steps since the first board time t_0. On frame k (board time t_k,
  outputs u_k) the simulator:
  1. advances from t_(k-1) to t_k with u_(k-1) held (zero-order hold);
  2. latches u_k;
  3. replies with the state at t_k.

  With a step `dt` the plant moves on the grid t_0 + n x dt, so board times
  between grid points need a rule, and the reference bridge uses a causal
  one:
  - the state that answers frame k is the state at the last grid point at
    or before t_k, t_0 + floor((t_k - t_0) / dt) x dt: less than one step
    early;
  - each step uses the outputs in effect at its start, those of the newest
    frame at or before that grid point. So u_k takes effect at the first grid
    point at or after t_k, less than one step late; a step that straddles
    t_k still uses u_(k-1); and of several frames before the same grid
    point (a loop faster than the step) only the newest acts.

  The reference bridge steps at 1000 us (`PHYSICS_STEP_US`) and does the
  count in integer microseconds.
- The first frame only sets t_0 and latches u_0. The reply carries the start
  state: at the origin, level, on the origin heading, at rest.
- No random numbers, unless they are seeded from a named, logged seed. The
  reference bridge uses none.
- Same frames in, bit-identical states out.
  `Tools/autotest/unittest/hil_bridge_unittest.py` replays a recorded session
  twice and compares the replies byte for byte. It checks the same with
  different read chunkings, with every PC clock replaced by one that raises,
  and with every random source (the `random` module, `random.Random`,
  `SystemRandom`, `os.urandom`) replaced by one that raises.

```
board time   t_0        t_1        t_2        t_3
frames       u_0        u_1        u_2        u_3
plant        |--u_0 held--|--u_1 held--|--u_2 held--|
replies      s(t_0)     s(t_1)     s(t_2)     s(t_3)
```

Determinism means the same frames always give the same states. Whether a whole
run repeats also depends on which frames the board sends. In real-time mode
that depends on when each reply arrives. Runs repeat exactly only in lockstep,
and only when everything else the vehicle sees is the same too (see
[Tests](#tests) for what that takes in SITL).

## Lockstep and real time

| | Real time (default) | Lockstep |
|---|---|---|
| Model string | `hil` | `hil:lockstep` |
| Where | SITL and boards | SITL only, never on a board (a hwdef that selects it does not build) |
| Board waits for the PC | Never | For the reply to every frame, before advancing its clock |
| `flags` bit 0 | 0 | 1 |
| Speed | Wall clock (SITL forces `SIM_SPEEDUP` 1) | As fast as the bridge answers, at any `SIM_SPEEDUP` (about 10x real time at a 200 Hz loop with the reference bridge in an autotest) |
| Repeatability | Same frames give the same states. The frames depend on reply timing. | Two runs with the same inputs repeat exactly. |
| Purpose | Rehearse the board's timing | Deterministic tests, fast autotests |

The bridge needs no option: it answers every frame the same way and notes the
lockstep bit when it first sees it. A bridge that withholds a reply in lockstep
stops SITL's clock. That is the lockstep contract, not a fault: without a
bridge a lockstep SITL does not run.

In SITL real-time mode each physics step waits until the wall clock reaches
the board time it advances to, so board time never runs ahead of the wall
clock, as on a board, and each frame leaves no earlier than its board time.
Without that, SITL would run physics steps in bursts of at least 5 ms
(`Aircraft::sync_frame_time`), so at 200 Hz two loops could end at the same
wall-clock instant and every seventh loop would see no new state. With it,
late replies in SITL are the bridge's, as on a board. Every step is held, not
only the frames: a task that waits in the main thread runs physics steps, and
held only at the frames such a wait raced ahead (the log file opening at
arming ran about 360 ms of board time in 25 ms) and the next frame came that
much late. From the first step with the port open, this is SITL's only
wall-clock pacing: `sync_frame_time` is switched off (`use_time_sync`),
because left on it overrode the pacing after its first sleep and the bursts
came back. As on a board, whose backend runs once per loop, a reply is
applied only with the next frame, not on the physics steps between frames, so
`Age` stays between one and two loop periods, as on a board.

In real-time mode late replies are not hidden. They are counted (`Stale`), and
their age is logged (`Age`, `AgeMx`). A PC-side run gate fails the run when they
exceed named limits; the Plane autotest is one (see [Tests](#tests)).

SITL note: above `SIM_SPEEDUP` 1, SITL holds its clock while its SERIAL0 output
queue is full (`SITL_State::wait_clock`), so a ground station that stops reading
also stops a lockstep run.

## Link health, reboots and gaps

### On the board

- Before the first accepted state, the vehicle sits at the origin, level, at
  rest. Arming is refused through an AP_Arming auxiliary authorisation (built
  with `AP_ARMING_AUX_AUTH_ENABLED`; the HIL hwdef include turns it on even
  without scripting). The gate only refuses arming; it never disarms. It is
  a pre-arm check, so a forced arm and disabled arming checks get around it
  (see [Safety model](#safety-model)).
- Link health is based on the state's **age**: board now minus the `time_usec`
  of the applied state. It is not based on arrival time.
  - LOST: the age exceeds 50 ms (`HILLink::LOST_AGE_US`): 10 loops at 200 Hz,
    2.5 at Plane's default 50 Hz.
  - Recovered: a fresh state is applied again.

  Each transition is reported once by status text and logged (the HIL record's
  `LS` field, and an `ERR` record: subsystem 26, failsafe sensors, code 1 when
  lost and 0 when recovered).
- On LOST the last state is held, and the run is invalid: the PC gate fails it.
  No sensor-library special cases are added. Those made the 2013-2021 HIL rot.

### On the PC (the bridge)

All of this is detected from board time only. No frame counter is used.

| Board time of the new frame | Meaning | The reference bridge |
|---|---|---|
| First frame | Start | Sets t_0, model at the origin. |
| Increases by up to `GAP_FACTOR` x the nominal loop period | Normal | Steps across. |
| Increases by more than that, up to `MAX_FORWARD_JUMP_US` | Lost frames | Steps across with the held outputs and reports a GAP. |
| Equal, with the same outputs, mode and flags | The same frame again: lockstep SITL resends a frame whose reply was lost with a connection | Answers with the same state; steps nothing (`stepper.retransmits`). |
| Equal, with other outputs, mode or flags | A broken board: board time strictly increases within one boot | Reports an ERROR, counts it (`stepper.conflicts`), answers with the state at that time; steps and latches nothing. |
| Goes back, or jumps more than `MAX_FORWARD_JUMP_US` | Board reboot | Re-initialises at the new t_0, at the origin. |

Reference values: nominal loop period = 1 / `--loop-rate-hz` (default 200 Hz,
from `SCHED_LOOP_RATE`); `GAP_FACTOR` 1.5, so one lost frame (twice the
period) is a GAP and up to half a period of jitter is not;
`MAX_FORWARD_JUMP_US` 2 s.

A re-initialisation while the previous frame had the armed bit is never
silent. The reference bridge prints an `ALERT ... ARMED` line and counts it in
`stepper.armed_reinits`.

A board reboot usually shows as board time going back. After a reboot within
a couple of seconds of boot, board time may instead jump forward by less than
`MAX_FORWARD_JUMP_US`. The bridge then treats it as a GAP and steps across.

The reference bridge reconnects by itself. A TCP client reconnects when SITL
restarts, a listener accepts the next client, and a serial port is reopened
after a USB re-enumeration. Each attempt waits `RECONNECT_INTERVAL_S` (1 s).

## Status texts and the HIL log

### Status texts

| Text | Severity | When |
|---|---|---|
| `PreArm: HIL: no simulator state` | pre-arm failure | Until the first accepted state. |
| `PreArm: HIL: no simulator state (link lost)` | pre-arm failure | While the link is LOST. |
| `PreArm: HIL: no simulator port` | pre-arm failure | No usable SIM HIL port. |
| `HIL: waiting for simulator on SERIAL<n>` | INFO | The port opened; no state yet. |
| `HIL: first state received` | INFO | The first state is applied. |
| `HIL: simulator state lost, holding the last one` | CRITICAL | Once, when the state age exceeds 50 ms. |
| `HIL: simulator state recovered` | WARNING | Once, when a fresh state is applied again. |
| `HIL: no port, set SERIALn_PROTOCOL 100` | CRITICAL | Once, when no SERIALn has protocol 100. |
| `HIL: SERIAL<n> did not start` | CRITICAL | Once, when the port could not be begun. |

SITL also prints `HIL: real-time mode` or `HIL: lockstep mode` on its console,
`HIL: real-time mode, SIM_SPEEDUP set to 1 (use hil:lockstep to run faster)`,
and, every 2 s while a lockstep frame waits, `HIL: lockstep waiting for the
reply to frame <time_usec> (Rx <n> Rej <n>)`.

### HIL log record

The `HIL` record is written at 10 Hz whatever the port state (set `LOG_DISARMED`
1 to see it before arming).

| Field | Meaning |
|---|---|
| `TimeUS` | Time since system startup. |
| `LS` | Link state: 0 no port, 1 waiting for the first state, 2 ok, 3 lost (the last state is held). |
| `Age` | Age of the applied state: board time minus the `time_usec` of the frame it answers; 0 before the first state. |
| `AgeMx` | Largest `Age` since the previous record. |
| `Stale` | Main-loop frames with no new state since the previous frame, counted after the first state. |
| `Rx` | MAVLink 2 HIL_STATE_QUATERNION frames from another system with a good CRC. |
| `Rej` | Received states not applied: invalid content, or a `time_usec` that is not a recent actuator frame newer than the applied state. |
| `Rpl` | Acceptable states replaced by a newer one in the same read. |
| `Tx` | HIL_ACTUATOR_CONTROLS frames written whole. |
| `TxDrop` | Actuator frames not written whole: no room for the frame, or a short or refused write. |
| `CrcE` | Frames with a bad CRC, including unknown message ids. |
| `ParE` | MAVLink header parse errors. |
| `Oth` | Good frames ignored: other message types, MAVLink 1, or the board's own system id. |
| `Lp` | Main loops completed (`AP_Scheduler::loops_completed32`). With the port open, `Tx` plus `TxDrop` grows by exactly one per loop. |

A healthy 200 Hz link shows `LS` 2, `Age` about one loop period plus the
transport latency, `AgeMx` below the 50 ms LOST limit, `Rej`, `CrcE`, `ParE`
and `TxDrop` not growing, `Stale` a small fraction of `Tx`, and `Tx` plus
`TxDrop` growing as `Lp`.

## Tests

Unit tests (gtest, SITL build):

```
./waf configure --board sitl
./waf build --targets tests/test_sim_hil,tests/test_sim_hil_link,tests/test_sim_output_gate,tests/test_scheduler_loops
./build/sitl/tests/test_sim_hil
./build/sitl/tests/test_sim_hil_link
./build/sitl/tests/test_sim_output_gate
./build/sitl/tests/test_scheduler_loops
```

`test_sim_hil_link` drives the link through a fake UART: every acceptance rule,
newest-wins, whole-frame transmit, the counters, the link states and the reboot
replay. `test_sim_hil` covers the Aircraft state, the airspeed from the applied
state through `Aircraft::fill_fdm()` to the sensor pressure, and the shared
airspeed model with its noise. `test_sim_output_gate` covers the rules that
keep channels outside `SIM_OH_MASK` off the hardware (`SITL::OutputGate`), and
`test_scheduler_loops` that a main loop counts as completed only once all its
tasks have run, which is when the frame is sent. Where the board's drivers
apply the gate (ChibiOS PWM, DShot, GPIO, IOMCU, DroneCAN) only builds for a
board, so that part is the board README's bench check.

The reference bridge:

```
PYTHONPATH=Tools/autotest python3 Tools/autotest/unittest/hil_bridge_unittest.py
```

It covers the board-clock stepping (whole steps, the causal output timing
between grid points, gaps, reboots, retransmitted and conflicting frames, a
reply lost with its connection while the model flies), the unit conversions,
the models, the transports, and replay determinism.

Plane autotests (`Tools/autotest/arduplane.py`), with the reference bridge in
the autotest process on a spare TCP port:

```
Tools/autotest/autotest.py build.Plane test.Plane.HILBackend test.Plane.HILBackendLockstep
```

- `HILBackend` (real time, static model): arming refused with
  `HIL: no simulator state` before the first state; the first-state text;
  arming; the frames' outputs equal `SERVO_OUTPUT_RAW` on all 16 channels;
  board time strictly increasing, `flags` 0; replies held while armed (LOST
  text, still armed, recovered text) and while disarmed (arming refused with
  `(link lost)`); the bridge dropping and re-opening the connection; six
  injected bad states (zero time, a time no frame had, a future time, a
  quaternion of norm 2, a NaN body rate, latitude beyond 90 degrees) all
  rejected and counted, so the log's `Rej` equals 6, plus a corrupted CRC
  (`CrcE`), a MAVLink 1 frame and a frame with the board's own system id
  (`Oth`). Outside the deliberate outages, the HIL log must show `LS` ok,
  `AgeMx` below 50 ms, no `TxDrop`, at most 5% of the loops stale, and
  exactly one actuator frame per completed main loop (`Tx` plus `TxDrop`
  against `Lp`). The link is then held for 120 s with nothing disturbing
  it, and the last 30 s of that hold must meet the same limits on their
  own: frames that stop being spread over the wall clock 10 to 20 s after
  the start (see [Lockstep and real time](#lockstep-and-real-time)) can
  average out under 5% over a short run.
- `HILBackendLockstep` (lockstep at the autotest's `SIM_SPEEDUP` 100, kinematic
  model): an AUTO takeoff, a 400 m square circuit at 50 m and a loiter, flown
  twice. The inputs are kept the same in both runs: a fixed GPS time base
  (`--start-time`), no RC packets (SITL's own constant sticks), the mission at
  fixed coordinates, and AUTO and arm sent while the bridge holds SITL's clock
  at board time 60 s, so they act at the same board time. Every frame must be
  answered, with `flags` 1, no stale loop, no state older than one loop, no
  rejected or dropped frame, and exactly one frame per completed main loop.
  The simulated airspeed sensor (`ARSP`) must read the equivalent airspeed
  the bridge sent, within 1% on average over the circuit. Then the two runs
  must be identical up to
  board time 180 s: every frame (board time, outputs, mode, flags) and every
  reply, and every onboard log record at every board time. Excepted are the
  records that follow the wall clock or the autotest's ground-station traffic
  (status texts, the mission upload, command and link statistics, log
  buffer statistics, `SIM2.ASpdU`, log headers) and the first records of a log
  whose file the log I/O thread opened later. The comparison covers 59 record
  types, ATT, POS, IMU, BARO, GPS, ARSP, XKF*, CTUN, NTUN, TECS, RCOU and HIL
  among them.

## Safety model

HIL firmware runs on a real board. The real outputs must stay dead.

- With `SIM_OH_MASK` 0, once the firmware runs (its IOMCU thread has
  started), no physical actuator output of any kind leaves the board. One set
  of rules, `SITL::OutputGate` (`libraries/SITL/SIM_OutputGate.h`), decides
  for every path:
  - FMU PWM: a channel outside `SIM_OH_MASK` is written as 0 (no pulse).
  - IOMCU MAIN outputs. The IOMCU keeps receiving direct PWM frames (of 0 for
    gated channels), so it never enters its FMU-failsafe mixer. Its failsafe PWM
    is 0, IOMCU mixing is refused, and a failsafe page, mixer or GPIO setup left
    by earlier firmware is reset at start-up.
  - DShot: a gated channel gets no DShot frame and no DShot command (beeps,
    LEDs, 3D or reverse mode, telemetry), on the FMU and the IOMCU.
  - GPIO: a gated servo channel is never set up as a GPIO
    (`SERVOn_FUNCTION` -1), on the FMU or the IOMCU, so relays, the advanced
    failsafe pins and Lua cannot drive it (nor read it as an input). If
    `SIM_OH_MASK` loses a channel after boot, its next GPIO write drives it
    low. Relays additionally need their instance in `SIM_OH_RELAY_MSK`
    (default 0 on a board).
  - Non-PWM actuator drivers: DroneCAN servo and ESC commands are masked by
    `SIM_OH_MASK`, DroneCAN relay (hardpoint) commands by `SIM_OH_RELAY_MSK`;
    the IOMCU SBUS output is refused and cleared; Volz, SBUS
    servo out, Robotis, FETtec, KDECAN, PiccoloCAN, Torqeedo and the Hirth EFI
    are not built (`hwdef/include/HIL.inc`).

  Before the firmware runs, nothing can be gated: after a reflash or an
  FMU-only reboot an IOMCU with safety off runs what the previous firmware
  left (failsafe PWM, RC mixer, GPIO and SBUS outputs) until the HIL firmware
  clears it. Flash with safety on and power-cycle the board before the first
  safety-off (the board README's flashing steps).

  The rules are unit tested in SITL (`test_sim_output_gate`); the drivers that
  apply them only build for a board, so bench-check them once per firmware
  build with a scope or servo tester on every output before trusting them (see
  the board README).
- Procedure, whatever the firmware guarantees:
  - Connect no ESCs, servos or other actuators, and fit no propellers.
  - Never set `SIM_OH_MASK` non-zero on a board wired to an airframe.
  - Do not point `CAN_SLCAN_SERNUM` at the HIL port. SLCAN opens ports by
    number, whatever their protocol.
- The arming gate decides whether a run is valid, not whether outputs are
  safe. It is a pre-arm check, so the usual ways around pre-arm checks get
  around it: a forced arm (`MAV_CMD_COMPONENT_ARM_DISARM` with param2 21196),
  `ARMING_CHECK` 0 or `ARMING_REQUIRE` 0 on Plane, and re-arming after a
  watchdog reset that happened while armed. A run armed that way is invalid,
  and the PC-side run gate must fail it.
- The HIL defaults turn off things the simulated sensors would mislead:
  `BATT_MONITOR` 0 (it would read the real ADC), and `BRD_HEAT_TARG` -1 (the
  heater would chase the simulated IMU temperature). Defaults apply only to
  parameters not already saved, so reset the parameters after flashing.
- Parameters saved under HIL firmware persist when flight firmware is flashed
  again: `GPS1_TYPE` 100, `ARSPD_TYPE` 100, `SERIALn_PROTOCOL`, `SIM_*` and the
  accelerometer calibration markers. Restore a pre-HIL parameter backup, or
  reset and recalibrate, before flying.
- Do not use Mission Planner's legacy Simulation tab. It drives the old,
  removed HIL protocol.

## Bandwidth and baud rate

Frame sizes are MAVLink 2 maximums: 10 header bytes, the payload and 2 CRC
bytes, no signing. MAVLink 2 trims trailing zero payload bytes, so frames may
be shorter.

| Message | Max bytes per frame | At 200 Hz |
|---|---|---|
| HIL_ACTUATOR_CONTROLS (payload 81) | 93 | 18.6 kB/s board to PC |
| HIL_STATE_QUATERNION (payload 64) | 76 | 15.2 kB/s PC to board |

A UART sends 10 bits per byte, so it needs a baud rate of at least
93 x 10 x `SCHED_LOOP_RATE` (186 kbit/s at 200 Hz). Allow at least a factor of
2 for bursts: 460800 or more, preferably 921600. USB CDC has ample margin.
Do not run the link on a 115200 baud telemetry radio.

## Writing your own bridge

`Tools/autotest/pysim/hil_bridge.py` is the reference. A bridge must:

1. Open the HIL port and parse MAVLink 2, common dialect. Ignore everything but
   HIL_ACTUATOR_CONTROLS, and ignore frames with its own system id.
2. Answer **every** HIL_ACTUATOR_CONTROLS, in order, with one
   HIL_STATE_QUATERNION whose `time_usec` is that frame's `time_usec`.
3. Step its physics on board time only (fixed step, whole steps since t_0,
   zero-order hold with a stated rule for board times between steps; the
   reference rule is in [Timing and determinism](#timing-and-determinism)),
   and never on the PC clock.
4. Re-initialise on board time going back or jumping beyond a named maximum.
   Report it loudly if the previous frame was armed. Step across smaller
   forward jumps and report them as GAPs.
5. Answer the same frame received again (same board time and content: a
   lockstep SITL resends a frame whose reply was lost with a connection) with
   the same state, without stepping or re-initialising. Report a second frame
   at the same board time with other content as an error.
6. Fill every field in the units above: quaternion normalised, `lat`/`lon`
   valid, `zacc` -1000 when level at rest, `ind_airspeed` as EAS.
7. Do its own mixing: `controls[]` are the board's servo PWM per channel, as
   the `SERVOn_FUNCTION` parameters assign them.
8. Seed any randomness from a named, logged seed.

The bridge class can also drive an autotest. Its hooks are `last_controls`,
`last_mode`, `last_flags`, `last_time_usec`, `last_event`, `frames`,
`hold_replies(bool)`, `drop_connection()`, `send_raw(bytes)` and
`send_state(**overrides)`:

```python
from pysim import hil_bridge

with hil_bridge.HILBridge("tcp:127.0.0.1:5765", hil_bridge.KinematicPlane()) as bridge:
    ...                                  # arm, fly, check bridge.last_controls
    bridge.hold_replies(True)            # the board's state ages: link LOST
    bridge.send_state(time_usec=0)       # a state the board must reject
```

## Why MAVLink HIL messages over a serial port

- Any board can carry the link: every flight controller has USB or a UART. On
  the master branch, ArduPilot's JSON backend for external physics on hardware
  (PR #31158) needs ArduPilot's networking stack (Ethernet, or PPP over a
  serial port), which most board builds do not include. In this fork's 4.6
  line, JSON is SITL-only.
- The board never waits for the PC. Master's JSON receive blocks the main loop
  until the physics answers, unless its no-lockstep option is set, so without
  a simulator the board appears not to boot. Here the board boots, runs and
  logs without a bridge. Arming is blocked instead.
- Physics time follows the board clock, and the delay is measured (`Age`)
  instead of assumed.
- The framing is CRC-checked binary with standard common-dialect messages, so
  any MAVLink tool can decode a capture.
- It does not repeat the old HIL's failures. That HIL (removed in 2021) shared
  the GCS link and patched every sensor library. This backend uses its own
  port and the existing simulated sensor backends, and it compiles only into
  simulation builds.

## Limitations

- Only servo channels 1 to 16 reach the simulator (`controls[16]`).
- The message has no angle of attack or sideslip. The board's air-relative
  velocity lies along the body x axis (only the barometer's wind compensation
  reads it, off by default). The wind lives in the simulator; the board side
  has none.
- The sensors are ArduPilot's simulated backends with their `SIM_*` noise and
  rate settings. On the board, the IMU is sampled at its own rate from the
  state held between replies. In SITL it follows the physics rate.
- The simulated GPS reports ArduPilot's default 0.2 s lag unless
  `GPS1_DELAY_MS` is set; the HIL defaults do not set it. Set it to the
  measured link and loop latency (the HIL log's `Age`).
- On LOST the last state is held, and the EKF sees a frozen world. The run is
  invalid by contract and the PC gate must fail it.
- Problems inherited from simulation on hardware: the simulated state is
  copied field by field without a snapshot (`SIMState::fdm_input_local`, main
  thread), while the simulated IMU reads it from the timer thread
  (`AP_InertialSensor_SITL`), so one IMU sample can mix two consecutive
  states, about one loop apart (HIL changes the state at most once per loop).
  A fix belongs in the shared simulation code, not in this backend. And
  `SIM_IMU_POS` must stay 0 (angular acceleration is not filled).
- The board parses the HIL port at the end of each main loop, outside the
  scheduler's task budget, up to what was waiting when it started. The port's
  receive buffer bounds that work. On ChibiOS it is the larger of the 1024
  bytes the backend asks for and 2 x max(512, `SERIALn_BAUD` / 400) bytes on
  boards with plenty of memory, so a high `SERIALn_BAUD` (even on USB, which
  otherwise ignores it) enlarges the worst case. Its time on the target has
  not been measured yet: check `PM.MaxT` and `PM.NLon` in the bench check.
- The backend is vehicle-agnostic: Copter and Rover use it unchanged, with the
  mixing done in the simulator. Plane is covered by the autotests; Copter and
  Rover only by a SITL smoke check (boot, arming gate, first state, arming)
  with the static model.
- The reference bridge's kinematic plane is a toy (no stall, no wind, no
  aerodynamics). It only shows the link working.
- A reply carries no boot identity, only the board time of the frame it
  answers. A reply from before a board reboot is accepted if its `time_usec`
  equals one of the new boot's recent frames exactly.
- The reference bridge has no run gate of its own: it reports GAPs, reboots
  and errors, but a runner must fail a run from the board's HIL log and
  status texts (the Plane autotest does so in SITL).
- Simulation, HIL included, never proves real flight, physical outputs, or a
  vehicle's behaviour beyond the model.
