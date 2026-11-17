/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Hardware-in-the-loop (HIL) backend, see SIM_HIL.h and SIM_HIL_Link.h.

  Board-side behaviour (link contract v2):
  - Before the first accepted state the vehicle sits at the origin,
    level with the home heading, at rest, specific force (0, 0, -g)
    (Aircraft::set_start_location and SIM_Aircraft.h).
  - Arming is refused through an AP_Arming auxiliary authorisation until
    the first state arrives and whenever the link is LOST. It never
    disarms.
  - When the link is LOST the last state is held: the simulated sensors
    keep reading it, so the run is invalid from then on and the PC side
    fails it. No simulated sensor library gets a special case.
*/

#include "SIM_HIL.h"

#if AP_SIM_HIL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <unistd.h>
#endif

extern const AP_HAL::HAL& hal;

using namespace SITL;

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL && defined(AP_SIM_FRAME_STRING)
// true if string s starts with prefix p (C++11 constexpr)
static constexpr bool starts_with(const char *s, const char *p)
{
    return *p == '\0' || (*s == *p && starts_with(s + 1, p + 1));
}
// true if string s contains the frame option ":lockstep"
static constexpr bool has_lockstep_option(const char *s)
{
    return *s != '\0' && (starts_with(s, ":lockstep") || has_lockstep_option(s + 1));
}
// a board follows its own clock and never waits for the PC
static_assert(!has_lockstep_option(AP_SIM_FRAME_STRING), "HIL lockstep is SITL only");
#endif

HIL::HIL(const char *frame_str) :
    Aircraft(frame_str)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (strcasecmp(frame_str, "hil:lockstep") == 0) {
        lockstep = true;
    } else if (strcasecmp(frame_str, "hil") != 0) {
        AP_HAL::panic("HIL: unknown frame %s (hil or hil:lockstep)", frame_str);
    }
    ::printf("HIL: %s mode\n", lockstep ? "lockstep" : "real-time");
#endif
}

/*
  open the SERIALn port whose protocol is SIM_HIL once parameters are
  loaded and AP_SerialManager has set up the ports. The port is latched
  only once begin() has made it usable.
 */
bool HIL::open_port(void)
{
    if (link.port() != nullptr) {
        return true;
    }
    if (!hal.scheduler->is_system_initialized()) {
        return false;
    }
    const uint32_t now_ms = AP_HAL::millis();
    if (port_attempted && now_ms - last_port_attempt_ms < PORT_RETRY_INTERVAL_MS) {
        return false;
    }
    port_attempted = true;
    last_port_attempt_ms = now_ms;

    const AP_SerialManager &serial_manager = AP::serialmanager();
    const auto protocol = AP_SerialManager::SerialProtocol_SIM_HIL;
    AP_HAL::UARTDriver *port = serial_manager.find_serial(protocol, 0);
    if (port == nullptr) {
        if (!port_notice_sent) {
            port_notice_sent = true;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "HIL: no port, set SERIALn_PROTOCOL %d",
                          int(protocol));
        }
        return false;
    }
    const int8_t number = serial_manager.find_portnum(protocol, 0);
    port->begin(serial_manager.find_baudrate(protocol, 0), UART_RX_BUFSIZE, UART_TX_BUFSIZE);
    if (!port->is_initialized()) {
        if (!port_notice_sent) {
            port_notice_sent = true;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "HIL: SERIAL%d did not start", int(number));
        }
        return false;
    }
    port_number = number;
    link.set_port(port);
    return true;
}

/*
  true once per completed main loop, when its outputs are final. On a
  board update() runs once per loop, at its end (AP_Scheduler::loop,
  hal.simstate->update). In SITL update() runs on every physics step,
  whenever the main thread waits; a step during the loop's tasks (a
  task that waits) must not send, so the frame follows the loop counter
  that advances after the tasks, not the tick that advances before them.
 */
bool HIL::loop_completed(void)
{
    const AP_Scheduler *scheduler = AP_Scheduler::get_singleton();
    if (scheduler == nullptr) {
        // no main loop (an example program): every update is a frame
        return true;
    }
    const uint32_t count = scheduler->loops_completed32();
    if (have_loop_count && count == last_loop_count) {
        return false;
    }
    have_loop_count = true;
    last_loop_count = count;
    return true;
}

void HIL::exchange_frame(const struct sitl_input &input, uint64_t now_us)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (!lockstep && link.port() != nullptr) {
        // as on a board, this loop ends no earlier on the wall clock than
        // its board time (pace_realtime_step held the step before), and a
        // reply that arrived meanwhile is applied before the frame counts
        // as stale or not
        pump_port();
        if (link.receive(mavlink_system.sysid)) {
            apply_state(link.state());
        }
    }
#endif
    link.start_frame();
    if (link.port() == nullptr) {
        return;
    }
    static_assert(ARRAY_SIZE(input.servos) >= HILLink::NUM_CONTROLS, "servo input smaller than controls");
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (lockstep) {
        lockstep_exchange(input, now_us);
        return;
    }
#endif
    link.send_actuators(now_us, input.servos, hal.util->get_soft_armed(), false, mavlink_system.sysid);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // put the frame on the socket now, not at the next IO pass
    pump_port();
#endif
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
/*
  SITL moves serial bytes between a socket and the port buffers in its
  IO pass, every 10 ms of simulated time and after each main loop
  (HALSITL::Scheduler::stop_clock, HAL_SITL::run), all in the main
  thread, as is this. The HIL port is also pumped at each frame, so a
  reply that arrived is seen at the next frame, as on a board, and in
  lockstep while the clock stands still.
 */
void HIL::pump_port(void)
{
    AP_HAL::UARTDriver *port = link.port();
    if (port != nullptr) {
        port->_timer_tick();
    }
}

// defined out of class (C++11): AP_Float::set() binds it to a reference,
// so an unoptimised build needs its storage
constexpr float HIL::REALTIME_SPEEDUP;

/*
  real-time mode rehearses the board, whose physics follows the wall
  clock: keep SIM_SPEEDUP at 1 (fill_fdm applies the parameter)
 */
void HIL::keep_realtime(void)
{
    if (sitl == nullptr || is_equal(sitl->speedup.get(), REALTIME_SPEEDUP)) {
        return;
    }
    sitl->speedup.set(REALTIME_SPEEDUP);
    ::printf("HIL: real-time mode, SIM_SPEEDUP set to %.0f (use hil:lockstep to run faster)\n",
             REALTIME_SPEEDUP);
}

/*
  real time: hold each physics step until the wall clock reaches the
  board time it advances to, so board time never runs ahead of the wall
  clock, as on a board. Each frame then also leaves no earlier than its
  board time. Aircraft::sync_frame_time sleeps only once SITL is
  min_sleep_time (5 ms) ahead of the wall clock, so it runs physics steps
  in bursts longer than a 200 Hz loop: two loops could end at the same
  wall-clock instant, and the second could never see the reply to the
  first. A board's loops are spread over the wall clock; this keeps
  SITL's frames spread the same way, so late replies are the bridge's,
  not SITL's.

  Every step is held, not only the step that sends a frame: a main loop
  that does not complete runs physics steps while a task waits in the
  main thread (SITL_State::wait_clock), and each step advances board
  time. Holding only the frames let such a wait race ahead of the wall
  clock (the 1 ms delay() polling loop of AP_Logger_File's arming wait
  ran about 360 ms of board time while the log file took about 25 ms to
  open), and the next frame then waited for all of it at once: the
  bridge saw a gap that long and the board reported the state lost.
  Held per step, a wait costs its real duration in board time.

  From the first step with the port open this is SITL's only wall-clock
  pacing: sync_frame_time is switched off (use_time_sync). The two cannot
  share the work. This one can only delay a step, and sync_frame_time
  lets the wall clock run ahead of board time (it never charges the wall
  time between its own two clock reads). Its first sleep left every later
  step late for its slot here, so this never slept again and the bursts
  were back: one loop in seven sent its frame before the reply to the
  previous one could arrive. Before the port opens sync_frame_time keeps
  SITL on the wall clock; lockstep never gets here.
 */
void HIL::pace_realtime_step(uint64_t board_us)
{
    const uint64_t wall_us = get_wall_time_us();
    if (!pace_anchored) {
        pace_anchored = true;
        pace_wall_anchor_us = wall_us;
        pace_board_anchor_us = board_us;
        use_time_sync = false;
        return;
    }
    // REALTIME_SPEEDUP is 1: board and wall clock run at the same rate
    const uint64_t slot_us = pace_wall_anchor_us + (board_us - pace_board_anchor_us);
    if (wall_us < slot_us) {
        ::usleep(slot_us - wall_us);
    } else if (wall_us - slot_us > REALTIME_RESYNC_US) {
        pace_wall_anchor_us = wall_us;
        pace_board_anchor_us = board_us;
    }
}

/*
  lockstep: SITL owns its clock, so it sends the frame and waits for the
  reply to it before time advances. The same inputs then give the same
  run whatever the wall-clock timing, at any SIM_SPEEDUP. No frame is
  dropped: without room to send (no bridge connected yet) it waits; a
  frame lost with a dropped connection, or whose reply was, is sent again,
  unchanged, once a bridge reconnects (the contract's bridge answers the
  same frame again with the same state, without stepping).
 */
void HIL::lockstep_exchange(const struct sitl_input &input, uint64_t now_us)
{
    const bool armed = hal.util->get_soft_armed();
    bool sent = false;
    uint64_t last_notice_us = get_wall_time_us();
    while (true) {
        pump_port();
        if (!sent) {
            if (link.tx_room()) {
                sent = link.send_actuators(now_us, input.servos, armed, true, mavlink_system.sysid);
                pump_port();
            }
        } else if (!link.tx_room()) {
            // a SITL TCP port has no room only while no bridge is connected
            sent = false;
        }
        if (sent && link.receive(mavlink_system.sysid)) {
            apply_state(link.state());
        }
        if (sent && link.have_state() && link.state().time_us == now_us) {
            return;
        }
        const uint64_t wall_us = get_wall_time_us();
        if (wall_us - last_notice_us >= LOCKSTEP_NOTICE_US) {
            last_notice_us = wall_us;
            const auto &stats = link.stats();
            ::printf("HIL: lockstep waiting for %s frame %llu (Rx %u Rej %u)\n",
                     sent ? "the reply to" : "room to send",
                     (unsigned long long)now_us, unsigned(stats.rx), unsigned(stats.rejected));
        }
        ::usleep(LOCKSTEP_POLL_US);
    }
}
#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL

void HIL::apply_state(const HILLink::State &s)
{
    s.attitude.rotation_matrix(dcm);
    gyro = s.gyro;

    // location exactly as sent; position is the same point in metres NED
    // from the origin, the inverse of update_position(). update_position()
    // is not called, so the origin never moves and stays at home.
    location = s.location;
    const Vector2d pos_ne = origin.get_distance_NE_double(location);
    position = Vector3d{pos_ne.x, pos_ne.y, (home.alt - location.alt) * 0.01};

    velocity_ef = s.velocity_ef;
    accel_body = s.accel_body;

    // the airspeed sensor reads the equivalent airspeed the bridge sent:
    // fill_fdm() passes airspeed_pitot as fdm.airspeed, which both HALs
    // turn into the sensor's differential pressure as EAS
    // (SITL::SIM::airspeed_sensor_pressure)
    airspeed = s.ind_airspeed;
    airspeed_pitot = s.ind_airspeed;
    // the bridge owns the wind, the board side has none. The message has no
    // angle of attack or sideslip, so the air-relative velocity is the true
    // airspeed along the body x axis.
    wind_ef.zero();
    velocity_air_bf = Vector3f{s.true_airspeed, 0.0f, 0.0f};
    velocity_air_ef = dcm * velocity_air_bf;
}

void HIL::report_link_state(HILLink::LinkState previous)
{
    switch (link.link_state()) {
    case HILLink::LinkState::NO_PORT:
        // the port is latched once open
        break;
    case HILLink::LinkState::WAITING:
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "HIL: waiting for simulator on SERIAL%d", int(port_number));
        break;
    case HILLink::LinkState::OK:
        if (previous == HILLink::LinkState::LOST) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "HIL: simulator state recovered");
#if HAL_LOGGING_ENABLED
            AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_SENSORS, LogErrorCode::FAILSAFE_RESOLVED);
#endif
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "HIL: first state received");
        }
        break;
    case HILLink::LinkState::LOST:
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "HIL: simulator state lost, holding the last one");
#if HAL_LOGGING_ENABLED
        AP::logger().Write_Error(LogErrorSubsystem::FAILSAFE_SENSORS, LogErrorCode::FAILSAFE_OCCURRED);
#endif
        break;
    }
}

/*
  refuse arming until the first state and while the link is LOST. An
  auxiliary authorisation only gates arming: it never disarms.
 */
void HIL::update_arming_gate(void)
{
#if AP_ARMING_AUX_AUTH_ENABLED
    AP_Arming *arming = AP_Arming::get_singleton();
    if (arming == nullptr) {
        return;
    }
    if (!have_auth_id) {
        if (auth_id_requested) {
            // all ids were taken: AP_Arming itself refuses arming
            // ("Too many auxiliary authorisers")
            return;
        }
        auth_id_requested = true;
        if (!arming->get_aux_auth_id(auth_id)) {
            return;
        }
        have_auth_id = true;
    }
    switch (link.link_state()) {
    case HILLink::LinkState::NO_PORT:
        arming->set_aux_auth_failed(auth_id, "HIL: no simulator port");
        break;
    case HILLink::LinkState::WAITING:
        arming->set_aux_auth_failed(auth_id, "HIL: no simulator state");
        break;
    case HILLink::LinkState::LOST:
        arming->set_aux_auth_failed(auth_id, "HIL: no simulator state (link lost)");
        break;
    case HILLink::LinkState::OK:
        arming->set_aux_auth_passed(auth_id);
        break;
    }
#endif
}

void HIL::write_log(uint64_t now_us)
{
#if HAL_LOGGING_ENABLED
    const uint32_t age_us = link.state_age_us(now_us);
    age_max_us = MAX(age_max_us, age_us);
    if (now_us - last_log_us < LOG_INTERVAL_US) {
        return;
    }
    last_log_us = now_us;
    const auto &stats = link.stats();

// @LoggerMessage: HIL
// @Description: Hardware-in-the-loop link of the HIL simulator backend
// @Field: TimeUS: Time since system startup
// @Field: LS: link state: 0 no port, 1 waiting for the first state, 2 ok, 3 lost (the last state is held)
// @Field: Age: age of the applied state: board time minus the time_usec of the actuator frame it answers; 0 before the first state
// @Field: AgeMx: largest Age since the previous HIL record
// @Field: Stale: main-loop frames with no new state since the previous frame, counted after the first state
// @Field: Rx: MAVLink 2 HIL_STATE_QUATERNION frames from another system with a good CRC
// @Field: Rej: received states not applied: invalid content, or a time_usec that is not a recent actuator frame newer than the applied state
// @Field: Rpl: acceptable states replaced by a newer one in the same read
// @Field: Tx: HIL_ACTUATOR_CONTROLS frames written whole
// @Field: TxDrop: actuator frames not written whole: no room for the frame, or a short or refused write
// @Field: CrcE: frames with a bad CRC, including unknown message ids
// @Field: ParE: MAVLink header parse errors
// @Field: Oth: good frames ignored: other message types, MAVLink 1, or the board's own system id
// @Field: Lp: main loops completed (AP_Scheduler::loops_completed32); with the port open, Tx plus TxDrop grows by exactly one per loop
    const AP_Scheduler *scheduler = AP_Scheduler::get_singleton();
    // the labels fill 63 of the 64 characters a log format allows
    AP::logger().WriteStreaming(
        "HIL",
        "TimeUS,LS,Age,AgeMx,Stale,Rx,Rej,Rpl,Tx,TxDrop,CrcE,ParE,Oth,Lp",
        "s-ss----------",
        "F-FF----------",
        "QBIIIIIIIIIIII",
        now_us,
        uint8_t(link.link_state()),
        age_us,
        age_max_us,
        stats.stale,
        stats.rx,
        stats.rejected,
        stats.replaced,
        stats.tx,
        stats.tx_drop,
        stats.crc_errors,
        stats.parse_errors,
        stats.other_msgs,
        scheduler != nullptr ? scheduler->loops_completed32() : 0U);
    age_max_us = 0;
#endif
}

/*
  one simulator step: send the outputs once per completed main loop,
  apply the newest state, keep the last state when none arrived, keep
  time
 */
void HIL::update(const struct sitl_input &input)
{
    const uint64_t now_us = AP_HAL::micros64();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (!lockstep) {
        keep_realtime();
    }
    // follow SIM_RATE_HZ as the other backends do (SIM_Aircraft.cpp update_dynamics)
    if (sitl != nullptr) {
        adjust_frame_time(constrain_float(sitl->loop_rate_hz, rate_hz-1, rate_hz+1));
    }
#endif

    open_port();
    if (!auth_id_requested) {
        // the gate holds from the first step, before the port opens
        update_arming_gate();
    }

    // a state is applied only with a frame, once per completed loop, as
    // on a board, whose backend runs once per loop: a reply that arrives
    // while SITL runs the physics steps between two frames waits for the
    // next frame, so the loop's tasks see the state a board would
    if (loop_completed()) {
        exchange_frame(input, now_us);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        pump_port();
#endif
        if (link.receive(mavlink_system.sysid)) {
            apply_state(link.state());
        }
    }

    HILLink::LinkState previous;
    if (link.update_link_state(now_us, previous)) {
        report_link_state(previous);
        update_arming_gate();
    }
    write_log(now_us);

    // no onboard physics: only the magnetic field follows the applied
    // location and attitude
    update_mag_field_bf();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // SITL owns its clock: advance one physics step. sync_frame_time
    // keeps it to the wall clock until, in real-time mode, the port is
    // open; from then the step waits here for the wall clock to reach
    // its new board time (pace_realtime_step), before the HAL clock
    // takes that time (SITL_State::_fdm_input_step, stop_clock)
    time_advance();
    if (!lockstep && link.port() != nullptr) {
        pace_realtime_step(time_now_us);
    }
#else
    // on the board AP_HAL::micros64() is real time and stop_clock() is a
    // no-op; the state's timestamp is the board time
    time_now_us = now_us;
#endif
}

#endif // AP_SIM_HIL_ENABLED
