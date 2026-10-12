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
  Board side of the hardware-in-the-loop (HIL) link, contract v2
  (libraries/SITL/examples/HIL). It owns the MAVLink framing on the HIL
  port and the rules for which states are applied, and has no
  dependency on the Aircraft, the scheduler or the GCS, so that every
  rule can be tested with a fake UART (libraries/SITL/tests).

  - board -> PC: HIL_ACTUATOR_CONTROLS, once per main loop. time_usec is
    the board time the outputs were computed, controls[] the servo PWM
    in microseconds (0 = no output; an ArduPilot convention, the MAVLink
    text says -1..1), mode has MAV_MODE_FLAG_SAFETY_ARMED when armed,
    flags is 0, or ACTUATOR_FLAG_LOCKSTEP in SITL lockstep mode.
  - PC -> board: HIL_STATE_QUATERNION, one reply per actuator frame,
    whose time_usec equals the time_usec of the frame it answers.
  - MAVLink 2 only, CRC checked, private parser state (never a GCS
    channel). Frames from the board's own system id are ignored.
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_HIL_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

namespace SITL {

class HILLink {
public:
    // link health, from the age of the applied state
    enum class LinkState : uint8_t {
        NO_PORT = 0,    // no usable SIM_HIL serial port
        WAITING = 1,    // port open, no state accepted yet
        OK      = 2,    // applied state at most LOST_AGE_US old
        LOST    = 3,    // applied state older than LOST_AGE_US; it is held
    };

    // a HIL_STATE_QUATERNION that passed every check, in Aircraft units
    struct State {
        uint64_t time_us;       // time_usec of the actuator frame it answers
        Quaternion attitude;    // body to NED, normalised
        Vector3f gyro;          // body rates, rad/s
        Location location;      // absolute: lat/lon degE7, alt cm above MSL
        Vector3f velocity_ef;   // NED ground velocity, m/s
        Vector3f accel_body;    // body-frame specific force, m/s/s
        float ind_airspeed;     // equivalent airspeed the pitot must read, m/s
        float true_airspeed;    // m/s
    };

    struct Stats {
        uint32_t tx;            // HIL_ACTUATOR_CONTROLS written whole
        uint32_t tx_drop;       // actuator frames not written whole (no room, short or refused write)
        uint32_t rx;            // MAVLink 2 HIL_STATE_QUATERNION from another system, good CRC
        uint32_t rejected;      // of rx: invalid content, or time_usec not a recent frame newer than the applied state
        uint32_t replaced;      // of rx: acceptable, but a newer state arrived in the same read
        uint32_t crc_errors;    // frames with a bad CRC, including unknown message ids
        uint32_t parse_errors;  // MAVLink header errors
        uint32_t other_msgs;    // good frames ignored: other message ids, MAVLink 1, own system id
        uint32_t stale;         // main-loop frames with no new state since the previous frame, counted after the first state
    };

    // HIL_ACTUATOR_CONTROLS.flags bit 0 (common.xml: "1: indicate
    // simulation using lockstep"); only SITL sets it
    static constexpr uint64_t ACTUATOR_FLAG_LOCKSTEP = 1U;
    // channels carried by HIL_ACTUATOR_CONTROLS.controls
    static constexpr uint8_t NUM_CONTROLS = 16;
    // a reply is accepted only if its time_usec is one of the last
    // SENT_RING_SIZE frames written whole. Replies to older frames are
    // rejected: at 400 Hz the ring spans 80 ms, more than LOST_AGE_US,
    // so a reply too old for the ring is from a link that is lost anyway
    static constexpr uint8_t SENT_RING_SIZE = 32;
    // the link is LOST when the applied state is older than this: 10 main
    // loops at 200 Hz (2.5 at Plane's default 50 Hz). A fresh state is one
    // loop plus the round trip old, so an older state means the link
    // stalled; the HIL autotest's link check fails a run on the same age
    static constexpr uint32_t LOST_AGE_US = 50000;
    // a received attitude quaternion further than this from unit length is
    // rejected as corrupt (the bridge sends normalised quaternions)
    static constexpr float QUAT_NORM_TOLERANCE = 0.01f;
    // body rates beyond this are rejected as corrupt: 16 revolutions per
    // second is far above any airframe, and the limit keeps every derived
    // sensor value (degrees/s in the IMU model) finite
    static constexpr float MAX_BODY_RATE_RADS = 100.0f;
    // largest altitude magnitude a Location holds (LOCATION_ALT_MAX_M)
    static constexpr int64_t ALT_LIMIT_CM = int64_t(LOCATION_ALT_MAX_M) * 100;
    // longest HIL_ACTUATOR_CONTROLS on the wire (MAVLink 2, unsigned)
    static constexpr uint16_t ACTUATOR_FRAME_MAX_LEN = MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS_LEN;

    // the dedicated port, already begun; nullptr = no port
    void set_port(AP_HAL::UARTDriver *port) { uart = port; }
    AP_HAL::UARTDriver *port() const { return uart; }

    // count one main-loop frame for the stale statistic; call once per
    // loop before send_actuators()
    void start_frame();

    // true if a whole actuator frame fits in the transmit buffer now
    bool tx_room() const;

    /*
      send one HIL_ACTUATOR_CONTROLS from system sysid. Without room for
      the whole frame nothing is written (a partial frame would corrupt
      the next one at the bridge). A write that still comes up short
      (only if another user of the port took the room) is counted as
      dropped, though the bytes the port took stay queued. Only a frame
      written whole can be answered.
     */
    bool send_actuators(uint64_t time_us, const uint16_t servos[NUM_CONTROLS],
                        bool armed, bool lockstep, uint8_t sysid);

    /*
      parse the bytes waiting on the port (at most the number waiting when
      the call starts, which the port's real receive buffer bounds) and
      apply the newest acceptable state among them. Returns true if a new
      state was applied.
     */
    bool receive(uint8_t own_sysid);

    // recompute the link state at board time now_us; returns true and
    // the state before when it changed
    bool update_link_state(uint64_t now_us, LinkState &previous);
    LinkState link_state() const { return link_state_; }

    bool have_state() const { return have_state_; }
    const State &state() const { return applied; }
    // board time now_us minus the applied state's time_usec; 0 without one
    uint32_t state_age_us(uint64_t now_us) const;

    const Stats &stats() const { return stats_; }

    /*
      checks and unit conversion of a received state, independent of the
      link: every float finite, unit quaternion within
      QUAT_NORM_TOLERANCE, body rates within MAX_BODY_RATE_RADS, lat/lon
      valid, altitude within a Location. False rejects the whole state.
     */
    static bool decode_state(const mavlink_hil_state_quaternion_t &pkt, State &out);

    // altitude mm -> cm rounded half away from zero, in 64-bit arithmetic;
    // false if the result does not fit ALT_LIMIT_CM
    static bool alt_mm_to_cm(int32_t alt_mm, int32_t &alt_cm);

private:
    bool sent_recently(uint64_t time_us) const;
    void record_sent(uint64_t time_us);
    LinkState compute_link_state(uint64_t now_us) const;

    AP_HAL::UARTDriver *uart = nullptr;

    // private MAVLink parser and sender state, never a GCS channel
    mavlink_message_t rx_buffer {};     // frame being parsed
    mavlink_status_t rx_status {};      // parser state
    mavlink_message_t rx_msg {};        // last completed frame
    mavlink_status_t rx_result {};      // per-byte result (parse error count)
    mavlink_status_t tx_status {};      // outgoing sequence number, MAVLink 2

    // time_usec of the last frames written whole, oldest overwritten
    uint64_t sent_times[SENT_RING_SIZE] {};
    uint8_t sent_next = 0;
    uint8_t sent_count = 0;

    bool have_state_ = false;
    bool state_since_frame = false;     // a state was applied since the last frame
    State applied {};
    LinkState link_state_ = LinkState::NO_PORT;

    Stats stats_ {};
};

} // namespace SITL

#endif // AP_SIM_HIL_ENABLED
