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
  Board side of the HIL link, see SIM_HIL_Link.h
*/

#include "SIM_HIL_Link.h"

#if AP_SIM_HIL_ENABLED

using namespace SITL;

// bytes read per UART read call (a stack buffer)
static constexpr uint16_t RX_CHUNK_BYTES = 128;

void HILLink::start_frame()
{
    if (have_state_ && !state_since_frame) {
        stats_.stale++;
    }
    state_since_frame = false;
}

bool HILLink::tx_room() const
{
    return uart != nullptr && uart->txspace() >= ACTUATOR_FRAME_MAX_LEN;
}

bool HILLink::send_actuators(uint64_t time_us, const uint16_t servos[NUM_CONTROLS],
                             bool armed, bool lockstep, uint8_t sysid)
{
    if (uart == nullptr) {
        return false;
    }
    if (!tx_room()) {
        stats_.tx_drop++;
        return false;
    }

    mavlink_hil_actuator_controls_t pkt {};
    pkt.time_usec = time_us;
    static_assert(ARRAY_SIZE(pkt.controls) == NUM_CONTROLS, "HIL_ACTUATOR_CONTROLS channel count");
    for (uint8_t i = 0; i < NUM_CONTROLS; i++) {
        // PWM in microseconds, 0 = no output
        pkt.controls[i] = servos[i];
    }
    pkt.mode = armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0;
    pkt.flags = lockstep ? uint64_t(ACTUATOR_FLAG_LOCKSTEP) : uint64_t(0);

    mavlink_message_t msg;
    mavlink_msg_hil_actuator_controls_encode_status(sysid, MAV_COMP_ID_AUTOPILOT1,
                                                    &tx_status, &msg, &pkt);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    // write() returns what the port took: less than len means the bridge
    // gets a broken frame (or none, e.g. while another user holds the port
    // lock), which it cannot answer
    if (uart->write(buf, len) != len) {
        stats_.tx_drop++;
        return false;
    }
    stats_.tx++;
    record_sent(time_us);
    return true;
}

void HILLink::record_sent(uint64_t time_us)
{
    sent_times[sent_next] = time_us;
    sent_next = (sent_next + 1) % SENT_RING_SIZE;
    if (sent_count < SENT_RING_SIZE) {
        sent_count++;
    }
}

bool HILLink::sent_recently(uint64_t time_us) const
{
    for (uint8_t i = 0; i < sent_count; i++) {
        if (sent_times[i] == time_us) {
            return true;
        }
    }
    return false;
}

bool HILLink::alt_mm_to_cm(int32_t alt_mm, int32_t &alt_cm)
{
    const int64_t mm = alt_mm;
    const int64_t cm = (mm >= 0) ? (mm + 5) / 10 : (mm - 5) / 10;
    if (cm > ALT_LIMIT_CM || cm < -ALT_LIMIT_CM) {
        return false;
    }
    alt_cm = int32_t(cm);
    return true;
}

bool HILLink::decode_state(const mavlink_hil_state_quaternion_t &pkt, State &out)
{
    // every float used must be finite (indexed: the MAVLink struct is packed)
    for (uint8_t i = 0; i < ARRAY_SIZE(pkt.attitude_quaternion); i++) {
        if (!isfinite(pkt.attitude_quaternion[i])) {
            return false;
        }
    }
    const float rates[] { pkt.rollspeed, pkt.pitchspeed, pkt.yawspeed };
    for (const float r : rates) {
        if (!isfinite(r) || fabsf(r) > MAX_BODY_RATE_RADS) {
            return false;
        }
    }
    Quaternion q {pkt.attitude_quaternion[0], pkt.attitude_quaternion[1],
                  pkt.attitude_quaternion[2], pkt.attitude_quaternion[3]};
    if (fabsf(q.length() - 1.0f) > QUAT_NORM_TOLERANCE) {
        return false;
    }
    if (!check_latlng(pkt.lat, pkt.lon)) {
        return false;
    }
    int32_t alt_cm;
    if (!alt_mm_to_cm(pkt.alt, alt_cm)) {
        return false;
    }

    out.time_us = pkt.time_usec;
    q.normalize();
    out.attitude = q;
    out.gyro = Vector3f{pkt.rollspeed, pkt.pitchspeed, pkt.yawspeed};
    out.location = Location{pkt.lat, pkt.lon, alt_cm, Location::AltFrame::ABSOLUTE};
    out.velocity_ef = Vector3f{float(pkt.vx), float(pkt.vy), float(pkt.vz)} * 0.01f;
    // mG -> m/s/s; level at rest (0, 0, -1000) is (0, 0, -g)
    out.accel_body = Vector3f{float(pkt.xacc), float(pkt.yacc), float(pkt.zacc)} * (GRAVITY_MSS * 0.001f);
    out.ind_airspeed = pkt.ind_airspeed * 0.01f;
    out.true_airspeed = pkt.true_airspeed * 0.01f;
    return true;
}

bool HILLink::receive(uint8_t own_sysid)
{
    if (uart == nullptr) {
        return false;
    }
    State newest {};
    bool have_newest = false;
    uint8_t buf[RX_CHUNK_BYTES];
    // read only the bytes already waiting: the port's real receive buffer,
    // whatever size begin() gave it, bounds the work per call, and a
    // backlog is taken in one call so that its newest state wins
    uint32_t budget = uart->available();
    while (budget > 0) {
        const ssize_t nread = uart->read(buf, uint16_t(MIN(uint32_t(sizeof(buf)), budget)));
        if (nread <= 0) {
            break;
        }
        budget -= MIN(uint32_t(nread), budget);
        for (ssize_t i = 0; i < nread; i++) {
            const uint8_t framing = mavlink_frame_char_buffer(&rx_buffer, &rx_status, buf[i],
                                                              &rx_msg, &rx_result);
            stats_.parse_errors += rx_result.packet_rx_drop_count;
            if (framing == MAVLINK_FRAMING_BAD_CRC) {
                stats_.crc_errors++;
                continue;
            }
            if (framing != MAVLINK_FRAMING_OK) {
                continue;
            }
            if (rx_msg.msgid != MAVLINK_MSG_ID_HIL_STATE_QUATERNION ||
                rx_msg.magic != MAVLINK_STX ||
                rx_msg.sysid == own_sysid) {
                // other messages, MAVLink 1 framing, or our own frames
                // looped back
                stats_.other_msgs++;
                continue;
            }
            stats_.rx++;
            mavlink_hil_state_quaternion_t pkt;
            mavlink_msg_hil_state_quaternion_decode(&rx_msg, &pkt);
            State candidate;
            if (!decode_state(pkt, candidate) ||
                !sent_recently(candidate.time_us) ||
                (have_state_ && candidate.time_us <= applied.time_us)) {
                // corrupt, unknown, from before a reboot, or not newer
                // than the applied state
                stats_.rejected++;
                continue;
            }
            if (have_newest) {
                // two acceptable states in one read: the newer one wins
                stats_.replaced++;
                if (candidate.time_us <= newest.time_us) {
                    continue;
                }
            }
            newest = candidate;
            have_newest = true;
        }
    }
    if (!have_newest) {
        return false;
    }
    applied = newest;
    have_state_ = true;
    state_since_frame = true;
    return true;
}

uint32_t HILLink::state_age_us(uint64_t now_us) const
{
    if (!have_state_ || now_us <= applied.time_us) {
        return 0;
    }
    return uint32_t(MIN(now_us - applied.time_us, uint64_t(UINT32_MAX)));
}

HILLink::LinkState HILLink::compute_link_state(uint64_t now_us) const
{
    if (uart == nullptr) {
        return LinkState::NO_PORT;
    }
    if (!have_state_) {
        return LinkState::WAITING;
    }
    return state_age_us(now_us) > LOST_AGE_US ? LinkState::LOST : LinkState::OK;
}

bool HILLink::update_link_state(uint64_t now_us, LinkState &previous)
{
    const LinkState now_state = compute_link_state(now_us);
    if (now_state == link_state_) {
        return false;
    }
    previous = link_state_;
    link_state_ = now_state;
    return true;
}

#endif // AP_SIM_HIL_ENABLED
