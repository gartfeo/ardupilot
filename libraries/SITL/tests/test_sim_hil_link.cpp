/*
  tests of the board side of the HIL link (SITL::HILLink, contract v2)
  through a fake UART: framing, acceptance rules, counters, link state
 */
#include <AP_gtest.h>

#include <SITL/SIM_HIL_Link.h>

#if AP_SIM_HIL_ENABLED

#include <vector>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using SITL::HILLink;
using LinkState = HILLink::LinkState;

static constexpr uint8_t BOARD_SYSID = 1;
static constexpr uint8_t BRIDGE_SYSID = 251;

/*
  a UART that records writes and serves injected bytes. txspace and the
  number of bytes a write takes are settable; arrives_during_read adds
  bytes while the link reads, as a real port receives during a read.
  AP_HAL::UARTDriver does not initialise its members (lock keys, read
  monitor): real ports are static objects or come from ArduPilot's
  zeroing operator new. Every FakeUART here is therefore a member of the
  test fixture, which gtest allocates with new; never one on the stack.
 */
class FakeUART : public AP_HAL::UARTDriver {
public:
    bool is_initialized() override { return true; }
    bool tx_pending() override { return false; }
    uint32_t txspace() override { return tx_space; }

    void inject(const uint8_t *data, size_t len) {
        rx.insert(rx.end(), data, data + len);
    }
    void inject(const std::vector<uint8_t> &data) {
        inject(data.data(), data.size());
    }

    std::vector<uint8_t> rx;
    size_t rx_pos = 0;
    std::vector<uint8_t> tx;
    uint32_t tx_space = 4096;
    size_t write_limit = SIZE_MAX;
    std::vector<uint8_t> arrives_during_read;

protected:
    void _begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) override {}
    void _end() override {}
    void _flush() override {}
    size_t _write(const uint8_t *buffer, size_t size) override {
        const size_t n = MIN(size, write_limit);
        tx.insert(tx.end(), buffer, buffer + n);
        return n;
    }
    ssize_t _read(uint8_t *buffer, uint16_t count) override {
        const size_t n = MIN(size_t(count), rx.size() - rx_pos);
        memcpy(buffer, rx.data() + rx_pos, n);
        rx_pos += n;
        if (!arrives_during_read.empty()) {
            inject(arrives_during_read);
            arrives_during_read.clear();
        }
        return n;
    }
    uint32_t _available() override { return rx.size() - rx_pos; }
    bool _discard_input() override { rx.clear(); rx_pos = 0; return true; }
};

// a valid state answering frame time_us: level, at rest, 584 m AMSL
static mavlink_hil_state_quaternion_t valid_state(uint64_t time_us)
{
    mavlink_hil_state_quaternion_t s {};
    s.time_usec = time_us;
    s.attitude_quaternion[0] = 1.0f;
    s.lat = -353632621;
    s.lon = 1491652374;
    s.alt = 584000;
    s.zacc = -1000;
    return s;
}

// a state as the bridge sends it on the wire
static std::vector<uint8_t> frame(const mavlink_hil_state_quaternion_t &s,
                                  uint8_t sysid = BRIDGE_SYSID, bool mavlink1 = false)
{
    mavlink_status_t status {};
    if (mavlink1) {
        status.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    }
    mavlink_message_t msg;
    mavlink_msg_hil_state_quaternion_encode_status(sysid, MAV_COMP_ID_ALL, &status, &msg, &s);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    return std::vector<uint8_t>(buf, buf + len);
}

static std::vector<uint8_t> frame_for(uint64_t time_us)
{
    return frame(valid_state(time_us));
}

// decode the HIL_ACTUATOR_CONTROLS frames the link wrote
static std::vector<mavlink_message_t> sent_messages(const FakeUART &uart)
{
    std::vector<mavlink_message_t> out;
    mavlink_message_t buffer {}, msg {};
    mavlink_status_t status {}, result {};
    for (const uint8_t c : uart.tx) {
        if (mavlink_frame_char_buffer(&buffer, &status, c, &msg, &result) == MAVLINK_FRAMING_OK) {
            out.push_back(msg);
        }
    }
    return out;
}

static const uint16_t SERVOS[HILLink::NUM_CONTROLS] {
    1500, 1100, 1900, 1000, 0, 2000, 1234, 0, 0, 0, 0, 0, 0, 0, 1700, 1300
};

class HILLinkTest : public ::testing::Test {
protected:
    void SetUp() override {
        link.set_port(&uart);
    }
    bool send(uint64_t time_us, bool lockstep = false) {
        return link.send_actuators(time_us, SERVOS, false, lockstep, BOARD_SYSID);
    }
    bool receive() {
        return link.receive(BOARD_SYSID);
    }
    FakeUART uart;
    HILLink link;
    // a second port, e.g. for the board after a reboot
    FakeUART uart2;
};

// ---------------------------------------------------------------- TX

TEST_F(HILLinkTest, sends_actuator_frame_per_contract)
{
    ASSERT_TRUE(link.send_actuators(123456, SERVOS, true, false, BOARD_SYSID));
    const auto msgs = sent_messages(uart);
    ASSERT_EQ(msgs.size(), 1U);
    const mavlink_message_t &msg = msgs[0];
    EXPECT_EQ(msg.msgid, uint32_t(MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS));
    EXPECT_EQ(msg.magic, MAVLINK_STX);
    EXPECT_EQ(msg.sysid, BOARD_SYSID);
    EXPECT_EQ(msg.compid, MAV_COMP_ID_AUTOPILOT1);
    mavlink_hil_actuator_controls_t pkt;
    mavlink_msg_hil_actuator_controls_decode(&msg, &pkt);
    EXPECT_EQ(pkt.time_usec, 123456U);
    for (uint8_t i = 0; i < HILLink::NUM_CONTROLS; i++) {
        EXPECT_FLOAT_EQ(pkt.controls[i], float(SERVOS[i])) << "channel " << unsigned(i + 1);
    }
    EXPECT_EQ(pkt.mode, MAV_MODE_FLAG_SAFETY_ARMED);
    // no frame counter: flags is 0 in real-time mode
    EXPECT_EQ(pkt.flags, 0U);
    EXPECT_EQ(link.stats().tx, 1U);
    EXPECT_EQ(link.stats().tx_drop, 0U);
}

TEST_F(HILLinkTest, disarmed_mode_and_lockstep_flag)
{
    ASSERT_TRUE(send(1000, true));
    ASSERT_TRUE(send(2000, false));
    const auto msgs = sent_messages(uart);
    ASSERT_EQ(msgs.size(), 2U);
    mavlink_hil_actuator_controls_t pkt;
    mavlink_msg_hil_actuator_controls_decode(&msgs[0], &pkt);
    EXPECT_EQ(pkt.mode, 0);
    const uint64_t lockstep_flag = HILLink::ACTUATOR_FLAG_LOCKSTEP;
    EXPECT_EQ(pkt.flags, lockstep_flag);
    mavlink_msg_hil_actuator_controls_decode(&msgs[1], &pkt);
    EXPECT_EQ(pkt.flags, 0U);
}

TEST_F(HILLinkTest, frame_without_room_is_skipped_whole)
{
    uart.tx_space = HILLink::ACTUATOR_FRAME_MAX_LEN - 1;
    EXPECT_FALSE(link.tx_room());
    EXPECT_FALSE(send(1000));
    EXPECT_TRUE(uart.tx.empty());
    EXPECT_EQ(link.stats().tx, 0U);
    EXPECT_EQ(link.stats().tx_drop, 1U);
    // a frame that was never sent cannot be answered
    uart.inject(frame_for(1000));
    EXPECT_FALSE(receive());
    EXPECT_EQ(link.stats().rejected, 1U);
}

TEST_F(HILLinkTest, short_write_counts_as_drop_and_is_not_answerable)
{
    uart.write_limit = 10;
    EXPECT_FALSE(send(1000));
    EXPECT_EQ(link.stats().tx, 0U);
    EXPECT_EQ(link.stats().tx_drop, 1U);
    uart.inject(frame_for(1000));
    EXPECT_FALSE(receive());
    EXPECT_EQ(link.stats().rejected, 1U);
}

TEST_F(HILLinkTest, no_port_sends_and_receives_nothing)
{
    HILLink no_port;
    EXPECT_FALSE(no_port.tx_room());
    EXPECT_FALSE(no_port.send_actuators(1000, SERVOS, false, false, BOARD_SYSID));
    EXPECT_FALSE(no_port.receive(BOARD_SYSID));
    EXPECT_EQ(no_port.stats().tx_drop, 0U);
}

// ---------------------------------------------------------------- RX acceptance

TEST_F(HILLinkTest, applies_reply_to_a_sent_frame_with_unit_conversion)
{
    ASSERT_TRUE(send(5000));
    mavlink_hil_state_quaternion_t s = valid_state(5000);
    // 30 degrees of roll
    const float half = radians(30.0f) * 0.5f;
    s.attitude_quaternion[0] = cosf(half);
    s.attitude_quaternion[1] = sinf(half);
    s.rollspeed = 0.1f;
    s.pitchspeed = -0.2f;
    s.yawspeed = 0.3f;
    s.alt = 584005;     // rounds half away from zero to 58401 cm
    s.vx = 1500;
    s.vy = -250;
    s.vz = 10;
    s.xacc = 100;
    s.yacc = -50;
    s.zacc = -1000;
    s.ind_airspeed = 1810;
    s.true_airspeed = 1900;
    uart.inject(frame(s));
    ASSERT_TRUE(receive());

    const HILLink::State &st = link.state();
    EXPECT_EQ(st.time_us, 5000U);
    float roll, pitch, yaw;
    st.attitude.to_euler(roll, pitch, yaw);
    EXPECT_NEAR(degrees(roll), 30.0f, 1.0e-4f);
    EXPECT_NEAR(pitch, 0.0f, 1.0e-6f);
    EXPECT_NEAR(yaw, 0.0f, 1.0e-6f);
    EXPECT_FLOAT_EQ(st.gyro.x, 0.1f);
    EXPECT_FLOAT_EQ(st.gyro.y, -0.2f);
    EXPECT_FLOAT_EQ(st.gyro.z, 0.3f);
    EXPECT_EQ(st.location.lat, s.lat);
    EXPECT_EQ(st.location.lng, s.lon);
    EXPECT_EQ(st.location.alt, 58401);
    EXPECT_FALSE(st.location.relative_alt);
    EXPECT_FLOAT_EQ(st.velocity_ef.x, 15.0f);
    EXPECT_FLOAT_EQ(st.velocity_ef.y, -2.5f);
    EXPECT_FLOAT_EQ(st.velocity_ef.z, 0.1f);
    EXPECT_FLOAT_EQ(st.accel_body.x, 0.1f * GRAVITY_MSS);
    EXPECT_FLOAT_EQ(st.accel_body.y, -0.05f * GRAVITY_MSS);
    EXPECT_FLOAT_EQ(st.accel_body.z, -GRAVITY_MSS);
    EXPECT_FLOAT_EQ(st.ind_airspeed, 18.1f);
    EXPECT_FLOAT_EQ(st.true_airspeed, 19.0f);
    EXPECT_EQ(link.stats().rx, 1U);
    EXPECT_EQ(link.stats().rejected, 0U);
}

TEST_F(HILLinkTest, rejects_times_that_answer_no_sent_frame)
{
    ASSERT_TRUE(send(5000));
    // zero, unknown, in the future
    uart.inject(frame_for(0));
    uart.inject(frame_for(4999));
    uart.inject(frame_for(9999));
    EXPECT_FALSE(receive());
    EXPECT_FALSE(link.have_state());
    EXPECT_EQ(link.stats().rx, 3U);
    EXPECT_EQ(link.stats().rejected, 3U);
}

TEST_F(HILLinkTest, rejects_replies_from_before_a_reboot)
{
    ASSERT_TRUE(send(7000000));
    uart.inject(frame_for(7000000));
    ASSERT_TRUE(receive());

    // the board reboots: a reply still queued from the old boot arrives
    // before the new link has sent anything, and again once the new
    // boot's clock (9 s) has passed the old frame's time, which v1
    // accepted (not zero, not in the board's future)
    HILLink rebooted;
    rebooted.set_port(&uart2);
    uart2.inject(frame_for(7000000));
    EXPECT_FALSE(rebooted.receive(BOARD_SYSID));
    ASSERT_TRUE(rebooted.send_actuators(9000000, SERVOS, false, false, BOARD_SYSID));
    uart2.inject(frame_for(7000000));
    EXPECT_FALSE(rebooted.receive(BOARD_SYSID));
    EXPECT_FALSE(rebooted.have_state());
    EXPECT_EQ(rebooted.stats().rejected, 2U);
}

TEST_F(HILLinkTest, rejects_states_not_newer_than_the_applied_one)
{
    ASSERT_TRUE(send(1000));
    ASSERT_TRUE(send(2000));
    uart.inject(frame_for(2000));
    ASSERT_TRUE(receive());
    // older reply (still in the ring) and a duplicate of the applied one
    uart.inject(frame_for(1000));
    uart.inject(frame_for(2000));
    EXPECT_FALSE(receive());
    EXPECT_EQ(link.state().time_us, 2000U);
    EXPECT_EQ(link.stats().rejected, 2U);
}

TEST_F(HILLinkTest, ring_holds_the_last_sent_frames_only)
{
    for (uint32_t i = 0; i <= HILLink::SENT_RING_SIZE; i++) {
        ASSERT_TRUE(send(1000 * (i + 1)));
    }
    // the first frame fell out of the ring
    uart.inject(frame_for(1000));
    EXPECT_FALSE(receive());
    EXPECT_EQ(link.stats().rejected, 1U);
    // the second is the oldest one kept
    uart.inject(frame_for(2000));
    EXPECT_TRUE(receive());
}

TEST_F(HILLinkTest, newest_state_in_a_burst_wins)
{
    ASSERT_TRUE(send(1000));
    ASSERT_TRUE(send(2000));
    ASSERT_TRUE(send(3000));
    uart.inject(frame_for(1000));
    uart.inject(frame_for(3000));
    uart.inject(frame_for(2000));
    ASSERT_TRUE(receive());
    EXPECT_EQ(link.state().time_us, 3000U);
    EXPECT_EQ(link.stats().rx, 3U);
    EXPECT_EQ(link.stats().replaced, 2U);
    EXPECT_EQ(link.stats().rejected, 0U);
}

TEST_F(HILLinkTest, frame_split_across_reads_is_parsed)
{
    ASSERT_TRUE(send(1000));
    const auto bytes = frame_for(1000);
    const size_t half = bytes.size() / 2;
    uart.inject(bytes.data(), half);
    EXPECT_FALSE(receive());
    uart.inject(bytes.data() + half, bytes.size() - half);
    EXPECT_TRUE(receive());
    EXPECT_EQ(link.stats().crc_errors, 0U);
    EXPECT_EQ(link.stats().parse_errors, 0U);
}

TEST_F(HILLinkTest, reads_only_the_bytes_waiting_when_it_starts)
{
    ASSERT_TRUE(send(1000));
    ASSERT_TRUE(send(2000));
    uart.inject(frame_for(1000));
    // the reply to the second frame arrives while the first is read
    uart.arrives_during_read = frame_for(2000);
    ASSERT_TRUE(receive());
    EXPECT_EQ(link.state().time_us, 1000U);
    // it is taken by the next call
    ASSERT_TRUE(receive());
    EXPECT_EQ(link.state().time_us, 2000U);
}

TEST_F(HILLinkTest, counts_crc_and_header_errors)
{
    ASSERT_TRUE(send(1000));
    auto bad_crc = frame_for(1000);
    bad_crc[bad_crc.size() - 1] ^= 0xFF;
    uart.inject(bad_crc);
    // the start of a MAVLink 2 header with an unknown incompatibility flag
    const auto good = frame_for(1000);
    const uint8_t bad_header[] { good[0], good[1], 0x80 };
    uart.inject(bad_header, sizeof(bad_header));
    // the parser resynchronises on the next frame
    uart.inject(good);
    ASSERT_TRUE(receive());
    EXPECT_EQ(link.stats().crc_errors, 1U);
    EXPECT_EQ(link.stats().parse_errors, 1U);
    EXPECT_EQ(link.stats().rx, 1U);
    EXPECT_EQ(link.stats().rejected, 0U);
}

TEST_F(HILLinkTest, ignores_other_messages_mavlink1_and_own_sysid)
{
    ASSERT_TRUE(send(1000));
    mavlink_status_t status {};
    mavlink_message_t msg;
    mavlink_heartbeat_t hb {};
    mavlink_msg_heartbeat_encode_status(BRIDGE_SYSID, MAV_COMP_ID_ALL, &status, &msg, &hb);
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uart.inject(buf, mavlink_msg_to_send_buffer(buf, &msg));
    uart.inject(frame(valid_state(1000), BRIDGE_SYSID, true));
    uart.inject(frame(valid_state(1000), BOARD_SYSID));
    EXPECT_FALSE(receive());
    EXPECT_EQ(link.stats().other_msgs, 3U);
    EXPECT_EQ(link.stats().rx, 0U);
    EXPECT_FALSE(link.have_state());
}

// ---------------------------------------------------------------- content checks

TEST(HILLinkDecode, rejects_non_finite_and_out_of_range_content)
{
    HILLink::State out;
    ASSERT_TRUE(HILLink::decode_state(valid_state(1), out));

    auto check = [&](const char *what, void (*corrupt)(mavlink_hil_state_quaternion_t &)) {
        mavlink_hil_state_quaternion_t s = valid_state(1);
        corrupt(s);
        EXPECT_FALSE(HILLink::decode_state(s, out)) << what;
    };
    check("NaN quaternion", [](mavlink_hil_state_quaternion_t &s) { s.attitude_quaternion[2] = nanf(""); });
    check("infinite quaternion", [](mavlink_hil_state_quaternion_t &s) { s.attitude_quaternion[0] = INFINITY; });
    check("quaternion norm 1.02", [](mavlink_hil_state_quaternion_t &s) { s.attitude_quaternion[0] = 1.02f; });
    check("NaN roll rate", [](mavlink_hil_state_quaternion_t &s) { s.rollspeed = nanf(""); });
    check("infinite yaw rate", [](mavlink_hil_state_quaternion_t &s) { s.yawspeed = -INFINITY; });
    check("absurd pitch rate", [](mavlink_hil_state_quaternion_t &s) { s.pitchspeed = 1.0e30f; });
    check("latitude 90.1", [](mavlink_hil_state_quaternion_t &s) { s.lat = 901000000; });
    check("longitude -180.1", [](mavlink_hil_state_quaternion_t &s) { s.lon = -1801000000; });
    // INT32_MIN has no positive int32 counterpart: labs() on a 32-bit
    // long (the board) cannot take its magnitude, a plain bound can
    check("latitude INT32_MIN", [](mavlink_hil_state_quaternion_t &s) { s.lat = INT32_MIN; });
    check("longitude INT32_MIN", [](mavlink_hil_state_quaternion_t &s) { s.lon = INT32_MIN; });
    check("latitude INT32_MAX", [](mavlink_hil_state_quaternion_t &s) { s.lat = INT32_MAX; });
    check("longitude INT32_MAX", [](mavlink_hil_state_quaternion_t &s) { s.lon = INT32_MAX; });
    check("altitude INT32_MAX mm", [](mavlink_hil_state_quaternion_t &s) { s.alt = INT32_MAX; });
    check("altitude INT32_MIN mm", [](mavlink_hil_state_quaternion_t &s) { s.alt = INT32_MIN; });

    // slightly off unit length is accepted and normalised
    mavlink_hil_state_quaternion_t s = valid_state(1);
    s.attitude_quaternion[0] = 1.005f;
    ASSERT_TRUE(HILLink::decode_state(s, out));
    EXPECT_NEAR(out.attitude.length(), 1.0f, 1.0e-6f);
    // a fast but physical rate is accepted
    s.rollspeed = HILLink::MAX_BODY_RATE_RADS;
    EXPECT_TRUE(HILLink::decode_state(s, out));
    // the poles and the antimeridian are valid positions
    const int32_t lat_limits[] {HILLink::MAX_LAT_E7, -HILLink::MAX_LAT_E7};
    const int32_t lng_limits[] {HILLink::MAX_LNG_E7, -HILLink::MAX_LNG_E7};
    for (const int32_t lat : lat_limits) {
        for (const int32_t lng : lng_limits) {
            s.lat = lat;
            s.lon = lng;
            EXPECT_TRUE(HILLink::decode_state(s, out)) << lat << " " << lng;
        }
    }
}

TEST_F(HILLinkTest, rejected_content_is_counted)
{
    ASSERT_TRUE(send(1000));
    mavlink_hil_state_quaternion_t s = valid_state(1000);
    s.rollspeed = nanf("");
    uart.inject(frame(s));
    EXPECT_FALSE(receive());
    EXPECT_EQ(link.stats().rx, 1U);
    EXPECT_EQ(link.stats().rejected, 1U);
}

TEST(HILLinkDecode, altitude_mm_to_cm_rounds_in_64_bit)
{
    const struct {
        int32_t mm;
        int32_t cm;
    } cases[] {
        {0, 0}, {4, 0}, {5, 1}, {14, 1}, {15, 2}, {-4, 0}, {-5, -1}, {-15, -2},
        {584000, 58400}, {-430000, -43000},
        // the Location limit itself
        {int32_t(LOCATION_ALT_MAX_M) * 1000, int32_t(LOCATION_ALT_MAX_M) * 100},
        {-int32_t(LOCATION_ALT_MAX_M) * 1000, -int32_t(LOCATION_ALT_MAX_M) * 100},
    };
    for (const auto &c : cases) {
        int32_t cm = 12345;
        EXPECT_TRUE(HILLink::alt_mm_to_cm(c.mm, cm)) << c.mm;
        EXPECT_EQ(cm, c.cm) << c.mm;
    }
    int32_t cm;
    EXPECT_FALSE(HILLink::alt_mm_to_cm(int32_t(LOCATION_ALT_MAX_M) * 1000 + 5, cm));
    EXPECT_FALSE(HILLink::alt_mm_to_cm(INT32_MAX, cm));
    EXPECT_FALSE(HILLink::alt_mm_to_cm(INT32_MIN, cm));
    EXPECT_FALSE(HILLink::alt_mm_to_cm(INT32_MAX - 4, cm));
}

// ---------------------------------------------------------------- stale, age, link state

TEST_F(HILLinkTest, stale_frames_are_counted_after_the_first_state)
{
    // no state yet: not stale
    link.start_frame();
    ASSERT_TRUE(send(1000));
    link.start_frame();
    EXPECT_EQ(link.stats().stale, 0U);

    uart.inject(frame_for(1000));
    ASSERT_TRUE(receive());
    // a state arrived since the last frame
    link.start_frame();
    EXPECT_EQ(link.stats().stale, 0U);
    // none since
    link.start_frame();
    link.start_frame();
    EXPECT_EQ(link.stats().stale, 2U);
}

TEST_F(HILLinkTest, link_state_no_port_waiting_ok_lost_recovered)
{
    // a link of its own, starting without a port
    HILLink fresh;
    LinkState previous = LinkState::OK;

    // no port: the initial state, no transition
    EXPECT_FALSE(fresh.update_link_state(1000, previous));
    EXPECT_EQ(fresh.link_state(), LinkState::NO_PORT);
    EXPECT_EQ(fresh.state_age_us(1000), 0U);

    fresh.set_port(&uart2);
    ASSERT_TRUE(fresh.update_link_state(2000, previous));
    EXPECT_EQ(previous, LinkState::NO_PORT);
    EXPECT_EQ(fresh.link_state(), LinkState::WAITING);
    // reported once
    EXPECT_FALSE(fresh.update_link_state(3000, previous));

    ASSERT_TRUE(fresh.send_actuators(10000, SERVOS, false, false, BOARD_SYSID));
    uart2.inject(frame_for(10000));
    ASSERT_TRUE(fresh.receive(BOARD_SYSID));
    ASSERT_TRUE(fresh.update_link_state(15000, previous));
    EXPECT_EQ(previous, LinkState::WAITING);
    EXPECT_EQ(fresh.link_state(), LinkState::OK);
    EXPECT_EQ(fresh.state_age_us(15000), 5000U);

    // the age counts, not the arrival: at the limit still OK, beyond it LOST
    EXPECT_FALSE(fresh.update_link_state(10000 + HILLink::LOST_AGE_US, previous));
    ASSERT_TRUE(fresh.update_link_state(10000 + HILLink::LOST_AGE_US + 1, previous));
    EXPECT_EQ(previous, LinkState::OK);
    EXPECT_EQ(fresh.link_state(), LinkState::LOST);
    EXPECT_FALSE(fresh.update_link_state(500000, previous));

    // a late reply to an old frame keeps the link LOST: its age decides
    ASSERT_TRUE(fresh.send_actuators(400000, SERVOS, false, false, BOARD_SYSID));
    ASSERT_TRUE(fresh.send_actuators(510000, SERVOS, false, false, BOARD_SYSID));
    uart2.inject(frame_for(400000));
    ASSERT_TRUE(fresh.receive(BOARD_SYSID));
    EXPECT_FALSE(fresh.update_link_state(520000, previous));
    EXPECT_EQ(fresh.link_state(), LinkState::LOST);

    // a fresh state: recovered
    uart2.inject(frame_for(510000));
    ASSERT_TRUE(fresh.receive(BOARD_SYSID));
    ASSERT_TRUE(fresh.update_link_state(520000, previous));
    EXPECT_EQ(previous, LinkState::LOST);
    EXPECT_EQ(fresh.link_state(), LinkState::OK);
}

#endif // AP_SIM_HIL_ENABLED

AP_GTEST_MAIN()
