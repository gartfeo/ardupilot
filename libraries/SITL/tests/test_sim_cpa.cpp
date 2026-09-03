#include <AP_gtest.h>

#include <SITL/SIM_CPA.h>

#if AP_SIM_CPA_ENABLED

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using namespace SITL;

// subclass to drive protected parameters and observe protected state
class CPASimTest : public CPASim {
public:
    void set_chunks(int32_t lat_hi_v, int32_t lat_lo_v,
                    int32_t lng_hi_v, int32_t lng_lo_v, int32_t alt_cm_v)
    {
        lat_hi.set(lat_hi_v);
        lat_lo.set(lat_lo_v);
        lng_hi.set(lng_hi_v);
        lng_lo.set(lng_lo_v);
        alt_cm.set(alt_cm_v);
    }
    void set_enable(int8_t v) { enable.set(v); }
    bool active() const { return _active; }
    bool faulted() const { return _faulted; }
    uint32_t epoch() const { return _epoch; }
    uint32_t seq() const { return _seq; }
    bool glob_valid() const { return _glob_best_valid; }
    const SegmentMin &glob() const { return _glob_best; }
    bool interval_valid() const { return _int_best_valid; }
    const SegmentMin &interval_best() const { return _int_best; }
};

static const Location ORIGIN(430000000, 60000000, 10000, Location::AltFrame::ABSOLUTE);

// chunks matching ORIGIN's coordinates with target altitude == origin
// altitude, so the target-relative residual equals the raw position
static void set_origin_target(CPASimTest &sim)
{
    sim.set_chunks(43000, 0, 6000, 0, 10000);
}

TEST(CPAChunks, reassemble)
{
    EXPECT_EQ(CPASim::reassemble_chunks(43123, 4567), 431234567);
    EXPECT_EQ(CPASim::reassemble_chunks(-43123, -4567), -431234567);
    EXPECT_EQ(CPASim::reassemble_chunks(43123, 0), 431230000);
    EXPECT_EQ(CPASim::reassemble_chunks(0, 9999), 9999);
    EXPECT_EQ(CPASim::reassemble_chunks(180000, 0), 1800000000);
}

TEST(CPAChunks, extreme_values_survive_float32_transport)
{
    // the transport premise: every chunk magnitude is exactly representable
    // in float32, so a REAL32 PARAM_SET round trip cannot alter it
    const int32_t extremes[] = {
        90000, -90000, 180000, -180000, 9999, -9999, 1000000, -1000000,
    };
    for (const int32_t v : extremes) {
        EXPECT_EQ((int32_t)(float)v, v);
    }
}

TEST(CPAChunks, validate_accepts_typical_target)
{
    int32_t lat_e7 = 0, lng_e7 = 0;
    EXPECT_TRUE(CPASim::validate_target(43123, 4567, 6123, 4567, 10000, lat_e7, lng_e7));
    EXPECT_EQ(lat_e7, 431234567);
    EXPECT_EQ(lng_e7, 61234567);
}

TEST(CPAChunks, validate_rejects_bad_configs)
{
    int32_t lat_e7 = 0, lng_e7 = 0;
    // remainder out of range
    EXPECT_FALSE(CPASim::validate_target(43123, 10000, 6123, 0, 0, lat_e7, lng_e7));
    // sign mismatch between chunks
    EXPECT_FALSE(CPASim::validate_target(43123, -1, 6123, 0, 0, lat_e7, lng_e7));
    EXPECT_FALSE(CPASim::validate_target(-43123, 1, 6123, 0, 0, lat_e7, lng_e7));
    // latitude beyond +/-90 degrees
    EXPECT_FALSE(CPASim::validate_target(90000, 1, 6123, 0, 0, lat_e7, lng_e7));
    // longitude beyond +/-180 degrees
    EXPECT_FALSE(CPASim::validate_target(43123, 0, 180000, 1, 0, lat_e7, lng_e7));
    // all-zero coordinates mean unconfigured
    EXPECT_FALSE(CPASim::validate_target(0, 0, 0, 0, 0, lat_e7, lng_e7));
    // altitude outside +/-10 km
    EXPECT_FALSE(CPASim::validate_target(43123, 0, 6123, 0, 1000001, lat_e7, lng_e7));
    EXPECT_FALSE(CPASim::validate_target(43123, 0, 6123, 0, -1000001, lat_e7, lng_e7));
}

TEST(CPASegment, interior_minimum)
{
    const CPASim::SegmentMin m = CPASim::segment_min(
        Vector3d(-1.0, 0.5, 0.0), Vector3d(1.0, 0.5, 0.0), 1000, 3000);
    EXPECT_NEAR(sqrt(m.d3_sq), 0.5, 1e-12);
    EXPECT_NEAR(m.dh, 0.5, 1e-12);
    EXPECT_NEAR(m.dv, 0.0, 1e-12);
    EXPECT_EQ(m.time_us, 2000u);
}

TEST(CPASegment, clamps_to_endpoints)
{
    // receding: minimum at the start
    const CPASim::SegmentMin start = CPASim::segment_min(
        Vector3d(1.0, 0.0, 0.0), Vector3d(2.0, 0.0, 0.0), 1000, 2000);
    EXPECT_NEAR(sqrt(start.d3_sq), 1.0, 1e-12);
    EXPECT_EQ(start.time_us, 1000u);

    // approaching without passing: minimum at the end
    const CPASim::SegmentMin end = CPASim::segment_min(
        Vector3d(3.0, 0.0, 0.0), Vector3d(1.0, 0.0, 0.0), 1000, 2000);
    EXPECT_NEAR(sqrt(end.d3_sq), 1.0, 1e-12);
    EXPECT_EQ(end.time_us, 2000u);
}

TEST(CPASegment, zero_length_segment)
{
    const CPASim::SegmentMin m = CPASim::segment_min(
        Vector3d(2.0, 0.0, 0.0), Vector3d(2.0, 0.0, 0.0), 1000, 2000);
    EXPECT_NEAR(sqrt(m.d3_sq), 2.0, 1e-12);
    EXPECT_NEAR(m.dh, 2.0, 1e-12);
    EXPECT_NEAR(m.dv, 0.0, 1e-12);
    EXPECT_EQ(m.time_us, 1000u);
}

TEST(CPASegment, vertical_component)
{
    const CPASim::SegmentMin m = CPASim::segment_min(
        Vector3d(-1.0, 0.0, 2.0), Vector3d(1.0, 0.0, 2.0), 0, 2000);
    EXPECT_NEAR(sqrt(m.d3_sq), 2.0, 1e-12);
    EXPECT_NEAR(m.dh, 0.0, 1e-12);
    EXPECT_NEAR(m.dv, 2.0, 1e-12);
}

TEST(CPASegment, time_rounding_is_deterministic)
{
    // f = 0.5 over an odd 3 us span: llround(1.5) rounds away from zero
    const CPASim::SegmentMin m = CPASim::segment_min(
        Vector3d(-1.0, 1.0, 0.0), Vector3d(1.0, 1.0, 0.0), 1000, 1003);
    EXPECT_EQ(m.time_us, 1002u);
}

TEST(CPAFrame, anchor_shift_residual_stability)
{
    // the residual subtracts two projections from the same anchor. Near the
    // CPA the projected endpoints are close together (vehicle ~ target, and
    // the 1 Hz origin rebase keeps the origin within seconds of flight of
    // both), so the projection distortion is common mode: the DIFFERENCE of
    // two nearby points must be stable against moving the anchor kilometres
    // away, even though each absolute projection shifts by millimetres
    const Location anchor_a(430000000, 60000000, 10000, Location::AltFrame::ABSOLUTE);
    const Location anchor_b(430180000, 60000000, 10000, Location::AltFrame::ABSOLUTE);  // ~2 km north
    const Location p(430050000, 60050000, 12000, Location::AltFrame::ABSOLUTE);
    const Location t(430054500, 60050000, 12000, Location::AltFrame::ABSOLUTE);         // ~50 m from p
    const Vector3d res_a = anchor_a.get_distance_NED_double(p) - anchor_a.get_distance_NED_double(t);
    const Vector3d res_b = anchor_b.get_distance_NED_double(p) - anchor_b.get_distance_NED_double(t);
    EXPECT_NEAR((res_a - res_b).length(), 0.0, 1e-3);
}

TEST(CPALifecycle, scores_a_straight_pass)
{
    CPASimTest sim;
    set_origin_target(sim);
    sim.set_enable(1);

    uint64_t t = 1000000;
    // seed 3 m abeam of the target, then fly a straight pass
    sim.update(Vector3d(-100.0, 3.0, 0.0), ORIGIN, t);
    EXPECT_TRUE(sim.active());
    EXPECT_EQ(sim.epoch(), 1u);

    for (int i = 1; i <= 200; i++) {
        t += 1000;
        sim.update(Vector3d(-100.0 + i, 3.0, 0.0), ORIGIN, t);
    }
    ASSERT_TRUE(sim.glob_valid());
    EXPECT_NEAR(sqrt(sim.glob().d3_sq), 3.0, 1e-9);
    EXPECT_NEAR(sim.glob().dh, 3.0, 1e-9);
    EXPECT_NEAR(sim.glob().dv, 0.0, 1e-9);
    // closest sample is exactly 100 steps after the seed
    EXPECT_EQ(sim.glob().time_us, 1100000u);
    // 200 ms of flight closed ten 20 ms intervals
    EXPECT_EQ(sim.seq(), 10u);
}

TEST(CPALifecycle, one_segment_spanning_multiple_boundaries)
{
    CPASimTest sim;
    set_origin_target(sim);
    sim.set_enable(1);

    // seed, then a single 50 ms physics segment crossing two 20 ms
    // boundaries; the module must split it at 1,020,000 and 1,040,000
    // with the same linear interpolation the minimum uses
    sim.update(Vector3d(-50.0, 0.0, 0.0), ORIGIN, 1000000);
    sim.update(Vector3d(50.0, 0.0, 0.0), ORIGIN, 1050000);

    EXPECT_EQ(sim.seq(), 2u);
    ASSERT_TRUE(sim.glob_valid());
    // the crossing lands inside the second interval: x runs -10 -> +30
    // over [1,020,000, 1,040,000], so x=0 at fraction 0.25
    EXPECT_NEAR(sqrt(sim.glob().d3_sq), 0.0, 1e-9);
    EXPECT_EQ(sim.glob().time_us, 1025000u);
    // the still-open third interval holds the trailing sub-segment
    // [+30, +50], whose minimum is 30 m at the 1,040,000 boundary
    ASSERT_TRUE(sim.interval_valid());
    EXPECT_NEAR(sqrt(sim.interval_best().d3_sq), 30.0, 1e-9);
    EXPECT_EQ(sim.interval_best().time_us, 1040000u);
}

TEST(CPALifecycle, retarget_starts_a_new_epoch)
{
    CPASimTest sim;
    set_origin_target(sim);
    sim.set_enable(1);

    uint64_t t = 1000000;
    sim.update(Vector3d(-100.0, 3.0, 0.0), ORIGIN, t);
    for (int i = 1; i <= 200; i++) {
        t += 1000;
        sim.update(Vector3d(-100.0 + i, 3.0, 0.0), ORIGIN, t);
    }
    ASSERT_NEAR(sqrt(sim.glob().d3_sq), 3.0, 1e-9);

    // lower the target by 10 m: config change while active = retarget
    sim.set_chunks(43000, 0, 6000, 0, 9000);
    t += 1000;
    sim.update(Vector3d(100.0, 3.0, 0.0), ORIGIN, t);
    EXPECT_TRUE(sim.active());
    EXPECT_EQ(sim.epoch(), 2u);

    // recede; the new epoch's minimum must exclude the old 3 m pass
    for (int i = 1; i <= 50; i++) {
        t += 1000;
        sim.update(Vector3d(100.0 + i, 3.0, 0.0), ORIGIN, t);
    }
    ASSERT_TRUE(sim.glob_valid());
    EXPECT_GT(sqrt(sim.glob().d3_sq), 50.0);
    EXPECT_NEAR(sim.glob().dv, 10.0, 1e-9);
}

TEST(CPALifecycle, non_monotonic_time_faults_until_enable_cycles)
{
    CPASimTest sim;
    set_origin_target(sim);
    sim.set_enable(1);

    uint64_t t = 1000000;
    sim.update(Vector3d(-100.0, 3.0, 0.0), ORIGIN, t);
    t += 1000;
    sim.update(Vector3d(-99.0, 3.0, 0.0), ORIGIN, t);
    EXPECT_TRUE(sim.active());

    // time going backwards is unknown corruption: fail closed
    sim.update(Vector3d(-98.0, 3.0, 0.0), ORIGIN, t - 500);
    EXPECT_TRUE(sim.faulted());
    EXPECT_FALSE(sim.active());

    // staying enabled must not recover
    t += 1000;
    sim.update(Vector3d(-97.0, 3.0, 0.0), ORIGIN, t);
    EXPECT_TRUE(sim.faulted());
    EXPECT_FALSE(sim.active());

    // cycling ENABLE off and on recovers with a fresh epoch, and the new
    // epoch's scoring shares no segment with the pre-fault trajectory
    sim.set_enable(0);
    t += 1000;
    sim.update(Vector3d(-96.0, 3.0, 0.0), ORIGIN, t);
    EXPECT_FALSE(sim.faulted());
    sim.set_enable(1);
    t += 1000;
    sim.update(Vector3d(500.0, 0.0, 0.0), ORIGIN, t);
    EXPECT_TRUE(sim.active());
    EXPECT_EQ(sim.epoch(), 2u);
    t += 1000;
    sim.update(Vector3d(499.0, 0.0, 0.0), ORIGIN, t);
    ASSERT_TRUE(sim.glob_valid());
    EXPECT_GT(sqrt(sim.glob().d3_sq), 400.0);
}

TEST(CPALifecycle, non_finite_state_faults)
{
    CPASimTest sim;
    set_origin_target(sim);
    sim.set_enable(1);

    uint64_t t = 1000000;
    sim.update(Vector3d(-100.0, 3.0, 0.0), ORIGIN, t);
    t += 1000;
    sim.update(Vector3d(NAN, 3.0, 0.0), ORIGIN, t);
    EXPECT_TRUE(sim.faulted());
    EXPECT_FALSE(sim.active());
}

TEST(CPALifecycle, origin_alt_change_faults)
{
    CPASimTest sim;
    set_origin_target(sim);
    sim.set_enable(1);

    uint64_t t = 1000000;
    sim.update(Vector3d(-100.0, 3.0, 0.0), ORIGIN, t);
    // the simulator only ever rebases origin lat/lng; an altitude change
    // is unknown corruption and must fail the epoch closed
    const Location moved(430000000, 60000000, 10100, Location::AltFrame::ABSOLUTE);
    t += 1000;
    sim.update(Vector3d(-99.0, 3.0, 0.0), moved, t);
    EXPECT_TRUE(sim.faulted());
    EXPECT_FALSE(sim.active());
}

TEST(CPALifecycle, origin_rebase_keeps_scoring_continuous)
{
    CPASimTest sim;
    set_origin_target(sim);
    sim.set_enable(1);

    // approach from the west, 3 m abeam
    uint64_t t = 1000000;
    Location origin = ORIGIN;
    Vector3d pos(-100.0, 3.0, 0.0);
    sim.update(pos, origin, t);
    for (int i = 1; i <= 50; i++) {
        t += 1000;
        pos.x += 1.0;
        sim.update(pos, origin, t);
    }

    // replicate Aircraft::update_position's 1 Hz rebase EXACTLY: shift the
    // origin to (roughly) the current location and subtract the same NE
    // vector from the raw position, so the physical point is unchanged
    Location new_origin = origin;
    new_origin.lat += 4500;  // ~50 m north of the old origin
    new_origin.lng += 2000;
    const Vector2d diff = origin.get_distance_NE_double(new_origin);
    pos.x -= diff.x;
    pos.y -= diff.y;
    origin = new_origin;

    // keep flying through the pass under the new anchor
    for (int i = 1; i <= 150; i++) {
        t += 1000;
        pos.x += 1.0;
        sim.update(pos, origin, t);
    }

    // no fault, same epoch, and the minimum matches the un-rebased pass
    // to well under a millimetre (the rebase is model-continuous)
    EXPECT_FALSE(sim.faulted());
    EXPECT_TRUE(sim.active());
    EXPECT_EQ(sim.epoch(), 1u);
    ASSERT_TRUE(sim.glob_valid());
    EXPECT_NEAR(sqrt(sim.glob().d3_sq), 3.0, 1e-4);
    EXPECT_NEAR(sim.glob().dv, 0.0, 1e-6);
}

TEST(CPALifecycle, invalid_config_does_not_activate)
{
    CPASimTest sim;
    // null island target must never activate scoring
    sim.set_chunks(0, 0, 0, 0, 0);
    sim.set_enable(1);
    uint64_t t = 1000000;
    for (int i = 0; i < 5; i++) {
        t += 1000;
        sim.update(Vector3d(-100.0, 3.0, 0.0), ORIGIN, t);
    }
    EXPECT_FALSE(sim.active());
    EXPECT_FALSE(sim.faulted());

    // correcting the config activates without cycling ENABLE
    set_origin_target(sim);
    t += 1000;
    sim.update(Vector3d(-100.0, 3.0, 0.0), ORIGIN, t);
    EXPECT_TRUE(sim.active());
    EXPECT_EQ(sim.epoch(), 1u);
}

#endif  // AP_SIM_CPA_ENABLED

AP_GTEST_MAIN()
