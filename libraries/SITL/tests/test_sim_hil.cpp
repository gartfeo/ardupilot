/*
  tests of the HIL backend's Aircraft state and of the simulated airspeed
  sensor it feeds (SITL::SIM::airspeed_sensor_pressure, the model the SITL
  HAL and simulation on hardware share)
 */
#include <AP_gtest.h>

#include <SITL/SITL.h>
#include <SITL/SIM_HIL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Scheduler/AP_Scheduler.h>

#if AP_SIM_HIL_ENABLED

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// the SIM parameters the simulated sensors read (AP::sitl())
static SITL::SIM sim;
// Aircraft::fill_fdm() logs its SIM2 record once per scheduler tick
// (AP::scheduler(), AP::logger()); this logger has no backend
static AP_Scheduler scheduler;
static AP_Logger logger;

// float rounding of a square and a square root at up to 35 m/s
static constexpr float AIRSPEED_TOLERANCE_MPS = 1.0e-4f;
// position round trip through the Location distance functions
static constexpr double POSITION_TOLERANCE_M = 0.01;
// samples of the noisy sensor pressure, and the spread they must show at
// least, as a fraction of SIM_ARSPD_RND: the noise is uniform over
// +-SIM_ARSPD_RND, so 50 samples span less than half of that range with
// a probability of about 1e-13
static constexpr uint8_t NOISE_SAMPLES = 50;
static constexpr float MIN_NOISE_SPREAD_FRACTION = 0.5f;

// what an airspeed driver makes of the differential pressure
// (AP_Airspeed: airspeed = sqrt(pressure * ARSPD_RATIO))
static float sensor_airspeed(float pressure, float ratio)
{
    return sqrtf(fabsf(pressure) * ratio);
}

class AirspeedSensorModel : public ::testing::Test {
protected:
    void SetUp() override {
        sim.airspeed[0].noise_param.set(0);
        sim.airspeed[0].fail.set(0);
        sim.airspeed[0].fail_pressure.set(0);
        sim.airspeed[0].signflip.set(0);
    }
};

// the sensor reads the equivalent airspeed it is given, at any altitude:
// no second EAS2TAS division (upstream b99fdd751a, PR #33713)
TEST_F(AirspeedSensorModel, reads_the_equivalent_airspeed_at_every_altitude)
{
    const float ratio = sim.airspeed[0].ratio;
    // sea level, a 584 m field, a 1400 m field, 3000 m
    const float alts_m[] {0.0f, 584.0f, 1400.0f, 3000.0f};
    // at rest, near stall, cruise, dive
    const float speeds_mps[] {0.0f, 12.0f, 20.0f, 35.0f};
    for (const float alt : alts_m) {
        sim.state.altitude = alt;
        for (const float eas : speeds_mps) {
            const float pressure = sim.airspeed_sensor_pressure(0, eas);
            EXPECT_NEAR(pressure, sq(eas) / ratio, 1.0e-3f) << "alt " << alt << " eas " << eas;
            EXPECT_NEAR(sensor_airspeed(pressure, ratio), eas, AIRSPEED_TOLERANCE_MPS)
                << "alt " << alt << " eas " << eas;
        }
    }
}

TEST_F(AirspeedSensorModel, applies_signflip_failure_and_noise_options)
{
    const float ratio = sim.airspeed[0].ratio;
    sim.airspeed[0].signflip.set(1);
    EXPECT_LT(sim.airspeed_sensor_pressure(0, 20.0f), 0.0f);
    sim.airspeed[0].signflip.set(0);

    // a failed sensor reads SIM_ARSPD_FAIL whatever the airspeed
    sim.airspeed[0].fail.set(7.0f);
    EXPECT_NEAR(sensor_airspeed(sim.airspeed_sensor_pressure(0, 20.0f), ratio), 7.0f, AIRSPEED_TOLERANCE_MPS);
    sim.airspeed[0].fail.set(0);

    // pressure noise: present, and within SIM_ARSPD_RND (in Pa) of the
    // noiseless pressure
    const float noise_pa = 2.0f;
    sim.airspeed[0].noise_param.set(noise_pa);
    ASSERT_FALSE(sim.noise_category_off(SITL::SIM::NOISE_AIRSPEED));
    const float noiseless = sq(20.0f) / ratio;
    float lowest = noiseless;
    float highest = noiseless;
    for (uint8_t i = 0; i < NOISE_SAMPLES; i++) {
        const float pressure = sim.airspeed_sensor_pressure(0, 20.0f);
        EXPECT_NEAR(pressure, noiseless, noise_pa + 1.0e-3f);
        lowest = MIN(lowest, pressure);
        highest = MAX(highest, pressure);
    }
    EXPECT_GT(highest - lowest, MIN_NOISE_SPREAD_FRACTION * noise_pa);

    // SIM_NOISE_OFF's airspeed category turns the noise off
    sim.noise_off.set(int32_t(1U << SITL::SIM::NOISE_AIRSPEED));
    for (uint8_t i = 0; i < NOISE_SAMPLES; i++) {
        EXPECT_NEAR(sim.airspeed_sensor_pressure(0, 20.0f), noiseless, 1.0e-3f);
    }
    sim.noise_off.set(0);
}

// exposes the Aircraft fields that only fill_fdm() reads
class TestHIL : public SITL::HIL {
public:
    TestHIL() : SITL::HIL("hil") {}
    float pitot() const { return airspeed_pitot; }
    const Vector3d &pos() const { return position; }
    const Vector3f &accel() const { return accel_body; }
    const Vector3f &air_bf() const { return velocity_air_bf; }
    const Vector3f &wind() const { return wind_ef; }
};

static SITL::HILLink::State state_at(const Location &loc)
{
    SITL::HILLink::State s {};
    s.time_us = 1000;
    s.attitude.from_euler(radians(10.0f), radians(-5.0f), radians(90.0f));
    s.gyro = Vector3f{0.1f, 0.2f, -0.3f};
    s.location = loc;
    s.velocity_ef = Vector3f{0.0f, 20.0f, -1.0f};
    s.accel_body = Vector3f{0.5f, 0.0f, -GRAVITY_MSS};
    s.ind_airspeed = 18.0f;
    s.true_airspeed = 19.5f;
    return s;
}

TEST(SIM_HIL, apply_state_sets_the_aircraft_state)
{
    TestHIL hil;
    const Location home {-353632621, 1491652374, 58400, Location::AltFrame::ABSOLUTE};
    hil.set_start_location(home, 0.0f);

    // 100 m north and 10 m above home
    Location loc = home;
    loc.offset(100.0f, 0.0f);
    loc.alt += 1000;
    const SITL::HILLink::State s = state_at(loc);
    hil.apply_state(s);

    EXPECT_EQ(hil.get_location().lat, loc.lat);
    EXPECT_EQ(hil.get_location().lng, loc.lng);
    EXPECT_EQ(hil.get_location().alt, loc.alt);
    EXPECT_NEAR(hil.pos().x, 100.0, POSITION_TOLERANCE_M);
    EXPECT_NEAR(hil.pos().y, 0.0, POSITION_TOLERANCE_M);
    EXPECT_NEAR(hil.pos().z, -10.0, POSITION_TOLERANCE_M);

    float roll, pitch, yaw;
    hil.get_dcm().to_euler(&roll, &pitch, &yaw);
    EXPECT_NEAR(degrees(roll), 10.0f, 1.0e-3f);
    EXPECT_NEAR(degrees(pitch), -5.0f, 1.0e-3f);
    EXPECT_NEAR(degrees(yaw), 90.0f, 1.0e-3f);
    EXPECT_TRUE(hil.get_gyro() == s.gyro);
    EXPECT_TRUE(hil.get_velocity_ef() == s.velocity_ef);
    EXPECT_TRUE(hil.accel() == s.accel_body);

    // the pitot gets the sent equivalent airspeed unchanged
    EXPECT_FLOAT_EQ(hil.pitot(), 18.0f);
    // the bridge owns the wind; the air-relative velocity is the true
    // airspeed along the body x axis
    EXPECT_TRUE(hil.wind().is_zero());
    EXPECT_FLOAT_EQ(hil.air_bf().x, 19.5f);
    EXPECT_NEAR((hil.get_velocity_air_ef() - hil.get_dcm() * Vector3f{19.5f, 0.0f, 0.0f}).length(), 0.0f, 1.0e-5f);
}

// from the applied state through Aircraft::fill_fdm(), as both HALs run
// it (SITL_State and SIMState fill state.airspeed_raw_pressure with
// airspeed_sensor_pressure(i, state.airspeed)): the simulated airspeed
// sensor reads the airspeed the bridge sent, at the Peron15 airfield
// heights where the double EAS2TAS division read 6-7% low. The HAL call
// itself is covered by the Plane HIL lockstep autotest (SITL) and the
// board bench check.
TEST(SIM_HIL, airspeed_sensor_reads_the_sent_airspeed)
{
    sim.airspeed[0].noise_param.set(0);
    const float ratio = sim.airspeed[0].ratio;
    TestHIL hil;
    const Location home {-353632621, 1491652374, 130000, Location::AltFrame::ABSOLUTE};
    hil.set_start_location(home, 0.0f);
    for (const int32_t alt_cm : {130000, 145000}) {
        Location loc = home;
        loc.alt = alt_cm;
        hil.apply_state(state_at(loc));
        SITL::sitl_fdm fdm {};
        hil.fill_fdm(fdm);
        EXPECT_NEAR(fdm.altitude, alt_cm * 0.01, 1.0e-6) << alt_cm;
        // the equivalent airspeed the bridge sent, not divided by EAS2TAS
        EXPECT_FLOAT_EQ(fdm.airspeed, 18.0f) << alt_cm;
        sim.state = fdm;
        const float pressure = sim.airspeed_sensor_pressure(0, sim.state.airspeed);
        EXPECT_NEAR(sensor_airspeed(pressure, ratio), 18.0f, AIRSPEED_TOLERANCE_MPS) << alt_cm;
    }
}

#endif // AP_SIM_HIL_ENABLED

AP_GTEST_MAIN()
