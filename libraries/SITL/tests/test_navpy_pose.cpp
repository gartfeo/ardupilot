#include <AP_gtest.h>
#include <SITL/SIM_NavPyPose.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
using namespace SITL;

static Location convert(const Location &origin, const Location &home, const Vector3d &p)
{
    Location result = origin;
    result.offset(p.x, p.y); // independent, linked production Location helper
    result.alt = static_cast<int32_t>(home.alt - p.z * 100.0f);
    return result;
}

TEST(NavPyPose, matches_real_location_for_signed_offsets_and_altitude)
{
    for (int sign : {-1, 1}) {
        Location origin(sign*403117414, sign*444552111, sign*129485, Location::AltFrame::ABSOLUTE);
        for (double north : {-1000.125, -25.999, -0.011, -0.0001, 0., 0.0001, 0.011, 25.999, 1000.125}) {
            for (double east : {-100.9, -0.002, 0., 0.002, 100.9}) {
                for (double down : {-13.9999, -0.005, 0., 0.005, 13.9999}) {
                    Vector3d position(north, east, down);
                    const auto rounded = convert(origin, origin, position);
                    const auto s = NavPyPoseSample::capture(origin, origin, position, rounded, 1, 1, 100, 1);
                    ASSERT_TRUE(s.valid);
                    EXPECT_EQ(s.ftype_size, 8);
                    EXPECT_DOUBLE_EQ(s.precast[0], (double(origin.lat)+s.dlat)*1e-7);
                    EXPECT_DOUBLE_EQ(s.precast[1], (double(origin.lng)+s.dlng)*1e-7);
                    EXPECT_DOUBLE_EQ(s.precast[2], s.altitude_cm*1e-2);
                    EXPECT_EQ(origin.lat + int32_t(s.dlat), rounded.lat);
                    EXPECT_EQ(int64_t(origin.lng) + int64_t(s.dlng), rounded.lng);
                    EXPECT_EQ(int32_t(s.altitude_cm), rounded.alt);
                }
            }
        }
    }
}

TEST(NavPyPose, exact_and_adjacent_truncation_boundaries)
{
    const Location origin(403117414,444552111,129485,Location::AltFrame::ABSOLUTE);
    constexpr float inverse = LATLON_TO_M_INV;
    for (double boundary : {-100., -1., 0., 1., 100.}) {
        for (double epsilon : {-1e-10, 0., 1e-10}) {
            const Vector3d position((boundary+epsilon)/inverse,0.,(boundary+epsilon)/100.);
            const auto rounded = convert(origin,origin,position);
            EXPECT_TRUE(NavPyPoseSample::capture(origin,origin,position,rounded,1,1,100,1).valid);
        }
    }
}

TEST(NavPyPose, rejects_nonfinite_wrapping_and_mismatched_shadow)
{
    const Location origin(403117414,444552111,129485,Location::AltFrame::ABSOLUTE);
    EXPECT_FALSE(NavPyPoseSample::capture(origin,origin,Vector3d(NAN,0,0),origin,1,1,100,1).valid);
    EXPECT_FALSE(NavPyPoseSample::capture(origin,origin,Vector3d(0,INFINITY,0),origin,1,1,100,1).valid);
    EXPECT_FALSE(NavPyPoseSample::capture(origin,origin,Vector3d(0,0,NAN),origin,1,1,100,1).valid);
    EXPECT_FALSE(NavPyPoseSample::capture(origin,origin,Vector3d(1e30,0,0),origin,1,1,100,1).valid);
    Location edge(899999999,1799999999,0,Location::AltFrame::ABSOLUTE);
    EXPECT_FALSE(NavPyPoseSample::capture(edge,edge,Vector3d(100.,0,0),edge,1,1,100,1).valid);
    edge.lat = 0;
    EXPECT_FALSE(NavPyPoseSample::capture(edge,edge,Vector3d(0,100.,0),edge,1,1,100,1).valid);
    EXPECT_FALSE(NavPyPoseSample::capture(origin,origin,Vector3d(1.,0,0),origin,1,1,100,1).valid);
}

TEST(NavPyPose, reset_preserves_conversion_before_origin_shift)
{
    NavPyPoseCapture capture;
    capture.enabled = true;
    Location origin(403117414,444552111,129485,Location::AltFrame::ABSOLUTE);
    const Location home = origin;
    Vector3d p(25.555,-3.123,1.234);
    const auto rounded = convert(origin,home,p);
    capture.raw(origin,home,p,rounded,1000);
    const int32_t original_origin = origin.lat;
    const Vector2d shift = origin.get_distance_NE_double(rounded);
    p.xy() -= shift;
    origin.lat = rounded.lat;
    origin.lng = rounded.lng;
    capture.reset_smoothing();
    const auto s = capture.publish(2000,true,rounded); // Plane advances time before FDM
    ASSERT_TRUE(s.valid);
    EXPECT_EQ(s.branch,3);
    EXPECT_EQ(s.conversion_us,1000U);
    EXPECT_EQ(s.publish_us,2000U);
    EXPECT_EQ(s.origin_lat,original_origin);
    EXPECT_EQ(s.rounded_lat,rounded.lat);
    EXPECT_EQ(s.physics_sequence,s.expected_physics_sequence);
    EXPECT_EQ(s.origin_lat+int32_t(s.dlat),rounded.lat);
    EXPECT_FALSE(capture.publish(2001,true,rounded).valid); // duplicate publish
}

TEST(NavPyPose, normal_smoothing_and_stale_generation)
{
    NavPyPoseCapture capture;
    capture.enabled = true;
    const Location origin(403117414,444552111,129485,Location::AltFrame::ABSOLUTE);
    const Vector3d p(2.555,3.123,1.234);
    const auto rounded = convert(origin,origin,p);
    capture.raw(origin,origin,p,rounded,1000);
    capture.smooth(origin,origin,p,rounded,2000);
    ASSERT_TRUE(capture.publish(2000,true,rounded).valid);
    capture.raw(origin,origin,p,rounded,2000);
    EXPECT_FALSE(capture.publish(3000,true,rounded).valid); // old smoothed generation
    capture.raw(origin,origin,p,rounded,3000);
    capture.smooth(origin,origin,p,rounded,4000);
    capture.stale_smoothing();
    EXPECT_FALSE(capture.publish(4000,true,rounded).valid);
    capture.raw(origin,origin,p,rounded,4000);
    EXPECT_TRUE(capture.publish(5000,false,rounded).valid);
    capture.enabled = false;
    capture.raw(origin,origin,p,rounded,5000);
    EXPECT_FALSE(capture.publish(6000,false,rounded).valid);
}

TEST(NavPyPose, distinct_home_and_smoothed_conversion_after_rebase)
{
    NavPyPoseCapture capture;
    capture.enabled = true;
    const Location home(403117414,444552111,129485,Location::AltFrame::ABSOLUTE);
    Location origin = home;
    origin.offset(700.,-200.);
    origin.alt += 10000; // horizontal origin height must not become home height
    Vector3d p(25.555,-3.123,1.234);
    const auto raw = convert(origin,home,p);
    capture.raw(origin,home,p,raw,1000);
    const Vector2d shift = origin.get_distance_NE_double(raw);
    p.xy() -= shift;
    origin.lat = raw.lat;
    origin.lng = raw.lng;
    const Vector3d smooth_position = p + Vector3d(0.004,-0.006,0.003);
    const auto smoothed = convert(origin,home,smooth_position);
    capture.smooth(origin,home,smooth_position,smoothed,2000);
    const auto s = capture.publish(2000,true,smoothed);
    ASSERT_TRUE(s.valid);
    EXPECT_EQ(s.branch,2);
    EXPECT_EQ(s.origin_lat,origin.lat);
    EXPECT_EQ(s.home_alt,home.alt);
    EXPECT_EQ(s.rounded_alt,smoothed.alt);
    EXPECT_EQ(s.origin_lat+int32_t(s.dlat),smoothed.lat);
    EXPECT_EQ(s.conversion_us,s.publish_us);
}

TEST(NavPyPose, rejects_location_changed_between_conversion_and_publish)
{
    NavPyPoseCapture capture;
    capture.enabled = true;
    const Location origin(403117414,444552111,129485,Location::AltFrame::ABSOLUTE);
    const Vector3d p(2.555,3.123,1.234);
    const auto rounded = convert(origin,origin,p);
    capture.raw(origin,origin,p,rounded,1000);
    capture.smooth(origin,origin,p,rounded,2000);
    Location changed = rounded;
    changed.lat++;
    EXPECT_FALSE(capture.publish(2000,true,changed).valid);
}

AP_GTEST_MAIN()
