#include <AP_gtest.h>
#include <SITL/SIM_NavPyPose.h>
#include <cstring>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

TEST(NavPyPoseLiterals, producer_scales_have_python_double_bits)
{
    for (int sign : {-1, 1}) {
        const Location origin(sign,sign,sign,Location::AltFrame::ABSOLUTE);
        const auto s = SITL::NavPyPoseSample::capture(origin,origin,Vector3d(0,0,0),origin,1,1,100,1);
        ASSERT_TRUE(s.valid);
        uint64_t bits[3];
        memcpy(bits,s.precast,sizeof(bits));
        const uint64_t negative = sign < 0 ? UINT64_C(0x8000000000000000) : 0;
        EXPECT_EQ(bits[0],UINT64_C(0x3e7ad7f29abcaf48) | negative);
        EXPECT_EQ(bits[1],UINT64_C(0x3e7ad7f29abcaf48) | negative);
        EXPECT_EQ(bits[2],UINT64_C(0x3f847ae147ae147b) | negative);
    }
}

AP_GTEST_MAIN()
