/*
  tests of the simulation-on-hardware output gate (SITL::OutputGate): only
  servo channels in SIM_OH_MASK may drive real hardware, through PWM,
  IOMCU failsafe PWM, DShot commands, GPIO or DroneCAN
 */
#include <AP_gtest.h>

#include <SITL/SITL.h>
#include <SITL/SIM_OutputGate.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

using SITL::OutputGate;

// servo channels a SIM_OH_MASK holds
static constexpr uint8_t MASK_CHANNELS = 32;
// SERVO1, SERVO3 and SERVO32 enabled
static constexpr uint32_t SOME_CHANNELS = (1U << 0) | (1U << 2) | (1U << 31);
static constexpr uint16_t PWM_US = 1500;
static constexpr uint8_t HIGH = 1;
// values, not references: C++11 needs no storage for the class constants
static constexpr uint16_t GATED_PWM = OutputGate::GATED_PWM;
static constexpr uint8_t GATED_LEVEL = OutputGate::GATED_GPIO_LEVEL;

TEST(SIM_OutputGate, mask_zero_gates_every_output)
{
    const OutputGate gate(0);
    for (uint16_t chan = 0; chan <= UINT8_MAX; chan++) {
        EXPECT_FALSE(gate.enabled(chan)) << chan;
        EXPECT_EQ(gate.pwm(chan, PWM_US), GATED_PWM) << chan;
        EXPECT_EQ(gate.gpio_level(chan, HIGH), GATED_LEVEL) << chan;
    }
    EXPECT_EQ(gate.allowed(UINT32_MAX), 0U);
    EXPECT_EQ(gate.gated(UINT32_MAX), UINT32_MAX);
    // gated means no pulse and a low pin
    EXPECT_EQ(GATED_PWM, 0U);
    EXPECT_EQ(GATED_LEVEL, 0U);
}

TEST(SIM_OutputGate, only_masked_channels_pass)
{
    const OutputGate gate(SOME_CHANNELS);
    for (uint8_t chan = 0; chan < MASK_CHANNELS; chan++) {
        const bool in_mask = (SOME_CHANNELS & (1U << chan)) != 0;
        EXPECT_EQ(gate.enabled(chan), in_mask) << unsigned(chan);
        EXPECT_EQ(gate.pwm(chan, PWM_US), in_mask ? PWM_US : GATED_PWM) << unsigned(chan);
        EXPECT_EQ(gate.gpio_level(chan, HIGH), in_mask ? HIGH : GATED_LEVEL) << unsigned(chan);
        // a low level is passed on as it is
        EXPECT_EQ(gate.gpio_level(chan, 0), 0U) << unsigned(chan);
    }
}

TEST(SIM_OutputGate, channels_beyond_the_mask_are_gated)
{
    // a full mask still gates channels it cannot hold, and the DShot
    // ALL_CHANNELS marker (255) is never one channel
    const OutputGate gate(UINT32_MAX);
    EXPECT_TRUE(gate.enabled(MASK_CHANNELS - 1));
    for (uint16_t chan = MASK_CHANNELS; chan <= UINT8_MAX; chan++) {
        EXPECT_FALSE(gate.enabled(chan)) << chan;
        EXPECT_EQ(gate.pwm(chan, PWM_US), GATED_PWM) << chan;
    }
}

TEST(SIM_OutputGate, channel_masks_split_into_allowed_and_gated)
{
    const OutputGate gate(SOME_CHANNELS);
    // e.g. the IOMCU failsafe channels, the IOMCU GPIO mask or the
    // DroneCAN servo and ESC masks
    const uint32_t chmask = 0x0000FFFFU;
    EXPECT_EQ(gate.allowed(chmask), chmask & SOME_CHANNELS);
    EXPECT_EQ(gate.gated(chmask), chmask & ~SOME_CHANNELS);
    EXPECT_EQ(gate.allowed(chmask) | gate.gated(chmask), chmask);
    EXPECT_EQ(gate.allowed(chmask) & gate.gated(chmask), 0U);
}

// the gate in use follows the SIM_OH_MASK parameter, live, and gates
// everything while no SIM parameters exist
TEST(SIM_OutputGate, current_follows_SIM_OH_MASK)
{
    ASSERT_TRUE(AP::sitl() == nullptr);
    EXPECT_FALSE(OutputGate::current().enabled(0));
    EXPECT_EQ(OutputGate::current().allowed(UINT32_MAX), 0U);

    static SITL::SIM sim;
    ASSERT_TRUE(AP::sitl() == &sim);
    // SIM_OH_MASK defaults to 0: nothing reaches hardware
    EXPECT_EQ(sim.on_hardware_output_enable_mask.get(), 0);
    EXPECT_EQ(OutputGate::current().allowed(UINT32_MAX), 0U);

    sim.on_hardware_output_enable_mask.set(int32_t(SOME_CHANNELS));
    EXPECT_EQ(OutputGate::current().allowed(UINT32_MAX), SOME_CHANNELS);
    EXPECT_TRUE(OutputGate::current().enabled(2));
    EXPECT_FALSE(OutputGate::current().enabled(1));

    // clearing it takes effect at once
    sim.on_hardware_output_enable_mask.set(0);
    EXPECT_FALSE(OutputGate::current().enabled(2));
}

AP_GTEST_MAIN()
