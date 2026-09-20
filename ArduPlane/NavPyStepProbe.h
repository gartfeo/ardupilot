#pragma once

// Linux SITL experiment only. This protocol never transports flight commands.
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && defined(__linux__)
#include <stdint.h>

namespace NavPyStep {
struct Sample {
    float roll, pitch, airspeed;
    int32_t target_roll, target_pitch;
    uint16_t servo[16];
    uint8_t mode;
};
void begin(uint32_t vehicle, uint32_t tick, const Sample &sample);
void end(uint32_t tick, const Sample &sample);
}
#endif
