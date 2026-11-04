#pragma once
// Test shim: only what NavPyStep.cpp uses from AP_HAL.
#include <atomic>
#include <stdint.h>
#define HAL_BOARD_SITL 3
#define CONFIG_HAL_BOARD HAL_BOARD_SITL
extern std::atomic<uint64_t> harness_now_us;
namespace AP_HAL { inline uint64_t micros64() { return harness_now_us.load(); } }
