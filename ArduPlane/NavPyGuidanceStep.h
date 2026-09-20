#pragma once
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && defined(__linux__)
namespace NavPyGuidance {
bool active();
bool internal_dispatch();
[[noreturn]] void fail(const char *reason);
}
#endif
