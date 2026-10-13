#include "NavPyStep.h"

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <atomic>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

namespace HALSITL {
namespace NavPyStep {
namespace {
std::atomic<uint32_t> advances{0};
std::atomic<uint64_t> jitter_state{0};
std::atomic<int32_t> helpers_alive{0};
std::atomic<int32_t> helpers_parked{0};
// Most a helper wait is stretched: enough to reorder helper wake-ups against
// the main loop at any speedup, small enough to keep runs short.
constexpr uint32_t MAX_HELPER_JITTER_US = 2000;

int read_enabled()
{
    const char *value = getenv("NAVPY_SERIAL_STEP");
    if (value == nullptr) {
        return 0;
    }
    if (strcmp(value, "1") != 0) {
        fail("configuration");
    }
    const char *seed = getenv("NAVPY_STEP_HELPER_JITTER_SEED");
    if (seed != nullptr) {
        char *end = nullptr;
        errno = 0;
        const unsigned long long n = strtoull(seed, &end, 10);
        if (errno || !*seed || *end || n == 0) {
            fail("jitter_configuration");
        }
        jitter_state = n;
    }
    ::printf("NAVPY_SERIAL_STEP EVAL-ONLY firmware mode\n");
    return 1;
}
} // namespace

bool enabled()
{
    static const int on = read_enabled();
    return on != 0;
}

uint32_t out_of_loop_advances()
{
    return advances.load();
}

void count_out_of_loop_advance()
{
    advances++;
}

uint32_t helper_jitter_us()
{
    uint64_t x = jitter_state.load();
    if (x == 0) {
        return 0;
    }
    uint64_t next;
    do {  // xorshift64, shared by every helper thread
        next = x;
        next ^= next << 13;
        next ^= next >> 7;
        next ^= next << 17;
    } while (!jitter_state.compare_exchange_weak(x, next));
    return uint32_t(next % (MAX_HELPER_JITTER_US + 1));
}

void helper_created() { helpers_alive++; }
void helper_exited() { helpers_alive--; }
void helper_parked(bool parked) { parked ? helpers_parked++ : helpers_parked--; }

void wait_helpers_parked(uint64_t wall_deadline_us)
{
    for (;;) {
        if (helpers_parked.load() >= helpers_alive.load()) {
            return;
        }
        timespec t{};
        clock_gettime(CLOCK_MONOTONIC, &t);
        if (uint64_t(t.tv_sec) * 1000000 + t.tv_nsec / 1000 >= wall_deadline_us) {
            fail("helper_gate");
        }
        usleep(50);
    }
}

[[noreturn]] void fail(const char *reason)
{
    fprintf(stderr, "NAVPY_SERIAL_STEP_INVALID %s\n", reason);
    fflush(stderr);
    ::_exit(72);
}

} // namespace NavPyStep
} // namespace HALSITL
#endif
