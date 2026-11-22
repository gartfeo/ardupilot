#include "NavPyStep.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SIM_Aircraft.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sched.h>
#include <unistd.h>

namespace HALSITL {
namespace NavPyStep {
namespace {
std::atomic<uint32_t> advances{0};
std::atomic<uint64_t> jitter_state{0};
// Most helper threads one SITL creates (scripting, logger IO, FTP, ...).
constexpr int32_t MAX_HELPERS = 32;
std::atomic<uint64_t> helper_until[MAX_HELPERS];  // 0: not waiting
std::atomic<bool> slot_used[MAX_HELPERS];
std::atomic<bool> slot_created[MAX_HELPERS];  // taken by thread_create: runs in turns
// A created helper's first wait ends here: the creator's clock plus 1 us, read
// at creation, so the new thread's start time never depends on when it runs.
std::atomic<uint64_t> start_until[MAX_HELPERS];
// The slot whose helper may run now (wait_helpers_parked); -1: none.
std::atomic<int32_t> turn{-1};
thread_local int32_t my_slot = -1;
// A helper retrying a lock (helper_take) runs at its next clock step, or when
// the main thread lends it a turn (main_take): it is lent one while its seen
// epoch differs from its slot's lend_epoch, and takes the lend by catching up.
std::atomic<uint32_t> lend_epoch[MAX_HELPERS];
std::atomic<bool> retrying[MAX_HELPERS];
std::atomic<uint32_t> retry_seen[MAX_HELPERS];
// Each slot's thread: main_take lends only when a helper in turns holds the lock.
std::atomic<pthread_t> slot_thread[MAX_HELPERS];
// Most a helper wait is stretched: enough to reorder helper wake-ups against
// the main loop at any speedup, small enough to keep runs short.
constexpr uint32_t MAX_HELPER_JITTER_US = 2000;
// The most wall time a clock step waits for its helpers: the coordinator's
// step deadline (STEP_DEADLINE_US), so a stuck helper is a fault, not a hang.
constexpr uint64_t HELPER_GATE_WALL_US = 30000000;
// Helpers wait on clock_cv for the clock; the gate waits on park_cv for them.
std::mutex gate_mutex;
std::condition_variable clock_cv;
std::condition_variable park_cv;

uint64_t wall_now_us()
{
    timespec t{};
    clock_gettime(CLOCK_MONOTONIC, &t);
    return uint64_t(t.tv_sec) * 1000000 + t.tv_nsec / 1000;
}

bool lent(int32_t slot)
{
    return retrying[slot].load() && retry_seen[slot].load() != lend_epoch[slot].load();
}

void notify_parked()
{
    std::lock_guard<std::mutex> hold(gate_mutex);
    park_cv.notify_all();
}

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

void seed_random()
{
    const char *value = getenv("NAVPY_STEP_RNG_SEED");
    if (value == nullptr) {
        return;
    }
    if (!enabled()) {
        fail("rng_configuration");  // EVAL-ONLY: never outside the mode
    }
    char *end = nullptr;
    errno = 0;
    const unsigned long long n = strtoull(value, &end, 10);
    if (errno || *value < '1' || *value > '9' || *end || n > UINT32_MAX) {
        fail("rng_configuration");
    }
    // glibc's rand() is random() and srand is srandom; both are seeded so
    // the rule holds on any libc.
    srandom(unsigned(n));
    srand(unsigned(n));
    seed_random16(uint32_t(n));
    SITL::Aircraft::seed_turbulence_random(uint32_t(n));
}

int32_t helper_created()
{
    if (!enabled()) {
        return -1;
    }
    for (int32_t i = 0; i < MAX_HELPERS; i++) {
        bool expected = false;
        if (slot_used[i].compare_exchange_strong(expected, true)) {
            helper_until[i] = 0;  // starting: the gate waits for its first wait
            start_until[i] = AP_HAL::micros64() + 1;
            slot_created[i] = true;
            return i;
        }
    }
    fail("helper_slots");  // more live helpers than slots
}

static void free_slot(int32_t slot)
{
    helper_until[slot] = 0;
    retrying[slot] = false;
    slot_thread[slot] = pthread_t();
    slot_created[slot] = false;
    slot_used[slot] = false;
    notify_parked();
}

void helper_create_failed(int32_t slot)
{
    if (slot >= 0) {
        free_slot(slot);
    }
}

void helper_exited()
{
    if (!enabled()) {
        return;
    }
    if (my_slot >= 0) {  // on the exiting thread: free its slot
        free_slot(my_slot);
        my_slot = -1;
    }
}
void helper_wait_begin(uint64_t until_us)
{
    if (!enabled()) {
        return;
    }
    for (int32_t i = 0; my_slot < 0 && i < MAX_HELPERS; i++) {
        bool expected = false;
        if (slot_used[i].compare_exchange_strong(expected, true)) {
            my_slot = i;
        }
    }
    if (my_slot < 0) {
        fail("helper_slots");  // more live helpers than slots
    }
    slot_thread[my_slot] = pthread_self();
    helper_until[my_slot] = until_us;
    notify_parked();
}

void helper_wait_end()
{
    if (enabled() && my_slot >= 0) {
        helper_until[my_slot] = 0;
    }
}

void wait_helpers_parked(uint64_t now_us, uint64_t wall_deadline_us)
{
    std::unique_lock<std::mutex> hold(gate_mutex);
    for (;;) {
        // Busy: the turn's helper still runs (its wait expired or ended), or
        // a created helper runs outside a turn (it is starting).
        const int32_t current = turn.load();
        bool busy = current >= 0 && slot_used[current].load() &&
            (helper_until[current].load() <= now_us || lent(current));
        int32_t next = -1;
        for (int32_t i = 0; !busy && i < MAX_HELPERS; i++) {
            if (!slot_used[i].load() || !slot_created[i].load()) {
                continue;
            }
            const uint64_t until = helper_until[i].load();
            if (until == 0) {
                busy = true;
            } else if ((until <= now_us || lent(i)) && next < 0) {
                next = i;
            }
        }
        if (!busy) {
            turn = next;
            if (next < 0) {
                return;
            }
            clock_cv.notify_all();
            continue;  // the turn's helper's wait has expired: busy until it waits again
        }
        if (wall_now_us() >= wall_deadline_us) {
            fail("helper_gate");
        }
        // A park or an exit notifies; the timeout only bounds a missed one.
        park_cv.wait_for(hold, std::chrono::microseconds(1000));
    }
}

void clock_set(uint64_t now_us)
{
    if (!enabled()) {
        return;
    }
    {
        std::lock_guard<std::mutex> hold(gate_mutex);
        clock_cv.notify_all();
    }
    wait_helpers_parked(now_us, wall_now_us() + HELPER_GATE_WALL_US);
}

void helper_start(int32_t slot)
{
    if (!enabled()) {
        return;
    }
    my_slot = slot;
    if (slot >= 0) {
        slot_thread[slot] = pthread_self();
    }
    helper_wait(slot >= 0 ? start_until[slot].load() : AP_HAL::micros64() + 1);
}

bool helper_in_turns()
{
    return enabled() && my_slot >= 0 && slot_created[my_slot].load();
}

void helper_take(bool (*try_take)(void *ctx), void *ctx)
{
    retry_seen[my_slot] = lend_epoch[my_slot].load();
    retrying[my_slot] = true;
    while (!try_take(ctx)) {
        helper_wait(AP_HAL::micros64() + 1);
    }
    retrying[my_slot] = false;
}

// The slot of the helper in turns that is the lock's holder; -1: none.
static int32_t holder_slot(pthread_t holder)
{
    for (int32_t i = 0; i < MAX_HELPERS; i++) {
        if (slot_used[i].load() && slot_created[i].load() &&
            pthread_equal(slot_thread[i].load(), holder)) {
            return i;
        }
    }
    return -1;
}

void main_take(bool (*try_take)(void *ctx), pthread_t (*holder)(void *ctx), void *ctx)
{
    const uint64_t deadline = wall_now_us() + HELPER_GATE_WALL_US;
    while (!try_take(ctx)) {
        // A parked helper may hold the lock while it retries another one that
        // this thread has since given: lend that helper, and no other, a turn
        // at this clock, and try again. A holder outside the turns runs on its
        // own, and this thread waits for it as a blocking take would; no other
        // helper is lent a turn, so none takes a lock at a time set by the wall
        // clock. An ABBA cycle is a fault, never a hang.
        if (wall_now_us() >= deadline) {
            fail("lock_wait");
        }
        const int32_t slot = holder_slot(holder(ctx));
        if (slot < 0) {
            sched_yield();
            continue;
        }
        {
            std::lock_guard<std::mutex> hold(gate_mutex);
            lend_epoch[slot]++;
            clock_cv.notify_all();
        }
        wait_helpers_parked(AP_HAL::micros64(), deadline);
        sched_yield();
    }
}

void helper_wait(uint64_t until_us)
{
    helper_wait_begin(until_us);
#ifdef NAVPY_STEP_TEST_PAUSE
    NAVPY_STEP_TEST_PAUSE();  // tests/navpy_step_helpers: deschedule here
#endif
    helper_sleep(until_us);  // even when that time has passed: only in its turn
    helper_wait_end();
}

void helper_sleep(uint64_t until_us)
{
    {
        std::unique_lock<std::mutex> hold(gate_mutex);
        const bool in_turns = my_slot >= 0 && slot_created[my_slot].load();
        for (;;) {
            const bool due = AP_HAL::micros64() >= until_us || (in_turns && lent(my_slot));
            if (due && (!in_turns || turn.load() == my_slot)) {
                if (in_turns) {  // running from now: the gate waits for its next wait
                    helper_until[my_slot] = 0;
                    retry_seen[my_slot] = lend_epoch[my_slot].load();  // a lend is taken
                }
                break;  // its time has come (or a lent turn) and it is its turn
            }
            // No early return: a helper that left here before its time and
            // then slept its jitter unlocked could find the clock moved past
            // its time, and run while another helper holds the turn.
            clock_cv.wait_for(hold, std::chrono::microseconds(1000));
        }
    }
    const uint32_t jitter = helper_jitter_us();
    if (jitter != 0) {
        usleep(jitter);
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
