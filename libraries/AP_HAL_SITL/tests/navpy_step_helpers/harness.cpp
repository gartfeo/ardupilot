// Two helpers never run at once, nor alongside the main loop (NavPyStep's
// helper gate), with seeded helper jitter on. Mirrors Scheduler::thread_create
// (helper_created, helper_start) and SITL_State::wait_clock (helper waits).
#include "NavPyStep.h"  // libraries/AP_HAL_SITL
#include <AP_HAL/AP_HAL.h>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <vector>
std::atomic<uint64_t> harness_now_us{1000000};
using namespace HALSITL;
static std::atomic<int> inside{0};
static std::atomic<bool> main_running{false};
static std::atomic<bool> stop{false};
static std::atomic<int> faults{0};
static std::atomic<int> done{0};
static void body() {
    if (inside.fetch_add(1) != 0 || main_running.load()) faults++;
    std::this_thread::sleep_for(std::chrono::microseconds(50));
    inside.fetch_sub(1);
}
static void helper(int32_t slot, uint32_t period_us) {
    NavPyStep::helper_start(slot);
    while (!stop.load()) {
        body();
        const uint64_t until = AP_HAL::micros64() + period_us;  // as wait_clock
        NavPyStep::helper_wait_begin(until);
        while (AP_HAL::micros64() < until && !stop.load()) NavPyStep::helper_sleep(until);
        NavPyStep::helper_wait_end();
    }
    NavPyStep::helper_exited();
    done++;
}
int main(int argc, char **argv) {
    const int steps = argc > 1 ? atoi(argv[1]) : 20000;
    setenv("NAVPY_SERIAL_STEP", "1", 1);
    setenv("NAVPY_STEP_HELPER_JITTER_SEED", "7", 1);
    std::vector<std::thread> threads;
    const uint32_t periods[] = {1000, 1500, 2500};
    for (uint32_t p : periods) {
        const int32_t slot = NavPyStep::helper_created();
        threads.emplace_back(helper, slot, p);
    }
    for (int i = 0; i < steps; i++) {
        main_running = false;
        harness_now_us += 500;  // the main loop's clock step
        NavPyStep::clock_set(harness_now_us.load());
        main_running = true;
        std::this_thread::sleep_for(std::chrono::microseconds(20));  // the loop body
    }
    stop = true;
    main_running = false;
    for (int k = 0; done.load() < 3 && k < 1000; k++) { harness_now_us += 100000; NavPyStep::clock_set(harness_now_us.load()); }
    for (auto &t : threads) t.join();
    printf("steps %d faults %d\n", steps, faults.load());
    return faults.load() ? 1 : 0;
}
