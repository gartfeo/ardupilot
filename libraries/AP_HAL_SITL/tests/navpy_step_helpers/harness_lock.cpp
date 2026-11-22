// A helper's blocking lock take while the main thread holds that lock across
// clock steps (the scheduler semaphore in a scripting binding): the helper
// parks and retries in its turn (NavPyStep::helper_take), so the gate never
// waits on a helper that is blocked on the main thread. Exit 0 = every take
// got the lock and no step hit the gate's wall deadline (that exits 72).
#include "NavPyStep.h"  // libraries/AP_HAL_SITL
#include <AP_HAL/AP_HAL.h>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <pthread.h>
#include <thread>
std::atomic<uint64_t> harness_now_us{1000000};
void harness_pause() {}
using namespace HALSITL;
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static std::atomic<bool> main_holds{false};
static std::atomic<int> takes{0};
static std::atomic<int> faults{0};
static std::atomic<bool> done{false};
static bool try_take(void *) { return pthread_mutex_trylock(&lock) == 0; }
static void helper(int32_t slot, int rounds) {
    NavPyStep::helper_start(slot);
    if (!NavPyStep::helper_in_turns()) faults++;
    for (int i = 0; i < rounds; i++) {
        NavPyStep::helper_take(try_take, nullptr);
        if (main_holds.load()) faults++;
        takes++;
        pthread_mutex_unlock(&lock);
        NavPyStep::helper_wait(AP_HAL::micros64() + 700);
    }
    NavPyStep::helper_exited();
    done = true;
}
int main(int argc, char **argv) {
    const int rounds = argc > 1 ? atoi(argv[1]) : 2000;
    setenv("NAVPY_SERIAL_STEP", "1", 1);
    setenv("NAVPY_STEP_HELPER_JITTER_SEED", "7", 1);
    std::thread t(helper, NavPyStep::helper_created(), rounds);
    for (int i = 0; !done.load(); i++) {
        const bool hold = i % 5 < 3;  // three steps in five with the lock held
        if (hold) { pthread_mutex_lock(&lock); main_holds = true; }
        harness_now_us += 500;
        NavPyStep::clock_set(harness_now_us.load());
        if (hold) { main_holds = false; pthread_mutex_unlock(&lock); }
    }
    t.join();
    printf("takes %d faults %d\n", takes.load(), faults.load());
    return faults.load() || takes.load() != rounds ? 1 : 0;
}
