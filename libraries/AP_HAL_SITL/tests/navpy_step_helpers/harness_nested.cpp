// A helper that holds lock A parks retrying lock B, which the main thread
// holds across a clock step; the main thread then gives B and takes A before
// its next clock step (Codex round 2 on todo 188). In real firmware the helper
// gets B at once and gives A; in serial-step mode the main thread lends the
// retrying helper a turn (NavPyStep::main_take). "old" takes A with a plain
// blocking lock, the pre-fix main thread: it never returns (run.sh times it
// out). Then a lock held by a thread outside the turns (Codex round 3): the
// main thread waits for it without lending, so the helper retrying B does not
// take B at a time set by that thread's wall-clock release. Exit 0 = every
// round got both locks in order and no lend went to an outside holder.
#include "NavPyStep.h"  // libraries/AP_HAL_SITL
#include <AP_HAL/AP_HAL.h>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <pthread.h>
#include <thread>
#include <unistd.h>
std::atomic<uint64_t> harness_now_us{1000000};
void harness_pause() {}
using namespace HALSITL;
static pthread_mutex_t lock_a = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t lock_b = PTHREAD_MUTEX_INITIALIZER;
static std::atomic<pthread_t> owner_a{pthread_t()};
static std::atomic<int> helper_rounds{0};
static std::atomic<int> round_go{-1};
static std::atomic<bool> holds_a{false};
static std::atomic<bool> wants_b{false};
static std::atomic<bool> took_b{false};
static std::atomic<bool> done{false};
static bool try_a(void *) {
    if (pthread_mutex_trylock(&lock_a) != 0) return false;
    owner_a = pthread_self();
    return true;
}
static bool try_b(void *) { return pthread_mutex_trylock(&lock_b) == 0; }
static pthread_t holder_a(void *) { return owner_a.load(); }
static void give_a() { owner_a = pthread_t(); pthread_mutex_unlock(&lock_a); }
static void step() {
    harness_now_us += 500;
    NavPyStep::clock_set(harness_now_us.load());
}
static void helper(int32_t slot, int rounds) {
    NavPyStep::helper_start(slot);
    for (int i = 0; i < rounds; i++) {
        while (round_go.load() < i) NavPyStep::helper_wait(AP_HAL::micros64() + 1);
        NavPyStep::helper_take(try_a, nullptr);
        holds_a = true;
        NavPyStep::helper_take(try_b, nullptr);  // the main thread holds B: parks holding A
        pthread_mutex_unlock(&lock_b);
        holds_a = false;
        give_a();
        helper_rounds++;
        NavPyStep::helper_wait(AP_HAL::micros64() + 300);
    }
    while (round_go.load() < rounds) NavPyStep::helper_wait(AP_HAL::micros64() + 1);
    wants_b = true;
    NavPyStep::helper_take(try_b, nullptr);  // retries B, holding nothing
    took_b = true;
    pthread_mutex_unlock(&lock_b);
    NavPyStep::helper_exited();
    done = true;
}
int main(int argc, char **argv) {
    const int rounds = argc > 1 ? atoi(argv[1]) : 500;
    const bool old = argc > 2 && strcmp(argv[2], "old") == 0;
    setenv("NAVPY_SERIAL_STEP", "1", 1);
    setenv("NAVPY_STEP_HELPER_JITTER_SEED", "7", 1);
    std::thread t(helper, NavPyStep::helper_created(), rounds);
    int bad = 0;
    for (int i = 0; i < rounds; i++) {
        pthread_mutex_lock(&lock_b);
        round_go = i;
        for (int s = 0; !holds_a.load(); s++) {  // steps until the helper parks holding A, retrying B
            step();
            if (s > 1000) { printf("helper never took A\n"); return 1; }
        }
        pthread_mutex_unlock(&lock_b);
        if (old) {
            pthread_mutex_lock(&lock_a);
            owner_a = pthread_self();
        } else {
            NavPyStep::main_take(try_a, holder_a, nullptr);  // no clock step in between
        }
        if (holds_a.load()) bad++;
        give_a();
        while (helper_rounds.load() <= i) step();
    }
    // A thread outside the turns holds A; the helper parks retrying B, held
    // here. B is given, then A taken: only the outside thread's release lets
    // the main thread on, and no lend lets the helper take B before that.
    pthread_mutex_lock(&lock_b);
    round_go = rounds;
    while (!wants_b.load()) step();
    std::atomic<bool> outside_holds{false};
    std::thread outside([&] {
        pthread_mutex_lock(&lock_a);
        owner_a = pthread_self();
        outside_holds = true;
        usleep(20000);
        give_a();
    });
    while (!outside_holds.load()) usleep(100);
    pthread_mutex_unlock(&lock_b);
    NavPyStep::main_take(try_a, holder_a, nullptr);
    const bool lent_to_outside = took_b.load();
    give_a();
    outside.join();
    while (!done.load()) step();  // the helper takes B at a clock step
    t.join();
    printf("nested rounds %d bad %d lent_to_outside %d\n", helper_rounds.load(), bad, int(lent_to_outside));
    return bad || lent_to_outside || helper_rounds.load() != rounds ? 1 : 0;
}
