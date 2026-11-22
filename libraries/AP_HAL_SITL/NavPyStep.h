#pragma once
/*
  NavPy serial-step (EVAL-ONLY): HAL-level state shared by the UART driver
  and the clock. Inert unless NAVPY_SERIAL_STEP=1 is set.
  Contract: navpy docs/design/lockstep_contracts.md (UART, Firmware helpers).
 */
#include <pthread.h>
#include <stdint.h>

namespace HALSITL {
namespace NavPyStep {

// NAVPY_SERIAL_STEP=1 in the environment (read once).
bool enabled();

// Clock advances the main thread made while waiting on a semaphore, not in
// the loop's own wait. Nonzero is a host fault.
uint32_t out_of_loop_advances();
void count_out_of_loop_advance();

// Extra wall microseconds for a helper thread's clock wait, seeded by
// NAVPY_STEP_HELPER_JITTER_SEED (acceptance (c)); 0 when unset.
uint32_t helper_jitter_us();

// Seed the SITL's random numbers from NAVPY_STEP_RNG_SEED (decimal,
// 1..4294967295; only in the mode), once at SITL start before any sensor
// draw: glibc random()/rand(), get_random16 and turbulence's own state
// (SITL::Aircraft::seed_turbulence_random). Unset: glibc's default seed 1,
// get_random16's fixed seeds and turbulence seed 1.
// Contract: navpy docs/design/deterministic_dev_sim.md, section 2.
void seed_random();

// Helper threads (Scheduler::thread_create): alive from creation to exit,
// and parked while they wait for sim time in wait_clock. A helper's slot is
// taken at creation, by the creating thread, so slots (the order helpers run
// in) do not depend on when a new thread first runs. Returns the slot, -1
// outside the mode; a failed create gives it back.
int32_t helper_created();
void helper_create_failed(int32_t slot);
void helper_exited();
// A helper waits for sim time until until_us; it counts as parked only
// while that deadline lies after the current sim time.
void helper_wait_begin(uint64_t until_us);
void helper_wait_end();
// Block (clock frozen at now_us) until every live helper is parked with a
// deadline after now_us, or fail at the wall deadline. The helpers whose wait
// expired run one at a time, lowest slot first, each until it waits again or
// exits: two helpers never run at once, so their order is fixed.
void wait_helpers_parked(uint64_t now_us, uint64_t wall_deadline_us);
// The main thread set the clock to now_us (Scheduler::stop_clock): wake the
// helpers whose wait expired and hold the clock there until every helper is
// parked again. So a helper (the FTP worker) runs only while the main thread
// waits, at a fixed sim time, never alongside the loop.
void clock_set(uint64_t now_us);
// A helper's wait for sim time until_us, in place of the wall poll: it
// returns only once its time has come and it holds the turn (a helper outside
// turns: once its time has come), then sleeps the seeded jitter in its turn.
void helper_sleep(uint64_t until_us);
// A helper's whole wait for sim time until_us (SITL_State::wait_clock,
// helper_start): registered, then helper_sleep, then done. Never a clock-only
// check after registering: a helper descheduled there would find its time
// passed and run without its turn.
void helper_wait(uint64_t until_us);
// A new helper's first step, before its body: it waits for the next clock
// step, so it never runs alongside the main thread that created it (the FTP
// worker starts on the first FTP request, mid-loop).
void helper_start(int32_t slot);
// A blocking lock take: true when the caller is a helper that runs in turns.
// Such a helper must never block in the kernel on a held lock: the holder (the
// main thread at a clock step, or a parked helper) gives it back only after
// this helper parks, so the gate and the helper would wait on each other.
bool helper_in_turns();
// That helper's blocking take: try_take(ctx) until it succeeds, parked until
// the next clock step after each failure, so it retries in its turn.
void helper_take(bool (*try_take)(void *ctx), void *ctx);
// The main thread's blocking lock take in serial-step mode: try, and while the
// lock is held lend the helpers retrying a lock a turn at the current clock
// (a helper may hold this lock while it retries one the main thread gave).
// It lends only while a helper in turns holds the lock (``holder``: the lock's
// owner thread); for any other holder it waits as a blocking take.
void main_take(bool (*try_take)(void *ctx), pthread_t (*holder)(void *ctx), void *ctx);

// Abort with the invalid marker (protocol or contract fault).
[[noreturn]] void fail(const char *reason);

} // namespace NavPyStep
} // namespace HALSITL
