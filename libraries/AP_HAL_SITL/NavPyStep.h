#pragma once
/*
  NavPy serial-step (EVAL-ONLY): HAL-level state shared by the UART driver
  and the clock. Inert unless NAVPY_SERIAL_STEP=1 is set.
  Contract: navpy docs/design/lockstep_contracts.md (UART, Firmware helpers).
 */
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
// draw: glibc random()/rand() and get_random16. Unset, nothing changes:
// glibc's default seed 1 and get_random16's fixed seeds, as before.
// Contract: navpy docs/design/deterministic_dev_sim.md, section 2.
void seed_random();

// Helper threads (Scheduler::thread_create): alive from creation to exit,
// and parked while they wait for sim time in wait_clock.
void helper_created();
void helper_exited();
// A helper waits for sim time until until_us; it counts as parked only
// while that deadline lies after the current sim time.
void helper_wait_begin(uint64_t until_us);
void helper_wait_end();
// Block (clock frozen at now_us) until every live helper is parked with a
// deadline after now_us, or fail at the wall deadline: a helper whose
// wait has expired first wakes, does its work and waits again.
void wait_helpers_parked(uint64_t now_us, uint64_t wall_deadline_us);
// The main thread set the clock to now_us (Scheduler::stop_clock): wake the
// helpers whose wait expired and hold the clock there until every helper is
// parked again. So a helper (the FTP worker) runs only while the main thread
// waits, at a fixed sim time, never alongside the loop.
void clock_set(uint64_t now_us);
// A helper's wait for sim time until_us, in place of the wall poll: it wakes
// on clock_set, or after 1 ms of wall time to look again, then sleeps the
// seeded jitter.
void helper_sleep(uint64_t until_us);

// Abort with the invalid marker (protocol or contract fault).
[[noreturn]] void fail(const char *reason);

} // namespace NavPyStep
} // namespace HALSITL
