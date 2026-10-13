#pragma once
/*
  NavPy serial-step (EVAL-ONLY): HAL-level state shared by the UART driver,
  the clock and MAVLink FTP. Inert unless NAVPY_SERIAL_STEP=1 is set.
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

// Helper threads (Scheduler::thread_create): alive from creation to exit,
// and parked while they wait for sim time in wait_clock.
void helper_created();
void helper_exited();
void helper_parked(bool parked);
// Block (clock frozen) until every live helper is parked, or fail at the
// wall deadline: helpers then do no work between ticks.
void wait_helpers_parked(uint64_t wall_deadline_us);

// Abort with the invalid marker (protocol or contract fault).
[[noreturn]] void fail(const char *reason);

} // namespace NavPyStep
} // namespace HALSITL
