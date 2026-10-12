/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  Hardware-in-the-loop (HIL) backend. An external simulator owns the
  vehicle physics. Once per main loop this backend sends the servo
  outputs as HIL_ACTUATOR_CONTROLS on the serial port whose
  SERIALn_PROTOCOL is SIM_HIL (100), and it applies the newest accepted
  HIL_STATE_QUATERNION reply to the Aircraft state, which the simulated
  sensor backends (IMU, baro, GPS, compass, airspeed) read through
  fill_fdm(). There is no onboard physics and no wind. The link rules
  are in SIM_HIL_Link.h, the contract in libraries/SITL/examples/HIL.

  SITL:  --model hil           real time, the board timing rehearsal
         --model hil:lockstep  SITL waits for each reply before its clock
                               advances: deterministic, any SIM_SPEEDUP
         plus --serialN=tcp:<port> and SERIALn_PROTOCOL 100
  board: a simulation-on-hardware hwdef that includes
         hwdef/include/HIL.inc (real time only; lockstep does not exist
         on a board)
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_HIL_ENABLED

#include "SIM_Aircraft.h"
#include "SIM_HIL_Link.h"

namespace SITL {

class HIL : public Aircraft {
public:
    HIL(const char *frame_str);

    /* send outputs once per main loop, apply the newest state, keep time */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return NEW_NOTHROW HIL(frame_str);
    }

    // set the Aircraft state from a state the link accepted
    void apply_state(const HILLink::State &s);

private:
    // receive buffer asked of the port: room for 13 HIL_STATE_QUATERNION
    // (76 bytes on the wire) if loops stall. begin() may allocate more
    // (ChibiOS sizes buffers up with the baud rate and on boards with
    // plenty of memory); the link reads at most what is waiting
    static constexpr uint16_t UART_RX_BUFSIZE = 1024;
    // transmit buffer asked of the port: room for 11 HIL_ACTUATOR_CONTROLS
    // (93 bytes on the wire); ChibiOS doubles it on USB and big-memory boards
    static constexpr uint16_t UART_TX_BUFSIZE = 1024;
    // HIL log record rate
    static constexpr uint32_t LOG_RATE_HZ = 10;
    static constexpr uint32_t LOG_INTERVAL_US = 1000000UL / LOG_RATE_HZ;
    // retry interval while no SIM_HIL port can be opened (the protocol
    // can be set without a reboot)
    static constexpr uint32_t PORT_RETRY_INTERVAL_MS = 1000;
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // real-time mode follows the wall clock, as the board does
    static constexpr float REALTIME_SPEEDUP = 1.0f;
    // lockstep: wall-clock sleep between polls of the port while waiting
    static constexpr uint32_t LOCKSTEP_POLL_US = 100;
    // lockstep: console notice interval while waiting for the bridge
    static constexpr uint64_t LOCKSTEP_NOTICE_US = 2000000;
    // real time: a frame later than its wall-clock slot by more than this
    // re-anchors the pacing (SITL was paused or runs slower than real
    // time) instead of sending frames back to back to catch up
    static constexpr uint64_t REALTIME_RESYNC_US = 100000;
#endif

    bool open_port(void);
    bool loop_completed(void);
    void exchange_frame(const struct sitl_input &input, uint64_t now_us);
    void report_link_state(HILLink::LinkState previous);
    void update_arming_gate(void);
    void write_log(uint64_t now_us);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    void keep_realtime(void);
    void pace_realtime_frame(uint64_t now_us);
    void pump_port(void);
    void lockstep_exchange(const struct sitl_input &input, uint64_t now_us);

    // --model hil:lockstep
    bool lockstep = false;

    // real time: wall-clock and board time of the pacing anchor
    bool pace_anchored = false;
    uint64_t pace_wall_anchor_us = 0;
    uint64_t pace_board_anchor_us = 0;
#endif

    HILLink link;

    // SERIALn of the open port, for messages
    int8_t port_number = -1;
    bool port_attempted = false;
    uint32_t last_port_attempt_ms = 0;
    bool port_notice_sent = false;

    // main loops completed (AP_Scheduler::loops_completed32) at the last frame
    bool have_loop_count = false;
    uint32_t last_loop_count = 0;

    // AP_Arming auxiliary authorisation: arming is refused until the first
    // state and while the link is LOST (AP_ARMING_AUX_AUTH_ENABLED)
    bool auth_id_requested = false;
    bool have_auth_id = false;
    uint8_t auth_id = 0;

    // largest state age since the last HIL log record
    uint32_t age_max_us = 0;
    uint64_t last_log_us = 0;
};

} // namespace SITL

#else  // AP_SIM_HIL_ENABLED

#include <type_traits>

namespace SITL {

class Aircraft;

/*
  stands in for the backend when it is compiled out, so that a hwdef
  that selects AP_SIM_FRAME_CLASS HIL without enabling the backend fails
  to build with this message (SIMState calls HIL::create)
 */
class HIL {
public:
    template <typename T = void>
    static Aircraft *create(const char *) {
        static_assert(!std::is_same<T, T>::value,
                      "AP_SIM_FRAME_CLASS HIL needs AP_SIM_HIL_ENABLED (include hwdef/include/HIL.inc)");
        return nullptr;
    }
};

} // namespace SITL

#endif // AP_SIM_HIL_ENABLED
