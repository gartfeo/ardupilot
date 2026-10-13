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
  Simulation on hardware: which servo channels may drive real hardware.

  Only the channels in SIM_OH_MASK (bit 0 = SERVO1) may. Every output
  path that can reach an actuator applies these rules to the others:
  - PWM (AP_HAL_ChibiOS RCOutput::write): written as GATED_PWM, no
    pulse, on the FMU and on the IOMCU, whose failsafe PWM for the
    channel is GATED_PWM too (RCOutput::set_failsafe_pwm)
  - DShot commands (RCOutput::send_dshot_command, dshot_send_command):
    none, and no DShot frame at all on the channel
  - GPIO (SERVOn_FUNCTION -1, AP_HAL_ChibiOS GPIO): never enabled as a
    GPIO output, and driven to GATED_GPIO_LEVEL if SIM_OH_MASK loses
    the channel after boot
  - DroneCAN servo and ESC commands (AP_DroneCAN::SRV_push_servos): none
  - serial LEDs (NeoPixel, ProfiLED data and clock,
    RCOutput::fill_DMA_buffer_serial_led): no waveform on the channel
  - serial ESC output (BLHeli passthrough, RCOutput::serial_setup_output):
    refused unless every channel it drives is allowed

  The rules are pure, so they are unit tested in SITL
  (libraries/SITL/tests/test_sim_output_gate.cpp). Their use on a board
  is covered by the bench check in hwdef/Pixhawk6X-HIL/README.md.
*/

#pragma once

#include <stdint.h>

namespace SITL {

class OutputGate {
public:
    // PWM width a gated channel is written with: no pulse, ArduPilot's
    // own "no output" (the IOMCU writes 0 itself while safety is on)
    static constexpr uint16_t GATED_PWM = 0;
    // level a gated channel's GPIO pin is driven to
    static constexpr uint8_t GATED_GPIO_LEVEL = 0;

    constexpr explicit OutputGate(uint32_t sim_oh_mask) : mask(sim_oh_mask) {}

#if AP_SIM_ENABLED
    // the gate SIM_OH_MASK sets now; every channel is gated while the
    // SIM parameters do not exist
    static OutputGate current(void);
#endif

    // true if servo channel chan (0 = SERVO1) may drive real hardware
    constexpr bool enabled(uint8_t chan) const {
        return chan < MASK_CHANNELS && (mask & (uint32_t(1U) << chan)) != 0;
    }

    // the PWM width the hardware gets for channel chan
    constexpr uint16_t pwm(uint8_t chan, uint16_t period_us) const {
        return enabled(chan) ? period_us : GATED_PWM;
    }

    // the GPIO level the hardware gets for channel chan
    constexpr uint8_t gpio_level(uint8_t chan, uint8_t level) const {
        return enabled(chan) ? level : GATED_GPIO_LEVEL;
    }

    // the channels of chmask that may drive hardware
    constexpr uint32_t allowed(uint32_t chmask) const {
        return chmask & mask;
    }

    // the channels of chmask that must not drive hardware
    constexpr uint32_t gated(uint32_t chmask) const {
        return chmask & ~mask;
    }

private:
    // channels a 32-bit mask holds; higher channels are always gated
    static constexpr uint8_t MASK_CHANNELS = 32;

    uint32_t mask;
};

} // namespace SITL
