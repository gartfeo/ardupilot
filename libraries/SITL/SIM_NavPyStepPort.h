#pragma once
/*
  NavPy serial-step port (EVAL-ONLY): a simulated serial device for
  --serialN=sim:navpystep. The coordinator's bytes for a tick go into an
  ingress FIFO that never drops; the UART's io tick admits from it what the
  read buffer has room for (at most 10000 bytes, the socket path's limit).
  Bytes the autopilot writes collect in an egress buffer that the step hook
  drains once per main loop. Contract: navpy lockstep_contracts.md (UART).
 */
#include "SIM_SerialDevice.h"

#include <stdint.h>

namespace SITL {

class NavPyStepPort : public SerialDevice {
public:
    explicit NavPyStepPort(uint8_t port_number);

    // the step hook's side
    static NavPyStepPort *for_port(uint8_t port_number);
    void push_ingress(const uint8_t *data, uint32_t len);
    uint32_t drain_egress(uint8_t *buffer, uint32_t size);
    uint32_t egress_pending() const;
    uint32_t fifo_bytes() const;

    // Usable bytes of the ingress FIFO and egress buffer: far above one
    // tick's 65535-byte wire limit, so only a coordinator that outruns the
    // autopilot for many ticks can fill them, and then the run stops.
    static constexpr uint32_t BUFFER_BYTES = 1U << 20;
    static constexpr uint8_t MAX_PORTS = 2;
};

} // namespace SITL
