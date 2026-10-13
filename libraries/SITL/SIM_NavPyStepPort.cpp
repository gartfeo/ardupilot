#include "SIM_NavPyStepPort.h"

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_HAL_SITL/NavPyStep.h>

using namespace SITL;

namespace {
NavPyStepPort *ports[NavPyStepPort::MAX_PORTS];
}

NavPyStepPort::NavPyStepPort(uint8_t port_number) :
    SerialDevice()
{
    if (!HALSITL::NavPyStep::enabled()) {
        HALSITL::NavPyStep::fail("navpystep_without_mode");
    }
    if (port_number >= MAX_PORTS || ports[port_number] != nullptr) {
        HALSITL::NavPyStep::fail("navpystep_port");
    }
    // Replace the base class's 512-byte buffers.
    delete to_autopilot;
    delete from_autopilot;
    to_autopilot = NEW_NOTHROW ByteBuffer{BUFFER_BYTES + 1};
    from_autopilot = NEW_NOTHROW ByteBuffer{BUFFER_BYTES + 1};
    if (to_autopilot == nullptr || from_autopilot == nullptr) {
        HALSITL::NavPyStep::fail("navpystep_buffers");
    }
    ports[port_number] = this;
}

NavPyStepPort *NavPyStepPort::for_port(uint8_t port_number)
{
    return port_number < MAX_PORTS ? ports[port_number] : nullptr;
}

void NavPyStepPort::push_ingress(const uint8_t *data, uint32_t len)
{
    if (len == 0) {
        return;
    }
    if (to_autopilot->space() < len || to_autopilot->write(data, len) != len) {
        HALSITL::NavPyStep::fail("ingress_fifo_full");  // never drop
    }
}

uint32_t NavPyStepPort::drain_egress(uint8_t *buffer, uint32_t size)
{
    return from_autopilot->read(buffer, size);
}

uint32_t NavPyStepPort::egress_pending() const
{
    return from_autopilot->available();
}

uint32_t NavPyStepPort::fifo_bytes() const
{
    return to_autopilot->available();
}
#endif
