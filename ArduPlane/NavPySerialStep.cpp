/*
  NavPy serial-step (EVAL-ONLY, inert unless NAVPY_SERIAL_STEP=1).

  Once per main loop, first among the FAST_TASKs, the autopilot sends the
  coordinator a frame (NVSTEP02) with the bytes it wrote on serial0 and
  serial1 since the last frame, then blocks, with the clock frozen, for the
  reply: the bytes each port receives. Those enter the port's ingress FIFO
  and reach the stock MAVLink parser through the UART's normal io tick.
  Layout and rules: navpy src/navpy/sim/lockstep/step_wire.py (golden
  vectors) and docs/design/lockstep_contracts.md (UART, Firmware helpers).
 */
#include "Plane.h"
#include "NavPySerialStep.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && defined(__linux__)
#include <AP_HAL_SITL/NavPyStep.h>
#include <AP_HAL_SITL/UARTDriver.h>
#include <SITL/SIM_NavPyStepPort.h>
#include <SITL/SITL.h>
#if AP_SCRIPTING_ENABLED
#include <AP_Scripting/lua_scripts.h>
#endif
#include <errno.h>
#include <poll.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

namespace NavPySerialStep {
namespace {

using HALSITL::NavPyStep::fail;

constexpr uint16_t VERSION = 2;
constexpr uint16_t PORTS = 2;
constexpr uint32_t MAX_BLOCK = 0xFFFF;  // per port and tick, both directions

struct PACKED FrameHeader {
    char magic[8];
    uint16_t version, ports;
    uint64_t boot[2], tick, source_us;
    uint32_t out_of_loop_advances;
    uint8_t helpers_quiet;
    uint8_t reserved[3];
};
struct PACKED FramePort {
    uint32_t fifo_bytes, read_buffer_bytes, egress_dropped_bytes, egress_len;
};
struct PACKED ReplyHeader {
    char magic[8];
    uint16_t version, ports;
    uint64_t tick;
};
static_assert(sizeof(FrameHeader) == 52, "NVSTEP02 frame header");
static_assert(sizeof(FramePort) == 16, "NVSTEP02 frame port");
static_assert(sizeof(ReplyHeader) == 20, "NVSTEP02 reply header");
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error NVSTEP02 requires a little-endian host
#endif

constexpr size_t MAX_FRAME = sizeof(FrameHeader) + PORTS * (sizeof(FramePort) + MAX_BLOCK);
constexpr size_t MAX_REPLY = sizeof(ReplyHeader) + PORTS * (2 + MAX_BLOCK);
uint8_t frame[MAX_FRAME];
uint8_t reply[MAX_REPLY + 1];  // one spare byte detects oversize replies

bool configured, enabled;
int fd = -1;
uint64_t boot_id, deadline_us, tick;

uint64_t wall_us()
{
    timespec t{};
    if (clock_gettime(CLOCK_MONOTONIC, &t)) { fail("clock"); }
    return uint64_t(t.tv_sec) * 1000000 + t.tv_nsec / 1000;
}

uint64_t env_number(const char *name)
{
    const char *value = getenv(name);
    if (!value || !*value || *value == '-') { fail("configuration"); }
    char *end = nullptr;
    errno = 0;
    const uint64_t n = strtoull(value, &end, 10);
    if (errno || *end || !n) { fail("configuration"); }
    return n;
}

void configure()
{
    configured = true;
    if (!HALSITL::NavPyStep::enabled()) { return; }
    if (getenv("NAVPY_GUIDANCE_MODE") || getenv("NAVPY_STEP_MODE")) { fail("exclusive_modes"); }
    boot_id = env_number("NAVPY_STEP_BOOT_ID");
    deadline_us = env_number("NAVPY_STEP_DEADLINE_US");
    for (uint8_t port = 0; port < PORTS; port++) {
        if (SITL::NavPyStepPort::for_port(port) == nullptr) { fail("ports_not_navpystep"); }
    }
    enabled = true;
}

void ready(short events, uint64_t deadline)
{
    for (;;) {
        const uint64_t now = wall_us();
        if (now >= deadline) { fail("deadline"); }
        pollfd p{fd, events, 0};
        const int result = poll(&p, 1, int((deadline - now + 999) / 1000));
        if (result < 0 && errno == EINTR) { continue; }
        if (result <= 0) { fail(result == 0 ? "deadline" : "poll"); }
        if (p.revents & events) { return; }
        fail("disconnected");
    }
}

void connect_peer()
{
    fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd < 0) { fail("socket"); }
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    strcpy(address.sun_path, "navpy-serial-step.sock");
    if (connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address))) { fail("connect"); }
}

bool helpers_quiet()
{
#if AP_SCRIPTING_ENABLED
    // aas_params.lua is one-shot and the only script in a lockstep root, so
    // a nonzero loaded and a zero running checksum mean it has finished
    // (with several scripts equal CRCs could cancel in the XOR).
    const bool scripts_done = lua_scripts::get_loaded_checksum() != 0 &&
                              lua_scripts::get_running_checksum() == 0;
#else
    const bool scripts_done = false;
#endif
    enum ap_var_type type;
    const bool aas_registered = AP_Param::find("AAS_NAV_CGT", &type) != nullptr;
    const AP_Param *backend = AP_Param::find("LOG_BACKEND_TYPE", &type);
    const bool logger_off = backend != nullptr && is_zero(backend->cast_to_float(type));
    return scripts_done && aas_registered && logger_off;  // FTP is always refused
}

size_t build_frame(uint8_t sysid, uint64_t source_us)
{
    FrameHeader header{};
    memcpy(header.magic, "NVSTEP02", 8);
    header.version = VERSION;
    header.ports = PORTS;
    header.boot[0] = boot_id;
    header.boot[1] = sysid;
    header.tick = tick;
    header.source_us = source_us;
    header.out_of_loop_advances = HALSITL::NavPyStep::out_of_loop_advances();
    header.helpers_quiet = helpers_quiet() ? 1 : 0;
    size_t offset = 0;
    memcpy(&frame[offset], &header, sizeof(header));
    offset += sizeof(header);
    for (uint8_t port = 0; port < PORTS; port++) {
        auto *step_port = SITL::NavPyStepPort::for_port(port);
        auto *uart = static_cast<HALSITL::UARTDriver *>(hal.serial(port));
        FramePort state{};
        state.fifo_bytes = step_port->fifo_bytes();
        state.read_buffer_bytes = uart->available();
        state.egress_dropped_bytes = uart->navpy_short_write_bytes();
        uint8_t *egress = &frame[offset + sizeof(state)];
        state.egress_len = step_port->drain_egress(egress, MAX_BLOCK);
        if (step_port->egress_pending() != 0) {
            fail("egress_overflow");  // never split one tick across frames
        }
        memcpy(&frame[offset], &state, sizeof(state));
        offset += sizeof(state) + state.egress_len;
    }
    return offset;
}

void apply_reply(size_t size)
{
    if (size > MAX_REPLY) { fail("reply_oversize"); }
    if (size < sizeof(ReplyHeader)) { fail("reply_truncated"); }
    ReplyHeader header{};
    memcpy(&header, reply, sizeof(header));
    if (memcmp(header.magic, "NVSTEP02", 8) || header.version != VERSION || header.ports != PORTS) {
        fail("reply_identity");
    }
    if (header.tick != tick) { fail("reply_tick"); }
    size_t offset = sizeof(header);
    const uint8_t *blocks[PORTS];
    uint16_t lengths[PORTS];
    for (uint8_t port = 0; port < PORTS; port++) {
        if (offset + 2 > size) { fail("reply_truncated"); }
        memcpy(&lengths[port], &reply[offset], 2);
        offset += 2;
        if (offset + lengths[port] > size) { fail("reply_truncated"); }
        blocks[port] = &reply[offset];
        offset += lengths[port];
    }
    if (offset != size) { fail("reply_trailing"); }
    for (uint8_t port = 0; port < PORTS; port++) {
        SITL::NavPyStepPort::for_port(port)->push_ingress(blocks[port], lengths[port]);
    }
}

} // namespace
} // namespace NavPySerialStep

void Plane::navpy_serial_step()
{
    using namespace NavPySerialStep;
    if (!configured) { configure(); }
    if (!enabled) { return; }
    const auto *sim = AP::sitl();
    if (sim == nullptr || !is_equal(sim->speedup.get(), 1.0f)) { fail("speedup"); }
    if (fd < 0) { connect_peer(); }
    tick++;
    // Helpers finish their work for times up to t_k before the frame.
    HALSITL::NavPyStep::wait_helpers_parked(AP_HAL::micros64(), wall_us() + deadline_us);
    const uint64_t source_us = AP_HAL::micros64();
    const size_t frame_size = build_frame(uint8_t(g.sysid_this_mav), source_us);
    const uint64_t deadline = wall_us() + deadline_us;
    ready(POLLOUT, deadline);
    if (send(fd, frame, frame_size, MSG_NOSIGNAL) != ssize_t(frame_size)) { fail("send"); }
    ready(POLLIN, deadline);
    const ssize_t size = recv(fd, reply, sizeof(reply), MSG_TRUNC);
    if (size <= 0) { fail("disconnected"); }
    apply_reply(size_t(size));
    if (AP_HAL::micros64() != source_us) { fail("clock_advanced_during_wait"); }
}
#endif
