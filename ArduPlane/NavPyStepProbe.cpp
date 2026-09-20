#include "Plane.h"
#include "NavPyStepProbe.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && defined(__linux__)
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/random.h>
#include <poll.h>
#include <unistd.h>
#include <time.h>
#include <errno.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>

namespace NavPyStep {
namespace {
constexpr unsigned COUNT = 1000;
constexpr unsigned FRAME_SIZE = 48;
// Resource bound for this experiment, not an aircraft timeout or latency model.
constexpr uint64_t DEADLINE_US = 5000000;
struct Row {
    uint64_t before, after, complete, elapsed, wall;
    uint32_t tick;
    Sample sample;
};
Row rows[COUNT];
unsigned count;
bool pending, finished, configured, handshake;
uint64_t boot[2], start_us;
uint32_t vehicle_id;
int fd = -1;

uint64_t wall_us()
{
    timespec t{};
    if (clock_gettime(CLOCK_MONOTONIC, &t) != 0) {
        ::_exit(72);
    }
    return uint64_t(t.tv_sec) * 1000000 + t.tv_nsec / 1000;
}

[[noreturn]] void fail(const char *reason)
{
    fprintf(stderr, "NAVPY_STEP_INVALID %s step=%u\n", reason, count + 1);
    fflush(stderr);
    if (fd >= 0) { close(fd); }
    ::_exit(72); // only this opt-in experimental SITL process
}

void ready(short events, uint64_t deadline)
{
    for (;;) {
        const uint64_t now = wall_us();
        if (now >= deadline) { fail("deadline"); }
        pollfd p{fd, events, 0};
        const int ms = int((deadline - now + 999) / 1000);
        const int result = poll(&p, 1, ms);
        if (result < 0 && errno == EINTR) { continue; }
        if (result <= 0) { fail(result == 0 ? "deadline" : "poll"); }
        if (p.revents & events) { return; }
        fail("disconnected");
    }
}

void put(uint8_t *dst, uint64_t value, unsigned bytes)
{
    for (unsigned i = 0; i < bytes; i++) { dst[i] = uint8_t(value >> (8*i)); }
}

void connect_peer()
{
    fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd < 0) { fail("socket"); }
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    // Relative to the launcher's isolated per-vehicle state directory.
    strcpy(addr.sun_path, "navpy-step.sock");
    if (connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
        fail("connect"); // a missing/full peer is not silently retried
    }
}

void exchange(uint32_t vehicle, uint32_t tick, uint64_t source_us)
{
    uint8_t request[FRAME_SIZE]{};
    memcpy(request, "NVSTEP01", 8);
    put(request + 8, boot[0], 8);
    put(request + 16, boot[1], 8);
    put(request + 24, count + 1, 8);
    put(request + 32, source_us, 8);
    put(request + 40, vehicle, 4);
    put(request + 44, tick, 4);
    const uint64_t deadline = wall_us() + DEADLINE_US;
    ready(POLLOUT, deadline);
    if (send(fd, request, sizeof(request), MSG_NOSIGNAL) != sizeof(request)) {
        fail("send");
    }
    ready(POLLIN, deadline);
    uint8_t reply[FRAME_SIZE + 1]{};
    const ssize_t size = recv(fd, reply, sizeof(reply), MSG_TRUNC);
    if (size != FRAME_SIZE) { fail(size == 0 ? "disconnected" : "frame_length"); }
    if (memcmp(request, reply, FRAME_SIZE) != 0) { fail("identity_or_version"); }
}

void configure()
{
    configured = true;
    const char *mode = getenv("NAVPY_STEP_MODE");
    if (mode == nullptr) { finished = true; return; }
    if (strcmp(mode, "observe") != 0 && strcmp(mode, "handshake") != 0) {
        fail("mode");
    }
    handshake = strcmp(mode, "handshake") == 0;
    const char *start = getenv("NAVPY_STEP_START_US");
    if (start == nullptr || *start == '\0' || *start == '-') { fail("start"); }
    char *endptr = nullptr;
    errno = 0;
    start_us = strtoull(start, &endptr, 10);
    if (errno || *endptr != '\0' || start_us == 0) { fail("start"); }
    if (getrandom(boot, sizeof(boot), 0) != sizeof(boot) || (boot[0] == 0 && boot[1] == 0)) { fail("boot_identity"); }
}

void save()
{
    // No file IO during the measured window. Exclusive creation preserves evidence.
    FILE *out = fopen("navpy-step.csv", "wx");
    if (out == nullptr) { fail("evidence_open"); }
    fprintf(out, "# NVSTEP01,boot=%016" PRIx64 "%016" PRIx64 ",handshake=%u,vehicle=%u\n",
            boot[0], boot[1], unsigned(handshake), unsigned(vehicle_id));
    fprintf(out, "step,tick,before_us,after_us,complete_us,wait_us,wall_us,mode,roll,pitch,airspeed,target_roll,target_pitch");
    for (unsigned j=0; j<16; j++) { fprintf(out, ",servo%u", j); }
    fprintf(out, "\n");
    for (unsigned i=0; i<COUNT; i++) {
        const Row &r = rows[i];
        fprintf(out, "%u,%u,%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%u,%.9g,%.9g,%.9g,%d,%d",
                i+1, r.tick, r.before, r.after, r.complete, r.elapsed, r.wall,
                unsigned(r.sample.mode), double(r.sample.roll), double(r.sample.pitch),
                double(r.sample.airspeed), r.sample.target_roll, r.sample.target_pitch);
        for (unsigned j=0; j<16; j++) { fprintf(out, ",%u", unsigned(r.sample.servo[j])); }
        fprintf(out, "\n");
    }
    const bool bad = ferror(out);
    const int closed = fclose(out);
    if (bad || closed != 0) { fail("evidence_write"); }
    printf("NAVPY_STEP_COMPLETE count=%u\n", COUNT);
    fflush(stdout);
}
} // namespace

static bool measuring()
{
    if (!configured) { configure(); }
    return !finished && AP_HAL::micros64() >= start_us;
}

static bool awaiting_end() { return pending; }

void begin(uint32_t vehicle, uint32_t tick, const Sample &sample)
{
    if (!configured) { configure(); }
    if (finished || AP_HAL::micros64() < start_us) { return; }
    if (pending) { fail("unfinished_step"); }
    if (count == 0 && handshake) { connect_peer(); }
    if (count > 0 && tick != rows[count-1].tick + 1) { fail("tick_order"); }
    Row &row = rows[count];
    row.tick = tick;
    vehicle_id = vehicle;
    row.sample = sample;
    row.before = AP_HAL::micros64();
    const uint64_t began = wall_us();
    row.wall = began;
    if (handshake) { exchange(vehicle, tick, row.before); }
    row.elapsed = wall_us() - began;
    row.after = AP_HAL::micros64();
    if (row.before != row.after) { fail("clock_advanced_during_wait"); }
    pending = true;
}

void end(uint32_t tick, const Sample &sample)
{
    if (!pending) { return; }
    Row &row = rows[count];
    if (row.tick != tick) { fail("tick_changed_before_control_completed"); }
    row.complete = AP_HAL::micros64();
    row.sample.target_roll = sample.target_roll;
    row.sample.target_pitch = sample.target_pitch;
    memcpy(row.sample.servo, sample.servo, sizeof(sample.servo));
    pending = false;
    if (++count == COUNT) {
        finished = true;
        if (fd >= 0) { close(fd); fd = -1; }
        save();
    }
}
} // namespace NavPyStep

void Plane::sim_companion_step()
{
    if (!NavPyStep::measuring()) { return; }
    NavPyStep::Sample sample{};
    sample.roll = ahrs.get_roll();
    sample.pitch = ahrs.get_pitch();
    sample.airspeed = nanf("");
    ahrs.airspeed_estimate(sample.airspeed);
    sample.mode = uint8_t(control_mode->mode_number());
    NavPyStep::begin(uint32_t(g.sysid_this_mav.get()), scheduler.ticks32(), sample);
}

void Plane::sim_companion_step_done()
{
    if (!NavPyStep::awaiting_end()) { return; }
    NavPyStep::Sample sample{};
    sample.target_roll = nav_roll_cd;
    sample.target_pitch = nav_pitch_cd;
    hal.rcout->read(sample.servo, 16);
    NavPyStep::end(scheduler.ticks32(), sample);
}
#endif
