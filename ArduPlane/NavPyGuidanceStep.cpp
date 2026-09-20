#include "Plane.h"
#include "NavPyGuidanceStep.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && defined(__linux__)
#include <SITL/SITL.h>
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

namespace NavPyGuidance {
namespace {
constexpr unsigned MAX_COUNT = 5000;
constexpr uint64_t DEADLINE_US = 5000000;
struct PACKED Header {
    char magic[8];
    uint64_t boot[2], step, source_us;
    uint32_t vehicle, tick;
};
struct PACKED State {
    uint64_t truth_us, applied_step;
    uint32_t applied_kind, mode, armed, status;
    uint64_t apply_before_us, apply_after_us;
    uint32_t log_disarmed, logger_ready;
    double observation[6], truth[6], limits[5];
    int32_t previous_targets[2];
    uint16_t previous_servos[16];
};
struct PACKED Request { Header header; State state; };
struct PACKED Command {
    uint64_t computed_step, apply_step;
    uint32_t kind, mask;
    float q[4], throttle;
};
struct PACKED Reply { Header header; Command command; };
static_assert(sizeof(Request) == 280, "snapshot layout");
static_assert(sizeof(Reply) == 92, "reply layout");
#if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
#error simulator guidance protocol requires a little-endian host
#endif
struct Row {
    uint64_t source_us, after_us, complete_us, wall_us, wait_us;
    uint32_t tick;
    int32_t targets[2];
    uint16_t servos[16];
};
Row rows[MAX_COUNT];
unsigned count, total;
uint64_t start_us, boot[2];
bool configured, enabled, finished, pending_end, dispatching;
int fd = -1;
Command queued{};

uint64_t wall_us()
{
    timespec t{};
    if (clock_gettime(CLOCK_MONOTONIC, &t)) { ::_exit(72); }
    return uint64_t(t.tv_sec)*1000000 + t.tv_nsec/1000;
}
uint64_t environment_number(const char *name)
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
    const char *mode = getenv("NAVPY_GUIDANCE_MODE");
    if (!mode) { return; }
    if (strcmp(mode, "closed-loop") || getenv("NAVPY_STEP_MODE")) { fail("configuration"); }
    start_us = environment_number("NAVPY_GUIDANCE_START_US");
    const uint64_t n = environment_number("NAVPY_GUIDANCE_COUNT");
    if (n < 3 || n > MAX_COUNT) { fail("window_size"); }
    total = unsigned(n);
    if (getrandom(boot, sizeof(boot), 0) != sizeof(boot) || !(boot[0] || boot[1])) { fail("boot"); }
    enabled = true;
}
void ready(short events, uint64_t deadline)
{
    for (;;) {
        const uint64_t now = wall_us();
        if (now >= deadline) { fail("deadline"); }
        pollfd p{fd, events, 0};
        const int result = poll(&p, 1, int((deadline-now+999)/1000));
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
    strcpy(address.sun_path, "navpy-guidance.sock");
    if (connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address))) { fail("connect"); }
}
void validate(const Reply &reply, const Header &header)
{
    if (memcmp(&reply.header, &header, sizeof(header))) { fail("identity_or_version"); }
    const Command &c = reply.command;
    if (c.computed_step != header.step || c.apply_step != header.step+1) { fail("apply_step"); }
    if (c.kind > 3) { fail("command_kind"); }
    float norm = 0;
    for (float v : c.q) {
        if (!isfinite(v)) { fail("nonfinite_command"); }
        norm += v*v;
    }
    if (!isfinite(c.throttle)) { fail("nonfinite_command"); }
    const float empty[5]{};
    if (c.kind == 1) {
        if ((c.mask != 132 && c.mask != 196) || fabsf(norm-1) > 0.0001f ||
            c.throttle < 0 || c.throttle > 1) { fail("attitude_command"); }
        Quaternion q(c.q[0], c.q[1], c.q[2], c.q[3]);
        if (fabsf(q.get_euler_yaw()) > 0.0001f) { fail("command_yaw"); }
    } else if (c.mask || memcmp(c.q, empty, sizeof(empty)) != 0) {
        fail("noncanonical_empty_command");
    }
    if (header.step == total && c.kind) { fail("undrained_command"); }
}
void exchange(const State &state, uint32_t vehicle, uint32_t tick)
{
    if (count == 0) { connect_peer(); }
    Row &row = rows[count];
    row.source_us = AP_HAL::micros64();
    row.tick = tick;
    if (count && (row.source_us <= rows[count-1].source_us || tick != rows[count-1].tick+1)) {
        fail("control_cadence");
    }
    Request request{};
    memcpy(request.header.magic, "NVGUID03", 8);
    memcpy(request.header.boot, boot, sizeof(boot));
    request.header.step = count+1;
    request.header.source_us = row.source_us;
    request.header.vehicle = vehicle;
    request.header.tick = tick;
    request.state = state;
    row.wall_us = wall_us();
    const uint64_t deadline = row.wall_us+DEADLINE_US;
    ready(POLLOUT, deadline);
    if (send(fd, &request, sizeof(request), MSG_NOSIGNAL) != sizeof(request)) { fail("send"); }
    ready(POLLIN, deadline);
    Reply reply{};
    const ssize_t size = recv(fd, &reply, sizeof(reply), MSG_TRUNC);
    if (size != sizeof(reply)) { fail(size == 0 ? "disconnected" : "frame_length"); }
    validate(reply, request.header);
    queued = reply.command;
    row.wait_us = wall_us()-row.wall_us;
    row.after_us = AP_HAL::micros64();
    if (row.after_us != row.source_us) { fail("clock_advanced_during_wait"); }
    pending_end = true;
}
void save()
{
    FILE *out = fopen("navpy-guidance.csv", "wx");
    if (!out) { fail("evidence_open"); }
    fprintf(out, "step,tick,source_us,after_us,complete_us,wall_us,wait_us,target_roll,target_pitch");
    for (unsigned j=0;j<16;j++) { fprintf(out, ",servo%u", j); }
    fprintf(out, "\n");
    for (unsigned i=0;i<total;i++) {
        const Row &r = rows[i];
        fprintf(out, "%u,%u,%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%" PRIu64 ",%d,%d",
                i+1,r.tick,r.source_us,r.after_us,r.complete_us,r.wall_us,r.wait_us,r.targets[0],r.targets[1]);
        for (unsigned j=0;j<16;j++) { fprintf(out, ",%u", unsigned(r.servos[j])); }
        fprintf(out, "\n");
    }
    const bool bad = ferror(out);
    if (fclose(out) || bad) { fail("evidence_write"); }
    printf("NAVPY_GUIDANCE_COMPLETE count=%u\n", total);
    fflush(stdout);
}
} // namespace

[[noreturn]] void fail(const char *reason)
{
    fprintf(stderr,"NAVPY_GUIDANCE_INVALID %s step=%u\n",reason,count+1);
    fflush(stderr);
    if (fd >= 0) { close(fd); }
    ::_exit(72);
}
bool active() { return enabled && !finished && AP_HAL::micros64() >= start_us; }
bool internal_dispatch() { return dispatching; }
static bool measuring()
{
    if (!configured) { configure(); }
    return active();
}
} // namespace NavPyGuidance

bool GCS_MAVLINK_Plane::sim_guidance_attitude(const mavlink_set_attitude_target_t &target)
{
    if (target.target_system != plane.g.sysid_this_mav || target.target_component != 0 ||
        plane.control_mode != &plane.mode_guided) { return false; }
    mavlink_message_t message{};
    mavlink_msg_set_attitude_target_encode(255, 0, &message, &target);
    NavPyGuidance::dispatching = true;
    handle_set_attitude_target(message);
    NavPyGuidance::dispatching = false;
    const uint32_t now = AP_HAL::millis();
    return uint32_t(plane.guided_state.last_forced_rpy_ms.x) == now &&
           uint32_t(plane.guided_state.last_forced_rpy_ms.y) == now &&
           ((target.type_mask & 64) || plane.guided_state.last_forced_throttle_ms == now);
}

void Plane::sim_guidance_step()
{
    using namespace NavPyGuidance;
    if (!measuring()) { return; }
    if (pending_end) { fail("unfinished_step"); }
    const Command &c = queued;
    State state{};
    state.apply_before_us = AP_HAL::micros64();
    state.log_disarmed = AP::logger().log_while_disarmed();
    state.logger_ready = AP::logger().logging_started();
    if (count && (c.computed_step != count || c.apply_step != count+1)) { fail("pending_identity"); }
    if (c.kind == 1) {
        mavlink_set_attitude_target_t target{};
        target.time_boot_ms = AP_HAL::millis();
        target.target_system = g.sysid_this_mav;
        target.type_mask = c.mask;
        memcpy(target.q, c.q, sizeof(target.q));
        target.thrust = c.throttle;
        auto *link = gcs().chan(0);
        if (!link || !link->sim_guidance_attitude(target)) { fail("attitude_rejected"); }
    } else if (c.kind == 2) {
        if (!state.log_disarmed || !state.logger_ready) { fail("logger_not_ready"); }
        if (arming.is_armed() || !set_mode(mode_takeoff, ModeReason::GCS_COMMAND) ||
            !arming.arm(AP_Arming::Method::MAVLINK)) { fail("takeoff_arm_rejected"); }
    } else if (c.kind == 3) {
        if (!arming.is_armed() || !set_mode(mode_guided, ModeReason::GCS_COMMAND)) { fail("guided_rejected"); }
    }
    state.apply_after_us = AP_HAL::micros64();
    if (state.apply_before_us != state.apply_after_us) { fail("command_advanced_clock"); }
    state.applied_step = count;
    state.applied_kind = c.kind;
    state.mode = uint32_t(control_mode->mode_number());
    state.armed = arming.is_armed();
    state.observation[0] = ahrs.get_roll();
    state.observation[1] = ahrs.get_pitch();
    if (!flight_option_enabled(FlightOptions::GCS_REMOVE_TRIM_PITCH)) {
        state.observation[1] -= radians(g.pitch_trim);
    }
    const Vector3f gyro = ahrs.get_gyro();
    state.observation[2]=gyro.x; state.observation[3]=gyro.y; state.observation[4]=gyro.z;
    float estimated_airspeed = nanf("");
    if (!ahrs.airspeed_estimate(estimated_airspeed)) { fail("airspeed_unavailable"); }
    state.observation[5] = estimated_airspeed;
    const auto *sim = AP::sitl();
    if (!sim) { fail("missing_simulator"); }
    state.truth_us = sim->state.timestamp_us;
    state.truth[0]=sim->state.latitude; state.truth[1]=sim->state.longitude; state.truth[2]=sim->state.altitude;
    state.truth[3]=sim->state.rollDeg; state.truth[4]=sim->state.pitchDeg; state.truth[5]=sim->state.yawDeg;
    const char *names[] = {"PTCH_LIM_MIN_DEG", "PTCH_LIM_MAX_DEG", "ROLL_LIMIT_DEG", "PTCH2SRV_TCONST"};
    for (unsigned i=0;i<4;i++) {
        enum ap_var_type type;
        const AP_Param *param = AP_Param::find(names[i], &type);
        if (!param) { fail("missing_law_parameter"); }
        state.limits[i] = param->cast_to_float(type);
    }
    // Explicit experiment profile: no throttle override, like direct_pixel_pn_child -tt -1.
    state.limits[4] = -1;
    if (count) {
        memcpy(state.previous_targets, rows[count-1].targets, sizeof(state.previous_targets));
        memcpy(state.previous_servos, rows[count-1].servos, sizeof(state.previous_servos));
    }
    exchange(state, g.sysid_this_mav, scheduler.ticks32());
}
void Plane::sim_guidance_step_done()
{
    using namespace NavPyGuidance;
    if (!pending_end) { return; }
    Row &row = rows[count];
    if (row.tick != scheduler.ticks32()) { fail("tick_changed"); }
    row.complete_us = AP_HAL::micros64();
    row.targets[0]=nav_roll_cd; row.targets[1]=nav_pitch_cd;
    hal.rcout->read(row.servos,16);
    pending_end=false;
    if (++count == total) {
        finished=true;
        close(fd); fd=-1;
        save();
    }
}
#endif
