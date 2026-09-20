#pragma once

// Private, opt-in NavPy experiment evidence. Never used by flight sensors.
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <stdint.h>
#include <limits.h>

namespace SITL {

struct NavPyPoseSample {
    uint64_t sequence{}, physics_sequence{}, expected_physics_sequence{};
    uint64_t conversion_us{}, publish_us{};
    int32_t origin_lat{}, origin_lng{}, home_alt{};
    int32_t rounded_lat{}, rounded_lng{}, rounded_alt{};
    double dlat{}, dlng{}, altitude_cm{};
    double precast[3]{};
    uint8_t branch{}, ftype_size{sizeof(ftype)};
    bool valid{};

    // Mirror Location's expressions, including ftype argument conversion and
    // FLOAT scaling constant, retaining only the discarded final fractions.
    // The real Location result remains authoritative and is checked here.
    static NavPyPoseSample capture(const Location &origin, const Location &home,
                                  const Vector3d &position, const Location &rounded,
                                  uint64_t sequence, uint64_t physics_sequence,
                                  uint64_t now, uint8_t branch);
};

class NavPyPoseCapture {
public:
    bool enabled{};

    void raw(const Location &origin, const Location &home, const Vector3d &position,
             const Location &rounded, uint64_t now)
    {
        if (!enabled) { return; }
        _raw = NavPyPoseSample::capture(origin, home, position, rounded,
                                       ++_sequence, ++_physics_sequence, now, 1);
    }

    void smooth(const Location &origin, const Location &home, const Vector3d &position,
                const Location &rounded, uint64_t now)
    {
        if (!enabled) { return; }
        _smoothed = NavPyPoseSample::capture(origin, home, position, rounded,
                                            ++_sequence, _physics_sequence, now, 2);
    }

    void reset_smoothing()
    {
        if (!enabled) { return; }
        _smoothed = _raw; // same conversion event, including pre-rebase origin
        _smoothed.branch = 3;
    }

    void stale_smoothing() { _smoothed.valid = false; }

    NavPyPoseSample publish(uint64_t now, bool smoothed, const Location &published)
    {
        NavPyPoseSample s = smoothed ? _smoothed : _raw;
        s.publish_us = now;
        s.expected_physics_sequence = _physics_sequence;
        s.valid = enabled && s.valid && s.sequence != 0 &&
                  s.rounded_lat == published.lat && s.rounded_lng == published.lng &&
                  s.rounded_alt == published.alt &&
                  s.physics_sequence == _physics_sequence &&
                  _physics_sequence > _published_physics_sequence;
        _published_physics_sequence = _physics_sequence;
        return s;
    }

private:
    uint64_t _sequence{}, _physics_sequence{}, _published_physics_sequence{};
    NavPyPoseSample _raw{}, _smoothed{};
};

} // namespace SITL
