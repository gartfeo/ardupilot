#include "SIM_NavPyPose.h"

namespace SITL {

NavPyPoseSample NavPyPoseSample::capture(const Location &origin, const Location &home,
                                  const Vector3d &position, const Location &rounded,
                                  uint64_t sequence, uint64_t physics_sequence,
                                  uint64_t now, uint8_t branch)
    {
        NavPyPoseSample s{};
        s.sequence = sequence;
        s.physics_sequence = physics_sequence;
        s.conversion_us = now;
        s.branch = branch;
        s.origin_lat = origin.lat;
        s.origin_lng = origin.lng;
        s.home_alt = home.alt;
        s.rounded_lat = rounded.lat;
        s.rounded_lng = rounded.lng;
        s.rounded_alt = rounded.alt;
        constexpr float inverse = LATLON_TO_M_INV;
        const ftype north = position.x, east = position.y;
        s.dlat = north * inverse;
        s.altitude_cm = home.alt - position.z * 100.0f;
        if (!isfinite(s.dlat) || !isfinite(s.altitude_cm) ||
            s.dlat < INT32_MIN || s.dlat > INT32_MAX ||
            s.altitude_cm < INT32_MIN || s.altitude_cm > INT32_MAX) {
            return s;
        }
        const int32_t dlat = int32_t(s.dlat);
        const int64_t lat = int64_t(origin.lat) + dlat;
        if (lat < -900000000 || lat > 900000000) {
            return s; // no latitude limiting supported by this experiment
        }
        s.dlng = (east * inverse) / Location::longitude_scale(origin.lat + dlat/2);
        // This narrower bound also makes the subsequent int64 conversion safe.
        if (!isfinite(s.dlng) || s.dlng < -3600000000.0 || s.dlng > 3600000000.0) {
            return s;
        }
        const int64_t lng = int64_t(origin.lng) + int64_t(s.dlng);
        if (lng < -1800000000 || lng > 1800000000) {
            return s; // no longitude wrapping supported by this experiment
        }
        // Explicit doubles: callers may use -fsingle-precision-constant.
        // Compute once here; the wire exporter only copies the stored values.
        constexpr double angle_scale = static_cast<double>(1.0e-7L);
        constexpr double height_scale = static_cast<double>(1.0e-2L);
        s.precast[0] = (double(origin.lat) + s.dlat) * angle_scale;
        s.precast[1] = (double(origin.lng) + s.dlng) * angle_scale;
        s.precast[2] = s.altitude_cm * height_scale;
        s.valid = lat == rounded.lat && lng == rounded.lng &&
                  int32_t(s.altitude_cm) == rounded.alt;
        return s;
    }

} // namespace SITL
