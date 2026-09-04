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
  true closest-point-of-approach (CPA) tracker against a fixed target,
  computed per physics step on the raw double-precision simulator state
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_CPA_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <AP_Param/AP_Param.h>

namespace SITL {

/*
  CPASim tracks the exact minimum of the piecewise-linear interpolant of
  the vehicle-to-target distance, in the simulator's origin-anchored
  double-precision NED frame, at physics rate.

  Frame choice: the raw physics state `position` is metric NED from
  `origin`, and the 1 Hz origin rebase keeps that anchor within seconds
  of flight of the vehicle. Projecting the target from the SAME origin
  makes the only flat-NED projection near the CPA one of a short
  (tens of metres) leg, which is exact to well below a millimetre. A
  home-anchored residual was rejected: it mixes the metric position leg
  with kilometre-lever home projections, whose triangle-closure error is
  millimetres — the size of the misses being measured.

  The target arrives as decimal-chunked integer parameters because the
  MAVLink parameter transport is float32: a single degE7 int32 above
  2^24 would round by whole LSBs (~0.36 m at typical latitudes). Each
  chunk stays below 2^24 and is therefore exact:
      value_degE7 = HI * 10000 + LO

  Results stream to the dataflash log:
   - SCPC: configuration / lifecycle events carrying the reconstructed
     integer target as the configuration authority (the PARAM_VALUE echo
     is float32 and is never part of the truth chain);
   - SCPA: one row per closed 20 ms sim-time interval with the exact
     interval minimum and the running epoch-global minimum.

  Interval contract (the analyzer relies on this exactly):
   - The epoch's SCPC EpUS is the first interval boundary; Seq starts at
     0; interval N spans [EpUS + N*20000, EpUS + (N+1)*20000).
   - A normal row's TimeUS is the exact interval-end boundary; physics
     segments crossing a boundary are split there with the same linear
     interpolation, so per-interval coverage is exact.
   - Seq advances even when a log write is dropped, so gaps are visible.
   - Partial (early-closed) intervals carry flag bits. Unknown
     discontinuities (non-monotonic time, non-finite state, an origin
     altitude change) fault the epoch closed; scoring stays off until
     ENABLE is cycled.
*/
class CPASim {
public:
    CPASim();

    // called at physics rate from Aircraft::update_dynamics with the raw
    // double-precision position (metric NED from origin), the current
    // origin, and the physics-step end time (the same value fill_fdm()
    // stamps into fdm.timestamp_us). Origin lat/lng changes are the
    // expected 1 Hz rebase; an origin altitude change is unknown
    // corruption and faults the epoch.
    void update(const Vector3d &position, const Location &origin, uint64_t now_us);

    // parameter table
    static const struct AP_Param::GroupInfo var_info[];

    // exact minimum over one segment of target-relative residuals
    struct SegmentMin {
        double d3_sq;      // squared 3D distance at the segment minimum
        double dh;         // horizontal distance at that point
        double dv;         // vertical distance at that point
        uint64_t time_us;  // interpolated time of that point
    };

    // pure helpers, public for unit tests
    static int64_t reassemble_chunks(int32_t hi, int32_t lo);
    static bool validate_target(int32_t lat_hi_v, int32_t lat_lo_v,
                                int32_t lng_hi_v, int32_t lng_lo_v,
                                int32_t alt_cm_v,
                                int32_t &lat_e7, int32_t &lng_e7);
    static SegmentMin segment_min(const Vector3d &a, const Vector3d &b,
                                  uint64_t ta_us, uint64_t tb_us);

protected:
    // parameters (protected so unit tests can drive them via a subclass)
    AP_Int8 enable;
    AP_Int32 lat_hi;
    AP_Int32 lat_lo;
    AP_Int32 lng_hi;
    AP_Int32 lng_lo;
    AP_Int32 alt_cm;

    // SCPC lifecycle event codes
    enum class Event : uint8_t {
        ENABLE = 1,
        DISABLE = 2,
        RETARGET = 3,
        FAULT = 4,
        INVALID_CONFIG = 5,
    };

    // SCPA flag bits
    static const uint8_t FLAG_PARTIAL = 1;  // interval closed before its boundary
    static const uint8_t FLAG_FINAL = 2;    // last row of its epoch

    static const uint64_t INTERVAL_US = 20000;

    // scoring state
    bool _active = false;
    bool _faulted = false;
    int8_t _last_enable = 0;
    uint32_t _epoch = 0;
    uint64_t _epoch_us = 0;
    int32_t _tgt_lat_e7 = 0;
    int32_t _tgt_lng_e7 = 0;
    int32_t _tgt_alt_cm = 0;
    // target in origin-anchored NED, refreshed when the origin rebases
    // (1 Hz) instead of recomputed at physics rate; near the CPA the
    // origin sits within seconds of flight of the target, so this is a
    // short-leg projection and the residual error is sub-millimetre
    Vector3d _tgt_ned_origin;
    int32_t _origin_lat = 0;
    int32_t _origin_lng = 0;
    int32_t _origin_alt = 0;
    Vector3d _prev_res;
    uint64_t _prev_time_us = 0;
    uint32_t _seq = 0;
    SegmentMin _int_best {};
    bool _int_best_valid = false;
    SegmentMin _glob_best {};
    bool _glob_best_valid = false;

    // config validation memo: revalidate only when the tuple changes, so
    // an invalid configuration is reported once, not at physics rate
    int32_t _cfg_seen[5] {};
    bool _cfg_seen_valid = false;

    // single-slot pending SCPC event, emitted once the logger is up; a
    // lost earlier event leaves the epoch without its SCPC, which the
    // analyzer treats as unscoreable (fail closed)
    bool _scpc_pending = false;
    Event _scpc_event = Event::DISABLE;
    uint64_t _scpc_time_us = 0;
    uint32_t _scpc_epoch = 0;
    uint64_t _scpc_epoch_us = 0;
    int32_t _scpc_lat_e7 = 0;
    int32_t _scpc_lng_e7 = 0;
    int32_t _scpc_alt_cm = 0;
    bool _scpc_emitted_for_epoch = false;

    void open_epoch(const Vector3d &position, const Location &origin,
                    uint64_t now_us, Event ev,
                    int32_t new_lat_e7, int32_t new_lng_e7, int32_t new_alt_cm);
    void close_epoch(Event ev, uint64_t now_us);
    void fault(uint64_t now_us);
    void accumulate_segment(const Vector3d &a, const Vector3d &b,
                            uint64_t ta_us, uint64_t tb_us);
    void close_interval(uint64_t boundary_us);
    void flush_partial_row(uint64_t now_us);
    void write_row(uint64_t time_us, uint32_t row_seq, uint8_t flags);
    void queue_scpc(Event ev, uint64_t now_us,
                    int32_t lat_e7_v, int32_t lng_e7_v, int32_t alt_cm_v);
    void flush_scpc();
    bool can_log() const;
    void refresh_target_projection(const Location &origin);
    Vector3d residual(const Vector3d &position) const;
    bool config_tuple_changed() const;
    void remember_config_tuple();
    static bool finite3(const Vector3d &v);
};

}  // namespace SITL

#endif  // AP_SIM_CPA_ENABLED
