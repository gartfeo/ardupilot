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

#include "SIM_CPA.h"

#if AP_SIM_CPA_ENABLED

#include <AP_Logger/AP_Logger.h>
#include <cmath>

using namespace SITL;

// CPASim parameters
const AP_Param::GroupInfo CPASim::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: CPA tracker enable
    // @Description: Enable the true closest-point-of-approach tracker. Set the target chunk parameters first and this last. Cycling it off and back on clears a fault and starts a new scoring epoch.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, CPASim, enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: LAT_HI
    // @DisplayName: CPA target latitude high chunk
    // @Description: Target latitude in 1e-7 degrees, divided by 10000 and truncated toward zero. Chunked so the float32 parameter transport stays exact: lat_degE7 = LAT_HI*10000 + LAT_LO.
    // @User: Advanced
    AP_GROUPINFO("LAT_HI", 2, CPASim, lat_hi, 0),

    // @Param: LAT_LO
    // @DisplayName: CPA target latitude low chunk
    // @Description: Remainder of the target latitude in 1e-7 degrees, magnitude below 10000, same sign as LAT_HI or zero.
    // @User: Advanced
    AP_GROUPINFO("LAT_LO", 3, CPASim, lat_lo, 0),

    // @Param: LNG_HI
    // @DisplayName: CPA target longitude high chunk
    // @Description: Target longitude in 1e-7 degrees, divided by 10000 and truncated toward zero. lng_degE7 = LNG_HI*10000 + LNG_LO.
    // @User: Advanced
    AP_GROUPINFO("LNG_HI", 4, CPASim, lng_hi, 0),

    // @Param: LNG_LO
    // @DisplayName: CPA target longitude low chunk
    // @Description: Remainder of the target longitude in 1e-7 degrees, magnitude below 10000, same sign as LNG_HI or zero.
    // @User: Advanced
    AP_GROUPINFO("LNG_LO", 5, CPASim, lng_lo, 0),

    // @Param: ALT_CM
    // @DisplayName: CPA target altitude
    // @Description: Target altitude AMSL in centimetres. Range limited to +/-1000000 (10 km) which is well inside the exact float32 integer range, so no chunking is needed.
    // @Units: cm
    // @Range: -1000000 1000000
    // @User: Advanced
    AP_GROUPINFO("ALT_CM", 6, CPASim, alt_cm, 0),

    AP_GROUPEND
};

CPASim::CPASim()
{
    AP_Param::setup_object_defaults(this, var_info);
}

int64_t CPASim::reassemble_chunks(int32_t hi, int32_t lo)
{
    return (int64_t)hi * 10000 + (int64_t)lo;
}

bool CPASim::validate_target(int32_t lat_hi_v, int32_t lat_lo_v,
                             int32_t lng_hi_v, int32_t lng_lo_v,
                             int32_t alt_cm_v,
                             int32_t &lat_e7, int32_t &lng_e7)
{
    // remainder magnitude and sign consistency catch mis-typed chunks
    if (lat_lo_v <= -10000 || lat_lo_v >= 10000 ||
        lng_lo_v <= -10000 || lng_lo_v >= 10000) {
        return false;
    }
    if ((lat_hi_v > 0 && lat_lo_v < 0) || (lat_hi_v < 0 && lat_lo_v > 0)) {
        return false;
    }
    if ((lng_hi_v > 0 && lng_lo_v < 0) || (lng_hi_v < 0 && lng_lo_v > 0)) {
        return false;
    }
    const int64_t lat64 = reassemble_chunks(lat_hi_v, lat_lo_v);
    const int64_t lng64 = reassemble_chunks(lng_hi_v, lng_lo_v);
    if (lat64 < -900000000 || lat64 > 900000000) {
        return false;
    }
    if (lng64 < -1800000000 || lng64 > 1800000000) {
        return false;
    }
    if (lat64 == 0 && lng64 == 0) {
        // all-zero coordinates mean "never configured", mirroring the
        // stream scorer's not-both-zero rule
        return false;
    }
    if (alt_cm_v < -1000000 || alt_cm_v > 1000000) {
        return false;
    }
    lat_e7 = (int32_t)lat64;
    lng_e7 = (int32_t)lng64;
    return true;
}

CPASim::SegmentMin CPASim::segment_min(const Vector3d &a, const Vector3d &b,
                                       uint64_t ta_us, uint64_t tb_us)
{
    const Vector3d d = b - a;
    const double dd = d * d;
    double f = 0.0;
    if (dd > 0.0) {
        f = constrain_value(-(a * d) / dd, 0.0, 1.0);
    }
    const Vector3d p = a + d * f;
    SegmentMin m;
    m.d3_sq = p * p;
    m.dh = sqrt(p.x * p.x + p.y * p.y);
    m.dv = fabs(p.z);
    m.time_us = ta_us + (uint64_t)llround(f * (double)(tb_us - ta_us));
    return m;
}

bool CPASim::finite3(const Vector3d &v)
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

void CPASim::refresh_target_projection(const Location &origin)
{
    // one flat-NED projection, anchored where the physics position is
    // anchored; the 1 Hz rebase keeps this a short leg near the CPA, so
    // the projection error there is far below a millimetre. Refreshing
    // mid-segment is fine: the cached vector and the raw position jump by
    // the same rebase offset up to that sub-millimetre model difference.
    const Location tgt_loc(_tgt_lat_e7, _tgt_lng_e7, _tgt_alt_cm, Location::AltFrame::ABSOLUTE);
    _tgt_ned_origin = origin.get_distance_NED_double(tgt_loc);
    _origin_lat = origin.lat;
    _origin_lng = origin.lng;
}

Vector3d CPASim::residual(const Vector3d &position) const
{
    // raw metric position minus the target's origin-anchored projection:
    // no home-lever projections enter the residual, so its frame error at
    // the CPA is set by the short origin->target leg alone
    return position - _tgt_ned_origin;
}

bool CPASim::config_tuple_changed() const
{
    if (!_cfg_seen_valid) {
        return true;
    }
    return _cfg_seen[0] != lat_hi.get() ||
           _cfg_seen[1] != lat_lo.get() ||
           _cfg_seen[2] != lng_hi.get() ||
           _cfg_seen[3] != lng_lo.get() ||
           _cfg_seen[4] != alt_cm.get();
}

void CPASim::remember_config_tuple()
{
    _cfg_seen[0] = lat_hi.get();
    _cfg_seen[1] = lat_lo.get();
    _cfg_seen[2] = lng_hi.get();
    _cfg_seen[3] = lng_lo.get();
    _cfg_seen[4] = alt_cm.get();
    _cfg_seen_valid = true;
}

bool CPASim::can_log() const
{
    AP_Logger *logger = AP_Logger::get_singleton();
    return logger != nullptr && logger->logging_started();
}

void CPASim::queue_scpc(Event ev, uint64_t now_us,
                        int32_t lat_e7_v, int32_t lng_e7_v, int32_t alt_cm_v)
{
    _scpc_pending = true;
    _scpc_event = ev;
    _scpc_time_us = now_us;
    _scpc_epoch = _epoch;
    _scpc_epoch_us = _epoch_us;
    _scpc_lat_e7 = lat_e7_v;
    _scpc_lng_e7 = lng_e7_v;
    _scpc_alt_cm = alt_cm_v;
}

void CPASim::flush_scpc()
{
    if (!_scpc_pending || !can_log()) {
        return;
    }
    // @LoggerMessage: SCPC
    // @Description: CPA tracker configuration and lifecycle events. The integer target fields are the configuration authority; the float32 PARAM_VALUE echo is never part of the truth chain.
    // @Field: TimeUS: event time
    // @Field: Ep: scoring epoch this event belongs to
    // @Field: EpUS: epoch start time, the first SCPA interval boundary
    // @Field: Ev: event, 1:enable 2:disable 3:retarget 4:fault 5:invalid config
    // @Field: LatE7: reconstructed target latitude, 1e-7 degrees
    // @Field: LngE7: reconstructed target longitude, 1e-7 degrees
    // @Field: AltCM: target altitude AMSL, centimetres
    AP::logger().WriteCritical("SCPC",
                               "TimeUS,Ep,EpUS,Ev,LatE7,LngE7,AltCM",
                               "QIQBiii",
                               (uint64_t)_scpc_time_us,
                               (uint32_t)_scpc_epoch,
                               (uint64_t)_scpc_epoch_us,
                               (uint8_t)_scpc_event,
                               (int32_t)_scpc_lat_e7,
                               (int32_t)_scpc_lng_e7,
                               (int32_t)_scpc_alt_cm);
    _scpc_pending = false;
    if (_scpc_event == Event::ENABLE || _scpc_event == Event::RETARGET) {
        _scpc_emitted_for_epoch = true;
    }
}

void CPASim::write_row(uint64_t time_us, uint32_t row_seq, uint8_t flags)
{
    if (!can_log() || !_scpc_emitted_for_epoch) {
        // rows before the epoch's SCPC reached the log are dropped; the
        // Seq gap makes the missing coverage visible to the analyzer
        return;
    }
    if (!_int_best_valid || !_glob_best_valid) {
        return;
    }
    // @LoggerMessage: SCPA
    // @Description: CPA tracker interval minima. One row per closed 20 ms sim-time interval; interval N spans EpUS+N*20000 to EpUS+(N+1)*20000. Fl bit 1 marks an interval closed before its boundary, bit 2 the last row of its epoch.
    // @Field: TimeUS: interval end boundary for normal rows, close time for partial rows
    // @Field: Ep: scoring epoch
    // @Field: Seq: interval index from 0 per epoch, advances even if a write is dropped
    // @Field: Fl: flag bits, 1:partial 2:final
    // @Field: CpaUS: interpolated time of the interval minimum
    // @Field: D3: interval minimum 3D distance
    // @Field: DH: horizontal distance at the interval minimum
    // @Field: DV: vertical distance at the interval minimum
    // @Field: GCpUS: interpolated time of the epoch-global minimum
    // @Field: GD3: epoch-global minimum 3D distance
    // @Field: GDH: horizontal distance at the global minimum
    // @Field: GDV: vertical distance at the global minimum
    AP::logger().Write("SCPA",
                       "TimeUS,Ep,Seq,Fl,CpaUS,D3,DH,DV,GCpUS,GD3,GDH,GDV",
                       "s---smmmsmmm",
                       "F---F000F000",
                       "QIIBQdddQddd",
                       (uint64_t)time_us,
                       (uint32_t)_epoch,
                       (uint32_t)row_seq,
                       (uint8_t)flags,
                       (uint64_t)_int_best.time_us,
                       (double)sqrt(_int_best.d3_sq),
                       (double)_int_best.dh,
                       (double)_int_best.dv,
                       (uint64_t)_glob_best.time_us,
                       (double)sqrt(_glob_best.d3_sq),
                       (double)_glob_best.dh,
                       (double)_glob_best.dv);
}

void CPASim::accumulate_segment(const Vector3d &a, const Vector3d &b,
                                uint64_t ta_us, uint64_t tb_us)
{
    const SegmentMin m = segment_min(a, b, ta_us, tb_us);
    if (!_int_best_valid || m.d3_sq < _int_best.d3_sq) {
        _int_best = m;
        _int_best_valid = true;
    }
    if (!_glob_best_valid || m.d3_sq < _glob_best.d3_sq) {
        _glob_best = m;
        _glob_best_valid = true;
    }
}

void CPASim::close_interval(uint64_t boundary_us)
{
    const uint32_t row_seq = _seq;
    _seq++;  // advances before the write attempt so a drop leaves a visible gap
    write_row(boundary_us, row_seq, 0);
    _int_best_valid = false;
}

void CPASim::flush_partial_row(uint64_t now_us)
{
    if (_int_best_valid) {
        const uint32_t row_seq = _seq;
        _seq++;
        write_row(now_us, row_seq, FLAG_PARTIAL | FLAG_FINAL);
    }
    _int_best_valid = false;
}

void CPASim::open_epoch(const Vector3d &position, const Location &origin,
                        uint64_t now_us, Event ev,
                        int32_t new_lat_e7, int32_t new_lng_e7, int32_t new_alt_cm)
{
    _tgt_lat_e7 = new_lat_e7;
    _tgt_lng_e7 = new_lng_e7;
    _tgt_alt_cm = new_alt_cm;
    refresh_target_projection(origin);
    _origin_alt = origin.alt;
    _epoch++;
    _epoch_us = now_us;
    _seq = 0;
    _int_best_valid = false;
    _glob_best_valid = false;
    _scpc_emitted_for_epoch = false;
    const Vector3d seed = residual(position);
    if (!finite3(seed)) {
        fault(now_us);
        return;
    }
    _prev_res = seed;
    _prev_time_us = now_us;
    queue_scpc(ev, now_us, _tgt_lat_e7, _tgt_lng_e7, _tgt_alt_cm);
    _active = true;
}

void CPASim::close_epoch(Event ev, uint64_t now_us)
{
    flush_partial_row(now_us);
    queue_scpc(ev, now_us, _tgt_lat_e7, _tgt_lng_e7, _tgt_alt_cm);
    _active = false;
}

void CPASim::fault(uint64_t now_us)
{
    // unknown corruption: no data row (the interval may be poisoned),
    // just the audit event; recovery requires cycling ENABLE off and on
    queue_scpc(Event::FAULT, now_us, _tgt_lat_e7, _tgt_lng_e7, _tgt_alt_cm);
    _active = false;
    _faulted = true;
    _int_best_valid = false;
    flush_scpc();
}

void CPASim::update(const Vector3d &position, const Location &origin, uint64_t now_us)
{
    flush_scpc();

    const bool enabled = enable.get() != 0;

    if (!enabled) {
        if (_active) {
            close_epoch(Event::DISABLE, now_us);
        }
        // a fresh OFF state is the agreed fault recovery point
        _faulted = false;
        _last_enable = 0;
        flush_scpc();
        return;
    }

    if (_faulted) {
        // fault latches until ENABLE is cycled off and back on
        _last_enable = 1;
        return;
    }

    if (!_active) {
        if (_last_enable == 1 && !config_tuple_changed()) {
            // config already reported invalid and unchanged: stay quiet
            return;
        }
        _last_enable = 1;
        remember_config_tuple();
        int32_t new_lat_e7 = 0;
        int32_t new_lng_e7 = 0;
        if (!validate_target(lat_hi.get(), lat_lo.get(),
                             lng_hi.get(), lng_lo.get(),
                             alt_cm.get(), new_lat_e7, new_lng_e7)) {
            queue_scpc(Event::INVALID_CONFIG, now_us, 0, 0, 0);
            flush_scpc();
            return;
        }
        open_epoch(position, origin, now_us, Event::ENABLE,
                   new_lat_e7, new_lng_e7, alt_cm.get());
        flush_scpc();
        return;
    }
    _last_enable = 1;

    // retarget or config change while active
    if (config_tuple_changed()) {
        remember_config_tuple();
        int32_t new_lat_e7 = 0;
        int32_t new_lng_e7 = 0;
        if (!validate_target(lat_hi.get(), lat_lo.get(),
                             lng_hi.get(), lng_lo.get(),
                             alt_cm.get(), new_lat_e7, new_lng_e7)) {
            close_epoch(Event::INVALID_CONFIG, now_us);
            flush_scpc();
            return;
        }
        flush_partial_row(now_us);
        open_epoch(position, origin, now_us, Event::RETARGET,
                   new_lat_e7, new_lng_e7, alt_cm.get());
        flush_scpc();
        return;
    }

    // an origin altitude change is unknown corruption (only lat/lng ever
    // rebase); fail the epoch closed
    if (origin.alt != _origin_alt) {
        fault(now_us);
        return;
    }
    // origin lat/lng changes are the expected 1 Hz rebase: re-anchor the
    // cached target projection; the raw position and the cache jump by the
    // same offset up to a sub-millimetre model difference, so the open
    // segment stays continuous
    if (origin.lat != _origin_lat || origin.lng != _origin_lng) {
        refresh_target_projection(origin);
    }
    if (now_us <= _prev_time_us) {
        fault(now_us);
        return;
    }
    const Vector3d cur_res = residual(position);
    if (!finite3(cur_res)) {
        fault(now_us);
        return;
    }

    // close every 20 ms interval this step crosses, splitting the segment
    // at each boundary with the same linear interpolation the minimum uses
    while (true) {
        const uint64_t boundary_us = _epoch_us + (uint64_t)(_seq + 1) * INTERVAL_US;
        if (now_us < boundary_us) {
            break;
        }
        const double span = (double)(now_us - _prev_time_us);
        const double f = (span > 0.0) ? (double)(boundary_us - _prev_time_us) / span : 0.0;
        const Vector3d mid = _prev_res + (cur_res - _prev_res) * f;
        accumulate_segment(_prev_res, mid, _prev_time_us, boundary_us);
        close_interval(boundary_us);
        _prev_res = mid;
        _prev_time_us = boundary_us;
    }
    if (now_us > _prev_time_us) {
        accumulate_segment(_prev_res, cur_res, _prev_time_us, now_us);
        _prev_res = cur_res;
        _prev_time_us = now_us;
    }
}

#endif  // AP_SIM_CPA_ENABLED
