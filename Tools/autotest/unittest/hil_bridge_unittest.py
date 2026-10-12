#!/usr/bin/env python3

'''
Unit tests of the HIL reference simulator bridge
(Tools/autotest/pysim/hil_bridge.py, link contract v2): board-clock
stepping with causal output timing, re-initialisation, retransmission
and GAP rules, unit conversions, the models, the reply time echo, replay
determinism and the rules that the physics path never reads the PC clock
and never draws a random number.

Run from the repository root:
  PYTHONPATH=Tools/autotest python3 Tools/autotest/unittest/hil_bridge_unittest.py

AP_FLAKE8_CLEAN
'''

import argparse
import math
import os
import random
import socket
import time
import unittest
from unittest import mock

from pymavlink.dialects.v20 import common as mavlink2

from pysim import hil_bridge as hb

BOARD_SYSID = 1
BOARD_COMPID = mavlink2.MAV_COMP_ID_AUTOPILOT1
LOOP_US = 5000                      # 200 Hz, the bridge's nominal loop period
T0_US = 1000000                     # board time of a first frame
CHANNELS = 16
# ArduPlane's default outputs at rest: aileron and elevator centred, throttle low
NEUTRAL = (1500.0, 1500.0, 1000.0, 1500.0) + (0.0,) * (CHANNELS - 4)
FULL_THROTTLE = (1500.0, 1500.0, 2000.0, 1500.0) + (0.0,) * (CHANNELS - 4)
NO_OUTPUT = (0.0,) * CHANNELS
# seed of the jitter in the recorded replay sequence (no unseeded randomness)
REPLAY_SEED = 20261012
# one 1e-7 degree is 1.1 cm; a round trip through degE7 stays within this
LATLON_ROUND_TRIP_M = 0.012
# bound on every wait for the threaded link tests
IO_TIMEOUT_S = 5.0


def controls_with(**channels):
    '''NEUTRAL with some channels (1-based, e.g. ch3=2000) replaced.'''
    values = list(NEUTRAL)
    for name, pwm in channels.items():
        values[int(name[2:]) - 1] = float(pwm)
    return tuple(values)


class RecordingModel(object):
    '''Records every step so the hold and step counts can be checked.'''
    name = "recording"

    def __init__(self):
        self.resets = 0
        self.steps = []

    def reset(self, origin):
        self.resets += 1

    def step(self, controls, dt_s):
        self.steps.append((tuple(controls), dt_s))

    def state(self):
        return hb.rest_state(0.0)


class FakeTransport(object):
    def __init__(self, fail_writes=False):
        self.written = bytearray()
        self.fail_writes = fail_writes
        self.closed = 0

    def open(self):
        pass

    def read(self):
        return b""

    def write(self, data):
        if self.fail_writes:
            raise OSError("port gone")
        self.written += data

    def close(self):
        self.closed += 1

    def shutdown(self):
        pass


class Board(object):
    '''Encodes what a board sends.'''

    def __init__(self, sysid=BOARD_SYSID):
        self.mav = mavlink2.MAVLink(None, srcSystem=sysid, srcComponent=BOARD_COMPID)

    def frame(self, time_usec, controls=NEUTRAL, mode=0, flags=0, mavlink1=False):
        msg = self.mav.hil_actuator_controls_encode(time_usec, list(controls), mode, flags)
        data = msg.pack(self.mav, force_mavlink1=mavlink1)
        self.mav.seq = (self.mav.seq + 1) % 256
        return data

    def heartbeat(self):
        msg = self.mav.heartbeat_encode(mavlink2.MAV_TYPE_FIXED_WING, mavlink2.MAV_AUTOPILOT_ARDUPILOTMEGA, 0, 0, 0)
        return msg.pack(self.mav)


def parse(data):
    parser = mavlink2.MAVLink(None)
    parser.robust_parsing = True
    return parser.parse_buffer(bytes(data)) or []


def make_bridge(model=None, **kwargs):
    transport = FakeTransport()
    said = []
    bridge = hb.HILBridge(transport, model or hb.StaticModel(), hb.DEFAULT_ORIGIN, say=said.append, **kwargs)
    assert bridge.connect()
    return bridge, transport, said


def recorded_sequence():
    '''A fixed board session: 200 Hz with loop jitter, throttle up, a turn
    and a pull-up, three lost frames, then a reboot and a short second boot.'''
    rng = random.Random(REPLAY_SEED)
    board = Board()
    frames = []
    t_us = T0_US
    for k in range(1600):
        if k < 100:
            controls = NEUTRAL
        elif k < 700:
            controls = controls_with(ch3=2000)
        elif k < 1100:
            controls = controls_with(ch1=1800, ch2=2000, ch3=2000)
        else:
            controls = controls_with(ch1=1300, ch3=1600)
        frames.append(board.frame(t_us, controls, mode=hb.ARMED_FLAG if k >= 100 else 0))
        t_us += LOOP_US + rng.randint(-300, 300)
        if k == 500:
            t_us += 3 * LOOP_US     # frames lost
    t_us = 2 * T0_US                # reboot: board time goes back
    for k in range(100):
        frames.append(board.frame(t_us, NEUTRAL))
        t_us += LOOP_US
    return frames


def replay(frames, chunk=None, clock=time.monotonic):
    bridge, transport, said = make_bridge(hb.KinematicPlane(), clock=clock)
    data = b"".join(frames)
    if chunk is None:
        for frame in frames:
            bridge.receive_bytes(frame)
    else:
        for i in range(0, len(data), chunk):
            bridge.receive_bytes(data[i:i + chunk])
    return bytes(transport.written), bridge


def raising_clock(*args, **kwargs):
    raise AssertionError("the physics path read the PC clock")


class StepperTest(unittest.TestCase):

    def stepper(self, model=None, **kwargs):
        return hb.BoardClockStepper(model or RecordingModel(), hb.DEFAULT_ORIGIN, **kwargs)

    def test_first_frame_initialises_without_stepping(self):
        model = RecordingModel()
        stepper = self.stepper(model)
        result = stepper.on_actuators(T0_US, NEUTRAL, False)
        self.assertEqual(result.event, hb.FIRST)
        self.assertEqual(result.steps, 0)
        self.assertEqual(stepper.t0_us, T0_US)
        self.assertEqual(stepper.plant_time_us, T0_US)
        self.assertEqual(model.steps, [])
        self.assertEqual(model.resets, 2)       # at construction and at t_0

    def test_plant_time_is_whole_steps_since_t0(self):
        stepper = self.stepper()
        stepper.on_actuators(T0_US, NEUTRAL, False)
        t_us = T0_US
        total = 0
        for jitter in (0, 1, 999, 1001, -250, 4321, 37):
            t_us += LOOP_US + jitter
            total += stepper.on_actuators(t_us, NEUTRAL, False).steps
            whole = (t_us - T0_US) // hb.PHYSICS_STEP_US
            self.assertEqual(stepper.steps_done, whole)
            self.assertEqual(stepper.plant_time_us, T0_US + whole * hb.PHYSICS_STEP_US)
            self.assertLessEqual(stepper.plant_time_us, t_us)
            self.assertLess(t_us - stepper.plant_time_us, hb.PHYSICS_STEP_US)
        self.assertEqual(total, stepper.steps_done)

    def test_zero_order_hold_uses_previous_outputs(self):
        model = RecordingModel()
        stepper = self.stepper(model)
        u0 = controls_with(ch1=1100)
        u1 = controls_with(ch1=1900)
        stepper.on_actuators(T0_US, u0, False)
        stepper.on_actuators(T0_US + LOOP_US, u1, False)
        stepper.on_actuators(T0_US + 2 * LOOP_US, NEUTRAL, False)
        steps_per_loop = LOOP_US // hb.PHYSICS_STEP_US
        self.assertEqual([c for c, _ in model.steps], [u0] * steps_per_loop + [u1] * steps_per_loop)
        self.assertTrue(all(dt == hb.PHYSICS_STEP_US / 1.0e6 for _, dt in model.steps))
        self.assertEqual(stepper.held, NEUTRAL)

    def test_time_going_back_reinitialises(self):
        model = RecordingModel()
        stepper = self.stepper(model)
        stepper.on_actuators(T0_US, NEUTRAL, False)
        stepper.on_actuators(T0_US + LOOP_US, NEUTRAL, False)
        result = stepper.on_actuators(T0_US // 2, NEUTRAL, False)
        self.assertEqual(result.event, hb.REINIT)
        self.assertEqual(result.steps, 0)
        self.assertEqual(stepper.t0_us, T0_US // 2)
        self.assertEqual(stepper.steps_done, 0)
        self.assertEqual(stepper.reinits, 1)
        self.assertEqual(model.resets, 3)

    def test_same_frame_again_is_a_retransmission(self):
        '''lockstep SITL resends a frame whose reply was lost with a
        connection: answered as before, nothing stepped, latched or reset'''
        model = RecordingModel()
        stepper = self.stepper(model)
        stepper.on_actuators(T0_US, NEUTRAL, True, hb.LOCKSTEP_FLAG)
        stepper.on_actuators(T0_US + LOOP_US, FULL_THROTTLE, True, hb.LOCKSTEP_FLAG)
        steps = list(model.steps)
        result = stepper.on_actuators(T0_US + LOOP_US, FULL_THROTTLE, True, hb.LOCKSTEP_FLAG)
        self.assertEqual(result.event, hb.RETRANSMIT)
        self.assertEqual(result.steps, 0)
        self.assertEqual((stepper.retransmits, stepper.conflicts, stepper.reinits), (1, 0, 0))
        self.assertEqual(model.steps, steps)
        self.assertEqual(model.resets, 2)
        self.assertEqual(stepper.plant_time_us, T0_US + LOOP_US)
        self.assertEqual(stepper.held, FULL_THROTTLE)
        # and the run goes on as if the frame had come once
        stepper.on_actuators(T0_US + 2 * LOOP_US, FULL_THROTTLE, True, hb.LOCKSTEP_FLAG)
        self.assertEqual(model.steps[len(steps):], [(FULL_THROTTLE, hb.PHYSICS_STEP_US / 1.0e6)] *
                         (LOOP_US // hb.PHYSICS_STEP_US))

    def test_same_time_with_other_content_is_a_conflict(self):
        '''board time strictly increases within one boot, so another frame
        at the same time breaks the contract: counted, nothing stepped,
        latched or reset'''
        for other in ((controls_with(ch1=1900), False, 0), (NEUTRAL, True, 0), (NEUTRAL, False, hb.LOCKSTEP_FLAG)):
            model = RecordingModel()
            stepper = self.stepper(model)
            stepper.on_actuators(T0_US, NEUTRAL, False, 0)
            stepper.on_actuators(T0_US + LOOP_US, NEUTRAL, False, 0)
            result = stepper.on_actuators(T0_US + LOOP_US, *other)
            self.assertEqual(result.event, hb.CONFLICT, other)
            self.assertEqual((stepper.conflicts, stepper.retransmits, stepper.reinits), (1, 0, 0))
            self.assertEqual(model.resets, 2)
            self.assertEqual(stepper.held, NEUTRAL)
            self.assertFalse(stepper.armed)
            self.assertEqual(stepper.plant_time_us, T0_US + LOOP_US)

    def test_outputs_take_effect_at_the_next_grid_point(self):
        '''causal zero-order hold between frames that fall between grid
        points: a step that straddles t_k still uses u_(k-1), and of two
        frames before the same grid point only the newer acts there'''
        model = RecordingModel()
        stepper = self.stepper(model, step_us=1000)
        u = [controls_with(ch1=1000 + 100 * k) for k in range(5)]
        stepper.on_actuators(T0_US, u[0], False)
        stepper.on_actuators(T0_US + 1500, u[1], False)     # acts from +2000
        stepper.on_actuators(T0_US + 3200, u[2], False)     # would act from +4000 ...
        stepper.on_actuators(T0_US + 4000, u[3], False)     # ... but u3 is newer there
        stepper.on_actuators(T0_US + 6000, u[4], False)
        self.assertEqual([c for c, _ in model.steps], [u[0], u[0], u[1], u[1], u[3], u[3]])
        self.assertEqual(stepper.held, u[4])

    def test_each_step_uses_the_newest_frame_at_or_before_its_start(self):
        '''the rule above over a long jittered session, frames faster and
        slower than the step'''
        rng = random.Random(REPLAY_SEED)
        model = RecordingModel()
        stepper = self.stepper(model, step_us=1000)
        frames = []
        t_us = T0_US
        for k in range(400):
            frames.append((t_us, controls_with(ch1=1000 + k)))
            t_us += rng.choice((1, 250, 999, 1000, 1001, 1500, 4321, 5000))
        for (t, controls) in frames:
            stepper.on_actuators(t, controls, False)
        want = []
        for n in range(len(model.steps)):
            start_us = T0_US + n * 1000
            want.append([c for (t, c) in frames if t <= start_us][-1])
        self.assertEqual([c for c, _ in model.steps], want)
        self.assertEqual(stepper.plant_time_us, T0_US + (frames[-1][0] - T0_US) // 1000 * 1000)

    def test_forward_jump_beyond_max_reinitialises(self):
        stepper = self.stepper()
        stepper.on_actuators(T0_US, NEUTRAL, False)
        at_max = stepper.on_actuators(T0_US + hb.MAX_FORWARD_JUMP_US, NEUTRAL, False)
        self.assertEqual(at_max.event, hb.GAP)
        beyond = stepper.on_actuators(T0_US + 2 * hb.MAX_FORWARD_JUMP_US + 1, NEUTRAL, False)
        self.assertEqual(beyond.event, hb.REINIT)
        self.assertEqual(stepper.t0_us, T0_US + 2 * hb.MAX_FORWARD_JUMP_US + 1)

    def test_gap_from_board_time_against_nominal_loop(self):
        stepper = self.stepper(loop_rate_hz=hb.DEFAULT_LOOP_RATE_HZ)
        threshold = int(hb.GAP_FACTOR * LOOP_US)
        stepper.on_actuators(T0_US, NEUTRAL, False)
        t_us = T0_US + threshold
        self.assertEqual(stepper.on_actuators(t_us, NEUTRAL, False).event, hb.STEP)
        t_us += threshold + 1
        gap = stepper.on_actuators(t_us, NEUTRAL, False)
        self.assertEqual(gap.event, hb.GAP)
        self.assertEqual(gap.lost_frames, 1)
        t_us += 4 * LOOP_US
        gap = stepper.on_actuators(t_us, NEUTRAL, False)
        self.assertEqual(gap.lost_frames, 3)
        self.assertEqual(stepper.gaps, 2)
        # stepped across with the held outputs, like any interval
        self.assertEqual(stepper.steps_done, (t_us - T0_US) // hb.PHYSICS_STEP_US)

    def test_loop_rate_sets_gap_threshold(self):
        stepper = self.stepper(loop_rate_hz=50)
        stepper.on_actuators(T0_US, NEUTRAL, False)
        self.assertEqual(stepper.on_actuators(T0_US + 20000, NEUTRAL, False).event, hb.STEP)

    def test_reboot_while_armed_is_counted(self):
        stepper = self.stepper()
        stepper.on_actuators(T0_US, NEUTRAL, True)
        result = stepper.on_actuators(T0_US // 2, NEUTRAL, False)
        self.assertTrue(result.was_armed)
        self.assertEqual(stepper.armed_reinits, 1)
        stepper.on_actuators(T0_US // 4, NEUTRAL, False)
        self.assertEqual(stepper.armed_reinits, 1)
        self.assertEqual(stepper.reinits, 2)

    def test_step_must_be_whole_microseconds(self):
        for bad in (0, -1000, 1000.5):
            with self.assertRaises(ValueError):
                self.stepper(step_us=bad)


class ConversionTest(unittest.TestCase):

    def test_quaternion_identity_and_heading(self):
        self.assertEqual(hb.euler_to_quaternion(0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
        w, x, y, z = hb.euler_to_quaternion(0.0, 0.0, math.pi / 2)
        self.assertAlmostEqual(w, math.sqrt(0.5))
        self.assertAlmostEqual(z, math.sqrt(0.5))
        self.assertEqual((x, y), (0.0, 0.0))

    def test_quaternion_and_ned_to_body_are_inverse(self):
        for roll, pitch, yaw in ((0.3, 0.2, 1.0), (-1.2, 0.7, 5.9), (0.0, -1.5, 3.1)):
            w, x, y, z = hb.euler_to_quaternion(roll, pitch, yaw)
            self.assertAlmostEqual(w * w + x * x + y * y + z * z, 1.0, places=12)
            # rotate body x by q (body to NED), then back with ned_to_body
            n = 1 - 2 * (y * y + z * z)
            e = 2 * (x * y + w * z)
            d = 2 * (x * z - w * y)
            back = hb.ned_to_body(n, e, d, roll, pitch, yaw)
            for got, want in zip(back, (1.0, 0.0, 0.0)):
                self.assertAlmostEqual(got, want, places=12)

    def test_ned_to_body_facing_east(self):
        x, y, z = hb.ned_to_body(1.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2)
        self.assertAlmostEqual(x, 0.0)
        self.assertAlmostEqual(y, -1.0)     # north is on the left
        self.assertAlmostEqual(z, 0.0)

    def test_body_rates_level(self):
        self.assertEqual(hb.euler_rates_to_body_rates(0.0, 0.0, 0.1, 0.2, 0.3), (0.1, 0.2, 0.3))

    def test_latlon_offset_matches_board_inverse(self):
        origin = hb.DEFAULT_ORIGIN
        for north, east in ((100.0, 0.0), (0.0, 100.0), (-2500.0, 1234.5)):
            lat, lon = hb.offset_latlon_e7(origin.lat_e7, origin.lon_e7, north, east)
            # Location::get_distance_NE_double, the board's inverse
            back_n = (lat - origin.lat_e7) * hb.LATLON_TO_M
            back_e = (lon - origin.lon_e7) * hb.LATLON_TO_M * hb.longitude_scale((lat + origin.lat_e7) // 2)
            self.assertLess(abs(back_n - north), LATLON_ROUND_TRIP_M)
            self.assertLess(abs(back_e - east), LATLON_ROUND_TRIP_M)
        self.assertEqual(hb.offset_latlon_e7(origin.lat_e7, origin.lon_e7, 100.0, 0.0)[0] - origin.lat_e7, 8983)

    def test_longitude_wraps(self):
        lat, lon = hb.offset_latlon_e7(0, 1799999990, 0.0, 100.0)
        self.assertLess(lon, 0)
        self.assertLessEqual(abs(lon), 180 * 10000000)
        self.assertEqual(hb.wrap_longitude_e7(-1800000001), 1799999999)

    def test_eas2tas_isa(self):
        self.assertAlmostEqual(hb.eas2tas(0.0), 1.0, delta=1e-3)
        # ISA density ratio 0.9075 at 1000 m: 1/sqrt(0.9075) = 1.0497
        self.assertAlmostEqual(hb.eas2tas(1000.0), 1.0497, delta=2e-3)
        self.assertGreater(hb.eas2tas(2000.0), hb.eas2tas(1000.0))
        self.assertEqual(hb.eas2tas(20000.0), hb.eas2tas(hb.TROPOPAUSE_ALT_M))

    def test_milli_g_and_clamps(self):
        self.assertEqual(hb.milli_g(-hb.GRAVITY_MSS), -1000)
        self.assertEqual(hb.milli_g(1.0e6), hb.INT16_MAX)
        self.assertEqual(hb.milli_g(-1.0e6), hb.INT16_MIN)
        self.assertEqual(hb.clamp_int(-5.0, 0, hb.UINT16_MAX), 0)
        self.assertEqual(hb.clamp_int(1.0e9, 0, hb.UINT16_MAX), hb.UINT16_MAX)

    def test_pwm_normalisation(self):
        self.assertEqual(hb.angle_from_pwm(1000.0), -1.0)
        self.assertEqual(hb.angle_from_pwm(1500.0), 0.0)
        self.assertEqual(hb.angle_from_pwm(2000.0), 1.0)
        self.assertEqual(hb.angle_from_pwm(2500.0), 1.0)
        self.assertEqual(hb.angle_from_pwm(hb.NO_OUTPUT_PWM_US), 0.0)
        self.assertEqual(hb.range_from_pwm(1000.0), 0.0)
        self.assertEqual(hb.range_from_pwm(1250.0), 0.25)
        self.assertEqual(hb.range_from_pwm(2000.0), 1.0)
        self.assertEqual(hb.range_from_pwm(900.0), 0.0)
        self.assertEqual(hb.range_from_pwm(hb.NO_OUTPUT_PWM_US), 0.0)

    def test_rest_state_fields(self):
        origin = hb.DEFAULT_ORIGIN
        fields = hb.state_fields(origin, hb.rest_state(origin.heading_deg))
        self.assertEqual((fields["lat"], fields["lon"]), (origin.lat_e7, origin.lon_e7))
        self.assertEqual(fields["alt"], 584000)
        self.assertEqual((fields["xacc"], fields["yacc"], fields["zacc"]), (0, 0, -1000))
        self.assertEqual((fields["vx"], fields["vy"], fields["vz"]), (0, 0, 0))
        self.assertEqual((fields["ind_airspeed"], fields["true_airspeed"]), (0, 0))
        w, x, y, z = fields["attitude_quaternion"]
        self.assertAlmostEqual(math.degrees(2 * math.atan2(z, w)) % 360.0, origin.heading_deg)
        self.assertEqual(set(fields) | {"time_usec"}, set(hb.STATE_FIELDS))

    def test_state_fields_units(self):
        origin = hb.Origin(0.0, 0.0, 1000.0, 0.0)
        state = hb.VehicleState(0.0, 0.0, -10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                12.34, -5.0, 400.0, 30.0, 0.0, 4.903325, -hb.GRAVITY_MSS)
        fields = hb.state_fields(origin, state)
        self.assertEqual(fields["alt"], 1010000)
        self.assertEqual((fields["vx"], fields["vy"], fields["vz"]), (1234, -500, hb.INT16_MAX))
        self.assertEqual(fields["true_airspeed"], 3000)
        self.assertEqual(fields["ind_airspeed"], round(3000 / hb.eas2tas(1010.0)))
        self.assertEqual((fields["yacc"], fields["zacc"]), (500, -1000))


class ModelTest(unittest.TestCase):
    STEP_S = hb.PHYSICS_STEP_US / 1.0e6

    def run_model(self, model, controls, seconds):
        for _ in range(int(round(seconds / self.STEP_S))):
            model.step(controls, self.STEP_S)
        return model.state()

    def plane(self):
        model = hb.KinematicPlane()
        model.reset(hb.DEFAULT_ORIGIN)
        return model

    def test_static_model_ignores_outputs(self):
        model = hb.StaticModel()
        model.reset(hb.DEFAULT_ORIGIN)
        self.assertEqual(self.run_model(model, FULL_THROTTLE, 2.0), hb.rest_state(hb.DEFAULT_ORIGIN.heading_deg))

    def test_plane_at_rest_without_outputs(self):
        model = self.plane()
        for controls in (NO_OUTPUT, NEUTRAL):
            state = self.run_model(model, controls, 1.0)
            self.assertEqual((state.north_m, state.east_m, state.down_m, state.tas_m_s), (0.0, 0.0, 0.0, 0.0))
            self.assertEqual((state.fx_m_s2, state.fz_m_s2), (0.0, -hb.GRAVITY_MSS))

    def test_throttle_rolls_along_the_heading(self):
        model = self.plane()
        state = self.run_model(model, FULL_THROTTLE, 1.0)
        self.assertGreater(state.tas_m_s, 5.0)
        self.assertLess(state.tas_m_s, hb.LIFTOFF_AIRSPEED_M_S)
        self.assertEqual(state.down_m, 0.0)
        self.assertEqual((state.roll_rad, state.pitch_rad), (0.0, 0.0))
        heading = math.atan2(state.east_m, state.north_m) % (2 * math.pi)
        self.assertAlmostEqual(heading, math.radians(hb.DEFAULT_ORIGIN.heading_deg), places=9)
        self.assertGreater(state.fx_m_s2, 0.0)  # accelerating forward

    def test_elevator_climbs_after_liftoff(self):
        model = self.plane()
        self.run_model(model, FULL_THROTTLE, 5.0)
        state = self.run_model(model, controls_with(ch2=2000, ch3=2000), 5.0)
        self.assertLess(state.down_m, -10.0)
        self.assertAlmostEqual(state.pitch_rad, hb.MAX_PITCH_RAD, delta=0.01)
        self.assertLess(state.vd_m_s, 0.0)

    def test_aileron_banks_and_turns_right(self):
        model = self.plane()
        self.run_model(model, FULL_THROTTLE, 5.0)
        self.run_model(model, controls_with(ch2=1700, ch3=2000), 2.0)
        start = model.state()
        state = self.run_model(model, controls_with(ch1=2000, ch3=2000), 5.0)
        self.assertAlmostEqual(state.roll_rad, hb.MAX_BANK_RAD, delta=0.01)
        turned = (state.yaw_rad - start.yaw_rad) % (2 * math.pi)
        self.assertGreater(turned, math.radians(30.0))
        self.assertGreater(state.r_rad_s, 0.0)

    def test_steady_turn_is_coordinated(self):
        model = self.plane()
        self.run_model(model, FULL_THROTTLE, 5.0)
        self.run_model(model, controls_with(ch2=1700, ch3=2000), 2.0)
        state = self.run_model(model, controls_with(ch1=1750, ch3=2000), 20.0)
        bank = state.roll_rad
        self.assertAlmostEqual(state.pitch_rad, 0.0, places=6)
        self.assertAlmostEqual(state.fy_m_s2, 0.0, delta=0.05)
        self.assertAlmostEqual(state.fz_m_s2, -hb.GRAVITY_MSS / math.cos(bank), delta=0.05)
        yaw_rate = hb.GRAVITY_MSS * math.tan(bank) / state.tas_m_s
        self.assertAlmostEqual(state.q_rad_s, yaw_rate * math.sin(bank), delta=1e-4)
        self.assertAlmostEqual(state.r_rad_s, yaw_rate * math.cos(bank), delta=1e-4)

    def test_cannot_sink_below_ground(self):
        model = self.plane()
        self.run_model(model, FULL_THROTTLE, 5.0)
        self.run_model(model, controls_with(ch2=2000, ch3=2000), 2.0)
        state = self.run_model(model, controls_with(ch2=1000, ch3=2000), 20.0)
        self.assertEqual(state.down_m, 0.0)
        self.assertEqual(state.vd_m_s, 0.0)

    def test_make_model(self):
        self.assertIsInstance(hb.make_model("static"), hb.StaticModel)
        self.assertIsInstance(hb.make_model("kinematic"), hb.KinematicPlane)
        with self.assertRaises(ValueError):
            hb.make_model("jsbsim")


class BridgeTest(unittest.TestCase):

    def test_every_frame_answered_with_its_time(self):
        bridge, transport, _ = make_bridge()
        board = Board()
        times = [T0_US + k * LOOP_US + (k % 3) * 7 for k in range(20)]
        for t_us in times:
            bridge.receive_bytes(board.frame(t_us))
        replies = parse(transport.written)
        self.assertEqual([m.get_type() for m in replies], ["HIL_STATE_QUATERNION"] * len(times))
        self.assertEqual([m.time_usec for m in replies], times)
        for msg in replies:
            self.assertEqual(msg.get_msgbuf()[0], mavlink2.PROTOCOL_MARKER_V2)
            self.assertEqual(msg.get_srcSystem(), hb.DEFAULT_SYSID)
            self.assertEqual(msg.get_srcComponent(), hb.BRIDGE_COMPID)
        self.assertEqual((bridge.frames, bridge.stats.replies), (len(times), len(times)))

    def test_reply_is_the_state_at_the_frame_time(self):
        bridge, transport, _ = make_bridge(hb.KinematicPlane())
        board = Board()
        for k in range(400):
            bridge.receive_bytes(board.frame(T0_US + k * LOOP_US, FULL_THROTTLE))
        last = parse(transport.written)[-1]
        want = hb.state_fields(hb.DEFAULT_ORIGIN, bridge.stepper.state())
        self.assertEqual(last.true_airspeed, want["true_airspeed"])
        self.assertGreater(last.true_airspeed, 0)
        self.assertEqual((last.lat, last.lon, last.alt), (want["lat"], want["lon"], want["alt"]))
        self.assertEqual(bridge.stepper.plant_time_us, T0_US + 399 * LOOP_US)

    def test_ignores_mavlink1_loopback_other_and_garbage(self):
        bridge, transport, _ = make_bridge()
        bridge.receive_bytes(Board().frame(T0_US, mavlink1=True))
        bridge.receive_bytes(Board(sysid=hb.DEFAULT_SYSID).frame(T0_US))
        bridge.receive_bytes(Board().heartbeat())
        garbage = bytearray(Board().frame(T0_US))
        garbage[20] ^= 0xFF                     # breaks the CRC
        bridge.receive_bytes(bytes(garbage))
        self.assertEqual(bytes(transport.written), b"")
        s = bridge.stats
        self.assertEqual((s.mavlink1, s.loopback, s.other_msgs, s.frames), (1, 1, 1, 0))
        self.assertGreater(s.bad_data, 0)
        bridge.receive_bytes(Board().frame(T0_US))
        self.assertEqual(len(parse(transport.written)), 1)

    def test_frames_split_across_reads(self):
        board = Board()
        data = b"".join(board.frame(T0_US + k * LOOP_US) for k in range(5))
        bridge, transport, _ = make_bridge()
        for i in range(len(data)):
            bridge.receive_bytes(data[i:i + 1])
        self.assertEqual([m.time_usec for m in parse(transport.written)], [T0_US + k * LOOP_US for k in range(5)])

    def test_hold_replies_keeps_stepping(self):
        bridge, transport, _ = make_bridge(hb.KinematicPlane())
        board = Board()
        bridge.receive_bytes(board.frame(T0_US, FULL_THROTTLE))
        bridge.hold_replies(True)
        for k in range(1, 101):
            bridge.receive_bytes(board.frame(T0_US + k * LOOP_US, FULL_THROTTLE))
        self.assertEqual(len(parse(transport.written)), 1)
        self.assertEqual(bridge.stats.held_replies, 100)
        self.assertEqual(bridge.stepper.plant_time_us, T0_US + 100 * LOOP_US)
        bridge.hold_replies(False)
        bridge.receive_bytes(board.frame(T0_US + 101 * LOOP_US, FULL_THROTTLE))
        replies = parse(transport.written)
        self.assertEqual(len(replies), 2)
        self.assertGreater(replies[-1].true_airspeed, replies[0].true_airspeed)

    def test_last_controls_mode_and_flags(self):
        bridge, _, said = make_bridge()
        controls = controls_with(ch1=1234, ch3=1600)
        bridge.receive_bytes(Board().frame(T0_US, controls, mode=hb.ARMED_FLAG, flags=hb.LOCKSTEP_FLAG))
        bridge.receive_bytes(Board().frame(T0_US + LOOP_US, controls, mode=hb.ARMED_FLAG, flags=hb.LOCKSTEP_FLAG))
        self.assertEqual(bridge.last_controls, controls)
        self.assertEqual(bridge.last_mode, hb.ARMED_FLAG)
        self.assertEqual(bridge.last_flags, hb.LOCKSTEP_FLAG)
        self.assertEqual(bridge.last_time_usec, T0_US + LOOP_US)
        self.assertEqual(sum("lockstep" in line for line in said), 1)

    def test_reboot_while_armed_is_reported_loudly(self):
        bridge, _, said = make_bridge()
        board = Board()
        bridge.receive_bytes(board.frame(T0_US, mode=hb.ARMED_FLAG))
        bridge.receive_bytes(board.frame(T0_US // 2))
        self.assertTrue(any("ALERT" in line and "ARMED" in line for line in said))
        del said[:]
        bridge.receive_bytes(board.frame(T0_US // 4))
        self.assertFalse(any("ALERT" in line for line in said))
        self.assertTrue(any("reboot" in line for line in said))

    def test_gap_is_reported(self):
        bridge, _, said = make_bridge()
        board = Board()
        bridge.receive_bytes(board.frame(T0_US))
        bridge.receive_bytes(board.frame(T0_US + 4 * LOOP_US))
        self.assertTrue(any("GAP" in line and "3 frames lost" in line for line in said))

    def test_send_state_overrides(self):
        bridge, transport, _ = make_bridge()
        bridge.receive_bytes(Board().frame(T0_US))
        self.assertTrue(bridge.send_state(time_usec=0, attitude_quaternion=[2.0, 0.0, 0.0, 0.0]))
        self.assertTrue(bridge.send_state(lat=950000000))
        replies = parse(transport.written)
        self.assertEqual(replies[1].time_usec, 0)
        self.assertEqual(list(replies[1].attitude_quaternion), [2.0, 0.0, 0.0, 0.0])
        self.assertEqual(replies[2].time_usec, T0_US)
        self.assertEqual(replies[2].lat, 950000000)
        with self.assertRaises(ValueError):
            bridge.send_state(latitude=1)

    def test_send_raw(self):
        bridge, transport, _ = make_bridge()
        self.assertTrue(bridge.send_raw(b"\x00\xfdjunk"))
        self.assertEqual(bytes(transport.written), b"\x00\xfdjunk")

    def test_write_failure_requests_reconnect(self):
        bridge, transport, _ = make_bridge()
        transport.fail_writes = True
        bridge.receive_bytes(Board().frame(T0_US))
        self.assertEqual(bridge.stats.replies, 0)
        self.assertEqual(bridge.stats.write_failures, 1)
        self.assertTrue(bridge._drop_request.is_set())

    def test_lost_reply_and_reconnect_give_the_uninterrupted_run(self):
        '''lockstep: the bridge handles frame k but its reply is lost with
        the connection, so SITL resends frame k to the reconnected bridge.
        The flying, armed model must carry on, not restart at the origin.'''
        board = Board()
        frames = [board.frame(T0_US + k * LOOP_US, NEUTRAL if k < 10 else FULL_THROTTLE,
                              mode=hb.ARMED_FLAG, flags=hb.LOCKSTEP_FLAG) for k in range(800)]
        lost = 600
        reference, _ = replay(frames)

        bridge, transport, said = make_bridge(hb.KinematicPlane())
        for frame in frames[:lost]:
            bridge.receive_bytes(frame)
        transport.fail_writes = True
        bridge.receive_bytes(frames[lost])          # stepped, but the reply is lost
        self.assertTrue(bridge._drop_request.is_set())
        self.assertGreater(bridge.stepper.state().tas_m_s, hb.LIFTOFF_AIRSPEED_M_S)
        bridge._close("connection lost")
        transport.fail_writes = False
        self.assertTrue(bridge.connect())
        for frame in frames[lost:]:                 # frame k again, then the rest
            bridge.receive_bytes(frame)

        def fields(data):
            '''the replies without their MAVLink sequence numbers'''
            return [m.to_dict() for m in parse(data)]
        got = fields(transport.written)
        self.assertEqual(len(got), len(frames))
        self.assertEqual(got, fields(reference))
        self.assertEqual((bridge.stepper.retransmits, bridge.stepper.reinits, bridge.stepper.armed_reinits), (1, 0, 0))
        self.assertEqual(bridge.last_event, hb.STEP)
        self.assertTrue(any("again" in line for line in said))
        self.assertFalse(any("ALERT" in line for line in said))

    def test_conflicting_frame_is_reported_and_answered(self):
        bridge, transport, said = make_bridge(hb.KinematicPlane())
        board = Board()
        bridge.receive_bytes(board.frame(T0_US, FULL_THROTTLE, mode=hb.ARMED_FLAG))
        bridge.receive_bytes(board.frame(T0_US + LOOP_US, FULL_THROTTLE, mode=hb.ARMED_FLAG))
        bridge.receive_bytes(board.frame(T0_US + LOOP_US, NEUTRAL, mode=hb.ARMED_FLAG))
        replies = parse(transport.written)
        self.assertEqual([m.time_usec for m in replies], [T0_US, T0_US + LOOP_US, T0_US + LOOP_US])
        self.assertEqual(replies[1].to_dict(), replies[2].to_dict())
        self.assertEqual((bridge.stepper.conflicts, bridge.stepper.reinits), (1, 0))
        self.assertEqual(bridge.last_event, hb.CONFLICT)
        self.assertTrue(any("ERROR" in line and "contract" in line for line in said))

    def test_drop_connection_without_thread(self):
        bridge, transport, said = make_bridge()
        self.assertTrue(bridge.drop_connection())
        self.assertEqual(transport.closed, 1)
        self.assertEqual(bridge.stats.disconnects, 1)
        self.assertFalse(bridge.send_raw(b"x"))


class DeterminismTest(unittest.TestCase):

    def test_replay_is_bit_identical(self):
        frames = recorded_sequence()
        first, bridge = replay(frames)
        second, _ = replay(frames)
        self.assertEqual(first, second)
        # the sequence exercised what it was meant to
        self.assertEqual(bridge.stepper.reinits, 1)
        self.assertEqual(bridge.stepper.armed_reinits, 1)
        self.assertGreaterEqual(bridge.stepper.gaps, 1)
        replies = parse(first)
        self.assertEqual(len(replies), len(frames))
        # it flew: climbed more than 5 m and turned
        self.assertTrue(any(m.alt > 584000 + 5000 for m in replies))
        self.assertTrue(any(abs(m.yawspeed) > 0.1 for m in replies))

    def test_replay_does_not_depend_on_chunking(self):
        frames = recorded_sequence()
        whole, _ = replay(frames)
        for chunk in (1, 7, 4096):
            self.assertEqual(replay(frames, chunk=chunk)[0], whole)

    def test_physics_path_never_reads_the_pc_clock(self):
        frames = recorded_sequence()
        reference, _ = replay(frames)
        bridge, transport, _ = make_bridge(hb.KinematicPlane(), clock=raising_clock)
        clocks = ("time", "time_ns", "monotonic", "monotonic_ns", "perf_counter", "perf_counter_ns",
                  "process_time", "process_time_ns")
        patches = [mock.patch.object(time, name, side_effect=raising_clock) for name in clocks]
        for patch in patches:
            patch.start()
        try:
            for frame in frames:
                bridge.receive_bytes(frame)
            bridge.send_state()
        finally:
            for patch in patches:
                patch.stop()
        self.assertEqual(bytes(transport.written)[:len(reference)], reference)

    def test_physics_path_draws_no_random_numbers(self):
        '''every random source raises while a session is replayed (the
        replay tests above show that the states repeat)'''
        frames = recorded_sequence()        # seeded jitter, drawn before the patches
        reference, _ = replay(frames)

        def no_random(*args, **kwargs):
            raise AssertionError("the physics path drew a random number")
        module_functions = ("random", "randint", "randrange", "uniform", "gauss", "normalvariate",
                            "choice", "choices", "shuffle", "sample", "getrandbits", "seed")
        patches = [mock.patch.object(random, name, no_random) for name in module_functions]
        # new generators: every draw of random.Random and SystemRandom goes through these
        patches += [mock.patch.object(cls, name, no_random)
                    for cls in (random.Random, random.SystemRandom) for name in ("random", "getrandbits")]
        patches += [mock.patch.object(os, "urandom", no_random)]
        for patch in patches:
            patch.start()
        try:
            got, _ = replay(frames)
        finally:
            for patch in patches:
                patch.stop()
        self.assertEqual(got, reference)


class CommandLineTest(unittest.TestCase):

    def test_parse_origin(self):
        self.assertEqual(hb.parse_origin("-35.363261,149.165230,584,353"), hb.DEFAULT_ORIGIN)
        for bad in ("1,2,3", "a,b,c,d", "91,0,0,0", "0,181,0,0", "0,0,0,360", "0,0,nan,0"):
            with self.assertRaises(argparse.ArgumentTypeError):
                hb.parse_origin(bad)

    def test_defaults(self):
        args = hb.parse_args(["--device", "COM11"])
        self.assertEqual((args.sysid, args.model, args.origin), (251, "static", hb.DEFAULT_ORIGIN))
        self.assertEqual(args.loop_rate_hz, hb.DEFAULT_LOOP_RATE_HZ)
        args = hb.parse_args(["--device", "tcp:127.0.0.1:5765", "--model", "kinematic", "--origin", "1,2,3,4"])
        self.assertEqual((args.model, args.origin), ("kinematic", hb.Origin(1.0, 2.0, 3.0, 4.0)))

    def test_make_transport(self):
        tcp = hb.make_transport("tcp:127.0.0.1:5765")
        self.assertIsInstance(tcp, hb.TcpTransport)
        self.assertEqual((tcp.host, tcp.port), ("127.0.0.1", 5765))
        with self.assertRaises(ValueError):
            hb.make_transport("tcp:5765")
        with self.assertRaises(ValueError):
            hb.make_transport("tcpin:x")
        for device in ("COM11", "/dev/ttyACM1"):
            serial = hb.make_transport(device, 57600)
            self.assertIsInstance(serial, hb.SerialTransport)
            self.assertEqual((serial.port, serial.baud), (device, 57600))
        listener = hb.make_transport("tcpin:0")
        try:
            self.assertEqual(listener.host, hb.LISTEN_HOST_DEFAULT)
            self.assertGreater(listener.port, 0)
        finally:
            listener.shutdown()


class ThreadedLinkTest(unittest.TestCase):
    '''The reader thread over real loopback sockets, with bounded waits.'''

    def setUp(self):
        patch = mock.patch.object(hb, "RECONNECT_INTERVAL_S", 0.05)
        patch.start()
        self.addCleanup(patch.stop)

    def exchange(self, sock, board, times):
        '''Send frames, return the replies' time_usec.'''
        sock.sendall(b"".join(board.frame(t) for t in times))
        parser = mavlink2.MAVLink(None)
        got = []
        deadline = time.monotonic() + IO_TIMEOUT_S
        while len(got) < len(times) and time.monotonic() < deadline:
            data = sock.recv(4096)
            if not data:
                break
            got += [m.time_usec for m in parser.parse_buffer(data) or []]
        return got

    def test_listener_drop_and_reconnect(self):
        bridge = hb.HILBridge("tcpin:127.0.0.1:0", hb.StaticModel(), say=lambda text: None)
        board = Board()
        with bridge:
            sock = socket.create_connection(("127.0.0.1", bridge.transport.port), timeout=IO_TIMEOUT_S)
            times = [T0_US + k * LOOP_US for k in range(10)]
            self.assertEqual(self.exchange(sock, board, times), times)
            self.assertTrue(bridge.drop_connection())
            self.assertEqual(sock.recv(4096), b"")      # the bridge closed it
            sock.close()
            sock = socket.create_connection(("127.0.0.1", bridge.transport.port), timeout=IO_TIMEOUT_S)
            times = [T0_US + k * LOOP_US for k in range(10, 20)]
            self.assertEqual(self.exchange(sock, board, times), times)
            sock.close()
        self.assertEqual(bridge.stats.connects, 2)
        self.assertEqual(bridge.frames, 20)
        self.assertIsNone(bridge.failure)

    def test_client_reconnects_after_server_restart(self):
        '''SITL listens on --serialN=tcp:<port> and drops the client when it
        restarts; the bridge connects again.'''
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind(("127.0.0.1", 0))
        server.listen(1)
        server.settimeout(IO_TIMEOUT_S)
        port = server.getsockname()[1]
        bridge = hb.HILBridge("tcp:127.0.0.1:%u" % port, hb.StaticModel(), say=lambda text: None)
        board = Board()
        try:
            with bridge:
                conn, _ = server.accept()
                conn.settimeout(IO_TIMEOUT_S)
                times = [T0_US + k * LOOP_US for k in range(5)]
                self.assertEqual(self.exchange(conn, board, times), times)
                conn.close()                            # "SITL restarts"
                conn, _ = server.accept()
                conn.settimeout(IO_TIMEOUT_S)
                times = [T0_US // 2 + k * LOOP_US for k in range(5)]
                self.assertEqual(self.exchange(conn, board, times), times)
                conn.close()
        finally:
            server.close()
        self.assertEqual(bridge.stats.connects, 2)
        self.assertEqual(bridge.stepper.reinits, 1)


if __name__ == '__main__':
    unittest.main()
