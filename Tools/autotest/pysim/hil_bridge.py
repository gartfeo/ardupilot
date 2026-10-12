#!/usr/bin/env python3

'''
Reference simulator bridge for ArduPilot's HIL simulator backend
(libraries/SITL/SIM_HIL.cpp), link contract v2
(libraries/SITL/examples/HIL/readme.md).

The flight controller (a board running HIL firmware, or SITL with
--model hil) sends HIL_ACTUATOR_CONTROLS once per main loop on a serial
port dedicated to the HIL link. This bridge answers every frame with one
HIL_STATE_QUATERNION carrying the same time_usec.

Rule zero: physics time follows only the board clock.
- The plant time is a whole number of fixed integration steps
  (PHYSICS_STEP_US) since the board time t_0 of the first frame: the
  plant moves on the grid t_0 + n x PHYSICS_STEP_US.
- On frame k (board time t_k, outputs u_k) the plant advances to the last
  grid point at or before t_k, and the reply is the state there. Each
  step uses the outputs in effect at its start (zero-order hold, causal):
  u_k takes effect at the first grid point at or after t_k, so a step
  that straddles t_k still uses u_(k-1).
- Board time going back, or jumping forward by more than
  MAX_FORWARD_JUMP_US, is a new boot: the plant is re-initialised at the
  origin (reported loudly when the last frame had the armed bit).
- The same frame again (same board time, outputs, mode and flags) is a
  retransmission, e.g. a lockstep SITL resending after a reconnect: it is
  answered with the same state and nothing is stepped. The same board
  time with other content breaks the contract: it is counted and
  reported, answered with the state at that time, and nothing is stepped
  or latched.
- A forward jump longer than GAP_FACTOR nominal loop periods means frames
  were lost: it is stepped across with the held outputs and reported as
  a GAP. No frame counter is used.
- The PC clock never enters the physics path (it is used only to pace
  reconnects and status lines), and nothing is random. The same frames in
  give bit-identical states out.

Models:
  static      level at the origin, origin heading, at rest; ignores outputs
  kinematic   a toy plane: throttle -> airspeed, aileron -> bank -> turn
              rate, elevator -> pitch -> climb, on ArduPlane's default
              outputs (SERVO1 aileron, SERVO2 elevator, SERVO3 throttle)

Devices:
  COM11, /dev/ttyACM1   serial port (needs pyserial)
  tcp:HOST:PORT         TCP client; SITL --serialN=tcp:<port> listens
  tcpin:PORT            TCP listener on 127.0.0.1:PORT, one client at a time
  tcpin:HOST:PORT       TCP listener on HOST

Example, against SITL started with Tools/autotest/sim_vehicle.py
-v ArduPlane -f hil (or hil:lockstep), whose SERIAL5 (SERIAL5_PROTOCOL
100, SIM HIL) listens on TCP port 5765:
  python3 Tools/autotest/pysim/hil_bridge.py --device tcp:127.0.0.1:5765 \\
      --model kinematic --origin -35.363261,149.165230,584,353

Only pymavlink is needed (plus pyserial for serial ports). The MAVLink 2
codec is pymavlink.dialects.v20.common, imported directly: it speaks
MAVLink 2 whatever MAVLINK20 says and whatever mavutil a host process
(an autotest) has already imported. The byte transport is the small one
below rather than mavutil's, because mavutil's blocking receive polls
with a 50 ms sleep on ports without a file descriptor (pyserial on
Windows), too slow for a 200 Hz link.

AP_FLAKE8_CLEAN
'''

import argparse
import math
import os
import select
import socket
import sys
import threading
import time
import traceback
from dataclasses import dataclass

from pymavlink.dialects.v20 import common as mavlink2

# --- link ---------------------------------------------------------------
# bridge system id by convention (contract v2); the board does not enforce it
DEFAULT_SYSID = 251
# there is no simulator component id; an onboard computer is the closest
BRIDGE_COMPID = mavlink2.MAV_COMP_ID_ONBOARD_COMPUTER
# HIL_ACTUATOR_CONTROLS.mode bit set while the vehicle is armed
ARMED_FLAG = mavlink2.MAV_MODE_FLAG_SAFETY_ARMED
# HIL_ACTUATOR_CONTROLS.flags bit 0: the board runs in lockstep and waits
# for every reply (SITL only). common.xml documents the bit in the field
# text ("1: indicate simulation using lockstep"); the fork's MAVLink has
# no named enum for it.
LOCKSTEP_FLAG = 1
ACTUATORS_TYPE = "HIL_ACTUATOR_CONTROLS"
BAD_DATA_TYPE = "BAD_DATA"          # pymavlink's name for bytes that are no valid frame
STATE_FIELDS = tuple(mavlink2.MAVLink_hil_state_quaternion_message.fieldnames)

# --- board clock stepping -----------------------------------------------
US_PER_S = 1000000
# fixed integration step: the state answering frame k is the state at the
# last whole step at or before t_k, so it is at most one step early, and
# the outputs of frame k act from the first whole step at or after t_k,
# so at most one step late
PHYSICS_STEP_US = 1000
# the board's SCHED_LOOP_RATE, by default the 200 Hz the HIL link is sized
# for (default_params/plane-hil.parm and rover-hil.parm set it; the
# vehicles' own defaults are 50 Hz for Plane and Rover, 400 Hz for
# Copter). Only the GAP report depends on it: --loop-rate-hz
DEFAULT_LOOP_RATE_HZ = 200
# one lost frame doubles the interval between frames; 1.5 nominal periods
# leaves half a period for loop jitter before an interval counts as a GAP
GAP_FACTOR = 1.5
# a running board never goes this long between frames; a longer forward
# jump is treated as a new boot rather than stepped across with outputs
# held for that long (the board reports its link LOST long before)
MAX_FORWARD_JUMP_US = 2000000

FIRST = "first"
STEP = "step"
GAP = "gap"
REINIT = "reinit"
# the same board time as the previous frame: the same frame again, or a
# frame with other content (a contract violation); neither is stepped
RETRANSMIT = "retransmit"
CONFLICT = "conflict"

# --- units --------------------------------------------------------------
# ArduPilot's GRAVITY_MSS (libraries/AP_Math/definitions.h); the board turns
# mG back into m/s/s with the same value
GRAVITY_MSS = 9.80665
MILLI_G_PER_G = 1000.0
CM_PER_M = 100.0
MM_PER_M = 1000.0
DEG_E7 = 1.0e7
# metres per 1e-7 degree of latitude and its inverse, ArduPilot's
# LATLON_TO_M and LATLON_TO_M_INV (libraries/AP_Math/definitions.h)
LATLON_TO_M = 0.011131884502145034
LATLON_TO_M_INV = 89.83204953368922
# Location::longitude_scale never goes below this (libraries/AP_Common/Location.cpp)
MIN_LONGITUDE_SCALE = 0.01
LON_HALF_TURN_E7 = 1800000000
LON_FULL_TURN_E7 = 3600000000
# HIL_STATE_QUATERNION integer field ranges: values outside are clamped,
# never wrapped
INT16_MIN = -32768
INT16_MAX = 32767
UINT16_MAX = 65535

# ISA troposphere as ArduPilot's AP_Baro (libraries/AP_Math/definitions.h)
SSL_AIR_DENSITY = 1.225             # kg/m^3
SSL_AIR_PRESSURE = 101325.01576     # Pa
SSL_AIR_TEMPERATURE = 288.15        # K
ISA_LAPSE_RATE = 0.0065             # K/m
ISA_GAS_CONSTANT = 287.26           # J/(kg K)
# the reference models fly in the troposphere only; higher is clamped
TROPOPAUSE_ALT_M = 11000.0

# --- servo outputs ------------------------------------------------------
# ArduPilot's simulators normalise PWM over 1000..2000 us
# (libraries/SITL/ServoModel.h)
PWM_MIN_US = 1000.0
PWM_MAX_US = 2000.0
PWM_CENTRE_US = 0.5 * (PWM_MIN_US + PWM_MAX_US)
PWM_HALF_RANGE_US = 0.5 * (PWM_MAX_US - PWM_MIN_US)
# contract: a control of 0 means no output on that channel; a servo without
# a signal holds its position, so it reads as neutral (angle 0, range 0)
NO_OUTPUT_PWM_US = 0.0

# --- KinematicPlane -----------------------------------------------------
# controls[] indices of ArduPlane's default outputs: SERVO1_FUNCTION 4
# (aileron), SERVO2_FUNCTION 19 (elevator), SERVO3_FUNCTION 70 (throttle)
AILERON_CHANNEL = 0
ELEVATOR_CHANNEL = 1
THROTTLE_CHANNEL = 2
# true airspeed at full throttle, and how fast it is approached
MAX_AIRSPEED_M_S = 25.0
SPEED_TIME_CONSTANT_S = 2.0
# bank angle at full aileron, and how fast it is approached
MAX_BANK_RAD = math.radians(45.0)
BANK_TIME_CONSTANT_S = 0.5
# pitch angle at full elevator (no angle of attack: the flight path
# follows the pitch), and how fast it is approached
MAX_PITCH_RAD = math.radians(20.0)
PITCH_TIME_CONSTANT_S = 0.5
# on the ground below this airspeed the wheels hold the toy level and on
# its heading
LIFTOFF_AIRSPEED_M_S = 12.0
# below this airspeed there is no turn: g tan(bank) / V would blow up
MIN_TURN_AIRSPEED_M_S = 1.0

# --- transport ----------------------------------------------------------
TCP_PREFIX = "tcp:"
TCP_LISTEN_PREFIX = "tcpin:"
# tcpin:PORT listens on loopback only; give tcpin:0.0.0.0:PORT to listen
# on every interface
LISTEN_HOST_DEFAULT = "127.0.0.1"
# USB CDC ignores the baud rate; on a real UART it must match SERIALn_BAUD
DEFAULT_BAUD = 921600
# longest one read (or one wait for a client) blocks, so stop and drop
# requests are seen promptly
READ_TIMEOUT_S = 0.1
# a write that cannot complete in this time means the port is gone
WRITE_TIMEOUT_S = 0.1
TCP_CONNECT_TIMEOUT_S = 2.0
# pause between attempts to reopen a port that is not there (a rebooting
# board is away for seconds)
RECONNECT_INTERVAL_S = 1.0
RX_CHUNK_BYTES = 4096
LISTEN_BACKLOG = 1                  # one board link at a time
# how long drop_connection() and stop() wait for the reader thread
THREAD_WAIT_TIMEOUT_S = 5.0
# CLI status line interval
STATUS_INTERVAL_S = 5.0


# =======================================================================
# Pure parts: origin, state, unit conversions
# =======================================================================

@dataclass(frozen=True)
class Origin:
    '''Where the vehicle starts: the ground under it. Must match the
    board's SIM_OPOS_* (or SITL's --home).'''
    lat_deg: float
    lon_deg: float
    alt_m: float                    # ground altitude above mean sea level
    heading_deg: float

    @property
    def lat_e7(self):
        return int(round(self.lat_deg * DEG_E7))

    @property
    def lon_e7(self):
        return int(round(self.lon_deg * DEG_E7))


# SITL's default home (Tools/autotest/locations.txt CMAC), also the
# defaults of SIM_OPOS_LAT/LNG/ALT/HDG (libraries/SITL/SITL.cpp)
DEFAULT_ORIGIN = Origin(-35.363261, 149.165230, 584.0, 353.0)


@dataclass(frozen=True)
class VehicleState:
    '''Vehicle state in SI units relative to the origin. Euler angles are
    3-2-1 (body to NED), body axes x forward, y right, z down.'''
    north_m: float
    east_m: float
    down_m: float                   # positive below the origin's ground level
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    p_rad_s: float                  # body rates
    q_rad_s: float
    r_rad_s: float
    vn_m_s: float                   # NED ground velocity
    ve_m_s: float
    vd_m_s: float
    tas_m_s: float                  # true airspeed (no wind)
    fx_m_s2: float                  # body specific force: what an accelerometer
    fy_m_s2: float                  # reads, (0, 0, -g) when level at rest
    fz_m_s2: float


def rest_state(heading_deg):
    '''Level at the origin on this heading, at rest.'''
    return VehicleState(0.0, 0.0, 0.0, 0.0, 0.0, math.radians(heading_deg) % (2.0 * math.pi),
                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -GRAVITY_MSS)


def euler_to_quaternion(roll, pitch, yaw):
    '''Body-to-NED quaternion (w, x, y, z) from 3-2-1 Euler angles, as
    QuaternionT::from_euler (libraries/AP_Math/quaternion.cpp).'''
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def ned_to_body(n, e, d, roll, pitch, yaw):
    '''A NED vector in body axes for 3-2-1 Euler angles.'''
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    x1, y1 = cy * n + sy * e, -sy * n + cy * e     # about z by the yaw
    x2, z2 = cp * x1 - sp * d, sp * x1 + cp * d     # about y by the pitch
    return x2, cr * y1 + sr * z2, -sr * y1 + cr * z2   # about x by the roll


def euler_rates_to_body_rates(roll, pitch, roll_rate, pitch_rate, yaw_rate):
    '''Body rates (p, q, r) from 3-2-1 Euler angle rates.'''
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    return (roll_rate - yaw_rate * sp,
            pitch_rate * cr + yaw_rate * sr * cp,
            -pitch_rate * sr + yaw_rate * cr * cp)


def longitude_scale(lat_e7):
    '''As Location::longitude_scale (libraries/AP_Common/Location.cpp).'''
    return max(math.cos(math.radians(lat_e7 / DEG_E7)), MIN_LONGITUDE_SCALE)


def wrap_longitude_e7(lon_e7):
    '''Into -180..180 degrees, as Location::wrap_longitude.'''
    if lon_e7 > LON_HALF_TURN_E7:
        return lon_e7 - LON_FULL_TURN_E7
    if lon_e7 < -LON_HALF_TURN_E7:
        return lon_e7 + LON_FULL_TURN_E7
    return lon_e7


def offset_latlon_e7(lat_e7, lon_e7, north_m, east_m):
    '''lat, lon (degE7) of a point north_m, east_m from (lat_e7, lon_e7),
    with the flat-earth scaling of Location::offset_latlng, so the board's
    Location::get_distance_NE_double gives the offset back.'''
    dlat = north_m * LATLON_TO_M_INV
    dlon = east_m * LATLON_TO_M_INV / longitude_scale(lat_e7 + dlat * 0.5)
    return int(round(lat_e7 + dlat)), wrap_longitude_e7(int(round(lon_e7 + dlon)))


def eas2tas(alt_amsl_m):
    '''True airspeed per equivalent airspeed at this altitude in the ISA
    troposphere, with AP_Baro's constants.'''
    alt = min(alt_amsl_m, TROPOPAUSE_ALT_M)
    temperature = SSL_AIR_TEMPERATURE - ISA_LAPSE_RATE * alt
    exponent = GRAVITY_MSS / (ISA_LAPSE_RATE * ISA_GAS_CONSTANT)
    pressure = SSL_AIR_PRESSURE * (temperature / SSL_AIR_TEMPERATURE) ** exponent
    density = pressure / (ISA_GAS_CONSTANT * temperature)
    return math.sqrt(SSL_AIR_DENSITY / density)


def clamp_int(value, lo, hi):
    return int(min(hi, max(lo, round(value))))


def milli_g(specific_force_m_s2):
    return clamp_int(specific_force_m_s2 / GRAVITY_MSS * MILLI_G_PER_G, INT16_MIN, INT16_MAX)


def state_fields(origin, state):
    '''HIL_STATE_QUATERNION fields (keyword arguments of
    hil_state_quaternion_encode) without time_usec.

    ind_airspeed carries the equivalent airspeed the board's airspeed
    sensor must read (no wind, so IAS = EAS = TAS / EAS2TAS).'''
    alt_amsl_m = origin.alt_m - state.down_m
    lat_e7, lon_e7 = offset_latlon_e7(origin.lat_e7, origin.lon_e7, state.north_m, state.east_m)
    eas_m_s = state.tas_m_s / eas2tas(alt_amsl_m)
    return {
        "attitude_quaternion": list(euler_to_quaternion(state.roll_rad, state.pitch_rad, state.yaw_rad)),
        "rollspeed": state.p_rad_s,
        "pitchspeed": state.q_rad_s,
        "yawspeed": state.r_rad_s,
        "lat": lat_e7,
        "lon": lon_e7,
        "alt": int(round(alt_amsl_m * MM_PER_M)),
        "vx": clamp_int(state.vn_m_s * CM_PER_M, INT16_MIN, INT16_MAX),
        "vy": clamp_int(state.ve_m_s * CM_PER_M, INT16_MIN, INT16_MAX),
        "vz": clamp_int(state.vd_m_s * CM_PER_M, INT16_MIN, INT16_MAX),
        "ind_airspeed": clamp_int(eas_m_s * CM_PER_M, 0, UINT16_MAX),
        "true_airspeed": clamp_int(state.tas_m_s * CM_PER_M, 0, UINT16_MAX),
        "xacc": milli_g(state.fx_m_s2),
        "yacc": milli_g(state.fy_m_s2),
        "zacc": milli_g(state.fz_m_s2),
    }


def angle_from_pwm(pwm_us):
    '''-1..1 deflection from PWM, as ServoModel::filter_angle (unfiltered).'''
    if pwm_us == NO_OUTPUT_PWM_US:
        return 0.0
    return min(1.0, max(-1.0, (pwm_us - PWM_CENTRE_US) / PWM_HALF_RANGE_US))


def range_from_pwm(pwm_us):
    '''0..1 throttle from PWM, as ServoModel::filter_range (unfiltered).'''
    if pwm_us == NO_OUTPUT_PWM_US:
        return 0.0
    return min(1.0, max(0.0, (pwm_us - PWM_MIN_US) / (PWM_MAX_US - PWM_MIN_US)))


# =======================================================================
# Models: reset(origin), step(controls, dt_s), state()
# =======================================================================

class StaticModel(object):
    '''Level at the origin on the origin heading, at rest, whatever the
    outputs: specific force (0, 0, -g), zacc -1000 mG.'''
    name = "static"

    def __init__(self):
        self._state = rest_state(0.0)

    def reset(self, origin):
        self._state = rest_state(origin.heading_deg)

    def step(self, controls, dt_s):
        pass

    def state(self):
        return self._state


class KinematicPlane(object):
    '''A toy plane, enough to show the outputs moving the vehicle; not an
    aircraft model. Throttle sets the airspeed target, aileron the bank
    target, elevator the pitch target (each approached with a first-order
    lag); the bank makes a coordinated turn and the pitch is the flight
    path angle. No wind, no angle of attack or sideslip, no stall: at zero
    airspeed it hangs where it is. It cannot sink below the origin's
    ground level, and on the ground below LIFTOFF_AIRSPEED_M_S it stays
    level and on its heading. Explicit Euler integration.'''
    name = "kinematic"

    def __init__(self):
        self.reset(DEFAULT_ORIGIN)

    def reset(self, origin):
        start = rest_state(origin.heading_deg)
        self._north = self._east = self._down = 0.0
        self._roll = self._pitch = 0.0
        self._yaw = start.yaw_rad
        self._tas = 0.0
        self._vel = (0.0, 0.0, 0.0)
        self._state = start

    def step(self, controls, dt_s):
        aileron = angle_from_pwm(controls[AILERON_CHANNEL])
        elevator = angle_from_pwm(controls[ELEVATOR_CHANNEL])
        throttle = range_from_pwm(controls[THROTTLE_CHANNEL])

        grounded = self._down >= 0.0 and self._tas < LIFTOFF_AIRSPEED_M_S
        bank_target = 0.0 if grounded else aileron * MAX_BANK_RAD
        pitch_target = 0.0 if grounded else elevator * MAX_PITCH_RAD
        roll_rate = (bank_target - self._roll) / BANK_TIME_CONSTANT_S
        pitch_rate = (pitch_target - self._pitch) / PITCH_TIME_CONSTANT_S
        if grounded or self._tas < MIN_TURN_AIRSPEED_M_S:
            yaw_rate = 0.0
        else:
            yaw_rate = GRAVITY_MSS * math.tan(self._roll) / self._tas
        tas_rate = (throttle * MAX_AIRSPEED_M_S - self._tas) / SPEED_TIME_CONSTANT_S

        self._roll += roll_rate * dt_s
        self._pitch += pitch_rate * dt_s
        self._yaw = (self._yaw + yaw_rate * dt_s) % (2.0 * math.pi)
        self._tas += tas_rate * dt_s

        horizontal = self._tas * math.cos(self._pitch)
        vn = horizontal * math.cos(self._yaw)
        ve = horizontal * math.sin(self._yaw)
        vd = -self._tas * math.sin(self._pitch)
        self._north += vn * dt_s
        self._east += ve * dt_s
        self._down += vd * dt_s
        if self._down > 0.0:
            # on the ground: no further down
            self._down = 0.0
            vd = 0.0

        # specific force = acceleration minus gravity, in body axes
        an = (vn - self._vel[0]) / dt_s
        ae = (ve - self._vel[1]) / dt_s
        ad = (vd - self._vel[2]) / dt_s
        self._vel = (vn, ve, vd)
        fx, fy, fz = ned_to_body(an, ae, ad - GRAVITY_MSS, self._roll, self._pitch, self._yaw)
        p, q, r = euler_rates_to_body_rates(self._roll, self._pitch, roll_rate, pitch_rate, yaw_rate)
        self._state = VehicleState(self._north, self._east, self._down,
                                   self._roll, self._pitch, self._yaw,
                                   p, q, r, vn, ve, vd, self._tas, fx, fy, fz)

    def state(self):
        return self._state


MODELS = {
    StaticModel.name: StaticModel,
    KinematicPlane.name: KinematicPlane,
}


def make_model(name):
    try:
        return MODELS[name]()
    except KeyError:
        raise ValueError("unknown model %r, expected one of %s" % (name, ", ".join(sorted(MODELS)))) from None


# =======================================================================
# Board clock stepping
# =======================================================================

@dataclass(frozen=True)
class StepResult:
    # FIRST, STEP, GAP (a STEP across lost frames), REINIT, RETRANSMIT or
    # CONFLICT (neither of the last two is stepped)
    event: str
    board_dt_us: int                # board time since the previous frame (0 unless STEP or GAP)
    steps: int                      # integration steps taken for this frame
    lost_frames: int                # estimate from the interval, GAP only
    was_armed: bool                 # the previous frame had the armed bit
    previous_time_us: int           # board time of the previous frame (-1 on FIRST)


class BoardClockStepper(object):
    '''Advance a model on the board clock with the outputs held between
    frames (zero-order hold). Integer microseconds throughout, so the step
    count is exact.

    The plant moves on the grid t_0 + n x step_us. Each step uses the
    outputs in effect at its start: those of the newest frame at or
    before that grid point. The outputs u_k of a frame at t_k therefore
    take effect at the first grid point at or after t_k; a step that
    straddles t_k still uses u_(k-1), and of several frames before the
    same grid point only the newest takes effect there.'''

    def __init__(self, model, origin, step_us=PHYSICS_STEP_US, loop_rate_hz=DEFAULT_LOOP_RATE_HZ,
                 gap_factor=GAP_FACTOR, max_jump_us=MAX_FORWARD_JUMP_US):
        if int(step_us) != step_us or step_us <= 0:
            raise ValueError("step_us must be a positive whole number of microseconds")
        if loop_rate_hz <= 0:
            raise ValueError("loop_rate_hz must be positive")
        self.model = model
        self.origin = origin
        self.step_us = int(step_us)
        self.step_s = self.step_us / float(US_PER_S)
        self.nominal_loop_us = US_PER_S / float(loop_rate_hz)
        self.gap_threshold_us = gap_factor * self.nominal_loop_us
        self.max_jump_us = max_jump_us
        self.t0_us = None
        self.last_us = None
        self.last_frame = None          # (controls, armed, flags) of the last frame
        self.held = None                # outputs the next step uses
        self.pending = None             # (grid index, outputs) of a frame whose grid point is ahead
        self.armed = False
        self.steps_done = 0
        self.reinits = 0
        self.armed_reinits = 0
        self.gaps = 0
        self.retransmits = 0
        self.conflicts = 0
        # before the first frame the state is the start state at the origin
        self.model.reset(origin)

    @property
    def plant_time_us(self):
        '''Board time of the plant state: t_0 plus whole steps.'''
        if self.t0_us is None:
            return None
        return self.t0_us + self.steps_done * self.step_us

    def state(self):
        return self.model.state()

    def _initialise(self, time_us, controls):
        self.model.reset(self.origin)
        self.t0_us = time_us
        self.steps_done = 0
        # t_0 is grid point 0: the outputs act from the first step
        self.held = controls
        self.pending = None

    def _apply_pending(self):
        '''the latched outputs take over once the plant is at their grid point'''
        if self.pending is not None and self.pending[0] <= self.steps_done:
            self.held = self.pending[1]
            self.pending = None

    def _latch(self, time_us, controls):
        '''u_k acts from the first grid point at or after t_k; if a newer
        frame arrives before that point, it supersedes u_k there'''
        self._apply_pending()
        first_grid_index = -((self.t0_us - time_us) // self.step_us)    # ceil((t_k - t_0) / step)
        self.pending = (first_grid_index, controls)
        self._apply_pending()

    def _advance_to(self, time_us):
        target = (time_us - self.t0_us) // self.step_us
        steps = 0
        while self.steps_done < target:
            self._apply_pending()
            self.model.step(self.held, self.step_s)
            self.steps_done += 1
            steps += 1
        return steps

    def on_actuators(self, time_us, controls, armed, flags=0):
        '''Frame k: advance to t_k with the outputs in effect, latch u_k.'''
        was_armed = self.armed
        previous = -1 if self.last_us is None else self.last_us
        controls = tuple(controls)
        frame = (controls, bool(armed), flags)
        if self.last_us is not None and time_us == self.last_us:
            # board time is strictly increasing within one boot: the same
            # time again is the same frame again (lockstep resends a frame
            # whose reply was lost with a connection), or a broken board
            if frame == self.last_frame:
                self.retransmits += 1
                event = RETRANSMIT
            else:
                self.conflicts += 1
                event = CONFLICT
            return StepResult(event, 0, 0, 0, was_armed, previous)
        if self.last_us is None:
            event, board_dt, steps, lost = FIRST, 0, 0, 0
            self._initialise(time_us, controls)
        elif time_us < self.last_us or time_us - self.last_us > self.max_jump_us:
            # board time went back or jumped: a new boot
            event, board_dt, steps, lost = REINIT, 0, 0, 0
            self.reinits += 1
            if was_armed:
                self.armed_reinits += 1
            self._initialise(time_us, controls)
        else:
            board_dt = time_us - self.last_us
            if board_dt > self.gap_threshold_us:
                event = GAP
                lost = max(1, int(round(board_dt / self.nominal_loop_us)) - 1)
                self.gaps += 1
            else:
                event, lost = STEP, 0
            steps = self._advance_to(time_us)
            self._latch(time_us, controls)
        self.armed = bool(armed)
        self.last_us = time_us
        self.last_frame = frame
        return StepResult(event, board_dt, steps, lost, was_armed, previous)


# =======================================================================
# Byte transports: open() raises OSError when the other end is not there,
# read() returns b'' after READ_TIMEOUT_S without data, any OSError from
# read() or write() means the connection is gone
# =======================================================================

class NoClientYet(OSError):
    '''A listener had no client within READ_TIMEOUT_S: keep waiting.'''


class SerialTransport(object):
    def __init__(self, port, baud=DEFAULT_BAUD):
        self.port = port
        self.baud = baud
        self._ser = None

    def open(self):
        import serial   # only serial ports need pyserial
        # serial.SerialException derives from IOError (OSError)
        self._ser = serial.Serial(self.port, self.baud, timeout=READ_TIMEOUT_S, write_timeout=WRITE_TIMEOUT_S)
        # drop what the PC buffered before the bridge opened the port
        self._ser.reset_input_buffer()

    def read(self):
        # wait for the first byte, then take what arrived with it, so a
        # frame is passed on without waiting for more
        data = self._ser.read(1)
        waiting = self._ser.in_waiting if data else 0
        if waiting:
            data += self._ser.read(min(waiting, RX_CHUNK_BYTES))
        return data

    def write(self, data):
        self._ser.write(data)

    def close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            except OSError:
                pass
            self._ser = None

    def shutdown(self):
        self.close()


class _SocketConnection(object):
    _sock = None

    def _adopt(self, sock):
        sock.settimeout(READ_TIMEOUT_S)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)   # one small frame per loop
        self._sock = sock

    def read(self):
        try:
            data = self._sock.recv(RX_CHUNK_BYTES)
        except socket.timeout:
            return b""
        if not data:
            raise ConnectionResetError("TCP peer closed the connection")
        return data

    def write(self, data):
        self._sock.sendall(data)

    def close(self):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None


class TcpTransport(_SocketConnection):
    '''TCP client.'''

    def __init__(self, host, port):
        self.host = host
        self.port = port

    def open(self):
        self._adopt(socket.create_connection((self.host, self.port), timeout=TCP_CONNECT_TIMEOUT_S))

    def shutdown(self):
        self.close()


class TcpListenTransport(_SocketConnection):
    '''TCP listener, one client at a time; after a disconnect the next
    open() accepts a new one. The port is bound at construction (port 0
    picks a free one; see .port).'''

    def __init__(self, host, port):
        self.host = host
        listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            if os.name == "nt":
                # SO_REUSEADDR on Windows lets two listeners share a port
                listener.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
            else:
                listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind((host, port))
            listener.listen(LISTEN_BACKLOG)
        except OSError:
            listener.close()
            raise
        self._listener = listener
        self.port = listener.getsockname()[1]

    def open(self):
        if self._listener is None:
            raise OSError("listener shut down")
        ready, _, _ = select.select([self._listener], [], [], READ_TIMEOUT_S)
        if not ready:
            raise NoClientYet("no client on %s:%u yet" % (self.host, self.port))
        conn, _ = self._listener.accept()
        self._adopt(conn)

    def shutdown(self):
        self.close()
        if self._listener is not None:
            try:
                self._listener.close()
            except OSError:
                pass
            self._listener = None


def _split_host_port(text, device):
    host, _, port = text.rpartition(":")
    if not port.isdigit():
        raise ValueError("bad port in %r" % device)
    return host, int(port)


def make_transport(device, baud=DEFAULT_BAUD):
    '''tcp:HOST:PORT -> TCP client; tcpin:[HOST:]PORT -> TCP listener
    (bound here); anything else is a serial port name.'''
    if device.startswith(TCP_LISTEN_PREFIX):
        host, port = _split_host_port(device[len(TCP_LISTEN_PREFIX):], device)
        return TcpListenTransport(host or LISTEN_HOST_DEFAULT, port)
    if device.startswith(TCP_PREFIX):
        host, port = _split_host_port(device[len(TCP_PREFIX):], device)
        if not host:
            raise ValueError("expected tcp:HOST:PORT, got %r" % device)
        return TcpTransport(host, port)
    return SerialTransport(device, baud)


# =======================================================================
# The bridge
# =======================================================================

@dataclass
class BridgeStats:
    connects: int = 0
    disconnects: int = 0
    open_failures: int = 0
    frames: int = 0                 # HIL_ACTUATOR_CONTROLS handled
    replies: int = 0                # HIL_STATE_QUATERNION replies written
    held_replies: int = 0           # frames not answered (hold_replies)
    write_failures: int = 0
    mavlink1: int = 0               # MAVLink 1 frames, ignored
    loopback: int = 0               # frames with the bridge's own sysid, ignored
    other_msgs: int = 0             # other message types, ignored
    bad_data: int = 0               # bytes that were no valid frame (CRC, framing)


class _Sink(object):
    '''File interface of the MAVLink encoder: write() goes to the bridge
    and remembers whether the bytes went out.'''

    def __init__(self, bridge):
        self._bridge = bridge
        self.ok = False

    def write(self, data):
        self.ok = self._bridge._write_bytes(data)


class HILBridge(object):
    '''The PC side of the HIL link: a reader thread answers every
    HIL_ACTUATOR_CONTROLS with HIL_STATE_QUATERNION at the same time_usec.

    device: a device string (see make_transport) or a transport object
            (open/read/write/close/shutdown), e.g. a fake in tests.
    model:  StaticModel, KinematicPlane or anything with
            reset(origin)/step(controls, dt_s)/state().
    origin: Origin; must match the board's SIM_OPOS_* or SITL's --home.
    clock:  PC clock for diagnostics only (never physics).
    say:    where reports go (print by default).

    Test hooks: last_controls, last_mode, last_flags, last_time_usec,
    last_event, frames, hold_replies(bool), drop_connection(),
    send_raw(bytes), send_state(**overrides), stats, stepper.'''

    def __init__(self, device, model, origin=DEFAULT_ORIGIN, sysid=DEFAULT_SYSID, baud=DEFAULT_BAUD,
                 loop_rate_hz=DEFAULT_LOOP_RATE_HZ, step_us=PHYSICS_STEP_US, max_jump_us=MAX_FORWARD_JUMP_US,
                 say=print, clock=time.monotonic):
        self.transport = make_transport(device, baud) if isinstance(device, str) else device
        self.origin = origin
        self.sysid = sysid
        self.stepper = BoardClockStepper(model, origin, step_us=step_us, loop_rate_hz=loop_rate_hz,
                                         max_jump_us=max_jump_us)
        self.stats = BridgeStats()
        self._say = say
        self._clock = clock
        self._lock = threading.RLock()          # physics state and test-hook reads
        self._write_lock = threading.Lock()     # encoder sequence number and port writes
        self._sink = _Sink(self)
        self._encoder = mavlink2.MAVLink(self._sink, srcSystem=sysid, srcComponent=BRIDGE_COMPID)
        self._parser = self._new_parser()
        self._hold = False
        self._connected = False
        self._waiting_for_client = False
        self._open_failure_reported = False
        self._stop = threading.Event()
        self._drop_request = threading.Event()
        self._dropped = threading.Event()
        self._thread = None
        self.failure = None
        self.last_controls = None
        self.last_mode = None
        self.last_flags = None
        self.last_time_usec = None
        self.last_frame_pc_s = None             # diagnostic: PC clock at the last frame
        self.last_event = None                  # StepResult.event of the last frame
        self._lockstep_reported = False

    # --- test hooks ----------------------------------------------------

    @property
    def frames(self):
        return self.stats.frames

    def hold_replies(self, hold):
        '''While held, frames are still received and stepped, but not
        answered: the board's state ages and its link goes LOST.'''
        self._hold = bool(hold)

    def drop_connection(self):
        '''Close the connection; the reader thread reopens it (a TCP client
        reconnects, a listener accepts the next client, a serial port is
        reopened) after RECONNECT_INTERVAL_S. True once closed.'''
        if self._thread is None or not self._thread.is_alive():
            self._close("dropped on request")
            return True
        self._dropped.clear()
        self._drop_request.set()
        return self._dropped.wait(THREAD_WAIT_TIMEOUT_S)

    def send_raw(self, data):
        '''Write bytes as they are (garbage, MAVLink 1, foreign frames).'''
        with self._write_lock:
            return self._write_bytes(bytes(data))

    def send_state(self, **overrides):
        '''Send one HIL_STATE_QUATERNION: the current state at the last
        frame's time_usec (0 before any frame), with fields replaced by
        overrides (any HIL_STATE_QUATERNION field, time_usec included).'''
        unknown = set(overrides) - set(STATE_FIELDS)
        if unknown:
            raise ValueError("not HIL_STATE_QUATERNION fields: %s" % ", ".join(sorted(unknown)))
        with self._lock:
            fields = state_fields(self.origin, self.stepper.state())
            fields["time_usec"] = 0 if self.last_time_usec is None else self.last_time_usec
        fields.update(overrides)
        return self._send(self._encoder.hil_state_quaternion_encode(**fields))

    # --- frame handling (the physics path: no PC clock) ----------------

    def receive_bytes(self, data):
        '''Parse bytes from the board and handle each complete frame.'''
        for msg in self._parser.parse_buffer(data) or []:
            self._on_message(msg)

    def _on_message(self, msg):
        kind = msg.get_type()
        if kind == BAD_DATA_TYPE:
            self.stats.bad_data += 1
        elif msg.get_msgbuf()[0] != mavlink2.PROTOCOL_MARKER_V2:
            self.stats.mavlink1 += 1
        elif msg.get_srcSystem() == self.sysid:
            self.stats.loopback += 1
        elif kind != ACTUATORS_TYPE:
            self.stats.other_msgs += 1
        else:
            self.handle_actuators(msg)

    def handle_actuators(self, msg):
        '''Step the model to the frame's board time and answer it. A
        retransmitted frame is answered with the same state; nothing is
        stepped for it.'''
        armed = bool(msg.mode & ARMED_FLAG)
        with self._lock:
            result = self.stepper.on_actuators(msg.time_usec, msg.controls, armed, msg.flags)
            self.last_event = result.event
            self.stats.frames += 1
            self.last_controls = tuple(msg.controls)
            self.last_mode = msg.mode
            self.last_flags = msg.flags
            self.last_time_usec = msg.time_usec
            fields = state_fields(self.origin, self.stepper.state())
        self._report(result, msg)
        if self._hold:
            self.stats.held_replies += 1
            return False
        reply = self._encoder.hil_state_quaternion_encode(time_usec=msg.time_usec, **fields)
        if self._send(reply):
            self.stats.replies += 1
            return True
        return False

    def _report(self, result, msg):
        t_us = msg.time_usec
        if result.event == FIRST:
            self._say("bridge: first frame at board time %u us: %s model at the origin" %
                      (t_us, getattr(self.stepper.model, "name", "the")))
        elif result.event == REINIT:
            why = ("board time went back from %u to %u us" % (result.previous_time_us, t_us)
                   if t_us <= result.previous_time_us else
                   "board time jumped %.3f s forward" % ((t_us - result.previous_time_us) / float(US_PER_S)))
            if result.was_armed:
                self._say("bridge: ALERT: %s while the board was ARMED: board reboot or new link; "
                          "the simulator was re-initialised at the origin, this run is invalid" % why)
            else:
                self._say("bridge: %s: board reboot, simulator re-initialised at the origin" % why)
        elif result.event == GAP:
            self._say("bridge: GAP of %.1f ms board time before %u us (about %u frames lost), "
                      "stepped across with the held outputs" %
                      (result.board_dt_us / 1000.0, t_us, result.lost_frames))
        elif result.event == RETRANSMIT:
            self._say("bridge: the frame at board time %u us again (resent after a reconnect): "
                      "answered with the same state, nothing stepped" % t_us)
        elif result.event == CONFLICT:
            self._say("bridge: ERROR: a second frame at board time %u us with other outputs, mode or "
                      "flags: the board broke the contract; ignored (answered with the state at that time)" % t_us)
        if msg.flags & LOCKSTEP_FLAG and not self._lockstep_reported:
            self._lockstep_reported = True
            self._say("bridge: the board runs in lockstep: it waits for every reply")

    # --- writing ----------------------------------------------------------

    def _send(self, msg):
        with self._write_lock:
            self._sink.ok = False
            self._encoder.send(msg)
            return self._sink.ok

    def _write_bytes(self, data):
        '''Called with _write_lock held.'''
        if not self._connected:
            self.stats.write_failures += 1
            return False
        try:
            self.transport.write(data)
        except OSError:
            self.stats.write_failures += 1
            self._drop_request.set()        # the reader thread closes it
            return False
        return True

    # --- connection and reader thread (I/O: the PC clock is allowed) ----

    def _new_parser(self):
        parser = mavlink2.MAVLink(None, srcSystem=self.sysid, srcComponent=BRIDGE_COMPID)
        parser.robust_parsing = True    # bad bytes come back as BAD_DATA instead of raising
        return parser

    def connect(self):
        '''Open the transport once; True when connected. A listener without
        a client has already waited READ_TIMEOUT_S for one.'''
        self._waiting_for_client = False
        try:
            self.transport.open()
        except NoClientYet:
            self._waiting_for_client = True
            return False
        except OSError as exc:
            self.stats.open_failures += 1
            if not self._open_failure_reported:
                self._open_failure_reported = True
                self._say("bridge: cannot open the link (%s); retrying every %.0f s" % (exc, RECONNECT_INTERVAL_S))
            return False
        self._open_failure_reported = False
        self._parser = self._new_parser()   # no half frame from an earlier connection
        self._connected = True
        self.stats.connects += 1
        self._say("bridge: link connected (#%u)" % self.stats.connects)
        return True

    def _close(self, why):
        with self._write_lock:
            was_connected = self._connected
            self._connected = False
            self.transport.close()
        if was_connected:
            self.stats.disconnects += 1
            self._say("bridge: link closed (%s)" % why)

    def _run(self):
        while not self._stop.is_set():
            if self._drop_request.is_set():
                self._drop_request.clear()
                self._close("dropped on request")
                self._dropped.set()
                self._stop.wait(RECONNECT_INTERVAL_S)
                continue
            if not self._connected:
                if not self.connect() and not self._waiting_for_client:
                    self._stop.wait(RECONNECT_INTERVAL_S)
                continue
            try:
                data = self.transport.read()
            except OSError as exc:
                self._close(str(exc))
                continue
            if data:
                self.last_frame_pc_s = self._clock()
                self.receive_bytes(data)

    def _thread_main(self):
        try:
            self._run()
        except Exception as exc:   # report, never die silently
            self.failure = exc
            self._say("bridge: reader thread failed:\n" + traceback.format_exc())
        finally:
            self._close("bridge stopped")

    def start(self):
        if self._thread is not None:
            raise RuntimeError("bridge already started")
        self._thread = threading.Thread(target=self._thread_main, name="hil-bridge", daemon=True)
        self._thread.start()
        return self

    def is_running(self):
        return self._thread is not None and self._thread.is_alive()

    def stop(self):
        '''Stop the reader thread and release the port; raises if the
        reader thread failed.'''
        self._stop.set()
        if self._thread is not None:
            self._thread.join(THREAD_WAIT_TIMEOUT_S)
        self._close("bridge stopped")
        self.transport.shutdown()
        if self.failure is not None:
            raise RuntimeError("HIL bridge reader thread failed: %r" % (self.failure,))

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, tb):
        self.stop()

    def status_line(self):
        with self._lock:
            state = self.stepper.state()
            s = self.stats
            alt = self.origin.alt_m - state.down_m
            return ("bridge: frames=%u replies=%u held=%u gaps=%u reinits=%u resent=%u conflicts=%u bad=%u | "
                    "alt=%.1f m tas=%.1f m/s hdg=%.0f deg roll=%.0f deg pitch=%.0f deg" %
                    (s.frames, s.replies, s.held_replies, self.stepper.gaps, self.stepper.reinits,
                     self.stepper.retransmits, self.stepper.conflicts, s.bad_data,
                     alt, state.tas_m_s, math.degrees(state.yaw_rad), math.degrees(state.roll_rad),
                     math.degrees(state.pitch_rad)))


# =======================================================================
# Command line
# =======================================================================

def parse_origin(text):
    '''"lat,lon,alt_m,hdg_deg" -> Origin.'''
    parts = text.split(",")
    if len(parts) != 4:
        raise argparse.ArgumentTypeError("expected lat,lon,alt_m,hdg_deg, got %r" % text)
    try:
        lat, lon, alt, hdg = (float(p) for p in parts)
    except ValueError:
        raise argparse.ArgumentTypeError("expected four numbers, got %r" % text) from None
    if not (abs(lat) <= 90.0 and abs(lon) <= 180.0 and 0.0 <= hdg < 360.0 and math.isfinite(alt)):
        raise argparse.ArgumentTypeError("origin out of range: %r" % text)
    return Origin(lat, lon, alt, hdg)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Reference simulator bridge for ArduPilot's HIL backend")
    parser.add_argument("--device", required=True,
                        help="COMx, /dev/ttyACMx, tcp:HOST:PORT or tcpin:[HOST:]PORT")
    parser.add_argument("--origin", type=parse_origin, default=DEFAULT_ORIGIN,
                        help="lat,lon,alt_m,hdg_deg; must match SIM_OPOS_* (default: SITL's CMAC home)")
    parser.add_argument("--model", choices=sorted(MODELS), default=StaticModel.name)
    parser.add_argument("--sysid", type=int, default=DEFAULT_SYSID, help="bridge MAVLink system id")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                        help="serial baud rate (ignored by USB CDC; match SERIALn_BAUD on a UART)")
    parser.add_argument("--loop-rate-hz", type=float, default=DEFAULT_LOOP_RATE_HZ,
                        help="the board's SCHED_LOOP_RATE (GAP reports only)")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    bridge = HILBridge(args.device, make_model(args.model), args.origin, sysid=args.sysid, baud=args.baud,
                       loop_rate_hz=args.loop_rate_hz)
    print("bridge: %s model on %s, origin %.7f,%.7f,%.1f m,%.0f deg; Ctrl+C to stop" %
          (args.model, args.device, args.origin.lat_deg, args.origin.lon_deg, args.origin.alt_m,
           args.origin.heading_deg))
    bridge.start()
    try:
        while bridge.is_running():
            time.sleep(STATUS_INTERVAL_S)
            print(bridge.status_line())
    except KeyboardInterrupt:
        pass
    finally:
        bridge.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
