#!/usr/bin/env python3
"""Check the SITL pitot EAS-to-pressure contract with the real sensor functions.

Run with python3 libraries/AP_HAL_SITL/tests/test_pitot_eas.py.
Requires g++. No simulator is launched. Two real functions are compiled:
SITL_State::_update_airspeed (--source) and the sensor model it calls,
SITL::SIM::airspeed_sensor_pressure (--sim-source), which simulation on
hardware shares. Only their environmental inputs are stubbed: zero
noise/failure flags and a supplied density factor.
This does not replace SITL integration tests or test the noise gate itself.
An alternate --source permits unchanged failing/passing regression checks.
"""
import argparse
from pathlib import Path
import subprocess
import tempfile

PREFIX = r"""
#include <initializer_list>
#include <cmath>
#include <cstdint>
#include <cstdio>
#define AIRSPEED_MAX_SENSORS 1
#define PASCAL_TO_VOLTS(p) ((p) / 819)
#define SSL_AIR_PRESSURE 101325
using std::pow;
using std::sqrt;
float ratio_factor = 1;
float sq(float x) { return x*x; }
bool is_positive(float x) { return x > 0; }
bool is_zero(float x) { return x == 0; }
float rand_float() { return 0; }
struct AP_Baro {
    static float get_EAS2TAS_for_alt_amsl(float) { return ratio_factor; }
    float get_pressure() { return 101325; }
};
namespace AP {
    AP_Baro& baro() { static AP_Baro b; return b; }
}
struct Sensor {
    float ratio = 1.99f, fail = 0, fail_pressure = 0, fail_pitot_pressure = 0;
    float signflip = 0, offset = 2013;
};
struct SIM {
    Sensor airspeed[1];
    struct { float altitude = 1325; float airspeed_raw_pressure[1]; } state;
    float airspeed_noise(uint8_t) const { return 0; }
    float airspeed_sensor_pressure(uint8_t instance, float eas) const;
};
struct SITL_State {
    SIM sim;
    SIM* _sitl = &sim;
    float airspeed_pin_voltage[1];
    void _update_airspeed(float);
};
"""
SUFFIX = r"""
int main()
{
    SITL_State s;
    int fail = 0;
    // EAS is already density-corrected: pressure must not depend on this factor.
    for (float factor : {1.0f, 1.067f, 1.2f}) {
        ratio_factor = factor;
        float eas = 24.7f;
        s._update_airspeed(eas);
        float measured = s.sim.state.airspeed_raw_pressure[0];
        float expected = eas * eas / 1.99f;
        printf("factor=%.4f pressure=%.6f expected=%.6f error=%.6f\n",
               factor, measured, expected, measured - expected);
        if (fabsf(measured - expected) > 0.0001f) {
            fail++;
        }
    }
    return fail ? 1 : 0;
}
"""


def main():
    libraries = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path,
                        default=libraries / 'AP_HAL_SITL' / 'sitl_airspeed.cpp')
    parser.add_argument('--sim-source', type=Path,
                        default=libraries / 'SITL' / 'SITL.cpp')
    args = parser.parse_args()
    source = args.source.read_text()
    start = source.index('void SITL_State::_update_airspeed(')
    body = source[start:source.index('\n#endif', start)]
    sim_source = args.sim_source.read_text()
    sim_start = sim_source.index('float SIM::airspeed_sensor_pressure(')
    sim_body = sim_source[sim_start:sim_source.index('\n}\n', sim_start) + 3]
    with tempfile.TemporaryDirectory(prefix='sitl-pitot-eas-') as directory:
        cpp = Path(directory) / 'pitot.cpp'
        binary = Path(directory) / 'pitot'
        cpp.write_text(PREFIX + sim_body + '\n' + body + SUFFIX)
        subprocess.run(['g++', '-std=c++17', '-O2', str(cpp), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
    print('PASS: equivalent airspeed is converted to pressure exactly once')


if __name__ == '__main__':
    main()
