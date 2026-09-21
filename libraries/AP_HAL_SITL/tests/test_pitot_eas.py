#!/usr/bin/env python3
"""Check the SITL pitot EAS-to-pressure contract with the real sensor function.

Run with python3 libraries/AP_HAL_SITL/tests/test_pitot_eas.py.
Requires g++. No simulator is launched. Only the function's environmental
inputs are stubbed: zero noise/failure flags and a supplied density factor.
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
struct Sim {
    Sensor airspeed[1];
    struct { float altitude = 1325; float airspeed_raw_pressure[1]; } state;
    float airspeed_noise(int) { return 0; }
};
struct SITL_State {
    Sim sim;
    Sim* _sitl = &sim;
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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path,
                        default=Path(__file__).resolve().parents[1] / 'sitl_airspeed.cpp')
    args = parser.parse_args()
    source = args.source.read_text()
    start = source.index('void SITL_State::_update_airspeed(')
    body = source[start:source.index('\n#endif', start)]
    with tempfile.TemporaryDirectory(prefix='sitl-pitot-eas-') as directory:
        cpp = Path(directory) / 'pitot.cpp'
        binary = Path(directory) / 'pitot'
        cpp.write_text(PREFIX + body + SUFFIX)
        subprocess.run(['g++', '-std=c++17', '-O2', str(cpp), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
    print('PASS: equivalent airspeed is converted to pressure exactly once')


if __name__ == '__main__':
    main()
