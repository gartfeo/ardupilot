/*
  SITL handling

  This simulates an analog airspeed sensor

  Andrew Tridgell November 2011
 */

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)

#include "AP_HAL_SITL.h"
#include "AP_HAL_SITL_Namespace.h"
#include "HAL_SITL_Class.h"
#include "SITL_State.h"
#include <SITL/SITL.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL& hal;

using namespace HALSITL;

// scaling value taken from AP_Airspeed_analog.cpp
#define VOLTS_TO_PASCAL 819
#define PASCAL_TO_VOLTS(_p) (_p/VOLTS_TO_PASCAL)

/*
  convert equivalent airspeed (EAS) in m/s to an airspeed sensor value
 */
void SITL_State::_update_airspeed(float eas)
{
    for (uint8_t i=0; i<AIRSPEED_MAX_SENSORS; i++) {
        // the model shared with simulation on hardware (SIMState)
        const float airspeed_pressure = _sitl->airspeed_sensor_pressure(i, eas);
        _sitl->state.airspeed_raw_pressure[i] = airspeed_pressure;

        // apply airspeed sensor offset for the analog pin only
        const float airspeed_raw = airspeed_pressure + _sitl->airspeed[i].offset;
        airspeed_pin_voltage[i] = PASCAL_TO_VOLTS(airspeed_raw);
    }
}

#endif
