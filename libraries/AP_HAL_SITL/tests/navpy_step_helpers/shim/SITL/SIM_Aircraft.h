#pragma once
// Test shim: only what NavPyStep.cpp uses from SITL.
#include <stdint.h>
namespace SITL { struct Aircraft { static void seed_turbulence_random(uint32_t) {} }; }
