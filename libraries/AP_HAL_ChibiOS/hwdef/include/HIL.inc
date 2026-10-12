# Hardware-in-the-loop (HIL) firmware for any board that can run
# simulation on hardware: the vehicle code runs on the real board while
# a PC simulator owns the physics. The HIL backend
# (libraries/SITL/SIM_HIL.cpp) sends the servo outputs as
# HIL_ACTUATOR_CONTROLS on the port whose SERIALn_PROTOCOL is 100 (SIM
# HIL) and feeds the simulated IMU, baro, compass, GPS and airspeed from
# each HIL_STATE_QUATERNION reply. Link contract and reference bridge:
# libraries/SITL/examples/HIL.
#
# Use it from a board directory:
#   include ../<Board>/hwdef.dat
#   include ../include/HIL.inc
# with a defaults.parm that starts with "@include ../include/HIL.parm".
#
# Real outputs: in simulation on hardware no channel outside SIM_OH_MASK
# (default 0) drives anything (SITL::OutputGate): FMU and IOMCU PWM get 0
# (no pulse), DShot channels get no frames or commands, the channel is
# never a GPIO, the IOMCU never mixes RC to its outputs and has its SBUS
# output off, DroneCAN servos and ESCs get no commands, and the serial and
# CAN actuator drivers below are not built. This holds from the moment the
# firmware runs; flash with safety on and power-cycle the board before the
# first safety-off (hwdef/Pixhawk6X-HIL/README.md).

include SimOnHW.inc

# the HIL simulator backend, created by libraries/AP_HAL/SIMState.cpp as
# SITL::AP_SIM_FRAME_CLASS::create(AP_SIM_FRAME_STRING). A board runs in
# real time only: lockstep ("hil:lockstep") is SITL only and does not
# build here.
define AP_SIM_HIL_ENABLED 1
define AP_SIM_FRAME_CLASS HIL
define AP_SIM_FRAME_STRING "hil"

# the HIL link defaults to the second USB CDC interface (OTG2). The
# generator applies HAL_OTG2_PROTOCOL only on boards with CAN; a board
# without CAN, or one that wants a UART, sets
# "define DEFAULT_SERIALn_PROTOCOL SerialProtocol_SIM_HIL" for its port
# (at 921600 baud or more for a 200 Hz loop).
undef HAL_OTG2_PROTOCOL
define HAL_OTG2_PROTOCOL SerialProtocol_SIM_HIL

# arming is refused until the simulator sends states, through an
# AP_Arming auxiliary authorisation (on even without scripting)
undef AP_ARMING_AUX_AUTH_ENABLED
define AP_ARMING_AUX_AUTH_ENABLED 1

# actuator drivers that bypass the PWM outputs are not built: serial
# servos and ESCs, SBUS servo output, CAN ESCs and engine control
undef AP_VOLZ_ENABLED AP_SBUSOUTPUT_ENABLED AP_ROBOTISSERVO_ENABLED AP_FETTEC_ONEWIRE_ENABLED
define AP_VOLZ_ENABLED 0
define AP_SBUSOUTPUT_ENABLED 0
define AP_ROBOTISSERVO_ENABLED 0
define AP_FETTEC_ONEWIRE_ENABLED 0
undef AP_KDECAN_ENABLED HAL_PICCOLO_CAN_ENABLE HAL_TORQEEDO_ENABLED AP_EFI_SERIAL_HIRTH_ENABLED
define AP_KDECAN_ENABLED 0
define HAL_PICCOLO_CAN_ENABLE 0
define HAL_TORQEEDO_ENABLED 0
define AP_EFI_SERIAL_HIRTH_ENABLED 0

# no bootloader is built for a HIL board directory, so none can be embedded
undef AP_BOOTLOADER_FLASHING_ENABLED
define AP_BOOTLOADER_FLASHING_ENABLED 0
