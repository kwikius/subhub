
Subhub node software for a simple distributed network version of Ardupilot flight controller.

For a reasonable sized conventional model plane, it is difficult to make use of the wings for 
electronics if they need to come apart. By embedding a "subhub" in each wing of the plane, 
which communicates with the fuselage mounted  flight controller by means of a high speed 
serial link, sensors and servos can be placed in the wings with only one simple 4 way 
connector which provides power and full duplex serial signalling.

The project is in very early stages but a pcb is available from OSHPark:

https://oshpark.com/shared_projects/9ENCnF8j

BUILD

The following environment variables have to be defined( for example exported in a bash script) 
These are examples only. You will need to define them for your system

# path to compiler above the bin dir
export TOOLCHAIN_PREFIX="/opt/gcc-arm-none-eabi-4_7-2013q2/"
# The compiler version
export TOOLCHAIN_GCC_VERSION="4.7.4"
Path to quan
export QUAN_INCLUDE_PATH="$(QUAN_ROOT)"
Path to the stm32 std 
export STM32_STD_PERIPH_LIB_DIR="/opt/stm32f0/STM32F0xx_StdPeriph_Lib_V1.1.0/Libraries/"

With the dependencies installed type make in the root directory.





