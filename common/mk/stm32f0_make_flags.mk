#General stm32f0 flags
SUBHUB_INCLUDE_PATH = $(patsubst %common/mk/,%libraries/,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
## exports
# export TOOLCHAIN_PREFIX="/opt/gcc-arm-none-eabi-4_7-2013q2/"
# export TOOLCHAIN_GCC_VERSION="4.7.4"
# export QUAN_INCLUDE_PATH=/home/andy/website/quan-trunk
# export STM32_STD_PERIPH_LIB_DIR=/opt/stm32f4/STM32F4xx_DSP_StdPeriph_Lib_V1.0.0/Libraries/

CC      = $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-g++
CC1     = $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-gcc
LD      = $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-g++
CP      = $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-objcopy
OD      = $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-objdump
AR      = $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-ar
SIZ     = $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-size

INCLUDES = $(STM32_STD_PERIPH_LIB_DIR)CMSIS/Include \
$(STM32_STD_PERIPH_LIB_DIR)CMSIS/Device/ST/STM32F0xx/Include \
$(QUAN_INCLUDE_PATH) $(SUBHUB_INCLUDE_PATH)

INIT_LIB_PREFIX = $(TOOLCHAIN_PREFIX)lib/gcc/arm-none-eabi/$(TOOLCHAIN_GCC_VERSION)/armv6-m/

DEFINES = STM32F051x8 QUAN_STM32F0 QUAN_NO_EXCEPTIONS QUAN_STM32_SUBFAMILY_05X HSE_VALUE=8000000 \
 QUAN_SYSTICK_TIMER_UINT32

STARTUP = startupf0.s
LINKER_SCRIPT = $(SUBHUB_INCLUDE_PATH)system/stm32f0.ld
SYSTEM_INIT = system_initf0
PROCESSOR_FLAGS = -mcpu=cortex-m0 -mthumb -mfloat-abi=soft

INIT_LIBS = $(INIT_LIB_PREFIX)crti.o $(INIT_LIB_PREFIX)crtn.o

CFLAG_EXTRAS = -Wno-psabi -fno-math-errno -Os
# for float printf format etc
# CFLAG_EXTRAS += -Wl,-u,vsprintf -lm

CFLAGS  = -std=c++11 -fno-rtti -fno-exceptions -c -g $(DEFINE_ARGS) $(INCLUDE_ARGS) \
  $(PROCESSOR_FLAGS) $(CFLAG_EXTRAS)

LFLAGS  = -T$(LINKER_SCRIPT)  -nostartfiles -nodefaultlibs $(PROCESSOR_FLAGS) $(INIT_LIBS) \
   --specs=nano.specs $(CFLAG_EXTRAS) -Wl,--gc-sections -Wl,--undefined=_sbrk

#LFLAGS  =  -v
CPFLAGS = -Obinary
#ODFLAGS = -D
ODFLAGS = -d
# to see source in listing. However with optimisation doesnt work too well
#ODFLAGS += -S

INCLUDE_ARGS = $(patsubst %,-I%,$(INCLUDES))

DEFINE_ARGS = $(patsubst %,-D%,$(DEFINES))
