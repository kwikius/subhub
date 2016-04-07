
SUBHUB_INCLUDE_PATH = $(patsubst %common/,%libraries/,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))

TOOLCHAIN_PREFIX =/opt/gcc-arm-none-eabi-4_7-2013q2/
TOOLCHAIN_GCC_VERSION = 4.7.4

CC      = $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-g++
CC1     = $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-gcc
LD      = $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-g++
CP      = $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-objcopy
OD      = $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-objdump
AR      = $(TOOLCHAIN_PREFIX)bin/arm-none-eabi-ar


  
INCLUDES = /opt/stm32f0/STM32F0xx_StdPeriph_Lib_V1.1.0/Libraries/CMSIS/Include \
/opt/stm32f0/STM32F0xx_StdPeriph_Lib_V1.1.0/Libraries/CMSIS/Device/ST/STM32F0xx/Include \
/home/andy/website/quan-trunk $(SUBHUB_INCLUDE_PATH)

INIT_LIB_PREFIX = $(TOOLCHAIN_PREFIX)/lib/gcc/arm-none-eabi/$(TOOLCHAIN_GCC_VERSION)/armv6-m/

DEFINES = QUAN_STM32F0 QUAN_NO_EXCEPTIONS QUAN_STM32_SUBFAMILY_05X HSE_VALUE=8000000

STARTUP = startupf0.s
LINKER_SCRIPT = stm32f0.ld
SYSTEM_INIT = system_initf0
PROCESSOR_FLAGS = -mcpu=cortex-m0 -mthumb -mfloat-abi=soft

INIT_LIBS = $(INIT_LIB_PREFIX)crti.o $(INIT_LIB_PREFIX)crtn.o

CFLAG_EXTRAS = -fno-math-errno -Os
# for float printf format etc
# CFLAG_EXTRAS += -Wl,-u,vsprintf -lm

CFLAGS  = -std=c++11 -fno-rtti -fno-exceptions -c -g $(DEFINE_ARGS) $(INCLUDE_ARGS) \
  $(PROCESSOR_FLAGS) $(CFLAG_EXTRAS)

LFLAGS  = -T$(LINKER_SCRIPT)  -nostartfiles -nodefaultlibs $(PROCESSOR_FLAGS) $(INIT_LIBS) \
   --specs=nano.specs $(CFLAG_EXTRAS)

#LFLAGS  =  -v
CPFLAGS = -Obinary
#ODFLAGS = -D
ODFLAGS = -d
# to see source in listing. However with optimisation doesnt work too well
#ODFLAGS += -S

INCLUDE_ARGS = $(patsubst %,-I%,$(INCLUDES))

DEFINE_ARGS = $(patsubst %,-D%,$(DEFINES))
