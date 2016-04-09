#!/bin/bash
## exports to build subhub
export TOOLCHAIN_PREFIX="/opt/gcc-arm-none-eabi-4_7-2013q2/"
export TOOLCHAIN_GCC_VERSION="4.7.4"
export QUAN_INCLUDE_PATH="/home/andy/website/quan-trunk"
export STM32_STD_PERIPH_LIB_DIR="/opt/stm32f0/STM32F0xx_StdPeriph_Lib_V1.1.0/Libraries/"
make
