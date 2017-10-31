
#include <stm32f0xx.h>
#include <quan/stm32/millis.hpp>

extern "C" void SysTick_Handler()
{
   ++quan::stm32::detail::systick_tick::current;
}