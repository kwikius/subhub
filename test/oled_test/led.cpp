

#include "led.hpp"

#include <quan/stm32/gpio.hpp>

using namespace quan::stm32;

namespace {

#if defined (QUAN_STM32F0_DISCOVERY_BOARD)
   typedef quan::mcu::pin<gpioa,11> led1;
#else
   typedef quan::mcu::pin<gpioa,5> led1;
#endif
}

void led::on()
{
  set<led1>();
}

void led::off()
{
  clear<led1>();
}

void led::complement()
{
   quan::stm32::complement<led1>();
}

void led::initialise()
{
   module_enable<led1::port_type>();
   apply<
      led1
      , gpio::mode::output
      , gpio::otype::push_pull
      , gpio::pupd::none
      , gpio::ospeed::slow
      , gpio::ostate::low
   >();

};
