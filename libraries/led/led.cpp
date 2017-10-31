
#include <quan/stm32/gpio.hpp>
#include "../../led.hpp"

using namespace quan::stm32;

namespace {

  /* on STM32F0 discovery board  
     LED3 is on PC9
     LED4 is on PC8
     but use another pin to put led on
  */

//  for subhub board Use Servo1 on PA5 TIM2_CH1 as LED
#if defined (QUAN_STM32F0_DISCOVERY_BOARD)
   typedef quan::mcu::pin<gpioc,8> led1;
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

}


