

#include "led.hpp"

#include <quan/stm32/gpio.hpp>

using namespace quan::stm32;

namespace {

//  Use Servo1 on PA5 TIM2_CH1 as LED
   typedef quan::mcu::pin<gpioa,5> led1;
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

void led::setup()
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
