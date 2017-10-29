
/*
based on
https://github.com/bitcraze/crazyflie-firmware/blob/master/src/drivers/src/ws2812_cf2.c

which is 
 * Copyright (C) 2011-2014 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * ws2812.c: Neopixel LED driver
 * Mostly from Elia's electonic blog: http://eliaselectronics.com/driving-a-ws2812-rgb-led-with-an-stm32/
 */

#include <stm32f0xx.h>
#include <quan/stm32/tim.hpp>
#include <quan/stm32/gpio.hpp>
#include <quan/stm32/rcc.hpp>
#include <quan/stm32/tim/temp_reg.hpp>
#include <quan/stm32/get_raw_timer_frequency.hpp>
#include "../../led_sequence.hpp"

/*
use pa6 as neo_pixel_pin 
 use TIM16_CH1  ( AF5 for pa6)
*/

namespace {
  typedef quan::stm32::tim16 led_seq_timer;

   typedef quan::mcu::pin<quan::stm32::gpioa,6> neopixel_out_pin;
}

void led_sequence::initialise()
{
    quan::stm32::module_enable<led_seq_timer>();
    
   // turn on pwm timer in rcc
   // turn on dma in rcc
   constexpr uint8_t dma_rcc_bit = 0U;
   quan::stm32::rcc::get()->ahbenr.setbit<dma_rcc_bit>();

   // set up pin as af5
   quan::stm32::module_enable<neopixel_out_pin::port_type>();
   quan::stm32::apply<
      neopixel_out_pin,
      quan::stm32::gpio::mode::af5,  
      quan::stm32::gpio::pupd::pull_down,
      quan::stm32::gpio::ospeed::medium_fast
   >();
   
   constexpr uint32_t raw_timer_freq = quan::stm32::get_raw_timer_frequency<led_seq_timer>();
   static_assert(raw_timer_freq == 48000000,"unexpected raw freq");
   constexpr uint32_t reqd_freq = 800000U;

   constexpr uint32_t prescaler = (raw_timer_freq / reqd_freq) - 1U;
   static_assert(raw_timer_freq % reqd_freq == 0, "inaccurate precaler");
   static_assert(prescaler == 59, "need to redo and check prescaler");

   led_seq_timer::get()->psc = prescaler;
   {
      // enable preload
      quan::stm32::tim::cr1_t cr1 = 0;

      cr1.arpe = true;

      led_seq_timer::get()->cr1.set(cr1.value);
   }
   {
      // set timer to pwm mode
      quan::stm32::tim::ccmr1_t ccmr1 = 0;

      ccmr1.cc1s = 0b00;  // output
      ccmr1.oc1m = 0b110; // PWM mode 1
      ccmr1.oc1pe = true; // want preload

      led_seq_timer::get()->ccmr1.set(ccmr1.value);
   }

   DMA1
   // dma setup;
   // by default tim16_ch1 is on channel 3
  
   // start timer
   //led_seq_timer::get()->cr1.setbit<0>(); // (CEN)

}




