
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

uint8_t led_sequence::dma_buffer[ 8U * bytes_per_led * 2U] = {0U};
rgb_value led_sequence::led_data [num_leds];

namespace {
   typedef quan::stm32::tim16 led_seq_timer;
   typedef quan::mcu::pin<quan::stm32::gpioa,6> neopixel_out_pin;

   constexpr uint32_t raw_timer_freq = quan::stm32::get_raw_timer_frequency<led_seq_timer>();
   static_assert(raw_timer_freq == 48000000,"unexpected raw freq");
   constexpr uint32_t reqd_freq = 800000U;

   constexpr uint32_t period = (raw_timer_freq / reqd_freq) ;
   static_assert(raw_timer_freq % reqd_freq == 0, "inaccurate period");
   static_assert(period == 60U, "need to redo timer period");

   static constexpr uint32_t zero_pwm = period / 3U;
   static constexpr uint32_t one_pwm  = (period * 2U) / 3U;
}

void led_sequence::initialise()
{
   // turn on pwm timer in rcc
   // turn on dma in rcc
   constexpr uint8_t dma_rcc_bit = 0U;
   quan::stm32::rcc::get()->ahbenr.setbit<dma_rcc_bit>();

   quan::stm32::module_enable<led_seq_timer>();

   // set up pin as af5
   quan::stm32::module_enable<neopixel_out_pin::port_type>();
   quan::stm32::apply<
      neopixel_out_pin,
      quan::stm32::gpio::mode::af5,  
      quan::stm32::gpio::pupd::pull_down,
      quan::stm32::gpio::ospeed::medium_fast
   >();
   
   led_seq_timer::get()->psc = 0;
   led_seq_timer::get()->arr = period - 1U;

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

    // ccer default

   // dma setup;
   // set up timer dma on cc1 compare
   {
      quan::stm32::tim::dier_t dier = 0;

      dier.cc1de = true;

      led_seq_timer::get()->dier.set(dier.value);
   }

   // BDTR moe
   led_seq_timer::get()->bdtr.setbit<15U>(); //(MOE)
   
   // enable dma irq
   NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

   //enable timer irq
   NVIC_EnableIRQ(TIM16_IRQn);
   
   DMA1_Channel3->CPAR = (uint32_t)&TIM16->CCR1;
   DMA1_Channel3->CMAR = (uint32_t)led_sequence::dma_buffer;

   // memory 8 bit default
   DMA1_Channel3->CCR  =
     (0b1 << 4U)      // (DIR) read from memory 
    | (0b1 << 7U)     // (MINC)
    | (0b01 << 8U)    // (PSIZE[0:1])  16 bit
    | (0b1 << 5U)     // (CIRC)
    | (0b11 << 12U)   // highest priority
    | (0b1 << 2U)     // (HTIE)
    | (0b1 << 1U)     // (TCIE)
   ;
}

namespace {
   uint32_t led_data_idx = 0U;
}

bool led_sequence::put(uint32_t index, rgb_value const & v)
{
   if ( index < num_leds){
      led_data[index ] = v;
      return true;
   }else{
      return false;
   }
}

void led_sequence::send()
{
    led_data_idx = 0U;
   // load the first 2 leds in the buffer
    led_sequence::refill(0U, led_data_idx);
    ++led_data_idx;
    led_sequence::refill(1U, led_data_idx);
    ++led_data_idx;
    led_seq_timer::get()->sr.set(0U);
    led_seq_timer::get()->ccr1 = 0U;
    led_seq_timer::get()->cnt = 0U;
    DMA1_Channel3->CNDTR = 2U * 8U * bytes_per_led;
      // start  dma
    DMA1_Channel3->CCR |= (0b1 << 0U); // (OE)
   // start timer
    led_seq_timer::get()->cr1.setbit<0>(); // (CEN)
}

void led_sequence::refill(uint32_t dma_buf_id, uint32_t data_idx)
{
   uint32_t const dma_idx = dma_buf_id * 8U * bytes_per_led;
   // green, red, blue
   auto const & led = led_data[data_idx];
   for ( uint8_t i = 0U; i < 8U * bytes_per_led; ++i){
      dma_buffer[dma_idx + i] = (((led.green << i) & 0x80) == 0U)?zero_pwm:one_pwm;
      dma_buffer[dma_idx + i + 8U] = (((led.red << i) & 0x80) == 0U)?zero_pwm:one_pwm;
      dma_buffer[dma_idx + i + 16U] = (((led.blue << i) & 0x80) == 0U)?zero_pwm:one_pwm;
   }
}

extern "C" void DMA1_Channel2_3_IRQHandler() 
{
   // half transfer complete
   if ( DMA1->ISR & (0b1 << 10U)/* (HTIF3) */ ){
      DMA1->IFCR = (0b1 << 10U);
      if (led_data_idx < (led_sequence::num_leds) ){
         led_sequence::refill(0U, led_data_idx);
         ++led_data_idx;
      }
   }
    // full transfer complete
   if ( DMA1->ISR & ( 0b1 << 9U)/* (TCIF3) */ ){
      DMA1->IFCR = ( 0b1 << 9U);
      if ( led_data_idx < led_sequence::num_leds){
         led_sequence::refill(1U, led_data_idx);
         ++led_data_idx;
      }else{
         DMA1_Channel3->CCR &= ~(0b1 << 0U); // (OE)
         static constexpr uint8_t update_interrupt_flag = 0U;
         led_seq_timer::get()->sr.clearbit<update_interrupt_flag>();
         led_seq_timer::get()->dier.setbit<update_interrupt_flag>();
      }
   }
}

extern "C" void  TIM16_IRQHandler()
{
    // disable update and  turn offf the timer 
   static constexpr uint8_t update_interrupt_flag = 0U;
   led_seq_timer::get()->sr.clearbit<update_interrupt_flag>(); //(UIF)
   led_seq_timer::get()->dier.clearbit<update_interrupt_flag>();
   led_seq_timer::get()->cr1.clearbit<0U>(); // (CEN)
}

