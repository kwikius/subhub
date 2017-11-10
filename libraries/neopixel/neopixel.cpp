
/*
Copyright (C) 2017 Andy Little
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
#include <quan/stm32/millis.hpp>
#include "../../neopixel.hpp"
#include "../../usarts.hpp"
#include "../../resources.hpp"


/*

 use TIM16_CH1  
choose pin as neo_pixel_pin 
     PA6 AF5
 or  PB8 AF2 
*/

uint8_t neopixel::dma_buffer[ 8U * bytes_per_led * 2U] = {0U};
rgb_value neopixel::user_led_data [num_leds];
rgb_value neopixel::dma_led_data [num_leds];

using quan::stm32::millis;

namespace{

   typedef  link_sp::serial_port xout;

   typedef decltype(millis()) ms;
   ms operator "" _ms(unsigned long long int v)
   {
      return static_cast<ms>(v);
   }

   void delay(ms const & t)
   {
      auto const now = millis();
      while ( (millis() - now ) < t){
        asm volatile ("nop":::);
      }
   }

}

namespace {
#if defined (NEOPIXEL_USE_TIM2)
   typedef quan::stm32::tim2 led_seq_timer;
#else
   typedef quan::stm32::tim16 led_seq_timer;
#endif
 
   constexpr uint32_t raw_timer_freq = quan::stm32::get_raw_timer_frequency<led_seq_timer>();
   static_assert(raw_timer_freq == 48000000,"unexpected raw freq");
   constexpr uint32_t reqd_freq = 800000U; // 800 kHz

   constexpr uint32_t arr_value = (raw_timer_freq / reqd_freq) ;

   static constexpr uint32_t zero_pwm = (arr_value) / 3U -1U;
   static constexpr uint32_t one_pwm  = (2 * arr_value) / 3U -1U;

   static constexpr uint32_t reset_period_us = 50U;

   static constexpr float timer_period_us = 1000000.f / reqd_freq;
   
   static constexpr uint16_t reset_prescaler = static_cast<uint16_t>( reset_period_us / timer_period_us) + 1U;
   
}

void neopixel::initialise()
{
   // turn on pwm timer in rcc
   // turn on dma in rcc
   constexpr uint8_t dma_rcc_bit = 0U;
   quan::stm32::rcc::get()->ahbenr.setbit<dma_rcc_bit>();

   quan::stm32::module_enable<led_seq_timer>();

   // set up pin as af2
   quan::stm32::module_enable<neopixel_pin::port_type>();
   quan::stm32::apply<
      neopixel_pin,
      quan::stm32::gpio::mode::af2,  // ok for both pin options
      quan::stm32::gpio::pupd::pull_down,
      quan::stm32::gpio::ospeed::medium_fast
   >();
   
   led_seq_timer::get()->psc = 0;
   led_seq_timer::get()->arr = arr_value - 1U;

   {
      // enable preload
      quan::stm32::tim::cr1_t cr1 = 0;

      cr1.arpe = true;

      led_seq_timer::get()->cr1.set(cr1.value);
   }

   {
      quan::stm32::tim::cr2_t cr2 = 0;
#if !defined (NEOPIXEL_USE_TIM2)
      cr2.ois1 = false; //inactive state of output
#endif
      cr2.ccds = false; // dma from compare not update

      led_seq_timer::get()->cr1.set(cr2.value);
   }


   {
      // set timer to pwm mode
      quan::stm32::tim::ccmr1_t ccmr1 = 0;

#if !defined (NEOPIXEL_USE_TIM2)
      ccmr1.cc1s = 0b00;  // output
      ccmr1.oc1m = 0b110; // PWM mode 1
      ccmr1.oc1pe = true; // want preload
#else
      ccmr1.cc2s = 0b00;  // output
      ccmr1.oc2m = 0b110; // PWM mode 1
      ccmr1.oc2pe = true; // want preload
#endif

      led_seq_timer::get()->ccmr1.set(ccmr1.value);
   }


#if !defined (NEOPIXEL_USE_TIM2)
   {
      quan::stm32::tim::bdtr_t bdtr = 0;

      bdtr.moe = true;
      bdtr.ossr = false;
      bdtr.ossi = false;

      led_seq_timer::get()->bdtr.set(bdtr.value);
   }
#endif

    // ccer 
   {
      quan::stm32::tim::ccer_t ccer = 0;
#if !defined (NEOPIXEL_USE_TIM2)
      ccer.cc1e = true;
      ccer.cc1p = false;
      ccer.cc1ne = false;
#else
      ccer.cc2e = true;
      ccer.cc2p = false;
#endif

      led_seq_timer::get()->ccer.set(ccer.value);
   }

   // dma setup;
   // set up timer dma on cc1 compare
   {
      quan::stm32::tim::dier_t dier = 0;
#if !defined (NEOPIXEL_USE_TIM2)
      dier.cc1de = true;
#else
      dier.cc2de = true;
#endif
      led_seq_timer::get()->dier.set(dier.value);
   }

   NVIC_SetPriority(DMA1_Channel2_3_IRQn,interrupt_priority::neopixel);
   NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

#if !defined (NEOPIXEL_USE_TIM2)
   NVIC_SetPriority(TIM16_IRQn,interrupt_priority::neopixel);
   NVIC_EnableIRQ(TIM16_IRQn);
#else
   NVIC_SetPriority(TIM2_IRQn,interrupt_priority::neopixel);
   NVIC_EnableIRQ(TIM2_IRQn);
#endif
   
   // memory 8 bit default
   DMA1_Channel3->CCR  =
     (0b1 << 4U)      // (DIR) read from memory 
    | (0b1 << 7U)     // (MINC)
#if !defined (NEOPIXEL_USE_TIM2)
    | (0b01 << 8U)    // (PSIZE[0:1])  16 bit
#else
    | (0b10 << 8U)    // (PSIZE[0:1])  32 bit
#endif
    | (0b1 << 5U)     // (CIRC)
    | (0b11 << 12U)   // highest priority
    | (0b1 << 2U)     // (HTIE)
    | (0b1 << 1U)     // (TCIE)
   ;

#if !defined (NEOPIXEL_USE_TIM2)
   DMA1_Channel3->CPAR = (uint32_t)&led_seq_timer::get()->ccr1;
#else
   DMA1_Channel3->CPAR = (uint32_t)&led_seq_timer::get()->ccr2;
#endif
   DMA1_Channel3->CMAR = (uint32_t)neopixel::dma_buffer;

#if !defined (NEOPIXEL_USE_TIM2)
   led_seq_timer::get()->ccr1 = 0U;
#else
   led_seq_timer::get()->ccr2 = 0U;
#endif

   led_seq_timer::get()->cr1.setbit<0>(); // (CEN)
}

namespace {
   volatile uint32_t led_data_idx = 0U;
   volatile bool in_progress = false;
   volatile bool in_reset_timing = false;
}

bool neopixel::put(uint32_t index, rgb_value const & v)
{
   if ( index < num_leds){
      user_led_data[index] = v;
      return true;
   }else{
      return false;
   }
}

/*
Todo
  at the end of the send add a reset period, using the timer
  during that time in_progress returns true.
Otherwise acts as if a much longer chain
*/

void neopixel::send()
{
    while (in_progress == true){
      asm volatile ("nop":::);
    }

    copy_user_to_dma();

    led_data_idx = 0U;
    auto * const timer = led_seq_timer::get();

   // load the first 2 leds in the buffer
    neopixel::refill(0U, led_data_idx++);
    neopixel::refill(1U, led_data_idx++);

    timer->cnt = one_pwm;
    timer->sr.set(0U);

    DMA1_Channel3->CNDTR = 2U * 8U * bytes_per_led;
    // force cc1 event
#if !defined (NEOPIXEL_USE_TIM2)
    timer->egr.setbit<1>(); // (CC1G)
#else
    timer->egr.setbit<2>(); // (CC2G)
#endif
    // start  dma
    DMA1_Channel3->CCR |= (0b1 << 0U); // (OE)
    while ((DMA1_Channel3->CCR & (0b1 << 0U)) == 0U){
      asm volatile ("nop":::);
    }

    in_progress = true;

}

void neopixel::copy_user_to_dma()
{
  for (uint32_t i = 0U; i < num_leds; ++i){
     dma_led_data[i] = user_led_data[i];
  }
}

/*
since this is called in an interrupt try to make it as fast as possible
*/
inline void neopixel::refill(uint32_t dma_buf_id, uint32_t data_idx)
{
   uint32_t const dma_idx = dma_buf_id * 8U * bytes_per_led;
   // green, red, blue
   auto const & led = dma_led_data[data_idx];

   uint8_t* ptr = dma_buffer + dma_idx;

   uint8_t colour = led.green;
   *ptr = ((colour & 0x80) == 0U)?zero_pwm:one_pwm;
   *++ptr = ((colour & 0x40) == 0U)?zero_pwm:one_pwm;
   *++ptr = ((colour & 0x20) == 0U)?zero_pwm:one_pwm;
   *++ptr = ((colour & 0x10) == 0U)?zero_pwm:one_pwm;
   *++ptr = ((colour & 0x08) == 0U)?zero_pwm:one_pwm;
   *++ptr = ((colour & 0x04) == 0U)?zero_pwm:one_pwm;
   *++ptr = ((colour & 0x02) == 0U)?zero_pwm:one_pwm;
   *++ptr = ((colour & 0x01) == 0U)?zero_pwm:one_pwm;
   
   colour = led.red;
   *++ptr = ((colour & 0x80) == 0U)?zero_pwm:one_pwm;
   *++ptr = ((colour & 0x40) == 0U)?zero_pwm:one_pwm;
   *++ptr = ((colour & 0x20) == 0U)?zero_pwm:one_pwm;
   *++ptr = ((colour & 0x10) == 0U)?zero_pwm:one_pwm;
   *++ptr = ((colour & 0x08) == 0U)?zero_pwm:one_pwm;
   *++ptr = ((colour & 0x04) == 0U)?zero_pwm:one_pwm;
   *++ptr = ((colour & 0x02) == 0U)?zero_pwm:one_pwm;
   *++ptr = ((colour & 0x01) == 0U)?zero_pwm:one_pwm;

   colour = led.blue;
   *++ptr = ((colour & 0x80) == 0U)?zero_pwm:one_pwm;;
   *++ptr = ((colour & 0x40) == 0U)?zero_pwm:one_pwm;
   *++ptr = ((colour & 0x20) == 0U)?zero_pwm:one_pwm;
   *++ptr = ((colour & 0x10) == 0U)?zero_pwm:one_pwm;
   *++ptr = ((colour & 0x08) == 0U)?zero_pwm:one_pwm;
   *++ptr = ((colour & 0x04) == 0U)?zero_pwm:one_pwm;
   *++ptr = ((colour & 0x02) == 0U)?zero_pwm:one_pwm;
   *++ptr = ((colour & 0x01) == 0U)?zero_pwm:one_pwm;
}

/*
 called every 30 usec when transmitting
 Needs high prio else wrong bit value can be sent
 Total calls =  num_leds 
*/
extern "C" void DMA1_Channel2_3_IRQHandler() 
{
   // half transfer complete
   if ( DMA1->ISR & (0b1 << 10U)/* (HTIF3) */ ){
      DMA1->IFCR = (0b1 << 10U);
      if (led_data_idx < (neopixel::num_leds) ){
         neopixel::refill(0U, led_data_idx++);
      }
   }
    // full transfer complete
   if ( DMA1->ISR & ( 0b1 << 9U)/* (TCIF3) */ ){
      DMA1->IFCR = ( 0b1 << 9U);
      if ( led_data_idx < neopixel::num_leds){
         neopixel::refill(1U, led_data_idx++);
      }else{
         DMA1_Channel3->CCR &= ~(0b1 << 0U); // (OE)
#if !defined (NEOPIXEL_USE_TIM2)
         // catch the next cc1 interrupt to stop the process
         static constexpr uint8_t cc_interrupt_flag = 1U;
#else
         static constexpr uint8_t cc_interrupt_flag = 2U;
#endif
         // catch the next cc interrupt to stop the process
         led_seq_timer::get()->sr.clearbit<cc_interrupt_flag>();
         led_seq_timer::get()->dier.setbit<cc_interrupt_flag>();
      }
   }
}

/*
 called 3x at end of transmission
*/
#if !defined (NEOPIXEL_USE_TIM2)
extern "C" void  TIM16_IRQHandler()
#else
extern "C" void  TIM2_IRQHandler()
#endif
{
#if !defined (NEOPIXEL_USE_TIM2)
   static constexpr uint8_t cc_interrupt_flag = 1U;
#else
   static constexpr uint8_t cc_interrupt_flag = 2U;
#endif
   static constexpr uint8_t update_interrupt_flag = 0U;
   auto * const timer = led_seq_timer::get();
   if (timer->dier.getbit<cc_interrupt_flag>()){
#if !defined (NEOPIXEL_USE_TIM2)
      timer->ccr1 = 0U;
#else
      timer->ccr2 = 0U;
#endif
      timer->psc = reset_prescaler - 1U;
      timer->dier.clearbit<cc_interrupt_flag>();
      timer->sr = 0U;
      timer->dier.setbit<update_interrupt_flag>();
   }else{
      // 1st update interrupt
      if ( in_reset_timing == false){
         in_reset_timing = true;
         timer->psc = 0U;
         timer->sr = 0U;
      }else{
         // 2nd update interrupt
         timer->dier.clearbit<update_interrupt_flag>(); // (
         timer->sr = 0U;
         // signal to main process that send sequence is complete
         in_reset_timing = false;
         in_progress = false;
      }
   }
}

