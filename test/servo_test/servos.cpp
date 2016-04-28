
#include <quan/stm32/tim.hpp>
#include <quan/stm32/gpio.hpp>
#include <quan/stm32/tim/temp_reg.hpp>
#include <quan/stm32/get_raw_timer_frequency.hpp>
#include <quan/constrain.hpp>
#include <quan/min.hpp>
#include "../../servos.hpp"
/*
pwm servos on TIM2
 Servo1 on PA5 TIM2_CH1
Servo2 on PA1   TIM2_CH2
*/

namespace {

   typedef quan::stm32::tim2 rc_out_timer;
   constexpr uint16_t max_pulsewidth = 2250U;
   constexpr uint16_t min_pulsewidth = 750U;

   
   typedef quan::mcu::pin<quan::stm32::gpioa,5> rc_out_ch1;
   typedef quan::mcu::pin<quan::stm32::gpioa,1> rc_out_ch2;

   constexpr uint32_t timer_freq = quan::stm32::get_raw_timer_frequency<rc_out_timer>();

   static_assert(timer_freq == 48000000, "unexpected timer freq");
   void enable_pins()
   {
       quan::stm32::apply<
         rc_out_ch1,
         quan::stm32::gpio::mode::af2,  
         quan::stm32::gpio::pupd::pull_down
      >();

      quan::stm32::apply<
         rc_out_ch2,
         quan::stm32::gpio::mode::af2,  
         quan::stm32::gpio::pupd::pull_down
      >();
   }

   void start_timer()
   {
      rc_out_timer::get()->cr1.setbit<0>(); // (CEN)
   }
}

void servo_t::enable()
{
   rc_out_timer::get()->ccer |= (1 << (m_chan * 4));
}

void servo_t::set(uint16_t pulsewidth_in_us)
{
   volatile uint32_t * ccrs = &rc_out_timer::get()->ccr1;
   ccrs[m_chan] = quan::constrain(pulsewidth_in_us,min_pulsewidth, max_pulsewidth);
}

uint16_t servo_t::get()const
{
   volatile uint32_t * ccrs = &rc_out_timer::get()->ccr1;
   return ccrs[m_chan];
}

uint16_t servo_t::min(){return min_pulsewidth;}
uint16_t servo_t::max(){return max_pulsewidth;}

void servo_setup()
{
   quan::stm32::module_enable<rc_out_timer>();

   enable_pins();

    {
         // set ch[1:2] to PWM mode 1
         quan::stm32::tim::ccmr1_t ccmr1 = 0;
         ccmr1.cc1s = 0b00;  // output
         ccmr1.oc1m = 0b110; // PWM mode 1
         ccmr1.cc2s = 0b00;  // output
         ccmr1.oc2m = 0b110; // PWM mode 1
         rc_out_timer::get()->ccmr1.set(ccmr1.value);
      }
//      {
//         quan::stm32::tim::ccmr2_t ccmr2 = 0;
//         ccmr2.cc3s = 0b00;  // output
//         ccmr2.oc3m = 0b110; // PWM mode 1
//         ccmr2.cc4s = 0b00;  // output
//         ccmr2.oc4m = 0b110; // PWM mode 1
//         rc_out_timer::get()->ccmr2.set(ccmr2.value);
//      }
      {
         // default disabled
         quan::stm32::tim::ccer_t ccer = 0;
//         ccer.cc1p =  false;
//         ccer.cc1np = false;
//         ccer.cc1e =  false; // disable ch1 default?
//         ccer.cc2p =  false;
//         ccer.cc2np = false;
//         ccer.cc2e =  false; // disable ch2
//         ccer.cc3p =  false;
//         ccer.cc3np = false;
//         ccer.cc3e =  false; // disable ch3
//         ccer.cc4p =  false;
//         ccer.cc4np = false;
//         ccer.cc4e =  false; // disable ch4
         rc_out_timer::get()->ccer.set(ccer.value);
      }

      rc_out_timer::get()->ccr1 = 1500;
      rc_out_timer::get()->ccr2 = 1500;

      // set the ocpe (preload) bits in ccmr1  
      rc_out_timer::get()->ccmr1 |= ((1 << 3) | (1 << 11));

      rc_out_timer::get()->psc = (timer_freq / 1000000 )-1;
      rc_out_timer::get()->arr = 20000 -1;
      rc_out_timer::get()->cnt = 0x0;
      {
         quan::stm32::tim::cr1_t cr1 = 0;
         cr1.arpe = true ;// auto preload
         rc_out_timer::get()->cr1.set(cr1.value);
      }
      rc_out_timer::get()->sr = 0;

      start_timer();

     
}


