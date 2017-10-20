

#include <quan/stm32/tim.hpp>
#include <quan/stm32/gpio.hpp>
#include <quan/stm32/tim/temp_reg.hpp>
#include <quan/stm32/get_raw_timer_frequency.hpp>
#include <quan/constrain.hpp>
#include <quan/min.hpp>

#include "../../resources.hpp"

namespace {

//tim1 ch4  is pwm
// n.b same as servo timer 

 typedef quan::stm32::tim2 led_timer;
 constexpr uint32_t timer_freq = quan::stm32::get_raw_timer_frequency<led_timer>();

}

// value of 0 to 100
void set_pwm(uint32_t val)
{
   led_timer::get()->ccr4 = quan::constrain(val * 100UL,0UL,9999UL);
}

void setup_pwm()
{
   quan::stm32::module_enable<led_timer>();

   quan::stm32::module_enable<led_pwm_pin::port_type>();
   quan::stm32::apply<
      led_pwm_pin
      ,quan::stm32::gpio::mode::af2
      , quan::stm32::gpio::otype::open_drain
      , quan::stm32::gpio::pupd::none
      , quan::stm32::gpio::ospeed::slow
     // , quan::stm32::gpio::ostate::high
   >();
#if 1
   {
      // set ch4 to pwm mode
      quan::stm32::tim::ccmr2_t ccmr2 = 0;
      ccmr2.cc4s = 0b00;  // output
      ccmr2.oc4m = 0b110; // PWM mode 1
      
      led_timer::get()->ccmr2.set(ccmr2.value);
   }

   // set the ocpe (preload) bits in ccmr1  
   led_timer::get()->ccmr2 |= (1 << 11);

   // 1 usec tick
   led_timer::get()->psc = (timer_freq / 1000000 )-1;
   led_timer::get()->arr = 10000 -1;
   led_timer::get()->cnt = 0x0;
   {
      quan::stm32::tim::cr1_t cr1 = 0;
      cr1.arpe = true ;// auto preload
      led_timer::get()->cr1.set(cr1.value);
   }

   // output polarity
   led_timer::get()->ccr4 = 0;
   led_timer::get()->ccer |= (1 << 13); // (CC4P)
   led_timer::get()->ccer |= (1 << 12); // (CC4E)
   led_timer::get()->sr = 0;
   led_timer::get()->cr1.setbit<0>(); // (CEN)
#endif
}




