
#include <stm32f0xx.h>
#include <quan/stm32/usart/irq_handler.hpp>
#include <quan/conversion/itoa.hpp>
#include <quan/constrain.hpp>
#include <quan/stm32/millis.hpp>
#include "../../usarts.hpp"
#include "../../touch.hpp"
#include "../../neopixel.hpp"
#include "../../led.hpp"
#include "../../delay.hpp"

extern "C" void setup();
void set_pwm(uint32_t val);

using quan::stm32::millis;

namespace{

   typedef  link_sp::serial_port xout;

   typedef decltype(millis()) ms;
   ms operator "" _ms(unsigned long long int v)
   {
      return static_cast<ms>(v);
   }
   constexpr auto count_on_threshold = 125U;

   constexpr auto count_idle = 176.f;
   constexpr auto count_saturated = 76.f;
   constexpr auto count_diff = count_idle - count_saturated;

   void neopixels_off(uint32_t n)
   {

      rgb_value off_value{4,0,4};
      for (uint8_t i = 0; i < neopixel::num_leds;++i){
         neopixel::put(i,off_value);
      }
      neopixel::send();
   }

   void neopixels_on()
   {
      rgb_value on_value{0,20,0};
      for (uint8_t i = 0; i < neopixel::num_leds;++i){
         neopixel::put(i,on_value);
      }
      neopixel::send();
   }

   void do_pwm(uint32_t n)
   {
      if ( n < count_idle){
         float v = (count_idle - n) * 100U / count_diff;
         set_pwm (static_cast<uint32_t>(v));
      }else{
         set_pwm(0U);
      }
   }
   
   template <typename IntType>
   void xout_write( IntType v)
   {
      char buf[sizeof(IntType)*8 + 3];
      quan::itoasc(v,buf,10);
      xout::write(buf);
   }
}

int main()
{
   setup();
   xout::write("Touch Test\n");
   delay(100_ms);

   xout::write("Touch Test 1\n");
   auto last_out = millis();

   for(;;) {
      delay(10_ms);
      if ( !touch::start_conversion()){
         xout::write("start touch conv failed\n");
         break;
      }
      auto conv_start_time = millis();
      
      while (!touch::conversion_complete()){

         if ( (millis() - conv_start_time ) > 100_ms ){
            xout::write("stalled, got touch count of ");
            uint32_t const n = touch::get_count();
            xout_write(n);
            xout::write("\n");
            if ( touch::timeout()){
               xout::write("touch timed out\n");
               break;
            }
            break;
         }
      }
      // conv completed
      if ( touch::conversion_good()){
         
         uint32_t const n = touch::get_count();
         do_pwm(n);
         if ( n < count_on_threshold){
            neopixels_on();
            led::on();
         }else{
            neopixels_off(n);
            led::off();
         }
         auto now = millis();
         if ( (now - last_out) > 50_ms){
             last_out = now;
             xout::write("count = ");
             xout_write(n);
             xout::write("\n");
         }
      }else{
         xout::write("touch conv failed\n");
      }
      xout::flush_tx();
   }

}
