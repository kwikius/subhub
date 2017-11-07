
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
   constexpr ms operator "" _ms(unsigned long long int v)
   {
      return static_cast<ms>(v);
   }
   constexpr auto count_on_threshold = 150U;
   constexpr auto count_off_threshold = 160U;

   constexpr auto count_idle = 176.f;
   constexpr auto count_saturated = 76.f;
   constexpr auto count_diff = count_idle - count_saturated;

   //constexpr rgb_value red = {255,0,0};
   constexpr rgb_value off_colour{12,2,2};

   constexpr rgb_value on_colour{2,12,2};
   constexpr rgb_value on_colour1 = {2,2,12};
   constexpr rgb_value on_colour2 = {12,2,2};
   

   void neopixels_off()
   {

      
      for (uint8_t i = 0; i < neopixel::num_leds;++i){
         neopixel::put(i,off_colour);
      }
      neopixel::send();
   }

   void neopixels_on()
   {

      for (uint8_t i = 0; i < neopixel::num_leds;++i){
         neopixel::put(i,on_colour);
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
   bool switch_on = false;

   void walking_led(
         rgb_value const & background_colour, 
         rgb_value const & walk_colour1,
         rgb_value const & walk_colour2,
         ms const & delay_duration,
         ms const & demo_duration)
   {

      uint8_t pos = 0;
      auto now = millis();
      while ( (millis() - now) < demo_duration){
  
           neopixel::put((pos +0) % 8,walk_colour1);
           neopixel::put((pos +1) % 8,walk_colour2);
           neopixel::put((pos +2) % 8,background_colour);
           neopixel::put((pos +3) % 8,walk_colour1);
           neopixel::put((pos +4) % 8,walk_colour2);
           neopixel::put((pos +5) % 8,background_colour);
           neopixel::put((pos +6) % 8,walk_colour1);
           neopixel::put((pos +7) % 8,walk_colour2);
//           neopixel::put((pos +5) % 8,background_colour);
//           neopixel::put((pos +6) % 8,walk_colour);
//           neopixel::put((pos +7) % 8,background_colour);
           neopixel::send();
           pos = (pos +1) % 8;
           delay(delay_duration);
      }
   }

}

void momentary();
void simple();
void toggle();

int main()
{
   setup();
   xout::write("Touch Test\n");
   delay(100_ms);
   neopixels_off();
   set_pwm(0U);

   momentary();
  // simple();
   //toggle();
}

namespace {
 bool toggled_on = false;
}

void toggle()
{
   xout::write("Toggle mode\n");

   auto last_out = millis();
   ms on_start = 0_ms;

   for(;;) {
      delay(5_ms);
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

         if ( n > count_off_threshold){
               if ( toggled_on){
                  neopixels_off();
                  led::off();
                  toggled_on = false;
                  set_pwm(40);
                  delay(100_ms);
                  set_pwm(0);
               }
         }else{
            if (toggled_on == false){
               neopixels_on();
               led::on();
               toggled_on = true;
               set_pwm(40);
               delay(120_ms);
               set_pwm(0);
               
            }
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


void simple()
{
   xout::write("Momentary mode\n");

   auto last_out = millis();
   ms on_start = 0_ms;

   for(;;) {
      delay(5_ms);
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

         if ( n > count_off_threshold){
               neopixels_off();
               led::off();
               set_pwm(0);
         }else{
            set_pwm(50);
            neopixels_on();
            led::on();
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

void momentary()
{
   xout::write("Momentary mode\n");

   auto last_out = millis();
   ms on_start = 0_ms;

   for(;;) {
      delay(5_ms);
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
         //do_pwm(n);
         if ( switch_on == false){
            if ( n < count_on_threshold){
               on_start = millis();
               led::on();
               switch_on = true;
               auto period = 120_ms;
               auto dt = 5_ms;
               int32_t constexpr max_pwm = 80;
               auto n_steps = period / dt;
               uint32_t decay_decr = max_pwm / n_steps;
               
               for ( int32_t i = max_pwm; i >= 0; i -= decay_decr){
                  set_pwm(static_cast<uint32_t>(i));
                  delay(dt);
               }
               set_pwm(0);
               walking_led(on_colour,on_colour1,on_colour2,70_ms, 560_ms);
            }
         } else{ // switch is on
            if ( n > count_off_threshold){
               if ( (millis() - on_start) > 200_ms){
                  neopixels_off();
                  led::off();
                  switch_on = false;
                  set_pwm(0);
               }
            }else{
               neopixels_on();
               set_pwm(50);
            }
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
