
#include <quan/fixed_quantity/literal.hpp>
#include <quan/stm32/millis.hpp>
#include "../../usarts.hpp"
#include "../../neopixel.hpp"

#include "light_show_examples.hpp"
#include "../../neopixel.hpp"


extern "C" void setup();

using quan::stm32::millis;

namespace{

  typedef  link_sp::serial_port xout;

  QUAN_QUANTITY_LITERAL(time,ms)


  typedef decltype(millis()) ms;

  void delay( ms const & t)
  {
      auto const now = millis();

      while ( (millis() - now ) < t){
         asm volatile ("nop":::);
      }
  }

}

int main()
{
   setup();

   xout::write("Led sequence Test\n");

   rgb_value red = {12,0,0};
   rgb_value blue = {0,0,12};
   rgb_value white = {4,4,4};
   rgb_value green = {0,12,0};

   for (;;){
      // this double sequence is just to test that reset occurs automatically
      // the red sequence should pass too fast to be visible. only the green should be seen.
      // If red is seen, then indicates that reset didnt occur ( the green data got passed out of the end of the string of neopixels)
      for ( uint8_t i = 0; i < 8; ++i){
         neopixel::put(i,red);
      }
      neopixel::send();
      for ( uint8_t i = 0; i < 8; ++i){
         neopixel::put(i,green);
      }
      neopixel::send();

      // should see all green here
      delay(1000_ms);

      walking_led(white, blue,250_ms, 8000_ms);
      blend(10000_ms);
      pulse(10000_ms);
      walking_led(green, blue,100_ms, 5000_ms);
      blend(10000_ms);
      walking_led(green, red,50_ms, 6000_ms);
      pulse(10000_ms);
      walking_led(red,blue,25_ms, 5000_ms);
      walking_led(red,white,50_ms, 4000_ms);
      blend(20000_ms);
      
  
      
   }
}
