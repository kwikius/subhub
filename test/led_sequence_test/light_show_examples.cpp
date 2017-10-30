
#include <stm32f0xx.h>
#include <quan/stm32/usart/irq_handler.hpp>
#include <quan/conversion/itoa.hpp>
#include <quan/stm32/millis.hpp>
#include "../../usarts.hpp"
#include "../../led_sequence.hpp"
#include "led.hpp"

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

void walking_led(
      rgb_value const & background_colour, 
      rgb_value const & walk_colour,
      ms const & delay_duration,
      ms const & demo_duration)
{

     uint8_t pos = 0;
     auto now = millis();
     while ( (millis() - now) < demo_duration){
        led_sequence::put(pos,background_colour);
        pos = (pos +1) % 8;
        led_sequence::put(pos,walk_colour);
        led_sequence::send();
        delay(delay_duration);
    }

}

/*

light displays
*/