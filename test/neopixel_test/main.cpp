
#include <stm32f0xx.h>
#include <quan/stm32/usart/irq_handler.hpp>
#include <quan/conversion/itoa.hpp>
#include <quan/stm32/millis.hpp>
#include "../../usarts.hpp"
#include "../../neopixel.hpp"
#include "led.hpp"
#include "light_show_examples.hpp"

extern "C" void setup();

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

int main()
{
   setup();

   led::off();
   xout::write("Led sequence Test\n");

   rgb_value red = {12,0,0};
   rgb_value blue = {0,0,12};
   rgb_value white = {4,4,4};
   rgb_value green = {0,12,0};

   for (;;){
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

extern "C" void USART2_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void USART2_IRQHandler()
{
   static_assert(
      std::is_same<
         aux_sp::serial_port::usart_type,quan::stm32::usart2
      >::value
   ,"invalid usart for serial_port irq");

   quan::stm32::usart::irq_handler<aux_sp::serial_port>();
}

extern "C" void USART1_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void USART1_IRQHandler()
{
   static_assert(
      std::is_same<
         link_sp::serial_port::usart_type,quan::stm32::usart1
      >::value
   ,"invalid usart for serial_port irq");

   quan::stm32::usart::irq_handler<link_sp::serial_port>();
}

//volatile uint32_t quan::stm32::detail::systick_tick::current = 0;

extern "C" void SysTick_Handler()
{
   ++quan::stm32::detail::systick_tick::current;
   
}