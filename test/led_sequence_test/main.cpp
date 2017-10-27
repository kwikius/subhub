
#include <stm32f0xx.h>
#include <quan/stm32/usart/irq_handler.hpp>
#include <quan/conversion/itoa.hpp>
#include <quan/stm32/millis.hpp>
#include "../../usarts.hpp"
#include "../../led_sequence.hpp"
#include "led.hpp"

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

   xout::write("Led sequence Test\n");

   led_sequence::put(0U,{128,0,0});
   led_sequence::put(1U,{0,128,0});
   led_sequence::put(3U,{0,0,128});
   led_sequence::put(5U,{100,100,100});

   led_sequence::send();

   for (uint8_t i = 0U; i < 8; ++i){
     led_sequence::put(i,{0,0,0});
   }
   
   uint8_t pos = 7;
   for ( ;;){
     led_sequence::put(pos,{0,0,0});
     pos = (pos + 1) % 8U;
     led_sequence::put(pos,{0,64,0});
     delay (500_ms);
     led_sequence::send();
   }

   xout::write("Led sequence test completed\n");

   for(;;){
      asm volatile ("nop":::);
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