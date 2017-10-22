
#include <stm32f0xx.h>
#include <quan/stm32/millis.hpp>
#include <quan/stm32/usart/irq_handler.hpp>
#include <quan/conversion/itoa.hpp>
#include "../../usarts.hpp"
#include "led.hpp"
#include "sh1106_oled.hpp"

/*
Test of I2C using a 24LC128 eeprom
 A led is set up on Servo 1 port for diagnostics
*/

extern "C" void setup();

using quan::stm32::millis;
namespace {

   typedef decltype(millis()) ms;

   ms operator "" _ms(unsigned long long int v)
   {
      return static_cast<ms>(v);
   }
   void delay(ms t)
   {
      ms elapsed = millis();
      while ((millis() - elapsed) < t) {;}
   }

   typedef link_sp::serial_port xout;

}



using quan::stm32::millis;

namespace {
   typedef link_sp::serial_port xout;
}

int main()
{
   setup();

   for (;;){
      sh1106_oled::set_buffer_to(0x0F);
      sh1106_oled::write_buffer();

      delay(500_ms);

      sh1106_oled::set_buffer_to(0xFF);
      sh1106_oled::write_buffer();

      delay(500_ms);

      sh1106_oled::set_buffer_to(0xF0);
      sh1106_oled::write_buffer();

      delay(500_ms);

      sh1106_oled::set_buffer_to(0x0);
      sh1106_oled::write_buffer();

      delay(500_ms);
   }

   xout::write("oled Test complete\n");
   while (1){;}

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
         xout::usart_type,quan::stm32::usart1
      >::value
   ,"invalid usart for serial_port irq");

   quan::stm32::usart::irq_handler<link_sp::serial_port>();
}

//volatile uint32_t quan::stm32::detail::systick_tick::current = 0;

extern "C" void SysTick_Handler()
{
   ++quan::stm32::detail::systick_tick::current;
   
}