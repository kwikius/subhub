extern "C" void setup();

#include "../../usarts.hpp"
#include <quan/stm32/usart/irq_handler.hpp>

/*
PA14 TXO
PA15 RXI
*/

int main()
{
   setup();

   aux_sp::serial_port::write("Hello Worldfrom aux sp\n");
   link_sp::serial_port::write("Hello World from link sp\n");

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
         link_sp::serial_port::usart_type,quan::stm32::usart1
      >::value
   ,"invalid usart for serial_port irq");

   quan::stm32::usart::irq_handler<link_sp::serial_port>();
}