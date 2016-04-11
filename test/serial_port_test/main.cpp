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

   aux_sp::serial_port::write("Hello World again\n");

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