
#include <stm32f0xx.h>
#include <quan/stm32/millis.hpp>
#include "../../usarts.hpp"
#include <quan/stm32/usart/irq_handler.hpp>
#include <quan/conversion/itoa.hpp>
#include "led.hpp"
#include <quan/stm32/millis.hpp>

extern "C" void setup();

void eeprom_tx_test();
void eeprom_rx_test();

using quan::stm32::millis;

int main()
{
   setup();

  // led::on();

   link_sp::serial_port::write("i2c Test\n");

// Need to wait a short time after startup for eeprom to get powered up.
   auto now = millis();
   typedef decltype (now) ms;
   while ( (millis() - now) < ms{500} ){;}

   eeprom_tx_test();

   eeprom_rx_test();

   link_sp::serial_port::write("i2c Test done\n");
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

volatile uint32_t quan::stm32::detail::systick_tick::current = 0;

extern "C" void SysTick_Handler()
{
   ++quan::stm32::detail::systick_tick::current;
   
}