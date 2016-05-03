
#include <stm32f0xx.h>
#include <quan/stm32/millis.hpp>
#include "../../usarts.hpp"
#include <quan/stm32/usart/irq_handler.hpp>
#include <quan/conversion/itoa.hpp>
#include "../../rc_in.hpp"

extern "C" void setup();

using quan::stm32::millis;

namespace {

   typedef link_sp::serial_port xout;

   // output a servo channel
   void output_channel(uint8_t channel)
   {
      xout::write("ch[");
      char buf[20];
      quan::itoasc(channel,buf,10);
      xout::write(buf);
      xout::write("] = ");
      quan::itoasc(rc_inputs::get_channel(channel),buf,10);
      xout::write(buf);
   }

   typedef decltype(millis()) ms;
   ms operator "" _ms(unsigned long long int v)
   {
      return static_cast<ms>(v);
   }

} // ~namespace

int main()
{
   setup();

   xout::write("RC Input Test\n");

   ms elapsed = millis();

   for(;;) {
      auto now = millis();
      if((now - elapsed) > 200_ms && rc_inputs::have_new_input()) {
         elapsed = now;
         auto const nchan = rc_inputs::get_num_channels();
         for(uint8_t chan = 0; chan < nchan; ++chan) {
            if(chan > 0) {
               xout::write(" ,");
            }
            output_channel(chan);
         }
         xout::put('\n');
      }
   }
}

extern "C" void USART2_IRQHandler() __attribute__((interrupt("IRQ")));
extern "C" void USART2_IRQHandler()
{
   static_assert(
      std::is_same<
      aux_sp::serial_port::usart_type,quan::stm32::usart2
      >::value
      ,"invalid usart for serial_port irq");

   quan::stm32::usart::irq_handler<aux_sp::serial_port>();
}

extern "C" void USART1_IRQHandler() __attribute__((interrupt("IRQ")));
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