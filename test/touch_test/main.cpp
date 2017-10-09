
#include <stm32f0xx.h>
#include <quan/stm32/usart/irq_handler.hpp>
#include <quan/conversion/itoa.hpp>
#include <quan/stm32/millis.hpp>
#include "../../usarts.hpp"
#include "../../touch.hpp"
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
}

int main()
{
   setup();

   xout::write("Touch Test\n");

   ms elapsed = millis();

   for(;;) {

      if ( !touch::start_conversion()){
         xout::write("start touch conv failed\n");
         break;
      }

      while (!touch::conversion_complete()){

         auto now1 = millis();
         if ( (now1 - elapsed) > 10_ms ) {
            elapsed = now1;
            xout::write("stalled, got touch count of ");
            uint32_t const n = touch::get_count();
            char buf[sizeof(uint32_t)*8 + 3];
            quan::itoasc(n,buf,10);
            xout::write(buf);
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
         if ( n < 215){
            led::on();
         }else{
            led::off();
         }
      }else{
         xout::write("touch conv failed\n");
      }

      while (!xout::tx_reg_empty()){
        asm volatile ("nop":::);
      }
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