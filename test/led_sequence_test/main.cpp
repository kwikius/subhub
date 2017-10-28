
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

//   for ( uint8_t i = 0U; i < 8U;++i){
//     led_sequence::put(i,{0x8,0x1F,0x0});
//   }


 #if 1
//   uint32_t c = 0;
//   
   rgb_value colours [] = {{12,0,0},{12,0,0},{0,6,6},{0,6,6},{0,0,12},{0,0,12},{0,12,0},{0,12,0}};
   for (uint8_t i = 0; i < 8; ++i){
 
       led_sequence::put(i,colours[i]);
   }
   for ( uint8_t i = 0; ; i = ( i+1) % 0x10000){
      delay (50_ms);
       for (uint8_t j = 0; j < 8; ++j){
         led_sequence::put(j,colours[(i + j)%8]);
       }
      led_sequence::send();
   }

  #else

   
   uint32_t pos = 0;
   for ( ;;){
     
        led_sequence::put((pos) % 8U,{0U,0U,0U});
        led_sequence::put((pos + 1) % 8U,{0xf,0xf,0xf});
     pos = (pos + 1U) % 8U;
     delay (500_ms);
     led_sequence::send();
   }
#endif

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