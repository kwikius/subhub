
#include <stm32f0xx.h>
#include <quan/stm32/usart/irq_handler.hpp>
#include <quan/conversion/itoa.hpp>
#include <quan/stm32/millis.hpp>
#include "../../usarts.hpp"
#include "../../led_sequence.hpp"
#include "led.hpp"

extern "C" void setup();
void set_pwm(uint32_t val);

using quan::stm32::millis;

namespace{

   typedef  link_sp::serial_port xout;

   typedef decltype(millis()) ms;
   ms operator "" _ms(unsigned long long int v)
   {
      return static_cast<ms>(v);
   }
   constexpr auto count_on_threshold = 210U;

   constexpr auto count_idle = 210.f;
   constexpr auto count_saturated = 145.f;
   constexpr auto count_diff = count_idle - count_saturated;

   void do_pwm(uint32_t n)
   {
      if ( n < count_idle){
         float v = (count_idle - n) * 100U / count_diff;
         set_pwm (static_cast<uint32_t>(v));
      }else{
         set_pwm(0U);
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

//   int const bytes_left = led_sequence::transfer_bytes_left();
//   xout::printf<100>("DMA start bytes = %d\n" ,bytes_left);

//   auto now = quan::stm32::millis();
//
//   while( led_sequence::transfer_bytes_left() > 0U){
//      asm volatile ("nop":::);
//      if ( (millis() - now) > 100_ms){
//        xout::printf<100>("Dma failed, DMA bytes left = %d\n" ,bytes_left); 
//        break;
//      }
//   }

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