
#include <stm32f0xx.h>
#include <quan/stm32/usart/irq_handler.hpp>
#include <quan/conversion/itoa.hpp>
#include <quan/stm32/millis.hpp>
#include "../../usarts.hpp"
#include "../../touch.hpp"
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

   xout::write("Touch Test\n");
   auto last_out = millis();

   for(;;) {

      if ( !touch::start_conversion()){
         xout::write("start touch conv failed\n");
         break;
      }
      auto const start_time = millis();
      while (!touch::conversion_complete()){
         if ( (millis() - start_time) > 10_ms ) {
            xout::write("stalled, got touch count of ");
            uint32_t const n = touch::get_count();
            char buf[sizeof(uint32_t)*8 + 3];
            quan::itoasc(n,buf,10);
            xout::write(buf);
            xout::write("\n");
            if ( touch::timeout()){
               xout::write("touch timed out\n");
            }
            break;
         }
      }
      // conv completed
      if ( touch::conversion_good()){
         
         uint32_t const n = touch::get_count();
         do_pwm(n);
         if ( n < count_on_threshold){
            led::on();
         }else{
            led::off();
         }
         auto now = millis();
         if ( (now - last_out) > 100_ms){
             last_out = now;
             xout::printf<100>("count = %u\n",n);
             
         }
      }else{
         xout::write("touch conv failed\n");
      }
      xout::flush_tx();
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