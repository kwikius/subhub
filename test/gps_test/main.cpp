//#include <apm/gps.hpp>
#include <quan/stm32/millis.hpp>
#include <quan/conversion/itoa.hpp>
#include <quan/conversion/float_to_ascii.hpp>
#include <quan/stm32/usart/irq_handler.hpp>
#include <apm/serial_port.hpp>
#include "../../serial_port.hpp"
#include "../../usarts.hpp"

extern "C" void setup();

using quan::stm32::millis;

void passthrough();
void run_gps();

namespace {

   typedef decltype(millis()) ms;

   ms operator "" _ms(unsigned long long int v)
   {
      return static_cast<ms>(v);
   }

   typedef link_sp::serial_port xout;
   typedef link_sp::serial_port xin;

   //void do_state(apm::gps_t & gps);

   void flush_input()
   {
     while (xin::in_avail())
     {
        (void) xin::get(); 
     }
   }
}

void led_on();
void led_off();

int main()
{
   setup();

   xout::write("GPS test\n\n");

   flush_input();
   
   xout::write("options\n\n");
   xout::write("passthrough = \'p\'");
   xout::write("run gps     = \'r\'");
   while (!xin::in_avail()){;}

   for (;;){
      switch ( xin::get()){
         case 'P':
         case 'p':
            passthrough();
         break;
         case 'R':
         case 'r':
            run_gps();
         break;
         default:
            xout::write("invalid option\n");
         break;
      }
   }
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
