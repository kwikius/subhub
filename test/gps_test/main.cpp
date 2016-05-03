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

namespace {

   typedef decltype(millis()) ms;

   ms operator "" _ms(unsigned long long int v)
   {
      return static_cast<ms>(v);
   }

   typedef link_sp::serial_port xout;

   //void do_state(apm::gps_t & gps);

}

void led_on();
void led_off();

int main()
{
   setup();

//   apm::gps_t gps;
//   gps.initialise(get_gps_sp()); // just attaches the port

   // want to see what if any GPS we have 

   xout::write("GPS test\n\n");

   apm::abc_serial_port & sp = get_gps_sp();

   sp.begin(38400);

   auto elapsed = millis();

   led_on();
   while ( (millis() - elapsed) < 1000_ms){;}
   led_off();
   elapsed = millis();
   while ( (millis() - elapsed) < 1000_ms){;}


   elapsed = millis();
   for (;;){
     if ( sp.available()){
         xout::put(sp.read());
     }
   }
}

#if 0
namespace {

   void output( float v )
   {
      char buf[50];
      quan::float_to_ascii<7>(v,buf);
      xout::write(buf);
   }

   void output_location(apm::gps_t::position_type const & loc)
   {
      
      xout::write("lat = ");
      output(loc.lat.numeric_value());
      xout::write("\nlon = ");
      output(loc.lon.numeric_value());
      xout::write("\nalt = ");
      output(loc.lat.numeric_value());
      xout::put('\n');
   }

   enum class gps_state_t {
      unknown,   have_driver, have_fix
   };

   gps_state_t gps_state = gps_state_t::unknown; 

   ms elapsed = 0_ms;

   void do_state(apm::gps_t & gps)
   {
      gps.update();

      switch (gps_state) {
      case gps_state_t::unknown:
         if ( gps.get_driver_id() != apm::gps_t::GPS_TYPE_NONE ){
            xout::write("got driver ");
            xout::write(gps.get_driver_name());
            xout::put('\n');
            gps_state = gps_state_t::have_driver;
         }else{
            if (millis() - elapsed > 2000_ms){
               elapsed = millis();
               xout::write("no driver\n");
            }
         }
         break;
      case gps_state_t::have_driver:
         if ( gps.have_3d_fix()){
            xout::write("Have 3D fix\n");
            gps_state = gps_state_t::have_fix;
         }
         break;
      case gps_state_t::have_fix:
         if ( (millis() - elapsed ) > 500_ms ){
            elapsed = millis();
            output_location(gps.get_location());
         }
         break;
      default:
         break;
      }
   }

}
#endif

//extern "C" void USART2_IRQHandler() __attribute__ ((interrupt ("IRQ")));
//extern "C" void USART2_IRQHandler()
//{
//   static_assert(
//      std::is_same<
//         aux_sp::serial_port::usart_type,quan::stm32::usart2
//      >::value
//   ,"invalid usart for serial_port irq");
//
//   quan::stm32::usart::irq_handler<aux_sp::serial_port>();
//}

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
