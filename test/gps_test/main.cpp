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
   xout::write("passthrough = \'P\'");

   while (!xin::in_avail()){;}

   for (;;){
      switch ( xin::get()){
         case 'P':
         case 'p':
            passthrough();
         break;
         default:
            xout::write("invalid option\n");
         break;
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
