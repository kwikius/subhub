
#include <apm/gps.hpp>
#include <quan/stm32/millis.hpp>
#include <quan/conversion/itoa.hpp>
#include <quan/conversion/float_to_ascii.hpp>
#include <quan/stm32/usart/irq_handler.hpp>
#include <apm/serial_port.hpp>
#include <type_traits>
#include "../../serial_port.hpp"
#include "../../usarts.hpp"

apm::abc_serial_port& get_gps_sp();

using quan::stm32::millis;

namespace {

   typedef decltype(millis()) ms;

   ms operator "" _ms(unsigned long long int v)
   {
      return static_cast<ms>(v);
   }

   typedef link_sp::serial_port xout;
   typedef link_sp::serial_port xin;

}

void console_out(const char* text)
{
   xout::write(text);
}

namespace {

   template <typename T>
   typename quan::where_<std::is_integral<T> >::type 
   output(T v)
   {
      char buf[20];
      quan::itoasc(v,buf,10);
      xout::write(buf);  
   }

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

}

void run_gps()
{
   apm::gps_t gps;
   gps.initialise(get_gps_sp());

   for (;;){
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
               xout::write("At ");
               output(elapsed.numeric_value());
               xout::write(" no driver\n");
            }
         }
         break;
      case gps_state_t::have_driver:
         if ( gps.have_3d_fix()){
            xout::write("Have 3D fix\n");
            gps_state = gps_state_t::have_fix;
         }else{
            if ( (millis() - elapsed ) > 500_ms ){
               elapsed = millis();
               switch( gps.get_fix_type()){
               case apm::gps_t::NO_GPS:
                  xout::write("No GPS");
                  break;
               case apm::gps_t::NO_FIX:
                  xout::write("No fix");
                  break;
               case apm::gps_t::FIX_2D:
                  xout::write("2F fix\n");
                  break;
               case apm::gps_t::FIX_3D:
                  xout::write("3D fix\n");
                  break;
               case apm::gps_t::FIX_3D_DGPS:
                  xout::write("3D fix DGPS\n");
                  break;
               case  apm::gps_t::FIX_3D_RTK: 
                  xout::write("3D fix RTK\n");
                  break;
               default:
                  xout::write("unknow fix status\n");
                  break;
               }
               xout::put('\n');
            }
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
