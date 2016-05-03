#include <apm/gps.hpp>
#include <quan/stm32/millis.hpp>
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

   void do_state(apm::gps_t & gps);


}

int main()
{
   setup();

   apm::gps_t gps;
   gps.initialise(get_gps_sp()); // just attaches the port

   // want to see what if any GPS we have 

   auto elapsed = millis();
   for (;;){
      auto now = millis();

      if ( (now - elapsed) >= 20_ms){
         do_state(gps);
      }
   }
   
}

namespace {

   enum class gps_state_t {
      unknown,   have_driver, have_fix
   };

   gps_state_t gps_state = gps_state_t::unknown; 

   void do_state(apm::gps_t & gps)
   {
      gps.update();

      switch (gps_state) {
      case gps_state_t::unknown:
         if ( gps.get_driver_id() != apm::gps_t::GPS_TYPE_NONE ){
            xout::write(gps.get_driver_name());
            gps_state = gps_state_t::have_driver;
         }
         break;
      case gps_state_t::have_driver:
      
         break;
      }
   }

}
