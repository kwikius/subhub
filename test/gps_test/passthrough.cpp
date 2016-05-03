/*
Passthrough function
Just send the input from the gps port out the link port
*/

#include <quan/stm32/millis.hpp>
#include <apm/serial_port.hpp>
#include "../../serial_port.hpp"
#include "../../usarts.hpp"
#include <quan/convert.hpp>
#include <quan/conversion/int_convert.hpp>

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
     while (xin::in_avail()){
        (void) xin::get(); 
     }
   }

   void delay(ms t)
   {
      ms elapsed = millis();
      while ((millis() - elapsed) < t) {;}
   }

   bool is_valid_baud(uint32_t baud)
   {
      switch(baud){
      case 4800U:
      case 9600U: 
      case 38400U:
      case 57600U:
      case 115200U:
      case 230400U:
         return true;
      default:
         return false;
      }
   }
}

void passthrough()
{
   apm::abc_serial_port & gps_sp = get_gps_sp();
   delay(250_ms);
   flush_input();
   constexpr uint32_t bufsize = 20;
   for (;;){
      xout::write("input baudrate: ");
      bool parsed = false;
      while (! parsed){ 
         char buffer[bufsize];
         uint8_t idx = 0;
         for ( ;;){
            if (xin::in_avail()){
               char ch = xin::get();
               if (isdigit(ch)){
                  buffer[idx] = ch;
               }else{
                  if ( ch == '\r'){
                     buffer[idx] = '\0';
                     uint32_t baud = quan::convert<uint32_t>(buffer);
                     if ( is_valid_baud(baud)){
                        xout::write("setting baud to ");
                        xout::write(buffer);
                        xout::put('\n');
                        gps_sp.begin(baud);
                        parsed = true;
                        break;
                     }else{
                        xout::write("invalid baud\n");
                        break;
                     }
                  }else{
                     // could do return from here
                     xout::write("invalid input\n");
                     break;
                  }
               }
               if ( ++idx == bufsize){
                  xout::write("too many chars\n");
                  break;
               }
            }
         }
      }
      flush_input();
      for (;;){
        if ( gps_sp.available()){
            xout::put(gps_sp.read());
        }
        if (xin::in_avail()){
           break;
        }
      }
   }
}




