/*
 Copyright (c) 2012 - 2013 Andy Little 

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <quan/stm32/usart/irq_handler.hpp>
#include <apm/serial_port.hpp>
#include "../../usarts.hpp"


/*
   Leaf port is connected to GPS or Rc rx
   Leaf in the sense it is an end node rather than a link
*/
namespace {

   typedef aux_sp::serial_port port;

   struct gps_sp_t final : apm::abc_serial_port{

      void  begin(uint32_t baud)
      {
         switch (baud){
            case 4800U:
               port::set_baudrate<4800,false>();
               break;
            case 9600U: 
               port::set_baudrate<9600,false>();
               break;
            case 38400U:
               port::set_baudrate<38400,false>();
               break;
            case 57600U:
               port::set_baudrate<57600,true>();
               break;
            case 115200U:
               port::set_baudrate<115200,true>();
               break;
            case 230400U:
               port::set_baudrate<230400,true>();
               break;
            default:
               break;

         }
      }

      int16_t   available()const
      {
         return port::in_avail();
      }

      int16_t   read()
      {
         if(port::in_avail()){
            return port::get();
         }else{
            return -1;
         }
      } 
        
      int16_t  txspace()const
      {
         return port::get_tx_buffer_free_space();
      }

      size_t    write(uint8_t c)
      {
         port::put((char)c);
         return 1;
      }

      size_t  write(const uint8_t *buffer, size_t size)
      {
         port::write((const char*)buffer,size);
         return size;
      }
   };

   gps_sp_t gps_sp;

} // namespace

apm::abc_serial_port& get_gps_sp()
{
   return gps_sp;
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

