
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//
//  Trimble GPS driver for ArduPilot.
//	Code by Michael Oborne
//
/*
#############################
Changed by Andy Little Apr 2016
###########################
*/

#ifndef APM_GPS_GSOF_HPP_INCLUDED
#define APM_GPS_GSOF_HPP_INCLUDED

#include <apm/gps.hpp>
#include "GPS_Backend.h"

namespace apm{

   class AP_GPS_GSOF : public AP_GPS_Backend{
   public:
       AP_GPS_GSOF(gps_t &_gps);

       gps_t::fix_type_t highest_supported_status(void) 
       {
           return gps_t::FIX_3D_RTK;
       }

       // Methods
       bool read();

       void inject_data(uint8_t *data, uint8_t len);

   private:

       bool parse(uint8_t temp);
       bool process_message();
       void requestBaud(uint8_t portindex);
       void requestGSOF(uint8_t messagetype, uint8_t portindex);
       double SwapDouble(uint8_t* src, uint32_t pos);
       float SwapFloat(uint8_t* src, uint32_t pos);
       uint32_t SwapUint32(uint8_t* src, uint32_t pos);
       uint16_t SwapUint16(uint8_t* src, uint32_t pos);


       struct gsof_msg_parser_t
       {
           enum
           {
               STARTTX = 0,
               STATUS,
               PACKETTYPE,
               LENGTH,
               DATA,
               CHECKSUM,
               ENDTX
           } gsof_state;

           uint8_t starttx;
           uint8_t status;
           uint8_t packettype;
           uint8_t length;
           uint8_t data[256];
           uint8_t checksum;
           uint8_t endtx;

           uint16_t read;
           uint8_t checksumcalc;
       } gsof_msg;

       static const uint8_t GSOF_STX = 0x02;
       static const uint8_t GSOF_ETX = 0x03;

       uint8_t packetcount = 0;

       uint32_t gsofmsg_time = 0;
       uint8_t gsofmsgreq_index = 0;
       uint8_t gsofmsgreq[5] = {1,2,8,9,12};

       uint32_t last_hdop = 9999;
       uint32_t crc_error_counter = 0;
       uint32_t last_injected_data_ms = 0;
   };

} // apm

#endif // APM_GPS_GSOF_HPP_INCLUDED
