
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

/*
  GPS driver backend class
 */

#ifndef APM_GPS_BACKEND_HPP_INCLUDED
#define APM_GPS_BACKEND_HPP_INCLUDED

#include "../gps.h"

namespace apm{

   class AP_GPS_Backend{
   public:
      AP_GPS_Backend(gps_t &_gps, gps_t::GPS_State &_state, SerialPort *_port);
       virtual ~AP_GPS_Backend(void) {}

       // The read() method is the only one needed in each driver. It
       // should return true when the backend has successfully received a
       // valid packet from the GPS.
       virtual bool read() = 0;

       virtual void inject_data(uint8_t *data, uint8_t len) { return; }

       // Highest status supported by this GPS. 
       // Allows external system to identify type of receiver connected.
       virtual gps_t::GPS_Status highest_supported_status(void) { return gps_t::GPS_OK_FIX_3D; }
   protected:
       SerialPort *port;           ///< UART we are attached to
       gps_t &gps;                        ///< access to frontend (for parameters)
       gps_t::GPS_State &state;           ///< public state for this instance

       // common utility functions
       int32_t swap_int32(int32_t v) const;
       int16_t swap_int16(int16_t v) const;

       /*
         fill in 3D velocity from 2D components
        */
       void fill_3d_velocity(void);

       /*
          fill in time_week_ms and time_week from BCD date and time components
          assumes MTK19 millisecond form of bcd_time
       */
       void make_gps_time(uint32_t bcd_date, uint32_t bcd_milliseconds);
   };

}// apm

#endif // APM_GPS_BACKEND_HPP_INCLUDED
