
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

/*
#############################
Changed by Andy Little Apr 2016
###########################
*/

#ifndef APM_GPS_BACKEND_HPP_INCLUDED
#define APM_GPS_BACKEND_HPP_INCLUDED

#include <apm/gps.hpp>

namespace apm{

   class AP_GPS_Backend{
   public:

       virtual ~AP_GPS_Backend(void) {}

       // The read() method is the only one needed in each driver. It
       // should return true when the backend has successfully received a
       // valid packet from the GPS.
       virtual bool read() = 0;

       virtual void inject_data(uint8_t *data, uint8_t len) { return; }

       // Highest status supported by this GPS. 
       // Allows external system to identify type of receiver connected.
       virtual gps_t::fix_type_t highest_supported_status(void) { return gps_t::FIX_3D; }
       apm::gps_t::driver_id_t get_driver_id()const { return m_driver_id;}
       const char* get_driver_name() const { return m_driver_name;}
   protected:
       AP_GPS_Backend(gps_t &_gps,const char* driver_name, apm::gps_t::driver_id_t driver_id);
       gps_t &gps;                ///< access to frontend 
       // common utility functions
       static int32_t swap_int32(int32_t v);
       static int16_t swap_int16(int16_t v);

       /*
         fill in 3D velocity from 2D components
        */
       void fill_3d_velocity(void);

       /*
          fill in time_week_ms and time_week from BCD date and time components
          assumes MTK19 millisecond form of bcd_time
       */
       void make_gps_time(uint32_t bcd_date, uint32_t bcd_milliseconds);
       
     private:
       apm::gps_t::driver_id_t m_driver_id;
       const char* const m_driver_name;
   };

}// apm

#endif // APM_GPS_BACKEND_HPP_INCLUDED
