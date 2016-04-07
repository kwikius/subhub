/*
   derived from Ardupilot GPS library
*/

// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#ifndef QUAN_SUBHUB_LIBRARIES_GPS_GPS_HPP_INCLUDED
#define QUAN_SUBHUB_LIBRARIES_GPS_GPS_HPP_INCLUDED

#include <quan/three_d/vect.hpp>
#include <quan/velocity.hpp>
#include <quan/time.hpp>
#include <quan/uav/position.hpp>

/*
   
   altitude is measured relative to sea level, unles othrewise stated
   gps time
   ( for more accurate definition maybe look at EGM96)
*/

namespace quan_apm{

   // abstract base class gps
   // represents a single gps receiver
   struct gps
   {
    protected:
      gps();
    public:
      virtual ~gps(){}
      typedef quan::angle_<int32_t>::deg10e7                   lat_lon_type;
      typedef quan::length_<int32_t>::mm                       altitude_type;
      typedef quan::uav::position<lat_lon_type,altitude_type>  position_type; 
      typedef quan::three_d::vect<quan::velocity::m_per_s>     velocity3d_type;
      typedef quan::three_d::vect<quan::length_<int32_t>::mm>  position_error_type; 
      struct gps_time{
         quan::time_<uint32_t>::ms week_time;
         uint16_t                  week_number;
      };
      //gps altitude is relative to mean sea level
      virtual position_type         get_position()const =0;
      virtual position_error_type   get_position_error()const =0;
      virtual velocity3d_type       get_velocity3d()const=0;
      virtual int                   get_num_sats()const=0;
      virtual gps_time              get_gps_time()const=0;
      
      static constexpr int no_gps  = -1;
      static constexpr int no_fix  = -2;
      static constexpr int fix_2d  = -3;

   };

   // return a pointer to  a gps if valid else null
   // once a valid pointer is returned then this must return
   // the same pointer to the same gps
   const gps * get_gps();

}
#endif // QUAN_SUBHUB_LIBRARIES_GPS_GPS_HPP_INCLUDED
