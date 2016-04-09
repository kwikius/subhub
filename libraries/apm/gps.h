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

#ifndef APM_GPS_GPS_HPP_INCLUDED
#define APM_GPS_GPS_HPP_INCLUDED

#include <cstdint>
#include <quan/three_d/vect.hpp>
#include <quan/velocity.hpp>
#include <quan/time.hpp>
#include <quan/uav/position.hpp>
#include <apm/gps/src/GPS_detect_state.h>

namespace apm{

   class AP_GPS_Backend;
   class SerialPort;

   class gps_t {
   public:
       // constructor
       gps_t();

       typedef quan::angle_<int32_t>::deg10e7                   lat_lon_type;
       typedef quan::length_<int32_t>::cm                       altitude_type; // n.b the AP type is differnt to the quan osd type
       typedef quan::uav::position<lat_lon_type,altitude_type>  position_type; 
       typedef quan::velocity::m_per_s                          velocity_type;
       typedef quan::three_d::vect<velocity_type>               velocity3d_type;
       typedef quan::three_d::vect<quan::length_<int32_t>::mm>  position_error_type; 

       /// Startup initialisation.
       void init(/*DataFlash_Class *dataflash, const AP_SerialManager& serial_manager */);

       /// Update GPS state based on possible bytes received from the module.
       /// This routine must be called periodically (typically at 10Hz or
       /// more) to process incoming data.
       void update();

       // GPS driver types
       enum GPS_Type {
           GPS_TYPE_NONE  = 0,
           GPS_TYPE_AUTO  = 1,
           GPS_TYPE_UBLOX = 2,
           GPS_TYPE_MTK   = 3,
           GPS_TYPE_MTK19 = 4,
           GPS_TYPE_NMEA  = 5,
           GPS_TYPE_SIRF  = 6,
           GPS_TYPE_HIL   = 7,
           GPS_TYPE_SBP   = 8,
           GPS_TYPE_PX4   = 9,
           GPS_TYPE_SBF   = 10,
           GPS_TYPE_GSOF  = 11,
       };

       /// GPS status codes
       enum GPS_Status {
           NO_GPS = 0,             ///< No GPS connected/detected
           NO_FIX = 1,             ///< Receiving valid GPS messages but no lock
           GPS_OK_FIX_2D = 2,      ///< Receiving valid messages and 2D lock
           GPS_OK_FIX_3D = 3,      ///< Receiving valid messages and 3D lock
           GPS_OK_FIX_3D_DGPS = 4, ///< Receiving valid messages and 3D lock with differential improvements
           GPS_OK_FIX_3D_RTK = 5,  ///< Receiving valid messages and 3D lock, with relative-positioning improvements
       };

       // GPS navigation engine settings. Not all GPS receivers support
       // this
       enum GPS_Engine_Setting {
           GPS_ENGINE_NONE        = -1,
           GPS_ENGINE_PORTABLE    = 0,
           GPS_ENGINE_STATIONARY  = 2,
           GPS_ENGINE_PEDESTRIAN  = 3,
           GPS_ENGINE_AUTOMOTIVE  = 4,
           GPS_ENGINE_SEA         = 5,
           GPS_ENGINE_AIRBORNE_1G = 6,
           GPS_ENGINE_AIRBORNE_2G = 7,
           GPS_ENGINE_AIRBORNE_4G = 8
       };

       /*
         The GPS_State structure is filled in by the backend driver as it
         parses each message from the GPS.
        */
      
       GPS_Status status(void) const 
       {
           return state.status;
       }

       // Query the highest status this GPS supports
       GPS_Status highest_supported_status(uint8_t instance) const;
       GPS_Status highest_supported_status(void) const;

       const position_type &location() const 
       {
           return state.location;
       }

       bool speed_accuracy(float &sacc) const 
       {
         if(state.have_speed_accuracy) {
            sacc = state.speed_accuracy;
            return true;
         }
         return false;
       }

       bool horizontal_accuracy(float &hacc) const 
       {
         if(state.have_horizontal_accuracy) {
            hacc = state.horizontal_accuracy;
            return true;
         }
         return false;
       }

       bool vertical_accuracy(float &vacc) const 
       {
           if(state.have_vertical_accuracy) {
               vacc = state.vertical_accuracy;
               return true;
           }
           return false;
       }

       // 3D velocity in NED format
       const velocity3d_type &velocity() const 
       {
           return state.velocity;
       }

       // ground speed in m/s
       float ground_speed() const 
       {
           return state.ground_speed;
       }

       // ground speed in cm/s
       uint32_t ground_speed_cm(void) 
       {
           return ground_speed() * 100;
       }

       // ground course in centidegrees
       int32_t ground_course_cd() const 
       {
           return state.ground_course_cd;
       }


       // number of locked satellites
       uint8_t num_sats() const 
       {
           return state.num_sats;
       }

       // GPS time of week in milliseconds
       uint32_t time_week_ms() const 
       {
           return state.time_week_ms;
       }

       // GPS week
       uint16_t time_week() const 
       {
           return state.time_week;
       }

       // horizontal dilution of precision
       uint16_t get_hdop(uint8_t instance) const
       {
           return state.hdop;
       }

       // vertical dilution of precision
       uint16_t get_vdop() const 
       {
           return state.vdop;
       }


       // the time we got our last fix in system milliseconds. This is
       // used when calculating how far we might have moved since that fix
       uint32_t last_fix_time_ms(uint8_t instance) const 
       {
           return timing.last_fix_time_ms;
       }


       // the time we last processed a message in milliseconds. This is
       // used to indicate that we have new GPS data to process
       uint32_t last_message_time_ms() const 
       {
           return timing.last_message_time_ms;
       }


       // return last fix time since the 1/1/1970 in microseconds
       uint64_t time_epoch_usec();


      // return true if the GPS supports vertical velocity values
       bool have_vertical_velocity() const 
       { 
           return state.have_vertical_velocity; 
       }

       // the expected lag (in seconds) in the position and velocity readings from the gps
       // could 
       static constexpr quan::time::ms get_lag() { return quan::time::ms{200.f}; }
       // handle sending of initialisation strings to the GPS
       
       bool   want_config_saved() const { return _save_config;}
       int8_t get_nav_filter()const{return _navfilter;}
       int8_t get_min_elevation()const {return _min_elevation;}
       int8_t get_gnss_mode()const{return _gnss_mode;}
       int8_t get_sbas_mode()const { return _sbas_mode;}

   private: 

        struct GPS_State {
           // all the following fields must all be filled by the backend driver
           GPS_Status status;                  ///< driver fix status
           uint32_t time_week_ms;              ///< GPS time (milliseconds from start of GPS week)
           uint16_t time_week;                 ///< GPS week number
           position_type location;                  ///< last fix location
           float ground_speed;                 ///< ground speed in m/sec
           int32_t ground_course_cd;           ///< ground course in 100ths of a degree
           uint16_t hdop;                      ///< horizontal dilution of precision in cm
           uint16_t vdop;                      ///< vertical dilution of precision in cm
           uint8_t num_sats;                   ///< Number of visible satelites        
           velocity3d_type velocity;           ///< 3D velocitiy in m/s, in NED format
           float speed_accuracy;
           float horizontal_accuracy;
           float vertical_accuracy;
           bool have_vertical_velocity:1;      ///< does this GPS give vertical velocity?
           bool have_speed_accuracy:1;
           bool have_horizontal_accuracy:1;
           bool have_vertical_accuracy:1;
           uint32_t last_gps_time_ms;          ///< the system time we got the last GPS timestamp, milliseconds
       };

       friend class AP_GPS_UBLOX;
       friend class AP_GPS_MTK;
       friend class AP_GPS_MTK19;
       friend class AP_GPS_NMEA;
       friend class AP_GPS_SBF;
       friend class AP_GPS_SBP;
       friend class AP_GPS_GSOF;
       friend class AP_GPS_SIRF;
       friend class AP_GPS_Backend;
        
       void send_blob_start( const char *_blob, uint16_t size);
       void send_blob_update();
       // lock out a GPS port, allowing another application to use the port
       void lock_port(bool locked);
       //Inject a packet of raw binary to a GPS
       void inject_data(uint8_t *data, uint8_t len);

       // configuration parameters
       int8_t _type; // [GPS_MAX_INSTANCES];
       int8_t _navfilter;
      // int8_t _auto_switch;
       int8_t _min_dgps;
     //  int16_t _sbp_logmask;
     //  int8_t _inject_to;
       uint32_t _last_instance_swap_ms;
       int8_t _sbas_mode;
       int8_t _min_elevation;
     //  int8_t _raw_data;
       int8_t _gnss_mode;
       int8_t _save_config;

       struct GPS_timing {
           // the time we got our last fix in system milliseconds
           uint32_t last_fix_time_ms;

           // the time we got our last fix in system milliseconds
           uint32_t last_message_time_ms;
       };
       GPS_timing timing;
       GPS_State state;
       AP_GPS_Backend *drivers;
       SerialPort *_port;

       // which ports are locked
       bool port_locked;

       // state of auto-detection process, per instance
       struct detect_state {
           uint32_t detect_started_ms;
           uint32_t last_baud_change_ms;
           uint8_t last_baud;
           struct UBLOX_detect_state ublox_detect_state;
           struct MTK_detect_state mtk_detect_state;
           struct MTK19_detect_state mtk19_detect_state;
           struct SIRF_detect_state sirf_detect_state;
           struct NMEA_detect_state nmea_detect_state;
           struct SBP_detect_state sbp_detect_state;
       } detect_state;//[GPS_MAX_INSTANCES];

       struct {
           const char *blob;
           uint16_t remaining;
       } initblob_state;//[GPS_MAX_INSTANCES];

       static const uint32_t  _baudrates[];
       static const char _initialisation_blob[];
       static const char _initialisation_raw_blob[];

       void detect_instance();
       void update_instance();
   };

} // namespace apm

#endif // APM_GPS_GPS_HPP_INCLUDED
