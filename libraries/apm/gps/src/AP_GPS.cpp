
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
#############################
Changed by Andy Little Apr 2016
###########################
*/

#include <cstring>
#include <apm/serial_port.hpp>
#include <apm/gps.hpp>
#include "AP_GPS_UBLOX.h"
#include "AP_GPS_MTK.h"
#include "AP_GPS_MTK19.h"
#include "AP_GPS_NMEA.h"
#include "AP_GPS_SIRF.h"
#include "AP_GPS_SBP.h"
#include "AP_GPS_SBF.h"
#include "AP_GPS_GSOF.h"

#include <quan/stm32/millis.hpp>

#define GPS_BAUD_TIME_MS 1200
#define GPS_RTK_INJECT_TO_ALL 127

apm::gps_t::gps_t()
: m_preset_firmware_type{1} // auto
,_navfilter{GPS_ENGINE_AIRBORNE_4G}
,_min_dgps{100}
,_sbas_mode{2}
,_min_elevation{-100}
,_gnss_mode{0}
,_save_config{0}
{
}


/// Startup initialisation.
void apm::gps_t::init(abc_serial_port& sp)
{port = &sp;}

// baudrates to try to detect GPSes with
const uint32_t apm::gps_t::_baudrates[] = {4800U, 38400U, 115200U, 57600U, 9600U, 230400U};

// initialisation blobs to send to the GPS to try to get it into the
// right mode
const char apm::gps_t::_initialisation_blob[] = UBLOX_SET_BINARY MTK_SET_BINARY SIRF_SET_BINARY;
const char apm::gps_t::_initialisation_raw_blob[] = UBLOX_SET_BINARY_RAW_BAUD MTK_SET_BINARY SIRF_SET_BINARY;

/*
  send some more initialisation string bytes if there is room in the
  UART transmit buffer
 */
void apm::gps_t::send_blob_start(const char *_blob, uint16_t size)
{
    initblob_state.blob = _blob;
    initblob_state.remaining = size;
}

/*
  send some more initialisation string bytes if there is room in the
  UART transmit buffer
 */
void apm::gps_t::send_blob_update()
{
    // exit immediately if no uart for this instance
    if (port == NULL) {
        return;
    }

    // see if we can write some more of the initialisation blob
    if (initblob_state.remaining > 0) {
        int16_t space = port->txspace();
        if (space > (int16_t)initblob_state.remaining) {
            space = initblob_state.remaining;
        }
        while (space > 0) {
            port->write(initblob_state.blob[0]);
            initblob_state.blob++;
            space--;
            initblob_state.remaining--;
        }
    }
}

/*
  run detection step If this finds a GPS then it
  will fill in drivers and change state.status
  from NO_GPS to NO_FIX.
 */
void apm::gps_t::detect()
{
    if (port == NULL) {
        // UART not available
        return;
    }

    AP_GPS_Backend *new_gps = NULL;
    struct detect_state *dstate = &detect_state;
    uint32_t now = quan::stm32::millis().numeric_value();

    state.status = NO_GPS;
    state.hdop = 9999;

	// by default the sbf/trimble gps outputs no data on its port, until configured.
	if (m_preset_firmware_type == GPS_TYPE_SBF) {
		//hal.console->print(" SBF ");
		new_gps = new AP_GPS_SBF(*this);
	} else if ((m_preset_firmware_type == GPS_TYPE_GSOF)) {
		//hal.console->print(" GSOF ");
		new_gps = new AP_GPS_GSOF(*this);
	}

    // record the time when we started detection. This is used to try
    // to avoid initialising a uBlox as a NMEA GPS
    if (dstate->detect_started_ms == 0) {
        dstate->detect_started_ms = now;
    }

    if (now - dstate->last_baud_change_ms > GPS_BAUD_TIME_MS) {
        // try the next baud rate
		dstate->last_baud++;
		if (dstate->last_baud == ARRAY_SIZE(_baudrates)) {
			dstate->last_baud = 0;
		}
      uint32_t baudrate = _baudrates[dstate->last_baud];
		port->begin(baudrate);
		dstate->last_baud_change_ms = now;
      send_blob_start( _initialisation_blob, sizeof(_initialisation_blob));
    }

    send_blob_update();

    while (initblob_state.remaining == 0 && port->available() > 0
            && new_gps == NULL) {
        uint8_t data = port->read();
        /*
          running a uBlox at less than 38400 will lead to packet
          corruption, as we can't receive the packets in the 200ms
          window for 5Hz fixes. The NMEA startup message should force
          the uBlox into 38400 no matter what rate it is configured
          for.
        */
        if ((m_preset_firmware_type == GPS_TYPE_AUTO || m_preset_firmware_type == GPS_TYPE_UBLOX) &&
            //pgm_read_dword(&_baudrates[dstate->last_baud]) >= 38400 && 
            _baudrates[dstate->last_baud] >= 38400 && 
            AP_GPS_UBLOX::_detect(dstate->ublox_detect_state, data)) {
            //hal.console->print(" ublox ");
            new_gps = new AP_GPS_UBLOX(*this);
        } 
		  else if ((m_preset_firmware_type == GPS_TYPE_AUTO || m_preset_firmware_type == GPS_TYPE_MTK19) &&
                 AP_GPS_MTK19::_detect(dstate->mtk19_detect_state, data)) {
			//hal.console->print(" MTK19 ");
			new_gps = new AP_GPS_MTK19(*this);
		} 
		else if ((m_preset_firmware_type == GPS_TYPE_AUTO || m_preset_firmware_type == GPS_TYPE_MTK) &&
                 AP_GPS_MTK::_detect(dstate->mtk_detect_state, data)) {
			//hal.console->print(" MTK ");
			new_gps = new AP_GPS_MTK(*this);
		}
        else if ((m_preset_firmware_type == GPS_TYPE_AUTO || m_preset_firmware_type == GPS_TYPE_SBP) &&
                 AP_GPS_SBP::_detect(dstate->sbp_detect_state, data)) {
            //hal.console->print(" SBP ");
            new_gps = new AP_GPS_SBP(*this);
        }
		// save a bit of code space on a 1280
		else if ((m_preset_firmware_type == GPS_TYPE_AUTO || m_preset_firmware_type == GPS_TYPE_SIRF) &&
                 AP_GPS_SIRF::_detect(dstate->sirf_detect_state, data)) {
			//hal.console->print(" SIRF ");
			new_gps = new AP_GPS_SIRF(*this);
		}
		else if (now - dstate->detect_started_ms > (ARRAY_SIZE(_baudrates) * GPS_BAUD_TIME_MS)) {
			// prevent false detection of NMEA mode in
			// a MTK or UBLOX which has booted in NMEA mode
			if ((m_preset_firmware_type == GPS_TYPE_AUTO || m_preset_firmware_type == GPS_TYPE_NMEA) &&
                AP_GPS_NMEA::_detect(dstate->nmea_detect_state, data)) {
				//hal.console->print(" NMEA ");
				new_gps = new AP_GPS_NMEA(*this);
			}
		}
	}

	if (new_gps != NULL) {
        state.status = NO_FIX;
        drivers = new_gps;
        timing.last_message_time_ms = now;
	}
}

apm::gps_t::GPS_Status 
apm::gps_t::get_highest_supported_status() const
{
   if (drivers != NULL){
      return drivers->highest_supported_status();
   }else{
      return apm::gps_t::GPS_OK_FIX_3D;
   }
}


/**
   calculate current time since the unix epoch in microseconds
 */
uint64_t apm::gps_t::get_time_epoch_usec()
{
    const GPS_State &istate = state;
    if (istate.last_gps_time_ms == 0) {
        return 0;
    }
    const uint64_t ms_per_week = 7000ULL*86400ULL;
    const uint64_t unix_offset = 17000ULL*86400ULL + 52*10*7000ULL*86400ULL - 15000ULL;
    uint64_t fix_time_ms = unix_offset + istate.time_week*ms_per_week + istate.time_week_ms;
    // add in the milliseconds since the last fix
    return (fix_time_ms + (quan::stm32::millis().numeric_value() - istate.last_gps_time_ms)) * 1000ULL;
}

//apm::gps_t::GPS_Status 
//apm::gps_t::highest_supported_status(void) const
//{
//    if (drivers[primary_instance] != NULL)
//        return drivers[primary_instance]->highest_supported_status();
//    return apm::gps_t::GPS_OK_FIX_3D;
//}


/*
  update one GPS instance. This should be called at 10Hz or greater
 */
void apm::gps_t::update()
{
    if (m_preset_firmware_type == GPS_TYPE_HIL) {
        // in HIL, leave info alone
        return;
    }
    if (m_preset_firmware_type == GPS_TYPE_NONE) {
        // not enabled
        state.status = NO_GPS;
        state.hdop = 9999;
        return;
    }
    if (port_locked ) {
        // the port is locked by another driver
        return;
    }

    if (drivers == NULL || state.status == NO_GPS) {
        // we don't yet know the GPS type of this one, or it has timed
        // out and needs to be re-initialised
        detect();
        return;
    }

    send_blob_update();

    // we have an active driver for this instance
    bool result = drivers->read();
    uint32_t tnow = quan::stm32::millis().numeric_value();

    // if we did not get a message, and the idle timer of 1.2 seconds
    // has expired, re-initialise the GPS. This will cause GPS
    // detection to run again
    if (!result) {
        if (tnow - timing.last_message_time_ms > 1200) {
            // free the driver before we run the next detection, so we
            // don't end up with two allocated at any time
            delete drivers;
            drivers = NULL;
            memset(&state, 0, sizeof(state));
          //  state.instance = instance;
            state.status = NO_GPS;
            state.hdop = 9999;
            timing.last_message_time_ms = tnow;
        }
    } else {
        timing.last_message_time_ms = tnow;
        if (state.status >= GPS_OK_FIX_2D) {
            timing.last_fix_time_ms = tnow;
        }
    }
}

/*
  update all GPS instances
 *//**
   Lock a GPS port, preventing the GPS driver from using it. This can
   be used to allow a user to control a GPS port via the
   SERIAL_CONTROL protocol
 */
void apm::gps_t::lock_port( bool lock)
{
  port_locked = lock;
}


void apm::gps_t::inject_data( uint8_t *data, uint8_t len)
{
    if (drivers != NULL){
        drivers->inject_data(data, len);
    }
}  
