
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

#include <cstring>
#include <ap_serialport/serialport.hpp>
#include <apm/gps.h>
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

static inline uint8_t pgm_read_byte(const void *s)
{
	return *(const uint8_t *)s;
}

static inline uint32_t pgm_read_dword(const void *s) 
{
	return *(const uint32_t *)s;
}


apm::gps_t::gps_t()
: _type{1}
,_navfilter{GPS_ENGINE_AIRBORNE_4G}
,_min_dgps{100}
,_sbas_mode{2}
,_min_elevation{-100}
,_gnss_mode{0}
,_save_config{0}
{
}


/// Startup initialisation.
void apm::gps_t::init(/*DataFlash_Class *dataflash, const AP_SerialManager& serial_manager */)
{}

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
    if (_port == NULL) {
        return;
    }

    // see if we can write some more of the initialisation blob
    if (initblob_state.remaining > 0) {
        int16_t space = _port->txspace();
        if (space > (int16_t)initblob_state.remaining) {
            space = initblob_state.remaining;
        }
        while (space > 0) {
            //_port->write(pgm_read_byte(initblob_state.blob),1);
           // _port->write((uint8_t const *)initblob_state.blob,1);
            _port->write(initblob_state.blob[0]);
            initblob_state.blob++;
            space--;
            initblob_state.remaining--;
        }
    }
}

/*
  run detection step for one GPS instance. If this finds a GPS then it
  will fill in drivers and change state.status
  from NO_GPS to NO_FIX.
 */
void apm::gps_t::detect_instance()
{
    AP_GPS_Backend *new_gps = NULL;
    struct detect_state *dstate = &detect_state;
    uint32_t now = quan::stm32::millis().numeric_value();

    if (_port == NULL) {
        // UART not available
        return;
    }

    //state.instance = instance;
    state.status = NO_GPS;
    state.hdop = 9999;

	// by default the sbf/trimble gps outputs no data on its port, until configured.
	if (_type == GPS_TYPE_SBF) {
		//hal.console->print(" SBF ");
		new_gps = new AP_GPS_SBF(*this,  _port);
	} else if ((_type == GPS_TYPE_GSOF)) {
		//hal.console->print(" GSOF ");
		new_gps = new AP_GPS_GSOF(*this,  _port);
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
		uint32_t baudrate = pgm_read_dword(&_baudrates[dstate->last_baud]);
		_port->begin(baudrate);
		_port->set_flow_control(SerialPort::FLOW_CONTROL_DISABLE);
		dstate->last_baud_change_ms = now;
      send_blob_start( _initialisation_blob, sizeof(_initialisation_blob));
    }

    send_blob_update();

    while (initblob_state.remaining == 0 && _port->available() > 0
            && new_gps == NULL) {
        uint8_t data = _port->read();
        /*
          running a uBlox at less than 38400 will lead to packet
          corruption, as we can't receive the packets in the 200ms
          window for 5Hz fixes. The NMEA startup message should force
          the uBlox into 38400 no matter what rate it is configured
          for.
        */
        if ((_type == GPS_TYPE_AUTO || _type == GPS_TYPE_UBLOX) &&
            pgm_read_dword(&_baudrates[dstate->last_baud]) >= 38400 && 
            AP_GPS_UBLOX::_detect(dstate->ublox_detect_state, data)) {
            //hal.console->print(" ublox ");
            new_gps = new AP_GPS_UBLOX(*this,  _port);
        } 
		  else if ((_type == GPS_TYPE_AUTO || _type == GPS_TYPE_MTK19) &&
                 AP_GPS_MTK19::_detect(dstate->mtk19_detect_state, data)) {
			//hal.console->print(" MTK19 ");
			new_gps = new AP_GPS_MTK19(*this,  _port);
		} 
		else if ((_type == GPS_TYPE_AUTO || _type == GPS_TYPE_MTK) &&
                 AP_GPS_MTK::_detect(dstate->mtk_detect_state, data)) {
			//hal.console->print(" MTK ");
			new_gps = new AP_GPS_MTK(*this,  _port);
		}
        else if ((_type == GPS_TYPE_AUTO || _type == GPS_TYPE_SBP) &&
                 AP_GPS_SBP::_detect(dstate->sbp_detect_state, data)) {
            //hal.console->print(" SBP ");
            new_gps = new AP_GPS_SBP(*this,  _port);
        }
		// save a bit of code space on a 1280
		else if ((_type == GPS_TYPE_AUTO || _type == GPS_TYPE_SIRF) &&
                 AP_GPS_SIRF::_detect(dstate->sirf_detect_state, data)) {
			//hal.console->print(" SIRF ");
			new_gps = new AP_GPS_SIRF(*this,  _port);
		}
		else if (now - dstate->detect_started_ms > (ARRAY_SIZE(_baudrates) * GPS_BAUD_TIME_MS)) {
			// prevent false detection of NMEA mode in
			// a MTK or UBLOX which has booted in NMEA mode
			if ((_type == GPS_TYPE_AUTO || _type == GPS_TYPE_NMEA) &&
                AP_GPS_NMEA::_detect(dstate->nmea_detect_state, data)) {
				//hal.console->print(" NMEA ");
				new_gps = new AP_GPS_NMEA(*this,  _port);
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
apm::gps_t::highest_supported_status() const
{
   if (drivers != NULL){
      return drivers->highest_supported_status();
   }else{
      return apm::gps_t::GPS_OK_FIX_3D;
   }
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
void apm::gps_t::update_instance()
{
    if (_type == GPS_TYPE_HIL) {
        // in HIL, leave info alone
        return;
    }
    if (_type == GPS_TYPE_NONE) {
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
        detect_instance();
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
 */
void apm::gps_t::update(void)
{
   // for (uint8_t i=0; i<GPS_MAX_INSTANCES; i++) {
        update_instance();
   // }

    // work out which GPS is the primary, and how many sensors we have
  //  for (uint8_t i=0; i<GPS_MAX_INSTANCES; i++) {
//        if (state.status != NO_GPS) {
//            num_instances = i+1;
//        }
//        if (_auto_switch) {            
//            if (i == primary_instance) {
//                continue;
//            }
//            if (state[i].status > state[primary_instance].status) {
//                // we have a higher status lock, change GPS
//                primary_instance = i;
//                continue;
//            }
//
//            bool another_gps_has_1_or_more_sats = (state[i].num_sats >= state[primary_instance].num_sats + 1);
//
//            if (state[i].status == state[primary_instance].status && another_gps_has_1_or_more_sats) {
//
//                uint32_t now = quan::stm32::millis().numeric_value();
//                bool another_gps_has_2_or_more_sats = (state[i].num_sats >= state[primary_instance].num_sats + 2);
//
//                if ( (another_gps_has_1_or_more_sats && (now - _last_instance_swap_ms) >= 20000) ||
//                     (another_gps_has_2_or_more_sats && (now - _last_instance_swap_ms) >= 5000 ) ) {
//                // this GPS has more satellites than the
//                // current primary, switch primary. Once we switch we will
//                // then tend to stick to the new GPS as primary. We don't
//                // want to switch too often as it will look like a
//                // position shift to the controllers.
//                primary_instance = i;
//                _last_instance_swap_ms = now;
//                }
//            }
//        } else {
//            primary_instance = 0;
//        }
   // }

	// update notify with gps status. We always base this on the primary_instance
   // AP_Notify::flags.gps_status = state[primary_instance].status;
}

/**
   Lock a GPS port, preventing the GPS driver from using it. This can
   be used to allow a user to control a GPS port via the
   SERIAL_CONTROL protocol
 */
void apm::gps_t::lock_port( bool lock)
{
  port_locked = lock;
//    if (instance >= GPS_MAX_INSTANCES) {
//        return;
//    }
//    if (lock) {
//        locked_ports = 1;
//    } else {
//        locked_ports = 0;
//    }
}

    //Inject a packet of raw binary to a GPS
//void apm::gps_t::inject_data(uint8_t *data, uint8_t len)
//{
//    //Support broadcasting to all GPSes.
//    if (_inject_to == GPS_RTK_INJECT_TO_ALL) {
//        for (uint8_t i=0; i<GPS_MAX_INSTANCES; i++) {
//            inject_data(i, data, len);
//        }
//    } else {
//        inject_data(_inject_to, data, len);
//    }
//}

void apm::gps_t::inject_data( uint8_t *data, uint8_t len)
{
    if (drivers != NULL){
        drivers->inject_data(data, len);
    }
}  
