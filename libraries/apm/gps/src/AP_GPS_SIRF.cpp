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

//
//  SiRF Binary GPS driver for ArduPilot and ArduPilotMega.
//	Code by Michael Smith.
//


#include <stdint.h>
#include <ap_math/ap_math.hpp>
#include <apm/serial_port.hpp>

#include "AP_GPS_SIRF.h"

// Initialisation messages
//
// Turn off all messages except for 0x29.
//
// XXX the bytes show up on the wire, but at least my test unit (EM-411) seems to ignore them.
//
const uint8_t apm::AP_GPS_SIRF::_initialisation_blob[] = {
    0xa0, 0xa2, 0x00, 0x08, 0xa6, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa8, 0xb0, 0xb3,
    0xa0, 0xa2, 0x00, 0x08, 0xa6, 0x00, 0x29, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd0, 0xb0, 0xb3 
};

apm::AP_GPS_SIRF::AP_GPS_SIRF(gps_t &_gps) :
    AP_GPS_Backend(_gps),
    _step(0),
    _gather(false),
    _payload_length(0),
    _payload_counter(0),
    _msg_id(0)
{
    gps.send_blob_start( (const char *)_initialisation_blob, sizeof(_initialisation_blob));
}


// Process bytes available from the stream
//
// The stream is assumed to contain only messages we recognise.  If it
// contains other messages, and those messages contain the preamble
// bytes, it is possible for this code to fail to synchronise to the
// stream immediately.  Without buffering the entire message and
// re-processing it from the top, this is unavoidable. The parser
// attempts to avoid this when possible.
//
bool apm::AP_GPS_SIRF::read(void)
{
    uint8_t data;
    int16_t numc;
    bool parsed = false;

    numc = gps.port->available();
    while(numc--) {

        // read the next byte
        data = gps.port->read();

        switch(_step) {

        // Message preamble detection
        //
        // If we fail to match any of the expected bytes, we reset
        // the state machine and re-consider the failed byte as
        // the first byte of the preamble.  This improves our
        // chances of recovering from a mismatch and makes it less
        // likely that we will be fooled by the preamble appearing
        // as data in some other message.
        //
        case 1:
            if (PREAMBLE2 == data) {
                _step++;
                break;
            }
            _step = 0;
        // FALLTHROUGH
        case 0:
            if(PREAMBLE1 == data)
                _step++;
            break;

        // Message length
        //
        // We always collect the length so that we can avoid being
        // fooled by preamble bytes in messages.
        //
        case 2:
            _step++;
            _payload_length = (uint16_t)data << 8;
            break;
        case 3:
            _step++;
            _payload_length |= data;
            _payload_counter = 0;
            _checksum = 0;
            break;

        // Message header processing
        //
        // We sniff the message ID to determine whether we are going
        // to gather the message bytes or just discard them.
        //
        case 4:
            _step++;
            _accumulate(data);
            _payload_length--;
            _gather = false;
            switch(data) {
            case MSG_GEONAV:
                if (_payload_length == sizeof(sirf_geonav)) {
                    _gather = true;
                    _msg_id = data;
                }
                break;
            }
            break;

        // Receive message data
        //
        // Note that we are effectively guaranteed by the protocol
        // that the checksum and postamble cannot be mistaken for
        // the preamble, so if we are discarding bytes in this
        // message when the payload is done we return directly
        // to the preamble detector rather than bothering with
        // the checksum logic.
        //
        case 5:
            if (_gather) {                                              // gather data if requested
                _accumulate(data);
                _buffer.bytes[_payload_counter] = data;
                if (++_payload_counter == _payload_length)
                    _step++;
            } else {
                if (++_payload_counter == _payload_length)
                    _step = 0;
            }
            break;

        // Checksum and message processing
        //
        case 6:
            _step++;
            if ((_checksum >> 8) != data) {
                _step = 0;
            }
            break;
        case 7:
            _step = 0;
            if ((_checksum & 0xff) != data) {
                break;
            }
            if (_gather) {
                parsed = _parse_gps();                                   // Parse the new GPS packet
            }
        }
    }
    return(parsed);
}

bool apm::AP_GPS_SIRF::_parse_gps(void)
{
    switch(_msg_id) {
    case MSG_GEONAV:
        //time                    = _swapl(&_buffer.nav.time);
        // parse fix type
        if (_buffer.nav.fix_invalid) {
            gps.state.status = gps_t::NO_FIX;
        }else if ((_buffer.nav.fix_type & FIX_MASK) == FIX_3D) {
            gps.state.status = gps_t::GPS_OK_FIX_3D;
        }else{
            gps.state.status = gps_t::GPS_OK_FIX_2D;
        }
        gps.state.location.lat      = gps_t::lat_lon_type{swap_int32(_buffer.nav.latitude)};
        gps.state.location.lon      = gps_t::lat_lon_type{swap_int32(_buffer.nav.longitude)};
        gps.state.location.alt      = gps_t::altitude_type{swap_int32(_buffer.nav.altitude_msl)};
        gps.state.ground_speed      = swap_int32(_buffer.nav.ground_speed)*0.01f;
        gps.state.ground_course_cd  = wrap_360_cd(swap_int16(_buffer.nav.ground_course));
        gps.state.num_sats          = _buffer.nav.satellites;
        fill_3d_velocity();
        return true;
    }
    return false;
}

void apm::AP_GPS_SIRF::_accumulate(uint8_t val)
{
    _checksum = (_checksum + val) & 0x7fff;
}



/*
  detect a SIRF GPS
 */
bool apm::AP_GPS_SIRF::_detect(struct SIRF_detect_state &det_state, uint8_t data)
{
	switch (det_state.step) {
	case 1:
		if (PREAMBLE2 == data) {
			det_state.step++;
			break;
		}
		det_state.step = 0;
	case 0:
		det_state.payload_length = det_state.payload_counter = det_state.checksum = 0;
		if (PREAMBLE1 == data)
			det_state.step++;
		break;
	case 2:
		det_state.step++;
		if (data != 0) {
			// only look for short messages
			det_state.step = 0;
		}
		break;
	case 3:
		det_state.step++;
		det_state.payload_length = data;
		break;
	case 4:
		det_state.checksum = (det_state.checksum + data) & 0x7fff;
		if (++det_state.payload_counter == det_state.payload_length)
			det_state.step++;
		break;
	case 5:
		det_state.step++;
		if ((det_state.checksum >> 8) != data) {
			det_state.step = 0;
		}
		break;
	case 6:
		det_state.step = 0;
		if ((det_state.checksum & 0xff) == data) {
			return true;
		}
    }
    return false;
}