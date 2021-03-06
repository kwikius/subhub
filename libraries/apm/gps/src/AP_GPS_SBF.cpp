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
//  Septentrio GPS driver for ArduPilot.
//  Code by Michael Oborne
//

/*
#############################
Changed by Andy Little Apr 2016
###########################
*/

#include <cstring>
#include <apm/math.hpp>
#include <apm/serial_port.hpp>
#include <quan/stm32/millis.hpp>

#include "AP_GPS_SBF.h"

apm::AP_GPS_SBF::AP_GPS_SBF(apm::gps_t &_gps) :
    AP_GPS_Backend(_gps,"SBF",apm::gps_t::GPS_TYPE_SBF)
{	
   sbf_msg.sbf_state = sbf_msg_parser_t::PREAMBLE1;
	gps.port->write((const uint8_t*)_initialisation_blob[0], strlen(_initialisation_blob[0]));
}

// Process all bytes available from the stream
//
bool apm::AP_GPS_SBF::read(void)
{
	uint32_t now = quan::stm32::millis().numeric_value();
	
	if (_init_blob_index < (sizeof(_initialisation_blob) / sizeof(_initialisation_blob[0]))) {
		if (now > _init_blob_time) {
			gps.port->write((const uint8_t*)_initialisation_blob[_init_blob_index], strlen(_initialisation_blob[_init_blob_index]));
			_init_blob_time = now + 70;
			_init_blob_index++;
		}
	}
	
    bool ret = false;
    while (gps.port->available() > 0) {
        uint8_t temp = gps.port->read();
        ret |= parse(temp);
    }

    return ret;
}

bool apm::AP_GPS_SBF::parse(uint8_t temp)
{
    switch (sbf_msg.sbf_state)
    {
        default:
        case sbf_msg_parser_t::PREAMBLE1:
            if (temp == SBF_PREAMBLE1) {
                sbf_msg.sbf_state = sbf_msg_parser_t::PREAMBLE2;
                sbf_msg.read = 0;
            }
            break;
        case sbf_msg_parser_t::PREAMBLE2:
            if (temp == SBF_PREAMBLE2) {
                sbf_msg.sbf_state = sbf_msg_parser_t::CRC1;
            }
            else
            {
                sbf_msg.sbf_state = sbf_msg_parser_t::PREAMBLE1;
            }
            break;
        case sbf_msg_parser_t::CRC1:
            sbf_msg.crc = temp;
            sbf_msg.sbf_state = sbf_msg_parser_t::CRC2;
            break;
        case sbf_msg_parser_t::CRC2:
            sbf_msg.crc += (uint16_t)(temp << 8);
            sbf_msg.sbf_state = sbf_msg_parser_t::BLOCKID1;
            break;
        case sbf_msg_parser_t::BLOCKID1:
            sbf_msg.blockid = temp;
            sbf_msg.sbf_state = sbf_msg_parser_t::BLOCKID2;
            break;
        case sbf_msg_parser_t::BLOCKID2:
            sbf_msg.blockid += (uint16_t)(temp << 8);
            sbf_msg.sbf_state = sbf_msg_parser_t::LENGTH1;
            break;
        case sbf_msg_parser_t::LENGTH1:
            sbf_msg.length = temp;
            sbf_msg.sbf_state = sbf_msg_parser_t::LENGTH2;
            break;
        case sbf_msg_parser_t::LENGTH2:
            sbf_msg.length += (uint16_t)(temp << 8);
            sbf_msg.sbf_state = sbf_msg_parser_t::DATA;
            if (sbf_msg.length % 4 != 0) {
                sbf_msg.sbf_state = sbf_msg_parser_t::PREAMBLE1;
                //Debug("bad packet length=%u\n", (unsigned)sbf_msg.length);
            }
            break;
        case sbf_msg_parser_t::DATA:
            if (sbf_msg.read >= sizeof(sbf_msg.data)) {
                //Debug("parse overflow length=%u\n", (unsigned)sbf_msg.read);
                sbf_msg.sbf_state = sbf_msg_parser_t::PREAMBLE1;
                break;
            }
            sbf_msg.data.bytes[sbf_msg.read] = temp;
            sbf_msg.read++;
            if (sbf_msg.read >= (sbf_msg.length - 8)) {
                uint16_t crc = crc16_ccitt((uint8_t*)&sbf_msg.blockid, 2, 0);
                crc = crc16_ccitt((uint8_t*)&sbf_msg.length, 2, crc);
                crc = crc16_ccitt((uint8_t*)&sbf_msg.data, sbf_msg.length - 8, crc);

                sbf_msg.sbf_state = sbf_msg_parser_t::PREAMBLE1;

                if (sbf_msg.crc == crc) {
                    return process_message();
                } else {
                   // Debug("crc fail\n");
                    crc_error_counter++;
                }
            }
            break;
    }

    return false;
}

void apm::AP_GPS_SBF::log_ExtEventPVTGeodetic(const msg4007 &temp)
{

#if 0
    if (gps._DataFlash == NULL || !gps._DataFlash->logging_started()) {
        return;
    }

    uint64_t now = AP_HAL::micros64();

    struct log_GPS_SBF_EVENT header = {
        LOG_PACKET_HEADER_INIT(LOG_GPS_SBF_EVENT_MSG),
        time_us:now,
        TOW:temp.TOW,
        WNc:temp.WNc,
        Mode:temp.Mode,
        Error:temp.Error,
        Latitude:ToDeg(temp.Latitude),
        Longitude:ToDeg(temp.Longitude),
        Height:temp.Height,
        Undulation:temp.Undulation,
        Vn:temp.Vn,
        Ve:temp.Ve,
        Vu:temp.Vu,
        COG:temp.COG
    };

    gps._DataFlash->WriteBlock(&header, sizeof(header));
#endif
}

bool apm::AP_GPS_SBF::process_message(void)
{
    uint16_t blockid = (sbf_msg.blockid & 4095u);

   // Debug("BlockID %d", blockid);

    // ExtEventPVTGeodetic
    if (blockid == 4038) {
        log_ExtEventPVTGeodetic(sbf_msg.data.msg4007u);
    }
    // PVTGeodetic
    if (blockid == 4007) {
        const msg4007 &temp = sbf_msg.data.msg4007u;

        // Update time state
        if (temp.WNc != 65535) {
            gps.state.time_week = temp.WNc;
            gps.state.time_week_ms = (uint32_t)(temp.TOW);
        }
		
		  gps.state.last_gps_time_ms = quan::stm32::millis().numeric_value();
        gps.state.hdop = last_hdop;

        // Update velocity state (dont use −2·10^10)
        if (temp.Vn > -200000) {
            gps.state.velocity.x = gps_t::velocity_type{temp.Vn};
            gps.state.velocity.y = gps_t::velocity_type{temp.Ve};
            gps.state.velocity.z = gps_t::velocity_type{-temp.Vu};
			
			   gps.state.have_vertical_velocity = true;

            float ground_vector_sq = gps.state.velocity[0].numeric_value() * gps.state.velocity[0].numeric_value() 
                  + gps.state.velocity[1].numeric_value() * gps.state.velocity[1].numeric_value();
            gps.state.ground_speed = (float)safe_sqrt(ground_vector_sq);
            gps.state.ground_course_cd = (int32_t)(100 * ToDeg(atan2f(gps.state.velocity[1].numeric_value(), gps.state.velocity[0].numeric_value())));
            gps.state.ground_course_cd = wrap_360_cd(gps.state.ground_course_cd);
			
			gps.state.horizontal_accuracy = (float)temp.HAccuracy * 0.01f;
			gps.state.vertical_accuracy = (float)temp.VAccuracy * 0.01f;
			gps.state.have_horizontal_accuracy = true;
			gps.state.have_vertical_accuracy = true;
        }

        // Update position state (dont use −2·10^10)
        if (temp.Latitude > -200000) {
            gps.state.location.lat = gps_t::lat_lon_type{temp.Latitude * RAD_TO_DEG_DOUBLE * 1e7};
            gps.state.location.lon = gps_t::lat_lon_type{temp.Longitude * RAD_TO_DEG_DOUBLE * 1e7};
            gps.state.location.alt = gps_t::altitude_type{(float)temp.Height * 1e2f};
        }

        if (temp.NrSV != 255) {
            gps.state.num_sats = temp.NrSV;
        }

        //Debug("temp.Mode=0x%02x\n", (unsigned)temp.Mode);
        switch (temp.Mode & 15) {
            case 0: // no pvt
                gps.state.status = gps_t::NO_FIX;
                break;
            case 1: // standalone
                gps.state.status = gps_t::FIX_3D;
                break;
            case 2: // dgps
                gps.state.status = gps_t::FIX_3D_DGPS;
                break;
            case 3: // fixed location
                gps.state.status = gps_t::FIX_3D;
                break;
            case 4: // rtk fixed
                gps.state.status = gps_t::FIX_3D_RTK;
                break;
            case 5: // rtk float
                gps.state.status = gps_t::FIX_3D_DGPS;
                break;
            case 6: // sbas
                gps.state.status = gps_t::FIX_3D;
                break;
            case 7: // moving rtk fixed
                gps.state.status = gps_t::FIX_3D_RTK;
                break;
            case 8: // moving rtk float
                gps.state.status = gps_t::FIX_3D_DGPS;
                break;
        }
        
        if ((temp.Mode & 64) > 0) // gps is in base mode
            gps.state.status = gps_t::NO_FIX;
        if ((temp.Mode & 128) > 0) // gps only has 2d fix
            gps.state.status = gps_t::FIX_2D;
                    
        return true;
    }
    // DOP
    if (blockid == 4001) {
        const msg4001 &temp = sbf_msg.data.msg4001u;

        last_hdop = temp.HDOP;
		
		gps.state.hdop = last_hdop;
    }

    return false;
}

void apm::AP_GPS_SBF::inject_data(uint8_t *data, uint8_t len)
{
    if (gps.port->txspace() > len) {
        last_injected_data_ms = quan::stm32::millis().numeric_value();
        gps.port->write(data, len);
    } else {
        //Debug("SBF: Not enough TXSPACE");
    }
}
