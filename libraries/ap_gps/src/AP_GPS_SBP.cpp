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
//  Swift Navigation SBP GPS driver for ArduPilot.
//	Code by Niels Joubert
//
//  Swift Binary Protocol format: http://docs.swift-nav.com/
//

#include <cstring>

#include <quan/stm32/millis.hpp>
#include <ap_serialport/serialport.hpp>
#include <ap_math/ap_math.hpp>

#include "../gps.h"
#include "AP_GPS_SBP.h"

#define SBP_TIMEOUT_HEATBEAT  4000
#define SBP_TIMEOUT_PVT       500

bool apm::AP_GPS_SBP::logging_started = false;


apm::AP_GPS_SBP::AP_GPS_SBP(apm::gps_t &_gps, apm::gps_t::GPS_State &_state,
                       apm::SerialPort *_port) :
    apm::AP_GPS_Backend(_gps, _state, _port),

    last_injected_data_ms(0),
    last_iar_num_hypotheses(0),
    last_full_update_tow(0),
    last_full_update_cpu_ms(0),
    crc_error_counter(0)
{

    //Debug("SBP Driver Initialized");

    parser_state.state = sbp_parser_state_t::WAITING;

    //Externally visible state
    state.status = gps_t::NO_FIX;
    state.have_vertical_velocity = true;
    state.last_gps_time_ms = last_heatbeat_received_ms = quan::stm32::millis().numeric_value();
}

// Process all bytes available from the stream
//
bool apm::AP_GPS_SBP::read(void)
{

    //Invariant: Calling this function processes *all* data current in the UART buffer.
    //
    //IMPORTANT NOTICE: This function is NOT CALLED for several seconds
    // during arming. That should not cause the driver to die. Process *all* waiting messages

    _sbp_process();

    return _attempt_state_update();

}

void  apm::AP_GPS_SBP::inject_data(uint8_t *data, uint8_t len)
{

    if (port->txspace() > len) {
        last_injected_data_ms = quan::stm32::millis().numeric_value();
        port->write(data, len);
    } else {
       // Debug("PIKSI: Not enough TXSPACE");
    }

}

//This attempts to reads all SBP messages from the incoming port.
//Returns true if a new message was read, false if we failed to read a message.
void apm::AP_GPS_SBP::_sbp_process() 
{

    while (port->available() > 0) {
        uint8_t temp = port->read();
        uint16_t crc;


        //This switch reads one character at a time,
        //parsing it into buffers until a full message is dispatched
        switch(parser_state.state) {
            case sbp_parser_state_t::WAITING:
                if (temp == SBP_PREAMBLE) {
                    parser_state.n_read = 0;
                    parser_state.state = sbp_parser_state_t::GET_TYPE;
                }
                break;

            case sbp_parser_state_t::GET_TYPE:
                *((uint8_t*)&(parser_state.msg_type) + parser_state.n_read) = temp;
                parser_state.n_read += 1;
                if (parser_state.n_read >= 2) {
                    parser_state.n_read = 0;
                    parser_state.state = sbp_parser_state_t::GET_SENDER;
                }
                break;

            case sbp_parser_state_t::GET_SENDER:
                *((uint8_t*)&(parser_state.sender_id) + parser_state.n_read) = temp;
                parser_state.n_read += 1;
                if (parser_state.n_read >= 2) {
                    parser_state.n_read = 0;
                    parser_state.state = sbp_parser_state_t::GET_LEN;
                }
                break;

            case sbp_parser_state_t::GET_LEN:
                parser_state.msg_len = temp;
                parser_state.n_read = 0;
                parser_state.state = sbp_parser_state_t::GET_MSG;
                break;

            case sbp_parser_state_t::GET_MSG:
                *((uint8_t*)&(parser_state.msg_buff) + parser_state.n_read) = temp;
                parser_state.n_read += 1;
                if (parser_state.n_read >= parser_state.msg_len) {
                    parser_state.n_read = 0;
                    parser_state.state = sbp_parser_state_t::GET_CRC;
                }
                break;

            case sbp_parser_state_t::GET_CRC:
                *((uint8_t*)&(parser_state.crc) + parser_state.n_read) = temp;
                parser_state.n_read += 1;
                if (parser_state.n_read >= 2) {
                    parser_state.state = sbp_parser_state_t::WAITING;

                    crc = crc16_ccitt((uint8_t*)&(parser_state.msg_type), 2, 0);
                    crc = crc16_ccitt((uint8_t*)&(parser_state.sender_id), 2, crc);
                    crc = crc16_ccitt(&(parser_state.msg_len), 1, crc);
                    crc = crc16_ccitt(parser_state.msg_buff, parser_state.msg_len, crc);
                    if (parser_state.crc == crc) {
                        _sbp_process_message();
                    } else {
                        //Debug("CRC Error Occurred!");
                        crc_error_counter += 1;
                    }

                    parser_state.state = sbp_parser_state_t::WAITING;                
                }
                break;

            default:
                parser_state.state = sbp_parser_state_t::WAITING;
                break;
            }
    }
}


//INVARIANT: A fully received message with correct CRC is currently in parser_state
void apm::AP_GPS_SBP::_sbp_process_message() {
    switch(parser_state.msg_type) {
        case SBP_HEARTBEAT_MSGTYPE:
            last_heatbeat_received_ms = quan::stm32::millis().numeric_value();
            break;

        case SBP_GPS_TIME_MSGTYPE:
            memcpy(&last_gps_time, parser_state.msg_buff, sizeof(last_gps_time));
            break;

        case SBP_VEL_NED_MSGTYPE:
            memcpy(&last_vel_ned, parser_state.msg_buff, sizeof(last_vel_ned));
            break;

        case SBP_POS_LLH_MSGTYPE: {
            struct sbp_pos_llh_t *pos_llh = (struct sbp_pos_llh_t*)parser_state.msg_buff;
            // Check if this is a single point or RTK solution
            // flags = 0 -> single point
            if (pos_llh->flags == 0) {
                last_pos_llh_spp = *pos_llh;
            } else if (pos_llh->flags == 1 || pos_llh->flags == 2) {
                last_pos_llh_rtk = *pos_llh;
            }
            break;
        }

        case SBP_DOPS_MSGTYPE:
            memcpy(&last_dops, parser_state.msg_buff, sizeof(last_dops));
            break;

        case SBP_TRACKING_STATE_MSGTYPE:
            //INTENTIONALLY BLANK
            //Currenly unhandled, but logged after switch statement.
            break;

        case SBP_IAR_STATE_MSGTYPE: {
            sbp_iar_state_t *iar = (struct sbp_iar_state_t*)parser_state.msg_buff;
            last_iar_num_hypotheses = iar->num_hypotheses;
            break;
        }

        default:
            // log anyway if it's an unsupported message. 
            // The log mask will be used to adjust or suppress logging
            break; 
    }

    logging_log_raw_sbp(parser_state.msg_type, parser_state.sender_id, parser_state.msg_len, parser_state.msg_buff);
}

bool apm::AP_GPS_SBP::_attempt_state_update()
{

    // If we currently have heartbeats
    //    - NO FIX
    //
    // If we have a full update available, save it
    //
    uint32_t now = quan::stm32::millis().numeric_value();
    bool ret = false;

    if (now - last_heatbeat_received_ms > SBP_TIMEOUT_HEATBEAT) {
        
        state.status = gps_t::NO_GPS;
       // Debug("No Heartbeats from Piksi! Driver Ready to Die!");
        ret = false;

    } else if (last_pos_llh_rtk.tow == last_vel_ned.tow
            && abs((int32_t) (last_gps_time.tow - last_vel_ned.tow)) < 10000
            && abs((int32_t) (last_dops.tow - last_vel_ned.tow)) < 60000
            && last_vel_ned.tow > last_full_update_tow) {

        // Use the RTK position
        sbp_pos_llh_t *pos_llh = &last_pos_llh_rtk;

        // Update time state
        state.time_week         = last_gps_time.wn;
        state.time_week_ms      = last_vel_ned.tow;

        state.hdop              = last_dops.hdop;

        // Update velocity state
        state.velocity[0]       = gps_t::velocity_type{last_vel_ned.n / 1000.0};
        state.velocity[1]       = gps_t::velocity_type{last_vel_ned.e / 1000.0};
        state.velocity[2]       = gps_t::velocity_type{last_vel_ned.d / 1000.0};

        float ground_vector_sq = state.velocity[0].numeric_value()*state.velocity[0].numeric_value() 
                                    + state.velocity[1].numeric_value()*state.velocity[1].numeric_value();
        state.ground_speed = safe_sqrt(ground_vector_sq);

        state.ground_course_cd = wrap_360_cd((int32_t) 100*ToDeg(atan2f(state.velocity[1].numeric_value(), state.velocity[0].numeric_value())));

        // Update position state

        state.location.lat      = gps_t::lat_lon_type{pos_llh->lat*1e7};
        state.location.lon      = gps_t::lat_lon_type{pos_llh->lon*1e7};
        state.location.alt      = gps_t::altitude_type{pos_llh->height*1e2};
        state.num_sats          = pos_llh->n_sats;

        if (pos_llh->flags == 0)
            state.status = gps_t::GPS_OK_FIX_3D;
        else if (pos_llh->flags == 2)
            state.status = gps_t::GPS_OK_FIX_3D_DGPS;
        else if (pos_llh->flags == 1)
            state.status = gps_t::GPS_OK_FIX_3D_RTK;
 
        last_full_update_tow = last_vel_ned.tow;
        last_full_update_cpu_ms = now;
        logging_log_full_update();
        ret = true;

    } else if (now - last_full_update_cpu_ms > SBP_TIMEOUT_PVT) {

        //INVARIANT: If we currently have a fix, ONLY return true after a full update.
        state.status = gps_t::NO_FIX;
        ret = true;
    } else {
        //No timeouts yet, no data yet, nothing has happened.
        ret = false;
    }
    return ret;
}

bool apm::AP_GPS_SBP::_detect(struct SBP_detect_state &state, uint8_t data)
{
    // This switch reads one character at a time, if we find something that
    // looks like our preamble we'll try to read the full message length,
    // calculating the CRC. If the CRC matches, we have an SBP GPS!

    switch(state.state) {
        case SBP_detect_state::WAITING:
            if (data == SBP_PREAMBLE) {
                state.n_read = 0;
                state.crc_so_far = 0;
                state.state = SBP_detect_state::GET_TYPE;
            }
            break;

        case SBP_detect_state::GET_TYPE:
            state.crc_so_far = crc16_ccitt(&data, 1, state.crc_so_far);
            state.n_read += 1;
            if (state.n_read >= 2) {
                state.n_read = 0;
                state.state = SBP_detect_state::GET_SENDER;
            }
            break;

        case SBP_detect_state::GET_SENDER:
            state.crc_so_far = crc16_ccitt(&data, 1, state.crc_so_far);
            state.n_read += 1;
            if (state.n_read >= 2) {
                state.n_read = 0;
                state.state = SBP_detect_state::GET_LEN;
            }
            break;

        case SBP_detect_state::GET_LEN:
            state.crc_so_far = crc16_ccitt(&data, 1, state.crc_so_far);
            state.msg_len = data;
            state.n_read = 0;
            state.state = SBP_detect_state::GET_MSG;
            break;

        case SBP_detect_state::GET_MSG:
            state.crc_so_far = crc16_ccitt(&data, 1, state.crc_so_far);
            state.n_read += 1;
            if (state.n_read >= state.msg_len) {
                state.n_read = 0;
                state.state = SBP_detect_state::GET_CRC;
            }
            break;

        case SBP_detect_state::GET_CRC:
            *((uint8_t*)&(state.crc) + state.n_read) = data;
            state.n_read += 1;
            if (state.n_read >= 2) {
                state.state = SBP_detect_state::WAITING;
                return state.crc == state.crc_so_far;
            }
            break;

        default:
            state.state = SBP_detect_state::WAITING;
            break;
    }
    return false;
}
