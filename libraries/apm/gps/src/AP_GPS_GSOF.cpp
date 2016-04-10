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
//  Trimble GPS driver for ArduPilot.
//  Code by Michael Oborne
//

#include "AP_GPS_GSOF.h"
#include <apm/serial_port.hpp>
#include <quan/stm32/millis.hpp>
#include <ap_math/ap_math.hpp>

apm::AP_GPS_GSOF::AP_GPS_GSOF(apm::gps_t &_gps) :
    apm::AP_GPS_Backend(_gps)
{
    gsof_msg.gsof_state = gsof_msg_parser_t::STARTTX;

    // baud request for port 0
    requestBaud(0);
    // baud request for port 3
    requestBaud(3);

    uint32_t now = quan::stm32::millis().numeric_value();
    gsofmsg_time = now + 110;
}

// Process all bytes available from the stream
//
bool apm::AP_GPS_GSOF::read(void)
{
    uint32_t now = quan::stm32::millis().numeric_value();

    if (gsofmsgreq_index < (sizeof(gsofmsgreq))) {
        if (now > gsofmsg_time) {
            requestGSOF(gsofmsgreq[gsofmsgreq_index], 0);
            requestGSOF(gsofmsgreq[gsofmsgreq_index], 3);
            gsofmsg_time = now + 110;
            gsofmsgreq_index++;
        }
    }

    bool ret = false;
    while (gps.port->available() > 0) {
        uint8_t temp = gps.port->read();
        ret |= parse(temp);
    }

    return ret;
}

bool apm::AP_GPS_GSOF::parse(uint8_t temp)
{
    switch (gsof_msg.gsof_state)
    {
    default:
    case gsof_msg_parser_t::STARTTX:
        if (temp == GSOF_STX)
        {
            gsof_msg.starttx = temp;
            gsof_msg.gsof_state = gsof_msg_parser_t::STATUS;
            gsof_msg.read = 0;
            gsof_msg.checksumcalc = 0;
        }
        break;
    case gsof_msg_parser_t::STATUS:
        gsof_msg.status = temp;
        gsof_msg.gsof_state = gsof_msg_parser_t::PACKETTYPE;
        gsof_msg.checksumcalc += temp;
        break;
    case gsof_msg_parser_t::PACKETTYPE:
        gsof_msg.packettype = temp;
        gsof_msg.gsof_state = gsof_msg_parser_t::LENGTH;
        gsof_msg.checksumcalc += temp;
        break;
    case gsof_msg_parser_t::LENGTH:
        gsof_msg.length = temp;
        gsof_msg.gsof_state = gsof_msg_parser_t::DATA;
        gsof_msg.checksumcalc += temp;
        break;
    case gsof_msg_parser_t::DATA:
        gsof_msg.data[gsof_msg.read] = temp;
        gsof_msg.read++;
        gsof_msg.checksumcalc += temp;
        if (gsof_msg.read >= gsof_msg.length)
        {
            gsof_msg.gsof_state = gsof_msg_parser_t::CHECKSUM;
        }
        break;
    case gsof_msg_parser_t::CHECKSUM:
        gsof_msg.checksum = temp;
        gsof_msg.gsof_state = gsof_msg_parser_t::ENDTX;
        if (gsof_msg.checksum == gsof_msg.checksumcalc)
        {
            return process_message();
        }
        break;
    case gsof_msg_parser_t::ENDTX:
        gsof_msg.endtx = temp;
        gsof_msg.gsof_state = gsof_msg_parser_t::STARTTX;
        break;
    }

    return false;
}

void apm::AP_GPS_GSOF::requestBaud(uint8_t portindex)
{
    uint8_t buffer[19] = {0x02,0x00,0x64,0x0d,0x00,0x00,0x00, // application file record
                          0x03, 0x00, 0x01, 0x00, // file control information block
                          0x02, 0x04, portindex, 0x07, 0x00,0x00, // serial port baud format
                          0x00,0x03
                         }; // checksum

    buffer[4] = packetcount++;

    uint8_t checksum = 0;
    for (uint8_t a = 1; a < (sizeof(buffer) - 1); a++) {
        checksum += buffer[a];
    }

    buffer[17] = checksum;

    gps.port->write((const uint8_t*)buffer, sizeof(buffer));
}

void apm::AP_GPS_GSOF::requestGSOF(uint8_t messagetype, uint8_t portindex)
{
    uint8_t buffer[21] = {0x02,0x00,0x64,0x0f,0x00,0x00,0x00, // application file record
                          0x03,0x00,0x01,0x00, // file control information block
                          0x07,0x06,0x0a,portindex,0x01,0x00,0x01,0x00, // output message record
                          0x00,0x03
                         }; // checksum

    buffer[4] = packetcount++;
    buffer[17] = messagetype;

    uint8_t checksum = 0;
    for (uint8_t a = 1; a < (sizeof(buffer) - 1); a++) {
        checksum += buffer[a];
    }

    buffer[19] = checksum;

    gps.port->write((const uint8_t*)buffer, sizeof(buffer));
}

double apm::AP_GPS_GSOF::SwapDouble(uint8_t* src, uint32_t pos)
{
    union {
        double d;
        char bytes[sizeof(double)];
    } doubleu;
    doubleu.bytes[0] = src[pos + 7];
    doubleu.bytes[1] = src[pos + 6];
    doubleu.bytes[2] = src[pos + 5];
    doubleu.bytes[3] = src[pos + 4];
    doubleu.bytes[4] = src[pos + 3];
    doubleu.bytes[5] = src[pos + 2];
    doubleu.bytes[6] = src[pos + 1];
    doubleu.bytes[7] = src[pos + 0];

    return doubleu.d;
}

float apm::AP_GPS_GSOF::SwapFloat(uint8_t* src, uint32_t pos)
{
    union {
        float f;
        char bytes[sizeof(float)];
    } floatu;
    floatu.bytes[0] = src[pos + 3];
    floatu.bytes[1] = src[pos + 2];
    floatu.bytes[2] = src[pos + 1];
    floatu.bytes[3] = src[pos + 0];

    return floatu.f;
}

uint32_t apm::AP_GPS_GSOF::SwapUint32(uint8_t* src, uint32_t pos)
{
    union {
        uint32_t u;
        char bytes[sizeof(uint32_t)];
    } uint32u;
    uint32u.bytes[0] = src[pos + 3];
    uint32u.bytes[1] = src[pos + 2];
    uint32u.bytes[2] = src[pos + 1];
    uint32u.bytes[3] = src[pos + 0];

    return uint32u.u;
}

uint16_t apm::AP_GPS_GSOF::SwapUint16(uint8_t* src, uint32_t pos)
{
    union {
        uint16_t u;
        char bytes[sizeof(uint16_t)];
    } uint16u;
    uint16u.bytes[0] = src[pos + 1];
    uint16u.bytes[1] = src[pos + 0];

    return uint16u.u;
}

bool apm::AP_GPS_GSOF::process_message(void)
{
    //http://www.trimble.com/OEM_ReceiverHelp/V4.81/en/default.html#welcome.html

    if (gsof_msg.packettype == 0x40) { // GSOF
        int valid = 0;
        // want 1 2 8 9 12
        for (uint32_t a = 3; a < gsof_msg.length; a++)
        {
            uint8_t output_type = gsof_msg.data[a];
            a++;
            uint8_t output_length = gsof_msg.data[a];
            a++;
            //Debug("GSOF type: " + output_type + " len: " + output_length);
            if (output_type == 1) // pos time
            {
                gps.state.time_week_ms = SwapUint32(gsof_msg.data, a);
                gps.state.time_week = SwapUint16(gsof_msg.data, a + 4);
                gps.state.num_sats = gsof_msg.data[a + 6];
                uint8_t posf1 = gsof_msg.data[a + 7];
                uint8_t posf2 = gsof_msg.data[a + 8];
                //Debug("POSTIME: " + posf1 + " " + posf2);
                if ((posf1 & 1) == 1)
                {
                    gps.state.status = gps_t::GPS_OK_FIX_3D;
                    if ((posf2 & 1) == 1)
                    {
                        gps.state.status = gps_t::GPS_OK_FIX_3D_DGPS;
                        if ((posf2 & 4) == 4)
                        {
                            gps.state.status = gps_t::GPS_OK_FIX_3D_RTK;
                        }
                    }
                }
                else
                {
                    gps.state.status = gps_t::NO_FIX;
                }
                valid++;
            }
            else if (output_type == 2) // position
            {
                gps.state.location.lat = gps_t::lat_lon_type{RAD_TO_DEG_DOUBLE * (SwapDouble(gsof_msg.data, a)) * 1e7}; // deg1e7
                gps.state.location.lon = gps_t::lat_lon_type{RAD_TO_DEG_DOUBLE * (SwapDouble(gsof_msg.data, a + 8)) * 1e7}; // deg1e7
                gps.state.location.alt = gps_t::altitude_type{SwapDouble(gsof_msg.data, a + 16) * 1e2}; //cm

                gps.state.last_gps_time_ms = gps.state.time_week_ms;

                valid++;
            }
            else if (output_type == 8) // velocity
            {
                uint8_t vflag = gsof_msg.data[a];
                if ((vflag & 1) == 1)
                {
                    gps.state.ground_speed = SwapFloat(gsof_msg.data, a + 1);
                    gps.state.ground_course_cd = (int32_t)(ToDeg(SwapFloat(gsof_msg.data, a + 5)) * 100);
                    fill_3d_velocity();
                    gps.state.velocity.z = gps_t::velocity_type{-SwapFloat(gsof_msg.data, a + 9)};
                    gps.state.have_vertical_velocity = true;
                }
                valid++;
            }
            else if (output_type == 9) //dop
            {
                gps.state.hdop = (uint16_t)(SwapFloat(gsof_msg.data, a + 4) * 100);
                valid++;
            }
            else if (output_type == 12) // position sigma
            {
                gps.state.horizontal_accuracy = (SwapFloat(gsof_msg.data, a + 4) + SwapFloat(gsof_msg.data, a + 8)) / 2;
                gps.state.vertical_accuracy = SwapFloat(gsof_msg.data, a + 16);
                gps.state.have_horizontal_accuracy = true;
                gps.state.have_vertical_accuracy = true;
                valid++;
            }

            a += output_length-1u;
        }

        if (valid == 5) {
            return true;
        } else {
            gps.state.status = gps_t::NO_FIX;
        }
    }

    return false;
}

void apm::AP_GPS_GSOF::inject_data(uint8_t *data, uint8_t len)
{

    if (gps.port->txspace() > len) {
        last_injected_data_ms = quan::stm32::millis().numeric_value();
        gps.port->write(data, len);
    } else {
       // Debug("GSOF: Not enough TXSPACE");
    }
}
