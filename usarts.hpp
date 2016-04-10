#ifndef QUANTRACKER_SERIAL_PORTS_HPP_INCLUDED
#define QUANTRACKER_SERIAL_PORTS_HPP_INCLUDED

/*
 Copyright (c) 2013 Andy Little 

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#include <quan/stm32/serial_port.hpp>
#include "resources.hpp"

struct comm_channel_sp{

   typedef comm_channel_txo_pin txo_pin;
   typedef comm_channel_rxi_pin rxi_pin;
   typedef comm_channel_uart uart;
   static constexpr uint32_t in_buf_size = 200;
   static constexpr uint32_t out_buf_size = 200;

   typedef quan::stm32::serial_port<
      uart,out_buf_size,in_buf_size,txo_pin,rxi_pin
   > serial_port;
};

struct aux_sp{
   typedef uart_txo_pin txo_pin;
   typedef uart_rxi_pin rxi_pin;

   static constexpr uint32_t in_buf_size = 100;
   static constexpr uint32_t out_buf_size = 100;
   typedef aux_uart uart;
   typedef quan::stm32::serial_port<
      uart,out_buf_size,in_buf_size,txo_pin,rxi_pin
   > serial_port;
};

#endif // QUANTRACKER_SERIAL_PORTS_HPP_INCLUDED
