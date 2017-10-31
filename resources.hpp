#ifndef FSK_TX_RESOURCES_HPP_INCLUDED
#define FSK_TX_RESOURCES_HPP_INCLUDED

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

#include <quan/stm32/usart.hpp>
#include <quan/stm32/gpio.hpp>
#include <quan/stm32/tim.hpp>

/*
  timers
   Systick for millis() fun
   TIM2   servos out
   TIM14  for PPM in
   TIM15 for ADC timer
   TIM16  for led_sequence timer

available
  TIM1
  TIM3
  TIM16
  TIM17
*/
/*
N.B on stm32F0, many pins are not 5 volt tolerant so beware
*/
typedef quan::mcu::pin<quan::stm32::gpioa,0>    analog1_in_pin; // user button on disco

typedef quan::mcu::pin<quan::stm32::gpioa,1>    servo2_out_pin;
typedef quan::mcu::pin<quan::stm32::gpioa,2>    analog_rssi_in_pin;

typedef quan::mcu::pin<quan::stm32::gpioa,5>    servo1_out_pin;


typedef quan::mcu::pin<quan::stm32::gpioa,6>    touch_electrode_pin;
typedef quan::mcu::pin<quan::stm32::gpioa,7>    touch_cap_pin;

typedef quan::mcu::pin<quan::stm32::gpioa,9>    link_txo_pin; 
typedef quan::mcu::pin<quan::stm32::gpioa,10>   link_rxi_pin; 

typedef quan::mcu::pin<quan::stm32::gpioa,11>   aux_led1_pin; 

typedef quan::mcu::pin<quan::stm32::gpioa,14>   uart_txo_pin; // swclk on disco
typedef quan::mcu::pin<quan::stm32::gpioa,15>   uart_rxi_pin;

typedef quan::mcu::pin<quan::stm32::gpiob,1>    ppm_in_pin;
typedef quan::mcu::pin<quan::stm32::gpiob,5>    digital_rssi_in_pin;
typedef quan::mcu::pin<quan::stm32::gpiob,6>    i2c_scl;
typedef quan::mcu::pin<quan::stm32::gpiob,7>    i2c_sda;
typedef quan::mcu::pin<quan::stm32::gpiob,8>    neopixel_pin;

// N.B 5V tolerant for connect via res to base of PNP transistor to 5V for LEDs
typedef quan::mcu::pin<quan::stm32::gpiob,11>    led_pwm_pin;

typedef quan::mcu::pin<quan::stm32::gpiob,13>    spi2_sck_pin;
typedef quan::mcu::pin<quan::stm32::gpiob,15>    spi2_mosi_pin;

typedef quan::stm32::usart1  link_uart;
typedef quan::stm32::usart2  aux_uart;

struct interrupt_priority{
   static constexpr uint32_t systick_timer = 3;
   static constexpr uint32_t gps_telem_port= 2;
   static constexpr uint32_t channel_port = 1;
   static constexpr uint32_t led_sequence = 0;
};

#endif // FSK_TX_RESOURCES_HPP_INCLUDED
