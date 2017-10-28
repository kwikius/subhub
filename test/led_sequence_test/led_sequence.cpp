
/*
based on
https://github.com/bitcraze/crazyflie-firmware/blob/master/src/drivers/src/ws2812_cf2.c

which is 
 * Copyright (C) 2011-2014 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * ws2812.c: Neopixel LED driver
 * Mostly from Elia's electonic blog: http://eliaselectronics.com/driving-a-ws2812-rgb-led-with-an-stm32/
 */

/*
use pa6 as neo_pixel_pin 
 use TIM16_CH1  ( AF5 for pa6)
*/
#include "../../led_sequence.hpp"


// set 