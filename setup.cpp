
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

#include <stm32f0xx.h>
#include "resources.hpp"
#include "usarts.hpp"

void setup_events()
{
   NVIC_SetPriority(SysTick_IRQn,interrupt_priority::systick_timer);
   SysTick_Config(SystemCoreClock / 1000);
}

void setup_comm_channel()
{
   // aux_sp::serial_port::init();
    link_sp::serial_port::init();
    
    link_sp::serial_port::set_irq_priority(interrupt_priority::channel_port);

}

// In setup setup comm channle only
extern "C" void setup()
{

  setup_comm_channel();

}

