
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

#include <quan/malloc_free.hpp>
#include <stm32f0xx.h>
#include <stddef.h> /* where ptrdiff_t is defined */
#include "resources.hpp"
#include "usarts.hpp"

extern "C" void __cxa_pure_virtual() { while (1); }
void *__dso_handle;


void * operator new(size_t size) throw()
{
  return quan::malloc(size);
}
void operator delete (void*p){ ;}

void setup_outputs()
{

}

void setup_events()
{
   NVIC_SetPriority(SysTick_IRQn,interrupt_priority::systick_timer);
   SysTick_Config(SystemCoreClock / 1000);
}

void setup_inputs()
{
    aux_sp::serial_port::init();
    comm_channel_sp::serial_port::init();
  // posdata_sp::serial_port::init();
   // add pullup to input
// inverted on v1 board
#if !defined(QUAN_DISCOVERY)
   typedef aux_sp::serial_port::usart_type usart;
   static constexpr uint8_t txinv_bit = 17;
//   bool const enabled = posdata_sp::serial_port::is_enabled();
//   if (enabled){
//      quan::stm32::disable<usart>();
//   }
   usart::get()->cr2.setbit<txinv_bit>();
//   if(enabled){
//      quan::stm32::enable<usart>();
//   }
#endif
   // N.B  for mavlink only
   // for GPS depends on config
  // posdata_sp::serial_port::set_baudrate<57600,false>();
   // posdata_sp::serial_port::set_irq_priority(interrupt_priority::telemetry_input_port);
}

extern "C" void setup()
{
  // first check if user wants to dialog
  // if not then ..
  // read config
  setup_outputs();
  setup_events();
  setup_inputs(); 
}

