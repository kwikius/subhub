
#include <quan/stm32/millis.hpp>
#include <cstring>
#include "../../usarts.hpp"
#include "sh1106_oled.hpp"


uint8_t sh1106_oled::buffer[(rows * columns) / 8U] = {0U};
bool sh1106_oled::wait_for_command_complete = true;
quan::time_<uint32_t>::ms sh1106_oled::max_cmd_wait{100};

using quan::stm32::millis;
namespace {

   typedef decltype(millis()) ms;

   ms operator "" _ms(unsigned long long int v)
   {
      return static_cast<ms>(v);
   }
   void delay(ms t)
   {
      ms elapsed = millis();
      while ((millis() - elapsed) < t) {;}
   }

   typedef link_sp::serial_port xout;

}

void sh1106_oled::initialise()
{
    delay(200_ms);
    apply(or_cmd::set_display_on,1); // turn on display
}

void sh1106_oled::set_pixel(uint32_t x, uint32_t y, bool colour)
{
   if ( (x < columns) && ( y < rows)){
       uint32_t const buffer_bit_pos = y * columns + x;
       uint32_t const buffer_byte = buffer_bit_pos / 8U;
       uint32_t const buffer_bit = buffer_bit_pos % 8U;
       uint32_t const mask = (1U << buffer_bit);
       if (colour){
           buffer[buffer_byte] |= mask;
       }else{
          buffer[buffer_byte] &= ~mask;
       }
   }
}

void sh1106_oled::write_buffer()
{
   apply(or_cmd::set_display_start_line, 0x00); 
   // set the page, column
   uint16_t buffer_idx = 0U;
   for ( uint8_t page = 0U; page < 8U; ++page){
      apply(or_cmd::set_lower_column_address, 0);
      apply(or_cmd::set_higher_column_address, 0);
      apply(or_cmd::set_page_address,page);
      write_data(buffer+buffer_idx,132);
      buffer_idx += 132U;
   }
}

void sh1106_oled::set_buffer_to(int val)
{
   auto const n = (rows * columns) / 8U;
   memset(buffer,val,n);
}
