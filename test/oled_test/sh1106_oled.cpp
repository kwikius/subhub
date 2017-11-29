
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
    apply(or_cmd::set_common_output_scan_direction,8); // flip the display vertically
    apply(or_cmd::set_segment_remap,1);
    apply(or_cmd::set_display_on,1); // turn on display
}

namespace {

   typedef quan::two_d::vect<int16_t> point;

   bool is_in_range(point const & p)
   {
      return (p.x >= 0) && (p.x < sh1106_oled::columns) && (p.y >= 0) && (p.y <  sh1106_oled::rows);
   }

   // requires is_in_range(p)
   // return a value from 0 to 7
   constexpr inline uint8_t get_page(point const & p) 
   {
         return p.y / 8;
   }

   // requires in range(p)
   // returns a value from 
   constexpr inline uint16_t get_byte_index(point const & p)
   {
      return sh1106_oled::columns * get_page(p) + p.x;
   }

   // requires is_in_range(p) 
   constexpr inline uint16_t get_bit_pos(point const & p)
   {
      return p.y % 8;
   }

   // convert from an index and bit to a point
   constexpr inline point get_point_from_index(int16_t idx, uint8_t bitpos)
   {
     return { idx % sh1106_oled::columns,(idx / sh1106_oled::columns ) * 8 + bitpos};
   }

}

void sh1106_oled::set_pixel(point const & p, bool colour)
{
    if ( is_in_range(p)){
      auto const byte_idx = get_byte_index(p);
      auto const bit_pos = get_bit_pos(p);
      auto & r = buffer[byte_idx];
      r = colour
         ?(r | (1U << bit_pos)) 
         :(r & ~(1U << bit_pos))
      ;
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
      write_data(buffer+buffer_idx,columns);
      buffer_idx += columns;
   }
}

void sh1106_oled::set_buffer_to(int val)
{
   auto const n = (rows * columns) / 8U;
   memset(buffer,val,n);
}
