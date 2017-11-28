
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

namespace {

 //  constexpr int num_columns = 132;
//   constexpr int num_rows = 64;

   typedef quan::two_d::vect<int16_t> point;

   bool is_in_range(point const & p)
   {
      return (p.x >= 0) && (p.x < sh1106_oled::columns) && (p.y >= 0) && (p.y <  sh1106_oled::rows);
        
   }

   // requires is_in_range(p)
   // return a value from 0 to 7
   constexpr uint8_t get_page(point const & p) 
   {
         return p.y / 8;
   }

   // requires in range(p)
   // returns a value from 
   constexpr uint16_t get_byte_index(point const & p)
   {
      return sh1106_oled::columns * get_page(p) + p.x;
   }

   // requires is_in_range(p) 
   constexpr uint16_t get_bit_pos(point const & p)
   {
      return p.y % 8;
   }

   // convert from an index and bit to a point
   point get_point_from_index(int16_t idx, uint8_t bitpos)
   {
      point result;
      result.x = idx % sh1106_oled::columns;
      result.y = (idx / sh1106_oled::columns ) * 8 + bitpos;
      return result;
   }

   //uint8_t buffer[132 * 8];

}

void sh1106_oled::set_pixel(point const & p, bool colour)
{
//   if ( (x < columns) && ( y < rows)){
//       uint32_t const buffer_bit_pos = y * columns + x;
//       uint32_t const buffer_byte = buffer_bit_pos / 8U;
//       uint32_t const buffer_bit = buffer_bit_pos % 8U;
//       uint32_t const mask = (1U << buffer_bit);
//       if (colour){
//           buffer[buffer_byte] |= mask;
//       }else{
//          buffer[buffer_byte] &= ~mask;
//       }
//   }
    if ( is_in_range(p)){
        if (colour){
          buffer[get_byte_index(p)] |= (1U << get_bit_pos(p));
        }else{
          buffer[get_byte_index(p)] &= ~(1U << get_bit_pos(p));
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
