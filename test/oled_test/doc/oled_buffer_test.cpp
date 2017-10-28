#include <iostream>

#include <quan/two_d/out/vect.hpp>
#include <quan_matters/test/test.hpp>
#include <quan/meta/max.hpp>
/*
access to sh1106 oled display pixels. How xy positions relate to memory
*/

namespace {

   constexpr int num_columns = 132;
   constexpr int num_rows = 64;

   typedef quan::two_d::vect<int16_t> point;

   bool is_in_range(point const & p)
   {
      return (p.x >= 0) && (p.x < num_columns) && (p.y >= 0) && (p.y < num_rows);
        
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
      return num_columns * get_page(p) + p.x;
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
      result.x = idx % num_columns;
      result.y = (idx / num_columns ) * 8 + bitpos;
      return result;
   }

   uint8_t buffer[132 * 8];

}

int errors = 0;
int main()
{
   
   QUAN_CHECK(is_in_range({0,0}) == true)
   QUAN_CHECK(is_in_range({num_columns - 1,num_rows - 1 }))
   QUAN_CHECK(is_in_range({num_columns - 1,num_rows - 1 }))

   QUAN_CHECK(is_in_range({num_columns,7}) == false)
   QUAN_CHECK(is_in_range({-1,7}) == false)
   QUAN_CHECK(is_in_range({20,num_rows}) == false)
   QUAN_CHECK(is_in_range({20,-1}) == false)

   QUAN_CHECK(get_page({0,0}) == 0)
   QUAN_CHECK(get_page({0,7}) == 0)
   QUAN_CHECK(get_page({0,8}) == 1)
   QUAN_CHECK(get_page({0,0xF}) == 1)
   QUAN_CHECK(get_page({0,num_rows -1}) == 7)

   QUAN_CHECK(get_byte_index({0,0}) == 0)

   QUAN_CHECK(get_byte_index({1,0}) == 1)

   QUAN_CHECK(get_byte_index({0,1}) == 0)
   QUAN_CHECK(get_byte_index({10,7}) == 10)
   QUAN_CHECK(get_byte_index({10,8}) == 142)
   QUAN_CHECK(get_byte_index({131,63}) == (num_columns * num_rows)/8 -1)

   QUAN_CHECK( (get_point_from_index(0,0) == point{0,0}) )
   QUAN_CHECK( (get_point_from_index(1,1) == point{1,1}) )
   QUAN_CHECK( (get_point_from_index(2,0) == point{2,0}) )
   QUAN_CHECK( (get_point_from_index(3,0) == point{3,0}) )
   QUAN_CHECK( (get_point_from_index(132,0) == point{0,8}) )

   QUAN_CHECK( (get_point_from_index(132,1) == point{0,9}) )
   QUAN_CHECK( (get_point_from_index(133,1) == point{1,9}) )

   QUAN_CHECK( (num_columns * num_rows)  < quan::meta::max_<int16_t>::value)
   
   point p1 = {131,7};

   /*
      to set the bit
   */
   buffer[get_byte_index(p1)] |= (1 << get_bit_pos(p1));

  EPILOGUE
}