#include <iostream>

#include <quan/two_d/out/vect.hpp>


namespace {

   typedef quan::two_d::vect<int> point;

   int get_page(point const & p) 
   {
      return p.y / 8;
   }

   int get_byte_index(point const & p)
   {
      return 132 * get_page(p) + p.x;
   }

   int get_bit_pos(point const & p)
   {
      return p.y % 8;
   }

   uint8_t buffer[132 * 8];

}

int main()
{
   point p = {1,8};

   std::cout << p <<'\n';

   std::cout << "page       = " << get_page(p) << '\n';

   std::cout << "byte index = " << get_byte_index(p) << '\n';

   std::cout << "bit pos    = " << get_bit_pos(p) << '\n';

   /*
      to set the bit
   */
   buffer[get_byte_index(p)] |= (1 << get_bit_pos(p));
}