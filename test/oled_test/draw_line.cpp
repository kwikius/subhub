
#include "sh1106_oled.hpp"
#include <cmath>
#include <utility>

void sh1106_oled::draw_line(
   point const & p0_in, 
   point const & p1_in, 
   bool c 
)
{
   point p0 = p0_in ;
   point p1 = p1_in;

   bool const steep = std::abs(p1.y - p0.y) > std::abs(p1.x - p0.x);

   if (steep){
      std::swap(p0.x,p0.y);
      std::swap(p1.x,p1.y);
   }

   if ( p0.x > p1.x){
      std::swap(p1,p0);
   }

   auto const deltax = p1.x - p0.x;
   auto const deltay = std::abs(p1.y - p0.y);
   int const ystep = (p0.y < p1.y)?1:-1;

   auto error = deltax / 2;
   auto y = p0.y ;

   for (int32_t x = p0.x ;x < p1.x ; ++x) {
      if (!steep) {
         set_pixel({x,y},c);
      }else{
         set_pixel({y,x},c);
      }
      error -= deltay;
      if (error < 0) {
         y     += ystep;
         error += deltax;
      }
   }
}