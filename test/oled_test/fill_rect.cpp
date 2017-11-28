#include <quan/min.hpp>
#include <quan/max.hpp>
#include "sh1106_oled.hpp"
/*
 
*/
void sh1106_oled::fill_rect(point const & corner1_in,point const & corner2_in, bool colour) 
{
   point const p1{quan::min(corner1_in.x,corner2_in.x),quan::min(corner1_in.y,corner2_in.y)};
   point const p2{quan::max(corner1_in.x,corner2_in.x),quan::max(corner1_in.y,corner2_in.y)};

   for (auto y = p1.y; y < p2.y; ++y) {
    draw_line( {p1.x,y},{p2.x,y}, colour);
   }
}