
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

/* 
 1 setup channel to FC
 Either the config of the subhub is fixed
so stup is fixed
 or FC sends config and setup is dependet on that

In fact only variable is whether RC in or GPS
can use both unless want SBUS or Frsky telem
*/


#include <apm/gps.hpp>
#include "serial_port.hpp"

/*
   input channel
   output channel use DMA? queue transactions. A single buffer for DMA. load buffer from fifo?
   

   rc in       : event from rc input. On new rc_in value send to output_channel
   servos out  : update dc event from chan input
   mag in out  : periodic every 1/100th sec (say) 1)request 2) read
   gps in      : event from sp input
   airspeed in : periodic every 1/100th sec say)
*/

extern "C" void setup();

int main()
{
   setup();

   apm::gps_t gps;
   gps.init(get_gps_sp());
   gps.update();
}
