
#include <quan/stm32/millis.hpp>

#include "../../usarts.hpp"
#include "sh1106_oled.hpp"

/*
from adafruit ssh1106 lib
*/

uint8_t sh1106_oled::buffer[rows * columns / 8U] = {0U};
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

    //do startup delay

    xout::write("in oled::initialaise\n");
 #if 1
    delay(100_ms);
    //xout::write("display off\n");
    apply(or_cmd::set_display_on,0); // turn off display
    //xout::write("set display clk ratio\n");
    apply(cmd::set_display_clock_divide_ratio,0x80);
   // xout::write("set multiplex ratio\n");
    apply(cmd::set_multiplex_ratio,0x3F);
    apply(cmd::set_display_offset,0x00);
    apply(or_cmd::set_display_start_line, 0x00); 
 //   apply(cmd::set_charge_pump,0x8B); 
  //  apply(or_cmd::set_charge_pump_voltage,0x02); // 8.4 V
    
    apply(cmd::set_memory_addressing_mode,0x00);
    apply(or_cmd::set_segment_remap, 0x01);
    apply(or_cmd::set_common_output_scan_direction,0x08);
    apply(cmd::set_common_pads_hardware_config,0x02);
    apply(cmd::set_contrast,0xCF);
    apply(cmd::set_precharge_period,0xF1);
    apply(cmd::set_vcom_deselect_level,0x40);
    // deactivate scroll, output ram to display?
    apply(or_cmd::set_invert_display,0);

    delay(100_ms);

    apply(or_cmd::set_display_on,1); // turn on display
    xout::write("in oled::initialaise complete\n");
#endif
}
