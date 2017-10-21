#ifndef SSH1106_OLED_HPP_INCLUDED
#define SSH1106_OLED_HPP_INCLUDED

#include <cstdint>
#include <quan/time.hpp>

extern "C" void setup();

struct sh1106_oled{

    static constexpr uint32_t columns = 128;
    static constexpr uint32_t rows = 64;
    static constexpr uint8_t i2c_address = (0x3C << 1U); // n.b the 7 bits of the address shifted by 1
    enum class cmd : uint8_t {
       set_charge_pump = 0xAD
       ,set_memory_addressing_mode = 0x20
       ,set_contrast = 0x81  // arg: 0 to FF, current increaseas a svalue increases
       ,set_vertical_scroll_area = 0xA3
       ,set_multiplex_ratio = 0xA8 // arg: 0 to 63 to chnage multiplex ratio
       ,set_page_address = 0xB0  // arg : page 0 to 15
       ,set_display_offset = 0xD3 //arg : 0 to 64
       ,set_display_clock_divide_ratio = 0xD5//arg: rtm suggested 0x80
       ,set_vcom_deselect_level = 0xDB //arg: rtm
       ,set_precharge_period = 0xD9 // arg rtm
       ,set_common_pads_hardware_config = 0xDA// arg rtm
       ,set_dc_dc = 0xAD// turn on or off dcdc conv arg:0x8A for off 0x8B for on for on.display must be off
       ,nop = 0xE3   // nop cmd no arg
   };

   enum class or_cmd : uint8_t {
       set_common_output_scan_direction = 0xC0// arg: 0 scan from 0 to (n-1] or 8 the other way
       ,set_segment_remap = 0xA0  // arg: 0 or 1 mirror screen around vertical axis?
       ,set_display_start_line = 0x40    //arg: start line for scrolling from 0 to 63
       ,set_lower_column_address = 0x00 // args 0 to 0xF
       ,set_higher_column_address = 0x10 // args 0 to 0xF
       ,set_charge_pump_voltage = 0x30 // arg 0 -> 6.4 V to 3 -> 9 V
       ,set_entire_display = 0xA4 // arg : 1 for all on (white?), 0 for normal
       ,set_invert_display = 0xA6 // arg 1 to invert, 0 to not invert
       ,set_display_on = 0xAE  // arg: 1 to turn on, 0 to turn off
       ,set_page_address = 0B0 // arg 0 to 7
       ,read_modify_write =0xE0  // arg 0 to start 0xE to end
   };

   // N.B These return when the i2c transaction has been started
   // not when it is completed
    static bool apply(cmd c);
    static bool apply(or_cmd c, uint8_t v);
    static bool apply(cmd c, uint8_t arg);

   static void set_pixel(int16_t x, int16_t y, bool colour);
   private:
   friend void ::setup();
   static void initialise();
   static bool do_command_tail( bool in);
   static uint8_t buffer[];
   static bool wait_for_command_complete;
   static quan::time_<uint32_t>::ms max_cmd_wait;
};

#endif // SSH1106_OLED_HPP_INCLUDED
