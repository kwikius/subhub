
#include <quan/stm32/millis.hpp>
#include <quan/stm32/gpio.hpp>
#include <quan/conversion/itoa.hpp>
#include <quan/fixed_quantity/literal.hpp>
#include "../../i2c.hpp"
#include "../../usarts.hpp"
#include "led.hpp"
#include "sh1106_oled.hpp"

namespace {
   typedef link_sp::serial_port xout;

   QUAN_QUANTITY_LITERAL(time,ms)
   

struct oled_writer{

    static void error_handler()
   {
      i2c::clear_irq_flags();
      i2c::disable_interrupts();
      xout::write( "got i2c errors\n");
   }
    static bool apply(uint8_t device_address,uint8_t const* data, uint8_t len, uint8_t const * preamble_byte = nullptr)
    {

      if (i2c::get_bus()){
         m_p_data = data;
         m_data_length = len;
         m_device_address = device_address ; // write address
         
         i2c::set_transmit_mode();
         i2c::set_slave_address_7bit(device_address); 
         
         i2c::set_autoend(true); 
         i2c::set_reload(false); 
         if ( preamble_byte != nullptr){
            i2c::set_transfer_size(len+1); 
            m_preamble_byte = preamble_byte[0];
            i2c::set_event_handler(on_preamble_byte);
         }else{
            i2c::set_transfer_size(len); 
            i2c::set_event_handler(on_data);
         }
         i2c::request_start_condition();
         i2c::enable_error_irqs(true);
         i2c::enable_tx_irq(true); 

         return true;
      }else{
         xout::write("couldnt get bus\n");
         return false;
      }
    }

   static void on_preamble_byte()
   {
      if (i2c::have_errors()) {
         error_handler();
         return;
      }
      i2c::set_event_handler(on_data);
      i2c::send_data(m_preamble_byte);
   }

   static void on_data()
   {
      if (i2c::have_errors()) {
        error_handler();
        return;
      }
      --m_data_length;
      if (m_data_length == 0){
        i2c::enable_tx_irq(false);
        i2c::enable_stop_irq(true);
        i2c::set_event_handler(on_stop);
      }
      i2c::send_data(*m_p_data);
      ++m_p_data;
   }

   static void on_stop()
   {
      if (i2c::have_errors()) {
         error_handler();
         return;
       }
       i2c::clear_stop_flag();
       i2c::enable_stop_irq(false);
       i2c::set_default_handlers();
       i2c::release_bus();
   }
private:
   static uint8_t const* m_p_data;  
   static uint8_t m_data_length; 
   static uint8_t m_device_address;
   static uint8_t m_preamble_byte;
};

   uint8_t const*    oled_writer::m_p_data = nullptr;
   uint8_t           oled_writer::m_data_length = 0U;
   uint8_t           oled_writer::m_device_address = 0U;
   uint8_t           oled_writer::m_preamble_byte = 0U;

   constexpr uint8_t i2c_buffer_len = 20;
   uint8_t i2c_buffer[i2c_buffer_len];

   

}// namespace

/*
  commands send a sequence of pairs, control byte followed by data byte
   the Co and DnotC bits are significant in the control byte
   The Co (continue bit bit7) is set in all pairs, except the last
   The DnotC  ( Dat not command) is always cleared
*/

bool sh1106_oled::apply(cmd c)
{
   // send the control then the command
   i2c_buffer[0] = 0U;
   i2c_buffer[1] = static_cast<uint8_t>(c);
   return do_command_tail(oled_writer::apply(i2c_address,i2c_buffer,2U));
}

bool sh1106_oled::apply(or_cmd c, uint8_t arg)
{
   i2c_buffer[0] = 0U;
   i2c_buffer[1] = static_cast<uint8_t>(c) | arg;
   return do_command_tail( oled_writer::apply(i2c_address,i2c_buffer,2U));
}

bool sh1106_oled::apply(cmd c, uint8_t arg)
{
   i2c_buffer[0] = 0b10000000;  // control with continue
   i2c_buffer[1] = static_cast<uint8_t>(c);
   i2c_buffer[2] = 0U;          // control no continue
   i2c_buffer[3] = arg;
   return do_command_tail( oled_writer::apply(i2c_address,i2c_buffer,4U));
}

using quan::stm32::millis;

bool sh1106_oled::do_command_tail( bool in)
{
   if ( in && wait_for_command_complete){
      auto start = quan::stm32::millis();
      while ( (millis() - start) < max_cmd_wait){
         if (!i2c::is_busy()){
            start= quan::stm32::millis();
            while ( (millis() - start) < 2_ms){
               asm volatile("nop":::);
            }
            return true;
         }
      }
      xout::write("timeout waiting command to complete\n");
      return false;
   }else{
      return in;
   }
}

bool sh1106_oled::write_data(uint8_t* buf, uint16_t len)
{
    uint8_t const preamble_byte = 0x40U;
    bool result = oled_writer::apply(i2c_address,buf,len,&preamble_byte);
    return do_command_tail(result);
}


