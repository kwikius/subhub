

#include <quan/stm32/millis.hpp>
#include "../../i2c.hpp"
#include "../../usarts.hpp"
#include <quan/stm32/gpio.hpp>
#include <quan/conversion/itoa.hpp>
#include "led.hpp"

namespace {
   typedef link_sp::serial_port xout;
}

struct eeprom_writer{

    static void error_handler()
   {
      i2c::clear_irq_flags();
      i2c::disable_interrupts();
      xout::write( "got i2c errors\n");
   }
    static bool apply(uint8_t device_address,uint16_t data_address, uint8_t const* data, uint16_t len)
    {

      if (i2c::get_bus()){
         m_p_data = data;
         m_data_length = len;
         m_data_address[0] = static_cast<uint8_t>((data_address & 0xFF00) >> 8U);
         m_data_address[1] = static_cast<uint8_t>(data_address & 0xFF);
         m_device_address = device_address ; // write address

         i2c::set_transmit_mode();
         i2c::set_slave_address_7bit(device_address); 
         i2c::set_transfer_size(len + 2);  // + 2 bytes to write the address  in eeprom to read from
         i2c::set_autoend(true); 
         i2c::set_reload(false); 
         i2c::set_event_handler(on_data_address1);
         i2c::request_start_condition();
         i2c::enable_error_irqs(true);
         i2c::enable_tx_irq(true); 

         return true;
      }else{
         xout::write("couldnt get bus\n");
         return false;
      }
    }

   // txis
   static void on_data_address1()
   {
       if (i2c::have_errors()) {
          error_handler();
          return;
       }
       i2c::set_event_handler(on_data_address2);
       i2c::send_data(m_data_address[0]);  // clears txis irq
   }

   static void on_data_address2()
   { 
      if (i2c::have_errors()) {
       error_handler();
       return;
      }
      i2c::set_event_handler(on_data);
      i2c::send_data(m_data_address[1]);  // clears txis irq
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
   static uint16_t m_data_length; 
   static uint8_t  m_data_address[2]; 
   static uint8_t  m_device_address;
};

   uint8_t const*    eeprom_writer::m_p_data = nullptr;
   uint16_t          eeprom_writer::m_data_length = 0U;
   uint8_t           eeprom_writer::m_data_address[] = {0U,0U};
   uint8_t           eeprom_writer::m_device_address = 0U;


constexpr uint16_t numbytes = 8U;
char data_out[numbytes] = {"7654321"};  // the data to write n.b in dma available memory

using quan::stm32::millis;

void eeprom_tx_test()
{
   static constexpr uint8_t eeprom_addr = 0b10100000;

   bool const result = eeprom_writer::apply( eeprom_addr ,5U,(uint8_t const *)data_out,8);

   auto const now = millis();
   typedef decltype (now) ms;
   while ( (millis() - now ) < ms{500}){;}

   if (i2c::is_busy()){
      xout::write( "i2c still busy ... hung\n");
   }else{
      xout::write( "i2c not busy ... OK\nWriter ");
   }
   // if timeout or i2c::is_busy() 
   xout::write( result? "success":"fail");
  
   xout::put('\n');
}
