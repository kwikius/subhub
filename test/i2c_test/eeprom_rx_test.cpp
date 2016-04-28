
#include <quan/stm32/millis.hpp>
#include "../../i2c.hpp"
#include "../../usarts.hpp"

#include <quan/conversion/itoa.hpp>
#include "led.hpp"

using quan::stm32::millis;

void print_flags(const char * name, uint32_t flags)
{
   link_sp::serial_port::write(name);
   link_sp::serial_port::write( "= 0x");
   char buf [ 50];
   quan::itoasc(flags,buf,16);
   link_sp::serial_port::write(buf);
   link_sp::serial_port::put('\n');
}

struct eeprom_reader{

   static void error_handler()
   {
      
      i2c::clear_irq_flags();
      i2c::disable_interrupts();
      link_sp::serial_port::write( "got i2c errors\n");
   }

   static bool apply(uint8_t device_address,uint16_t data_address, uint8_t* data, uint16_t len)
   {
      if (i2c::get_bus()){

#if 1

  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
  /* (1) open drain for I2C signals */
  /* (2) AF1 for I2C signals */
  /* (3) Select AF mode (10) on PB6 and PB7 */
  GPIOB->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7; /* (1) */
  GPIOB->AFR[0] = (GPIOB->AFR[0] & ~(GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7)) \
                  | (1 << ( 6 * 4 )) | (1 << (7 * 4)); /* (2) */
  GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER6 | GPIO_MODER_MODER7)) \
                 | (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); /* (3) */

RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
 RCC->CFGR3 |= RCC_CFGR3_I2C1SW;

  /* Configure I2C2, master */
  /* (1) Timing register value is computed with the AN4235 xls file,
   fast Mode @400kHz with I2CCLK = 48MHz, rise time = 140ns, fall time = 40ns */
  /* (2) Periph enable */
  /* (3) Slave address = 0x5A, write transfer, 1 byte to transmit, autoend */
  I2C1->TIMINGR = (uint32_t)0x00B01A4B; /* (1) */
  I2C1->CR1 = I2C_CR1_PE | I2C_CR1_TXIE; /* (2) */
  I2C1->CR2 =  I2C_CR2_AUTOEND | (2<<16) | ((uint32_t)device_address); /* (3) */

#else
        
          i2c::enable();
         m_p_data = data;
         m_data_length = len;
         m_data_address[0] = static_cast<uint8_t>((data_address & 0xFF00) >> 8U);
         m_data_address[1] = static_cast<uint8_t>(data_address & 0xFF);
         m_device_address = device_address ; // write address
//
         i2c::set_transmit_mode(); //ck
         i2c::set_slave_address_7bit(device_address); //ck
         i2c::set_transfer_size(2);  // want to write the address //ck
         i2c::set_autoend(false); //ck
         i2c::set_reload(false); //ck
         i2c::set_event_handler(on_data_address1);
 
        
#endif
        i2c::set_event_handler(on_data_address1);
         uint32_t flags = i2c::get_status();
         print_flags("isr",flags);
         if ( flags & 1){
             i2c::send_data(m_data_address[0]);
         }
         
         i2c::request_start_condition();
         i2c::enable_error_irqs(true); //
         i2c::enable_tx_irq(true); //ck


         auto const now = millis();
         typedef decltype (now) ms;
         while ( (millis() - now ) < ms{500}){
            if ( i2c::get_status() & (1 << 1) ){
               link_sp::serial_port::write("got txis\n");
               break;
            }
            if (  i2c::get_status() != flags){
               link_sp::serial_port::write("got status change\n");
               print_flags("isr",i2c::get_status());
               break;
            }
         ;}
         print_flags("cr1", i2c::get_cr1());
         print_flags("cr2", i2c::get_cr2());
    
         return true;
      }else{
         link_sp::serial_port::write("couldnt get bus\n");
         return false;
      }
   }

   // txis
   static void on_data_address1()
   {
       led::on();
       if (i2c::have_errors()) {
          error_handler();
       }
       i2c::set_event_handler(on_data_address2);
       i2c::send_data(m_data_address[0]);  // clears txis irq
       
   }

   // txis
   static void on_data_address2()
   {
       if (i2c::have_errors()) {
          error_handler();
       }
       i2c::enable_tx_irq(false);
       i2c::enable_tc_irq(true);
       i2c::set_event_handler(on_data_address_tc);
       i2c::send_data(m_data_address[1]);  // clears txis irq
   }

    // end of data address transmission
     // tc
   static void on_data_address_tc()
   {
      if (i2c::have_errors()) {
         error_handler();
      }
      i2c::set_receive_mode();
      i2c::set_slave_address_7bit(m_device_address); // redo the address
      i2c::set_transfer_size(m_data_length);
      i2c::set_autoend(true);
      i2c::set_reload(false);
      i2c::enable_rx_irq(true);
      i2c::set_event_handler(on_rxne);
      i2c::request_start_condition(); // clears tc irq
   }
   
   static void on_rxne()
   {
      if (i2c::have_errors()) {
         error_handler();
      }
      *m_p_data = i2c::receive_data(); // clears rxne irq
      ++m_p_data;
      
      if (i2c::transfer_is_complete()){
         i2c::enable_rx_irq(false);
         i2c::set_event_handler(on_stop);
         i2c::enable_stop_irq(true);
      }
   }

   static void on_stop()
   {  
       if (i2c::have_errors()) {
         error_handler();
       }
       i2c::clear_stop_flag();
       i2c::enable_stop_irq(false);
       i2c::set_default_handlers();
   }

private:
   static uint8_t* m_p_data;  
   static uint16_t m_data_length; 
   static uint8_t  m_data_address[2]; 
   static uint8_t  m_device_address;
};

   uint8_t*          eeprom_reader::m_p_data = nullptr;
   uint16_t          eeprom_reader::m_data_length = 0U;
   uint8_t           eeprom_reader::m_data_address[] = {0U,0U};
   uint8_t           eeprom_reader::m_device_address = 0U;


constexpr uint16_t numbytes = 8U;
char data_in[numbytes] = {"-------"};  // the data to write n.b in dma available memory

void eeprom_rx_test()
{
   static constexpr uint8_t eeprom_addr = 0b10100000;

   bool const result = eeprom_reader::apply( eeprom_addr ,5U,(uint8_t*)data_in,8);

   auto const now = millis();
   typedef decltype (now) ms;
   while ( (millis() - now ) < ms{500}){;}

   if (i2c::is_busy()){
      link_sp::serial_port::write( "i2c still busy ... hung\n");
   }
   // if timeout or i2c::is_busy() 
   link_sp::serial_port::write( result? "success":"fail");
   link_sp::serial_port::write(" got ");
   link_sp::serial_port::write(data_in,8);
   link_sp::serial_port::put('\n');
   
}