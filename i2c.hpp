#ifndef QUAN_SUBHUB_I2C_HPP_INCLUDED
#define QUAN_SUBHUB_I2C_HPP_INCLUDED

#include <cstdint>

extern "C" void I2C1_IRQHandler() __attribute__ ( (interrupt ("IRQ")));
extern "C" void DMA1_Channel2_3_IRQHandler() __attribute__ ( (interrupt ("IRQ")));

struct i2c{
   
   static void disable();
   static void enable();
   static void initialise();
   static bool is_busy() ;

   static bool get_bus();
   static bool release_bus();

   static void default_event_handler();
   static void default_dma_handler();

   static void set_default_handlers();
   static void set_event_handler( void(*pfn_event)());
   static void set_dma_handler( void(*pfn_event)());

   static void set_slave_address_7bit(uint8_t address);
   static void set_transmit_mode();
   static void set_receive_mode();
   static void set_autoend(bool b);
   static void set_reload(bool b);
   static void set_transfer_size(uint8_t n);
   static void enable_tx_irq(bool b);
   static void enable_rx_irq(bool b);
   static void enable_tc_irq(bool b);
   static void enable_stop_irq(bool b);
   static void enable_error_irqs(bool b);

   static void clear_irq_flags();
   static void disable_interrupts();
   
   static bool transfer_is_complete();
   static bool have_errors();
   static void clear_stop_flag();
   static void send_data(uint8_t data);
   
   static uint8_t receive_data();
   static void request_start_condition();

   static uint32_t get_status();
   static uint32_t get_cr1();
   static uint32_t get_cr2();
    
private:
   friend void ::I2C1_IRQHandler();
   friend void ::DMA1_Channel2_3_IRQHandler();
   static volatile bool m_bus_taken_token;

   static void (* volatile pfn_event_handler)();
   static void (* volatile pfn_dma_handler)();

   i2c() = delete;
   i2c(i2c const &) = delete;
   i2c& operator =(i2c const &) = delete;

};

#endif // QUAN_SUBHUB_I2C_HPP_INCLUDED
