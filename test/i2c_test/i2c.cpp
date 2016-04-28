
#include <stm32f0xx.h>
#include <quan/stm32/rcc.hpp>
#include <quan/stm32/gpio.hpp>
#include <quan/stm32/i2c/module_enable_disable.hpp>
#include "../../i2c.hpp"
#include "../../usarts.hpp"

void panic(const char* str)
{
   link_sp::serial_port::write("PANIC: ");
   link_sp::serial_port::write(str);
   link_sp::serial_port::put('\n');
}

/*
setup i2c

i2c is on i2c1

 PB7 SDA
 PB6 SCL

 DMA channel 2 is I2C1_TX
 DMA Channel 3 is I2C1_RX
*/

namespace {

   typedef quan::stm32::i2c1 i2c_type;

   typedef quan::mcu::pin<quan::stm32::gpiob,6> scl_pin;
   typedef quan::mcu::pin<quan::stm32::gpiob,7> sda_pin;
}

void verify_i2c()
{
   link_sp::serial_port::write("verifying i2c addresses\n");
   static_assert( (uint32_t)i2c_type::get() == 0x40005400 ,"invalid i2c addr");  
   if( ((uint32_t)&i2c_type::get()->cr1) != 0x40005400){
      panic("invalid i2c->cr1 addr");
   }
   if( ((uint32_t)&i2c_type::get()->cr2) != 0x40005404){
      panic("invalid i2c->cr2 addr");
   }
   if( ((uint32_t)&i2c_type::get()->oar1) != 0x40005408){
      panic("invalid i2c->oar1 addr");
   }
   if( ((uint32_t)&i2c_type::get()->oar2) != 0x4000540C){
      panic("invalid i2c->oar2 addr");
   }
   if( ((uint32_t)&i2c_type::get()->timingr) != 0x40005410){
      panic("invalid i2c->timingr addr");
   }
   if( ((uint32_t)&i2c_type::get()->timeoutr) != 0x40005414){
      panic("invalid i2c->timeoutr addr");
   }
   if( ((uint32_t)&i2c_type::get()->isr) != 0x40005418){
      panic("invalid i2c->isr addr");
   }
   if( ((uint32_t)&i2c_type::get()->icr) != 0x4000541C){
      panic("invalid i2c->icr addr");
   }
   if( ((uint32_t)&i2c_type::get()->pecr) != 0x40005420){
      panic("invalid i2c->pecr addr");
   }
   if( ((uint32_t)&i2c_type::get()->rxdr) != 0x40005424){
      panic("invalid i2c->rxdr addr");
   }
   if( ((uint32_t)&i2c_type::get()->txdr) != 0x40005428){
      panic("invalid i2c->txdr addr");
   }
}

volatile bool i2c::m_bus_taken_token=false;
void (* volatile i2c::pfn_event_handler)() = i2c::default_event_handler;
void (* volatile i2c::pfn_dma_handler)()   = i2c::default_dma_handler;

uint32_t i2c::get_status()
{
  return i2c_type::get()->isr.get();
}

uint32_t i2c::get_cr1()
{
  return i2c_type::get()->cr1.get();
}

uint32_t i2c::get_cr2()
{
  return i2c_type::get()->cr2.get();
}

void i2c::request_start_condition()
{
   constexpr uint8_t cr2_start = 13U;
   i2c_type::get()->cr2.setbit<cr2_start>();
}

bool i2c::transfer_is_complete()
{
   constexpr uint8_t isr_tc = 6;
   return i2c_type::get()->isr.getbit<isr_tc>();
}

void i2c::set_transfer_size(uint8_t n)
{
   i2c_type::get()->cr2.set(
      (i2c_type::get()->cr2.get() & ~(0xFF << 16U))
      | (n << 16u)
   );
}

uint8_t i2c::receive_data()
{
   return i2c_type::get()->rxdr;
}

void i2c::send_data(uint8_t data)
{
   i2c_type::get()->txdr = data;
}

void i2c::enable_error_irqs(bool b)
{
   constexpr uint8_t cr1_errie = 7;
   i2c_type::get()->cr1.putbit<cr1_errie>(b);
}

bool i2c::have_errors()
{
   return  (i2c_type::get()->isr.get()  & ( 7 << 8 ) ) != 0;
}

void i2c::clear_irq_flags()
{
   constexpr uint32_t mask = 0b11111100111000;
   i2c_type::get()->icr.set(mask);
}

void i2c::disable_interrupts()
{
    constexpr uint32_t mask = 0b11111110;
    i2c_type::get()->cr1 &= ~mask;
}


void i2c::enable_rx_irq(bool b)
{
   constexpr uint8_t cr1_rxie = 2;
   i2c_type::get()->cr1.putbit<cr1_rxie>(b); 
}

void i2c::enable_tx_irq(bool b)
{
   constexpr uint8_t cr1_txie = 1;
   i2c_type::get()->cr1.putbit<cr1_txie>(b);
  // i2c_type::get( )->cr1.set(i2c_type::get()->cr1.get() | 0xFF);
}

void i2c::enable_tc_irq(bool b)
{
   constexpr uint8_t cr1_tcie = 6;
   i2c_type::get()->cr1.putbit<cr1_tcie>(b);
}

void i2c::enable_stop_irq(bool b)
{
   uint8_t constexpr cr1_stop = 5;
   i2c_type::get()->cr1.putbit<cr1_stop>(b);
}

void i2c::clear_stop_flag()
{
   uint8_t constexpr icr_stopcf = 5;
   i2c_type::get()->icr.setbit<icr_stopcf>();
}

void i2c::set_transmit_mode()
{
   constexpr uint8_t cr2_rd_wrn = 10U;
   i2c_type::get()->cr2.clearbit<cr2_rd_wrn>();
}

void i2c::set_receive_mode()
{
   constexpr uint8_t cr2_rd_wrn = 10U;
   i2c_type::get()->cr2.setbit<cr2_rd_wrn>();  
}

void i2c::set_autoend(bool b)
{
   constexpr uint8_t cr2_autoend = 25U;
   i2c_type::get()->cr2.putbit<cr2_autoend>(b);
}

void i2c::set_reload(bool b)
{
   constexpr uint8_t cr2_reload = 24;
   i2c_type::get()->cr2.putbit<cr2_reload>(b);
}

void i2c::set_slave_address_7bit(uint8_t address)
{
   constexpr uint8_t cr2_add10_bit = 11;
   constexpr auto p = i2c_type::get();
   constexpr uint32_t and_mask = ~(0x3FF | (1U << cr2_add10_bit)) ;    // clear 7/10 bit address
   p->cr2.set( (p->cr2.get() & and_mask) | ( static_cast<uint32_t>(address) ));
}

void i2c::init()
{

#if 1

#else
  verify_i2c();
  // set up rcc clock config for system clock rcc cfgr3 bit 4 I2C1SW
  // 1 is sysclock 0 is HSI
   quan::stm32::rcc::get()->cfgr3.setbit<3>();

   quan::stm32::module_reset<i2c_type>();
  // enable the module
   quan::stm32::module_enable<i2c_type>();

   if ( ( quan::stm32::rcc::get()->apb1enr.get() & (1 << 21)) == 0){
      link_sp::serial_port::write("i2c not turned on\n");
   }else{
      link_sp::serial_port::write("i2c turned on\n");
   }
 #endif

#if 1
 // setup the pins
   quan::stm32::apply<
      scl_pin
      ,quan::stm32::gpio::mode::af1  
      ,quan::stm32::gpio::otype::open_drain
      ,quan::stm32::gpio::pupd::none         //  Use external pullup 5V tolerant pins
      ,quan::stm32::gpio::ospeed::slow 
   >();

   quan::stm32::apply<
      sda_pin
      ,quan::stm32::gpio::mode::af1
      ,quan::stm32::gpio::otype::open_drain
      ,quan::stm32::gpio::pupd::none     //  Use external pullup 5V tolerant pins
      ,quan::stm32::gpio::ospeed::slow 
   >();
#else
   {
      // filters
      // anfof and DNF[3:0] in cr1
      uint32_t cr1 = i2c_type::get()->cr1.get();
      cr1 &= ~(1 << 12) ; // (ANFOF)
      cr1 &= ~(0b1111 << 8); // (DNF)
      i2c_type::get()->cr1.set(cr1);
   }
   {
      uint32_t timingr = i2c_type::get()->timingr.get();
      // PRESC [3:0]  = 0xB
      timingr = (timingr & ~(0b1111 << 28)) | (0xB << 28); // (PRESC)
      // SCLDEL[3:0] = 0x4
      timingr = (timingr & ~(0b1111 << 20)) | ( 0x4 << 20); // (SCLDEL)
      // SDADEL[3:0] = 0x2
      timingr = (timingr & ~(0b1111 << 16)) | ( 0x2 << 16); // (SDADEL)
      // SCLH[7:0]   = 0xF
      timingr = (timingr & ~(0xFF << 8)) | ( 0xF << 8);  // (SCLH)
      // SCLL[7:0]   = 0x13
      timingr = (timingr & ~(0xFF << 0)) | ( 0x13 << 0); // (SCLL)
      i2c_type::get()->timingr.set(timingr);
   }
  #endif
   NVIC_EnableIRQ(I2C1_IRQn);
   NVIC_SetPriority(I2C1_IRQn, 0);
// dma channel 2_3
   NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
   NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0);

   // dont enable the peripheral

}

void i2c::disable()
{
   i2c_type::get()->cr1.clearbit<0>();
}
void i2c::enable()
{
   i2c_type::get()->cr1.setbit<0>();
}

bool i2c::is_busy()
{
   static constexpr uint8_t isr_busy_bit = 15;
   return i2c_type::get()->isr.getbit<isr_busy_bit>();
}

bool i2c::get_bus()
{
   if (m_bus_taken_token || is_busy()){
      return false;
   }
   return m_bus_taken_token = true;
}

bool i2c::release_bus()
{
   m_bus_taken_token = false;
   return true;
}

void i2c::default_event_handler()
{
   panic("default i2c event handler called");
}

void i2c::default_dma_handler()
{
   panic("default i2c dma handler called");
}

void i2c::set_default_handlers()
{
   pfn_event_handler = default_event_handler;
   pfn_dma_handler = default_dma_handler;
}

void i2c::set_event_handler( void(*pfn_event)())
{
   pfn_event_handler = pfn_event;
}

void i2c::set_dma_handler( void(*pfn_event)())
{
   pfn_dma_handler = pfn_event;
}

extern "C" void I2C1_IRQHandler() __attribute__ ( (interrupt ("IRQ")));
extern "C" void I2C1_IRQHandler() 
{
   i2c::pfn_event_handler();
}

extern "C" void DMA1_Channel2_3_IRQHandler() __attribute__ ( (interrupt ("IRQ")));
extern "C" void DMA1_Channel2_3_IRQHandler()
{
   i2c::pfn_dma_handler();
}
