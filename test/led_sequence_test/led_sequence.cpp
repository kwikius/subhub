
#include <stm32f0xx.h>
#include <cstring>
#include <quan/stm32/tim.hpp>
#include <quan/stm32/tim/temp_reg.hpp>
#include <quan/stm32/get_raw_timer_frequency.hpp>
#include <quan/stm32/gpio.hpp>
#include <quan/stm32/rcc.hpp>
#include <quan/stm32/millis.hpp>

#include "../../resources.hpp"
#include "../../led_sequence.hpp"
#include "../../usarts.hpp"

#define QUAN_USE_DMA
//#define QUAN_USE_IRQ

/*
  uses spi2,  mosi on pb15
  if reqd spi2 clk on pb13
*/
uint8_t  led_sequence::led_data [led_data_size + preamble] = {0U};


using quan::stm32::millis;

namespace{

   typedef  link_sp::serial_port xout;

   typedef decltype(millis()) ms;
   ms operator "" _ms(unsigned long long int v)
   {
      return static_cast<ms>(v);
   }

   void delay(ms const & t)
   {
      auto const now = millis();
      while ( (millis() - now ) < t){
        asm volatile ("nop":::);
      }
   }
}

void led_sequence::initialise()
{
   // clear the array, then set the bits
   memset(led_data ,0,led_data_size + preamble);
   // each led uses 24 bits , each led bit uses 4 bits
   uint32_t constexpr num_ar_bits = num_leds * 24U * 4U;

  // static_assert(num_ar_bits == (led_data_size * 8U),"unexpected data size");
   for ( uint32_t i = 0U; i < num_ar_bits; i += 4U){
      putbit(i,true);
   }

   // init the led sequence spi pin
   quan::stm32::module_enable<led_sequence_pin::port_type>();
   quan::stm32::apply<
      led_sequence_pin
      ,quan::stm32::gpio::mode::output
      ,quan::stm32::gpio::otype::push_pull
      ,quan::stm32::gpio::pupd::pull_down
      ,quan::stm32::gpio::ospeed::fast
   >();

   quan::stm32::apply<
      led_sequence_pin
      ,quan::stm32::gpio::mode::af0
   >();

#if defined QUAN_USE_DMA
   static constexpr uint32_t dma_en_bit = 0U;
   quan::stm32::rcc::get()->ahbenr |= (0b1 << dma_en_bit); //(DMA_EN)
#endif
   // turn on the spi rcc module
   static constexpr uint8_t spi2_en_bit = 14U;
   quan::stm32::rcc::get()->apb1enr |= (1U << spi2_en_bit); //(SPI2_EN)

   // reset spi
   quan::stm32::rcc::get()->apb1rstr |= ( 1U << spi2_en_bit ); // (SPI2_RsT)
   quan::stm32::rcc::get()->apb1rstr &= ~( 1U << spi2_en_bit ); // (SPI2_RST)

 /*Configure the serial clock baud rate using the BR[2:0] bits
    3 MHz baud. Divide 48 Mhz fpclk by 16
*/                         
   SPI2->CR1 = ( SPI2->CR1 & ~(0b111 << 3U) ) | (0b11 << 3U); // (BR[2:0] )

/*
b) Configure the CPOL and CPHA bits combination to define one of the four
relationships between the data transfer and the serial clock
*/
  SPI2->CR1 = ( SPI2->CR1 & ~(0b11 << 0U) ) | (0b11 << 0U); // (CPOL(bit1) CPHA (bit0)
  
/*
c) Select simplex or half-duplex mode by configuring RXONLY or BIDIMODE and
BIDIOE 
  no bidi not rxonly no bidioe
  yes bidi not rxonly yes bidioe
*/
                  // (BIDIMODE) ( BIDIOE) 
 // SPI2->CR1 |=  ( 0b1 << 15U) | (0b1 << 14U); 

/*
d) Configure the LSBFIRST bit to define the frame format
   lsb first
*/
  SPI2->CR1 |= (0b1 << 7U); // (LSBFIRST)

/*
e) Configure the CRCL and CRCEN bits if CRC is needed.
   no crc
*/
  SPI2->CR1 &= ~(0b1 << 13U);  //(CRCEN)

/*
f) Configure SSM and SSI
   set ssm and ssi
*/
  SPI2->CR1 |= (0b1 << 9U);  //(SSM)
  SPI2->CR1 |= (0b1 << 8U);  //(SSI)
/*
g) Configure the MSTR bit
   master mode
*/
  SPI2->CR1 |= (0b1 << 2U); //(MSTR)



/*
Write to SPI_CR2 register:
a) Configure the DS[3:0] bits to select the data length for the transfer
   8 bit transfer
*/
  SPI2->CR2 = ( SPI2->CR2 & ~(0b1111 << 8U) ) | (0b1111 << 8U); // (DS

/*
b) Configure SSOE .
  leave as default
*/

/*
c) Set the FRF bit if the TI protocol is required .
   clear FRF bit default

d) Set the NSSP bit if the NSS pulse mode between two data units is required (keep
CHPA and TI bits cleared in NSSP mode).
   keep bits cleared

e) Configure the FRXTH bit. The RXFIFO threshold must be aligned to the read
access
keep FRXTH clear for 8 bit transfer

f) Initialize LDMA_TX and LDMA_RX bits if DMA is used in packed mode.
   If the DMA transfers are 8 bit then LDMA_TX and LDMA_RX can be kept clear.
   (see ref man26.3.8

4. Write to SPI_CRCPR register: Configure the CRC polynomial if needed.
   no CRC
*/
  SPI2->CRCPR = 7U;

#if defined QUAN_USE_IRQ
   NVIC_EnableIRQ(SPI2_IRQn);
#else
#if defined QUAN_USE_DMA
/*
5. Write proper DMA registers: Configure DMA streams dedicated for SPI Tx and Rx in
DMA registers if the DMA streams are used.

   8 bit
   memory to peripheral
   inc memory
*/

   // 8 bit default
   DMA1_Channel5->CCR  =
      (0b1 << 7U) // (MINC)
    | (0b1 << 4U) // (DIR) read from memory 
    | (0b01 << 10U)
    | (0b01 << 8U) // 16 bit periphera
    | ( 0b11 << 12U)
   ;

   DMA1->IFCR = (0b1111 << 16U);
   DMA1_Channel5->CCR |= ( 0b1 << 1U); // (TCIE)
   NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
   SPI2->CR2 |= (0b1 << 1U); // (TXDMAEN)
#else
   // SPI2->CR1 |= (0b1 << 6U); // (SPE)
#endif
#endif

 SPI2->CR1 |= (0b1 << 6U); // (SPE)
   delay (1_ms);
}

namespace {

 uint32_t data_idx = 0U;
}
void led_sequence::send()
{
#if defined QUAN_USE_DMA
   //SPI2->CR1 &= ~(0b1 << 6U); // (SPE)
   DMA1_Channel5->CCR &= ~(0b1 << 0U); // (OE)
   while ( DMA1_Channel5->CCR & (0b1 << 0U)){
      asm volatile ("nop":::);
   }
   
   DMA1_Channel5->CPAR = (uint32_t)&SPI2->DR;
   DMA1_Channel5->CMAR = (uint32_t)led_data ;
   DMA1_Channel5->CNDTR = (led_data_size + preamble) / 2U ;
    // Clear DMA flags
   DMA1->IFCR = (0b1111 << 16U);
   DMA1_Channel5->CCR |= (0b1 << 1U); // (TCIE)
  // SPI2->DR = led_data[0];
   DMA1_Channel5->CCR |= (0b1 << 0U); // (OE)
   while ( ! DMA1_Channel5->CCR & (0b1 << 0U)){
      asm volatile ("nop":::);
   }
   SPI2->CR1 |= (0b1 << 6U); // (SPE)
   
#else
#if defined  QUAN_USE_IRQ
    data_idx= 0U;
    SPI2->CR1 &= ~(0b1 << 6U); // (SPE)
    SPI2->CR2 |= (0b1 << 7U); // TXEIE
    SPI2->CR1 |= (0b1 << 6U); // (SPE)
#else
      //enable the SPI 
   //SPI2->CR1 |= (0b1 << 6U); // (SPE)
   for ( uint32_t i = 0U; i < led_data_size; ++i){
      while ( (SPI2->SR & (0b1 << 1U)) == 0U){
         asm volatile ("nop":::);
      }
    *(__IO uint8_t *)&SPI2->DR = led_data[i + preamble];
   }
#endif
#endif
   
}

void led_sequence::putbit(uint32_t bit_idx_in, bool val)
{
   uint32_t const byte_idx = bit_idx_in / 8U ;
   uint32_t const bit_idx = bit_idx_in  % 8U;
   if ( val ) {
       led_data[byte_idx + preamble] |= (0b1 << bit_idx);
   }else{
       led_data[byte_idx + preamble] &= ~(0b1 << bit_idx);
   }
}

bool led_sequence::put(uint32_t index, rgb_value const & v)
{
   if ( index < num_leds){
      // bit position of start of the rgb entry in the output array
      uint32_t const out_arr_bitpos = index * 24U * 4U;
      for ( uint32_t i = 0U; i < 8U; ++i){
         uint8_t const bitmask = 1U << i;

         uint32_t const green_bit_pos = out_arr_bitpos + 29U - 4U * i;
         putbit(green_bit_pos,(v.green & bitmask) != 0U);
          putbit(green_bit_pos +1U,(v.green & bitmask) != 0U);

         uint32_t const red_bit_pos = green_bit_pos + 32U;
         putbit(red_bit_pos,(v.red & bitmask ) != 0U);
         putbit(red_bit_pos+1U,(v.red & bitmask ) != 0U);

         uint32_t const blue_bit_pos = green_bit_pos + 64U;
         putbit(blue_bit_pos,(v.blue & bitmask) != 0U);
         putbit(blue_bit_pos +1U,(v.blue & bitmask) != 0U);
      }
      return true;
   }else{
      return false;
   }
}

uint32_t led_sequence::transfer_bytes_left()
{
  return DMA1_Channel5->CNDTR;
}

extern "C" void SPI2_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void SPI2_IRQHandler()
{
    *(__IO uint8_t *)&SPI2->DR = led_sequence::led_data[data_idx];
    if ( ++ data_idx == led_sequence::led_data_size/2U){
       SPI2->CR2 &= ~(0b1 << 7U); // TXEIE
    }
}

extern "C" void DMA1_Channel4_5_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void DMA1_Channel4_5_IRQHandler()
{
   
   // disable DMA interrupt flag
 //  DMA1_Channel5->CCR &= ~( 0b1 << 1U); // (TCIE)
   // Clear DMA flags
   DMA1->IFCR = (0b1111 << 16U);
 // *(__IO uint8_t *)&SPI2->DR = 0U;

//   // disable dma
    DMA1_Channel5->CCR &= ~(0b1 << 0U); // (OE)
//   // disable the SPI 
//   SPI2->CR1 &= ~(0b1 << 6U); // (SPE)
//   SPI2->CR2 &= ~(0b1 << 1); // (TXDMAEN)
}




