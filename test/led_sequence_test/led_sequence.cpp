
#include <stm32f0xx.h>
#include <cstring>
#include <quan/stm32/tim.hpp>
#include <quan/stm32/tim/temp_reg.hpp>
#include <quan/stm32/get_raw_timer_frequency.hpp>
#include <quan/stm32/gpio.hpp>
#include <quan/stm32/rcc.hpp>

#include "../../resources.hpp"
#include "../../led_sequence.hpp"
#include "../../usarts.hpp"

/*
  uses spi2,  mosi on pb15
  if reqd spi2 clk on pb13
*/
uint8_t  led_sequence::led_data [led_data_size] __attribute__ ((aligned (16))) = {0U};

namespace{

   typedef  link_sp::serial_port xout;
}

void led_sequence::initialise()
{
   // clear the array, then set the bits
   memset(led_data ,0,led_data_size);
   // each led uses 24 bits , each led bit uses 3 bits
   uint32_t constexpr num_ar_bits = num_leds * 24U * 3U;
   for ( uint32_t i = 0U; i < num_ar_bits; i += 3U){
      putbit(i,true);
   }

  // init the led sequence spi pin
   quan::stm32::module_enable<spi2_sck_pin::port_type>();
   quan::stm32::apply<
      spi2_sck_pin
      ,quan::stm32::gpio::mode::af0
      ,quan::stm32::gpio::otype::push_pull
      ,quan::stm32::gpio::pupd::pull_down
      ,quan::stm32::gpio::ospeed::fast
   >();
   // init the led sequence spi pin
   quan::stm32::module_enable<led_sequence_pin::port_type>();
   quan::stm32::apply<
      led_sequence_pin
      ,quan::stm32::gpio::mode::af0
      ,quan::stm32::gpio::otype::push_pull
      ,quan::stm32::gpio::pupd::pull_down
      ,quan::stm32::gpio::ospeed::fast
   >();

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
  SPI2->CR1 = ( SPI2->CR1 & ~(0b11 << 0U) ) | (0b00 << 0U); // (CPOL CPHA)

/*
c) Select simplex or half-duplex mode by configuring RXONLY or BIDIMODE and
BIDIOE 
  no bidi not rxonly no bidioe
*/
                  // (BIDIMODE) ( BIDIOE) (RXONLY)
  SPI2->CR1 &= ~( ( 0b1 << 15U) | (0b1 << 14U) | (0b1 << 10U) ); 

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
  SPI2->CR2 = ( SPI2->CR2 & ~(0b1111 << 8U) ) | (0b0111 << 8U); // (DS

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
/*

5. Write proper DMA registers: Configure DMA streams dedicated for SPI Tx and Rx in
DMA registers if the DMA streams are used.

   8 bit
   memory to peripheral
   inc memory
*/
//   static constexpr uint32_t dma_en_bit = 0U;
//   quan::stm32::rcc::get()->ahbenr |= (0b1 << dma_en_bit); //(DMA_EN)
//   
//   // 8 bit default
//   DMA1_Channel5->CCR  =
//      (0b1 << 7U) // (MINC)
//    | (0b1 << 4U) // (DIR) read from memory 
//   ;
//
//   DMA1->IFCR = (0b1111 << 16U);
//   NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);
}

void led_sequence::send()
{
//    // Clear DMA flags
//   DMA1->IFCR = (0b1111 << 16U);
//
//   DMA1_Channel5->CPAR = (uint32_t)&SPI2->DR;
//   DMA1_Channel5->CMAR = (uint32_t)led_data;
//   DMA1_Channel5->CNDTR = led_data_size;
//
//   // enable DMA transfer complete interrupt flag
//   DMA1_Channel5->CCR |= ( 0b1 << 1U); // (TCIE)
//   
//   // enable DMA
//   DMA1_Channel5->CCR |= (0b1 << 0U); // (OE)
//   // TXDMAEN should be set before enabling spi
//   SPI2->CR2 |= (0b1 << 1U); // (TXDMAEN)
   // enable the SPI 
   SPI2->CR1 |= (0b1 << 6U); // (SPE)

   SPI2->DR = led_data[0];
   xout::printf<100>("spi sr flags = %d\n",static_cast<uint32_t>(SPI2->SR));
   

//   for ( uint32_t i = 0U; i < led_data_size; ++i){
//      while ( (SPI2->SR & (0b1 << 1U)) != 0U){
//         asm volatile ("nop":::);
//  //  *(__IO uint8_t *)&SPI2->DR = led_data[i];

  // }
   
}

void led_sequence::putbit(uint32_t bit_idx_in, bool val)
{
   uint32_t const byte_idx = bit_idx_in / 8U;
   uint32_t const bit_idx  = bit_idx_in  % 8U;
   if ( val ) {
       led_data[byte_idx] |= (0b1 << bit_idx);
   }else{
       led_data[byte_idx] &= ~(0b1 << bit_idx);
   }
}

bool led_sequence::put(uint32_t index, rgb_value const & v)
{
   if ( index < num_leds){
      // bit position of start of the rgb entry in the output array
      uint32_t const out_arr_bitpos = index * 24U * 3U;
      for ( uint32_t i = 0U; i < 8U; ++i){
         uint8_t const bitmask = 1U << i;

         uint32_t const green_bit_pos = out_arr_bitpos + 22U - 3U * i;
         putbit(green_bit_pos,(v.green & bitmask) != 0U);

         uint32_t const red_bit_pos = green_bit_pos + 24U;
         putbit(red_bit_pos,(v.red & bitmask ) != 0U);

         uint32_t const blue_bit_pos = green_bit_pos + 48U;
         putbit(blue_bit_pos,(v.blue & bitmask) != 0U);
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

extern "C" void DMA1_Channel4_5_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void DMA1_Channel4_5_IRQHandler()
{
   
   // disable DMA interrupt flag
 //  DMA1_Channel5->CCR &= ~( 0b1 << 1U); // (TCIE)
   // Clear DMA flags
   DMA1->IFCR = (0b1111 << 16U);

//   // disable dma
//   DMA1_Channel5->CCR &= ~(0b1 << 0U); // (OE)
//   // disable the SPI 
//   SPI2->CR1 &= ~(0b1 << 6U); // (SPE)
//   SPI2->CR2 &= ~(0b1 << 1); // (TXDMAEN)
}




