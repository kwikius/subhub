
#include <stm32f0xx.h>

#include <quan/stm32/tim.hpp>
#include <quan/stm32/tim/temp_reg.hpp>
#include <quan/stm32/get_raw_timer_frequency.hpp>
#include <quan/stm32/gpio.hpp>
#include <quan/stm32/rcc.hpp>

#include "../../touch.hpp"
/*
  touch pin on pa6 , TSC_G2_IO3 (AF3)
  cap on pa7, TSC_G2_IO4 (AF3)

  no load capacitance is around 38 pF
  touch det capacitance is (say) 50 pF
*/

void touch::initialise()
{
   // turn on the tsc in rcc
  // in rcc bit 24 in AHB reg
   static constexpr uint32_t tsc_en_bit = 24U;
   quan::stm32::rcc::get()->ahbenr |= (1U << tsc_en_bit); //(TSC_EN)
   // reset tsc
   quan::stm32::rcc::get()->ahbrstr |= ( 1U << tsc_en_bit ); // (TSC_EN)
   quan::stm32::rcc::get()->ahbrstr &= ~( 1U << tsc_en_bit ); // (TSC_EN)

   // enable TSC in CR 
   TSC->CR |= (1U << 0U); // (TSCE)

   // set up the pins
   // electrode on PA6

   typedef quan::mcu::pin<quan::stm32::gpioa,6>    electrode_pin; // user button on disco
   // sample capacitor on PA7
   typedef quan::mcu::pin<quan::stm32::gpioa,7>    sample_cap_pin; // user button on disco

  // The touch group is G2
/*
ref man 27.3.7
Sampling capacitor I/O mode
To allow the control of the sampling capacitor I/O by the TSC peripheral, the corresponding
GPIO must be first set to alternate output open drain mode and then the corresponding
Gx_IOy bit in the TSC_IOSCR register must be set.
Only one sampling capacitor per analog I/O group must be enabled at a time.
*/

   quan::stm32::module_enable<sample_cap_pin::port_type>();
   quan::stm32::apply<
      sample_cap_pin
      , quan::stm32::gpio::mode::output
      , quan::stm32::gpio::otype::open_drain
      , quan::stm32::gpio::ostate::low
   >();



// setup electrode pin
/*
ref man 27.3.7
Channel I/O mode
To allow the control of the channel I/O by the TSC peripheral, the corresponding GPIO must
be first set to alternate output push-pull mode and the corresponding Gx_IOy bit in the
TSC_IOCCR register must be set
*/
   quan::stm32::module_enable<electrode_pin::port_type>();
   quan::stm32::apply<
      electrode_pin
      , quan::stm32::gpio::mode::output
      , quan::stm32::gpio::otype::push_pull
      , quan::stm32::gpio::ostate::low
   >();
// discharge
   for ( uint32_t i = 0; i < 500; ++i){
    asm volatile ("nop":::);
   }

// set pins to af
   quan::stm32::apply<
      electrode_pin
      , quan::stm32::gpio::mode::af3
   >();

   quan::stm32::apply<
      sample_cap_pin
      , quan::stm32::gpio::mode::af3
   >();

   TSC->CR |= (0b100 << 12U);// (PGPSC)
   TSC->CR |= (6U << 28U); // (CTPH)
   TSC->CR |= (6U << 24U); // (CTPL)

   // enable G2E group 
   TSC->IOGCSR |= (1U << 1U); //(G2E)

   // set max count value to 16383
   TSC->CR |= (0b110 << 5U); // (MCV)

   // disable hysteresis
   TSC->IOHCR &= ~(0b11 << 6U);
   // enable analogue switch control
   TSC->IOASCR |= (0b11 << 6U);
   // set sample cap to TSC_G2_IO4
   // N.B.only one sample cap per group allowed
   TSC->IOSCR |= (1U << 7U); // IOSCR_G2_IO4)
   // set electrode to TSC_G2_IO3
   TSC->IOCCR |= (1U << 6U); // IOCCR_G2_IO3)

// clocking
  // clock is derived from fHCLK which is 48 MHz I think

  // TSC pulse generator clk is fPGCLK
  // fPGCLK can be divided by 1 to 128 (in powers of 2) from fHCLK
  // PULSE is set t by TSC->CR PGPSC to divison of sysclk on F0 part.
  // TSC_CR reg

  // set num fPGCLK of electrode charging in tsc_CR->CTPH
/*
  // use inline electrode resistor of 100 R
      sample cap of 10 nF form PA7 to ground
  // chrging discharging 100 pF electro load takes 100 e-12 * 100 * 10 = 0.1 usec
  // so set charge transfer time to 1 usec
  // charge time depends on clk speed and CTPL reg
  // divide clock by 16
  // then count 3 in ctph
  
*/


  // To start a conversion TSC->CR start flag
  // Then look for conv complete  in TSC->IOGCSR
  // If threshold then then count in TSC->IOGXCR[1]
}

bool touch::start_conversion()
{
   // set iodef low to discharge the IOs
   // if conversion in prog return false
   // eg CR start is set
   if ( (TSC->CR & ( 1U << 1U) ) == 0U /*(START)*/){
      // clear flags
      TSC->ICR = 0b11;
      while ( TSC->ICR  & 0b11 ){
         asm volatile ("nop":::);
      }
      TSC->CR |= (1U << 1U); // (START)
      return true;
   }else{
      return false;
   }
}

bool touch::conversion_complete()
{
   return  (TSC->IOGCSR & (1 << 17)) != 0U;
}

bool touch::timeout()
{
   return  (TSC->ISR & (1U << 1U) ) != 0U;  // (MCEF)
}

bool touch::conversion_good()
{
   return  (TSC->ISR & (1U << 0U) ) == (1U << 0U); //(EOAF)
}

uint32_t touch::get_count()
{
   return TSC->IOGXCR[1]; // Careful as array off by 1
}


