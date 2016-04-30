
#include <stm32f0xx.h>
#include <quan/min.hpp>
#include <quan/stm32/tim.hpp>
#include <quan/stm32/tim/temp_reg.hpp>
#include <quan/stm32/get_raw_timer_frequency.hpp>
#include <quan/stm32/gpio.hpp>
#include <quan/stm32/rcc.hpp>
#include <quan/constrain.hpp>

/*
   ADC on 
   PA0  e.g airspeed
   PA2(RSSI)

   ADC DMA on dma channel 1

   ADC can be triggered by  TIM1, TIM2, TIM3 or TIM15

  // trigger adc at 100 Hz
  // use tim15
   
*/

namespace {

   typedef quan::stm32::tim15 adc_timer;

   volatile uint16_t adc_results [2] = {0,0};

   typedef quan::mcu::pin<quan::stm32::gpioa,0> analog_pin1;
   typedef quan::mcu::pin<quan::stm32::gpioa,2> analog_pin2;

   template <typename Pin>
   void setup_adc_pin()
   {
      quan::stm32::module_enable<typename Pin::port_type>();
      quan::stm32::apply<
         Pin
         ,quan::stm32::gpio::mode::analog
         ,quan::stm32::gpio::pupd::none
      >();
   };
}

void adc_setup()
{
   setup_adc_pin<analog_pin1>();
   setup_adc_pin<analog_pin2>();

   quan::stm32::module_enable<adc_timer>();
   
   {
      constexpr uint32_t timer_freq = quan::stm32::get_raw_timer_frequency<adc_timer>();
      static_assert((timer_freq % static_cast<uint32_t>(1000000U))==0U,"unexpected raw timer frequency");
      constexpr uint32_t psc = (timer_freq / static_cast<uint32_t>(1000000U)) - 1U;
      // presc to 1 MHz
      adc_timer::get()->cr1 = 0;
      adc_timer::get()->psc = psc;
      adc_timer::get()->arr = 10000U-1U;  // 100Hz overflow 
      adc_timer::get()->cnt = 0;
      adc_timer::get()->sr = 0;
   }

   {  // connect TRGO to timer update
      quan::stm32::tim::cr2_t cr2 = adc_timer::get()->cr2.get();
      cr2.mms = 0b010; // TRGO is timer update
      adc_timer::get()->cr2.set(cr2.value);
   }

   // enable adc in rcc apb2 bit 9
   quan::stm32::rcc::get()->apb2enr.setbit<9>();
// calibration step
   ADC1->CR |= (1 << 31);
   while (  ADC1->CR & (1 << 31)) {;}
   
   ADC1->CR |= (1 << 0) ; // (ADEN)
   // setup adc clock
   // set up clock mode N.B ADC must be disabled
   // set to internal clock (pclk) at pclk/4
   ADC1->CFGR2 = (0b10 << 30); //(CKMODE[1:0])
   // select channels to be converted PA0 --> ADC_IN0, PA2 --> ADC_IN2
   ADC1->CHSELR = (1 << 0) | ( 1 << 2); 
   // scan direction up 
   ADC1->CFGR1 &= ~( 1<< 2);
   // sampling time Min 1 us for 12 bit accuracy
   // clock is 12 MHz . Say 2 us for simplicity 
   // so min 24 cycles nearest available is 28.5 cyc ( according to d/s 25 k input impedance ok)
   ADC1->SMPR = 0b011;
   // set 12 bit resolution
   ADC1->CFGR1 &= ~(0b11 << 3); // 
   // to start with just use irq
   // single conversion triggered by tim15 overflow
   ADC1->CFGR1 &= ~(1 << 13U); // (CONT)
   // select trigger 4 which is TIM15 ref man table 32
   ADC1->CFGR1 = (ADC1->CFGR1 & ~(0b111 << 6)) | ( 0b100 << 6); // (EXTSEL[2:0])
   // select rising edge
   ADC1->CFGR1 = (ADC1->CFGR1 & ~(0b11 << 10)) | ( 0b01 << 10); // (EXTEN[1:0])
   // right aligned
   ADC1->CFGR1 &= ~(1 << 5);// (ALIGN)
   // do whole sequence  get EOSEQ irq
   ADC1->CFGR1 |= (1<<16);// (DISCEN)
   // start timer
   adc_timer::get()->cr1.setbit<0>(); //(CEN)
   // enable interrupts
   ADC1->IER |= (1 << 3) | (1 << 2) ; // (EOSEQIE) (EOCIE)
   ADC1->CR |= (1 << 2); // (ADSTRT)
}

namespace {

   uint8_t cur_adc_chan = 0;
}

extern "C" void ADC1_COMP_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void ADC1_COMP_IRQHandler()
{
   adc_results[cur_adc_chan] = ADC1->DR; // clears EOC
   if (ADC1->ISR & ( 1 << 3)){ // (EOSEQ)
      ADC1->ISR |= (1 << 3);   // clear EOSEQ
      cur_adc_chan = 0;
   }else{
      ++ cur_adc_chan;
   }
}


