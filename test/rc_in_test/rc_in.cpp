
#include <stm32f0xx.h>
#include <quan/stm32/tim.hpp>
#include <quan/stm32/gpio.hpp>
#include <quan/stm32/tim/temp_reg.hpp>
#include <quan/stm32/get_raw_timer_frequency.hpp>
#include <quan/constrain.hpp>
#include <quan/min.hpp>
#include "../../rc_in.hpp"

/*
  rc in on PB1
  TIM14_CH1
*/

namespace {

   // tim14 : 16 bit 48 MHz clk
   typedef quan::stm32::tim14 rc_in_timer;
   typedef quan::mcu::pin<quan::stm32::gpiob,1> rc_input_pin;

   constexpr uint32_t timer_freq = quan::stm32::get_raw_timer_frequency<rc_in_timer>();
   static_assert(timer_freq == 48000000,"unexpected timer frequency");
   constexpr uint16_t min_sync_pulse = 3000U ;
   constexpr uint16_t min_pulsewidth = 900U ;
   constexpr uint16_t max_pulsewidth = 2100U ;
   constexpr uint8_t  min_num_channels = 5U;
   constexpr uint8_t  max_num_channels = 11U;

   void rc_input_timer_setup()
   {
      quan::stm32::module_enable<rc_input_pin::port_type>();
      quan::stm32::module_enable<rc_in_timer>();

      // TI1 input --> IC1 capture on falling edge
      quan::stm32::apply<
         rc_input_pin,
         quan::stm32::gpio::mode::af0,  
         quan::stm32::gpio::pupd::pull_up
      >();
      {
         quan::stm32::tim::ccmr1_t ccmr1 = 0;
         ccmr1.cc1s   = 0b01;// CC1 channel is configured as input, IC1 is mapped on TI1
         ccmr1.ic1f   = 0b00 ;// no filter 
         ccmr1.ic1psc = 0b00; // no prescaler
         rc_in_timer::get()->ccmr1.set(ccmr1.value);
      }
      {
         quan::stm32::tim::ccer_t ccer = 0;
         ccer.cc1p  = true; // capture falling edges
         ccer.cc1np = false; // capture falling edges
         ccer.cc1ne = false; // applies to output only
         ccer.cc1e  = true;  // enable capture
         rc_in_timer::get()->ccer.set(ccer.value);
      }
      {
         quan::stm32::tim::dier_t dier= 0;
         dier.cc1ie = true;
         rc_in_timer::get()->dier.set(dier.value);
      }

      rc_in_timer::get()->psc = (timer_freq / 1000000 )-1;
      rc_in_timer::get()->arr = 0xFFFF;   
      rc_in_timer::get()->cnt = 0x0;
      rc_in_timer::get()->sr  = 0;  // clear flags

      NVIC_SetPriority(TIM14_IRQn,15); // low priority
      NVIC_EnableIRQ(TIM14_IRQn);
   }

   void start_timer()
   {
      rc_in_timer::get()->cr1.setbit<0>(); // (CEN)
   }

   // caoture reg value at last capture event
   uint16_t last_edge = 0U;
   // number of channels detected
   volatile uint8_t  m_num_channels = 0;
   // set num_channels to this out of range value to represent sync lost
   volatile uint8_t  channel_idx = max_num_channels + 3;
   // set to true when input is detected
   volatile bool     m_new_input = false;

   bool have_sync()
   {
      return channel_idx <= max_num_channels;
   }

   volatile uint16_t m_input_rc_channels [max_num_channels] ;

   void on_edge()
   {
      uint16_t const edge = rc_in_timer::get()->ccr1;
      
      uint16_t const pulse = edge - last_edge;
      last_edge = edge;
      if (pulse < min_sync_pulse ) {
         if (channel_idx < max_num_channels){
            m_input_rc_channels[channel_idx++]
            = quan::constrain(
               pulse
               ,min_pulsewidth 
               ,max_pulsewidth 
            );
         } 
      }else{ // sync pulse
         if(channel_idx <= max_num_channels){
            m_num_channels = channel_idx ;
            m_new_input = true;
            channel_idx = 0;
         }else{ // waiting for enough sync pulses
            -- channel_idx;
         }
      }
   }
   // n.b call from irq only or disable irqs
   void reset()
   {
      last_edge = 0U;
      m_num_channels = 0U;
      channel_idx = max_num_channels + 3;
      m_new_input = false;
   }
}

extern "C" void TIM1_CC_IRQHandler() __attribute__ ( (interrupt ("IRQ")));
extern "C" void TIM14_IRQHandler() __attribute__ ((interrupt ("IRQ")));

/*
   a new falling edge event
   TODO
   ##################################################
   input capture event
   If the time from last pulse is greater than some x ( e.g 1/50th sec + some factor)
   then reset to sync lost
   overflow event
   If  more than one overflow has occurred without a pulse
   then reset
   #####################################################
*/
extern "C" void TIM14_IRQHandler()
{
   //TODO check overflow 
   rc_in_timer::get()->sr.set(0);
   on_edge();
}

void rc_inputs::init()
{
   for ( auto & pulse : m_input_rc_channels){ 
      pulse = (min_pulsewidth + max_pulsewidth)/2;
   }
   rc_input_timer_setup();
   start_timer();
}

bool rc_inputs::have_new_input() 
{
   if ( have_sync()){
      bool const result = m_new_input;
      m_new_input = false;
      return result;
   }else{
      return false;
   }
}

uint8_t rc_inputs::get_num_channels() 
{
   if ( have_sync()){
      return m_num_channels;
   }else{
      return 0U;
   }
}

uint16_t rc_inputs::get_channel(uint8_t ch)
{
   if ( have_sync() && (ch < max_num_channels) ){
      return m_input_rc_channels[ch];
   }else{
      return (ch == 2)?900:1500; /* throttle (ch[2]) should be low, for safety */
   }
}
