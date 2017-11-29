
#include <stm32f0xx.h>
#include <quan/stm32/millis.hpp>
#include <quan/stm32/usart/irq_handler.hpp>
#include <quan/conversion/itoa.hpp>
#include "../../usarts.hpp"
#include "led.hpp"
#include "sh1106_oled.hpp"

/*
Test of I2C using a 24LC128 eeprom
 A led is set up on Servo 1 port for diagnostics
*/

// TODO som fonts have own named function
//GFXfont const * get_fontGaruda10pt7b();
GFXfont const * get_font();

extern "C" void setup();

using quan::stm32::millis;
namespace {

   typedef decltype(millis()) ms;

   ms operator "" _ms(unsigned long long int v)
   {
      return static_cast<ms>(v);
   }
   void delay(ms t)
   {
      ms elapsed = millis();
      while ((millis() - elapsed) < t) {;}
   }

   typedef link_sp::serial_port xout;
}

using quan::stm32::millis;

namespace {
   typedef link_sp::serial_port xout;

   const char* commands[] = {
       "  Sit."
      ,"Lie Down."
      ," Speak."
      ,"  Beg."
      ," Treat?"
      ,"  Paw."
   };  
}

void show_splash()
{
     sh1106_oled::set_buffer_to(0);
     
     sh1106_oled::draw_text({15,25},"Oled Display",get_font(),true);
     sh1106_oled::draw_text({15,50},"V1.0",get_font(),true);
     sh1106_oled::write_buffer();
     delay(3000_ms);
}

int main()
{
   setup();

   typedef sh1106_oled::point point;

   show_splash();

   uint32_t constexpr num_commands = sizeof(commands) / sizeof(commands[0]);
   uint32_t cmd_idx = 0U;
   for ( ;;){
      sh1106_oled::set_buffer_to(0);

      sh1106_oled::draw_text({30,40},commands[cmd_idx],get_font(),true);
      sh1106_oled::write_buffer();
      cmd_idx = (cmd_idx +1) % num_commands;
      delay(2000_ms);
   }

   while (1){;}

}

extern "C" void USART2_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void USART2_IRQHandler()
{
   static_assert(
      std::is_same<
         aux_sp::serial_port::usart_type,quan::stm32::usart2
      >::value
   ,"invalid usart for serial_port irq");

   quan::stm32::usart::irq_handler<aux_sp::serial_port>();
}

extern "C" void USART1_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void USART1_IRQHandler()
{
   static_assert(
      std::is_same<
         xout::usart_type,quan::stm32::usart1
      >::value
   ,"invalid usart for serial_port irq");

   quan::stm32::usart::irq_handler<link_sp::serial_port>();
}

//volatile uint32_t quan::stm32::detail::systick_tick::current = 0;

extern "C" void SysTick_Handler()
{
   ++quan::stm32::detail::systick_tick::current;
   
}