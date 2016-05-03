
#include <stm32f0xx.h>
#include <quan/stm32/millis.hpp>
#include "../../usarts.hpp"
#include <quan/stm32/usart/irq_handler.hpp>
#include "../../servos.hpp"
#include <quan/conversion/itoa.hpp>

extern "C" void setup();

using quan::stm32::millis;

int main()
{
   setup();

   servo_t servo1{0};
   servo_t servo2{1};

   servo1.enable();
   servo2.enable();
   
   link_sp::serial_port::write("Servo Test\n");

   auto elapsed = millis();
   bool servo1_dir = true;
   quan::time_<uint32_t>::ms sp_out_last{0};
   bool servo2_dir = false;
   for(;;){
      auto const now = millis();
      if ( (now - sp_out_last) > quan::time_<uint32_t>::ms{100U}){
         sp_out_last = now;
         if ( link_sp::serial_port::tx_reg_empty()){
        
            link_sp::serial_port::write("srv1 = ");
            char buf[50];
            quan::itoasc(servo1.get(),buf,10);
            link_sp::serial_port::write(buf);
            link_sp::serial_port::write(", srv2 = ");
            quan::itoasc(servo2.get(),buf,10);
            link_sp::serial_port::write(buf);
            link_sp::serial_port::write("\n");
         }
         if ( now > elapsed){
            elapsed = now;
            {
               uint16_t const pos1 = servo1.get();
               if (servo1_dir){
                  if (  pos1 < 2000U){
                     servo1.set(pos1 + 1);
                  }else{
                     servo1_dir = false;
                  }
               }else{
                  if (  pos1 > 1000U){
                     servo1.set(pos1 - 1);
                  }else{
                     servo1_dir = true;
                  }
               }
            }
            {
               uint16_t const pos2 = servo2.get();
               if (servo2_dir){
                  if (  pos2 < 2000U){
                     servo2.set(pos2 + 1);
                  }else{
                     servo2_dir = false;
                  }
               }else{
                  if (  pos2 > 1000U){
                     servo2.set(pos2 - 1);
                  }else{
                     servo2_dir = true;
                  }
               }
            }
         }
      }
   }
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
         link_sp::serial_port::usart_type,quan::stm32::usart1
      >::value
   ,"invalid usart for serial_port irq");

   quan::stm32::usart::irq_handler<link_sp::serial_port>();
}

//uint32_t volatile quan::stm32::detail::systick_tick::current = 0;

extern "C" void SysTick_Handler()
{
   ++quan::stm32::detail::systick_tick::current;
   
}