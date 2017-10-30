
#include <stm32f0xx.h>
#include <quan/stm32/usart/irq_handler.hpp>
#include <quan/conversion/itoa.hpp>
#include <quan/stm32/millis.hpp>
#include "../../usarts.hpp"
#include "../../led_sequence.hpp"
#include "led.hpp"
#include "light_show_examples.hpp"

extern "C" void setup();

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



int main()
{
   setup();

  led::off();
   xout::write("Led sequence Test\n");

//   for ( uint8_t i = 0U; i < 8U;++i){
//     led_sequence::put(i,{0x8,0x1F,0x0});
//   }
   rgb_value red = {12,0,0};
   rgb_value blue = {0,0,12};
   rgb_value white = {4,4,4};
   rgb_value green = {0,12,0};

for (;;){
    walking_led(white, blue,250_ms, 8000_ms);
    blend(10000_ms);
    pulse(10000_ms);
    walking_led(green, blue,100_ms, 5000_ms);
    blend(10000_ms);
    walking_led(green, red,50_ms, 6000_ms);
    pulse(10000_ms);
    walking_led(red,blue,25_ms, 5000_ms);
    walking_led(red,white,50_ms, 4000_ms);
    blend(20000_ms);
}

#if 0
 #if 0
//   uint32_t c = 0;
//   
   rgb_value red = {12,0,0};
   rgb_value blue = {0,0,12};
   rgb_value white = {4,4,4};
   rgb_value green = {0,12,0};
    
   rgb_value colours1 [] = {blue, red, white,green, blue, red, white,green,};

   rgb_value colours2 [] = {green, blue, red, white,green,blue, red, white,};

    float ratio = 0.5f;
    float incr = 0.01f;
    bool pos_dir = true;

   for (;;){
       if (pos_dir){
         if( ratio >= 1.f){
            pos_dir = false;
            ratio -= incr;
          }else{
            ratio += incr;
          }
       }else {
         if ( ratio <= 0.f){
            pos_dir = true;
            ratio +=incr;
         }else{
            ratio -= incr;
         }
       }
       for (uint32_t i = 0; i < 8; ++i){
         rgb_value blend = add_colours(colours1[i],colours2[i],ratio);
         led_sequence::put(i,blend);
       }
      
       led_sequence::send();
       delay(500_ms);
  }

//       for ( uint32_t i = 0U; i < 10U; ++i){
//             for (uint32_t j = 0; j < 8; ++i){
//               led_sequence::put(j,colours[4]);
//              }
//            delay(2000_ms);
//            led_sequence::send();
//       }

//       for (uint8_t i = 0; i < 8; ++i){
//         led_sequence::put((i+1)%8U,colours[i]);
//       }
//       delay(2000_ms);
//       led_sequence::send();
//
//       for (uint8_t i = 0; i < 8; ++i){
//         led_sequence::put(i,red);
//       }
//       delay(2000_ms);
//       led_sequence::send();
//
//       for (uint8_t i = 0; i < 8; ++i){
//         led_sequence::put(i,colours[i]);
//       }
//       delay(2000_ms);
//       led_sequence::send();

 // }

  #else
 #if 1
   rgb_value color = {0,0,0};
   int state = 0;
   uint32_t pos = 0;
   constexpr uint32_t maxv = 15U;
   for ( ;;){

        switch (state){
            case 0: {
               if ( color.red ==maxv){
                 state =1;
                  --color.red;
               }else{
                  ++color.red;
               }
               break;
            }
            case 1:{
              if ( color.red == 0){
                  ++color.green ;
                  state =2;
               }else{
                  --color.red;
               }
             break;
            }
            case 2:{
              if ( color.green == maxv){
                  state = 3;
                  --color.green;
               }else{
                  ++color.green;
               }
             break;
            }
            case 3:{
               if ( color.green ==0){
                  state = 4;
                  ++color.blue;
               }else{
                  --color.green;
               }
             break;
            }
            case 4: {
               if ( color.blue ==maxv){
                  state =5;
                  -- color.blue;
               }else{
                  ++color.blue;
               }
             break;
            }
            case 5: {
               if ( color.blue ==0){
                  state =0;
                  ++color.red;
               }else{
                  --color.blue;
               }
             break;
            }

        }
     
//        led_sequence::put((pos) % 8U,{0U,0U,0U});
//        led_sequence::put((pos + 1) % 8U,color);
          for ( uint32_t i = 0; i < 8; ++i){
           led_sequence::put((pos + i) % 8U,color);
          }
     pos = (pos + 1U) % 8U;
     delay (20_ms);
     led_sequence::send();
   }
  #else
    uint32_t pos = 0U;
   rgb_value red = {12,0,0};
   rgb_value blue = {0,0,12};
   rgb_value white = {4,4,4};
   rgb_value green = {0,12,0};
    for ( ;;){
       led_sequence::put(pos,green);
        pos = (pos +1) % 8;
        led_sequence::put(pos,blue);

       led_sequence::send();
       delay(125_ms);
    }
  #endif
   
   
#endif
#endif

   xout::write("Led sequence test completed\n");

   for(;;){
      asm volatile ("nop":::);
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

//volatile uint32_t quan::stm32::detail::systick_tick::current = 0;

extern "C" void SysTick_Handler()
{
   ++quan::stm32::detail::systick_tick::current;
   
}