
#include <stm32f0xx.h>
#include <quan/stm32/usart/irq_handler.hpp>
#include <quan/conversion/itoa.hpp>
#include <quan/stm32/millis.hpp>
#include <quan/constrain.hpp>
#include "../../usarts.hpp"
#include "../../neopixel.hpp"
#include "led.hpp"

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

void walking_led(
      rgb_value const & background_colour, 
      rgb_value const & walk_colour,
      ms const & delay_duration,
      ms const & demo_duration)
{

     uint8_t pos = 0;
     auto now = millis();
     while ( (millis() - now) < demo_duration){
        neopixel::put(pos,background_colour);
        pos = (pos +1) % 8;
        neopixel::put(pos,walk_colour);
        neopixel::send();
        delay(delay_duration);
    }

}

namespace {
   rgb_value blend_colours(rgb_value const & c1,rgb_value const & c2, float ratio_in)
   {
        float ratio = quan::constrain(ratio_in,0.f,1.f);
        rgb_value result;
         result.red = c1.red* ratio + c2.red * (1-ratio);
         result.green = c1.green* ratio + c2.green * (1-ratio);
         result.blue = c1.blue* ratio + c2.blue * (1-ratio);
        return result;
   }
   constexpr rgb_value red = {12,0,0};
   constexpr rgb_value blue = {0,0,12};
   constexpr rgb_value white = {4,4,4};
   constexpr rgb_value green = {0,12,0};
   constexpr rgb_value black = {0,12,0};
    
   rgb_value colours1 [] = {blue, red,  white, green, blue, red,  black,green};

   rgb_value colours2 [] = {green, blue, black, red,   red, blue, red,  black,};
}

void blend(ms const & demo_duration)
{
    float ratio = 0.5f;
    float incr = 0.1f;
    bool pos_dir = true;
   auto now = millis();
   while ((millis() - now) < demo_duration){
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
         rgb_value blend = blend_colours(colours1[i],colours2[i],ratio);
         neopixel::put(i,blend);
       }
      
       neopixel::send();
       delay(100_ms);
  }
}

void pulse(ms const & demo_duration)
{

   rgb_value color = {0,0,0};
   int state = 0;
   uint32_t pos = 0;
   constexpr uint32_t maxv = 15U;
   auto now = millis();
   while ( (millis() - now ) < demo_duration){

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

          for ( uint32_t i = 0; i < 8; ++i){
           neopixel::put((pos + i) % 8U,color);
          }
     pos = (pos + 1U) % 8U;

     neopixel::send();
     delay (30_ms);
     
     }
}

/*

light displays
*/