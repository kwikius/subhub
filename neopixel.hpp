#ifndef SUBHUB_NEOPIXEL_HPP_INCLUDED
#define SUBHUB_NEOPIXEL_HPP_INCLUDED

#include <cstdint>

extern "C" void setup();

#if defined (NEOPIXEL_USE_TIM2)
// note both options use same dma channel
extern "C" void DMA1_Channel2_3_IRQHandler() __attribute__ ( (interrupt ("IRQ")));
extern "C" void  TIM2_IRQHandler() __attribute__ ( (interrupt ("IRQ")));
#else
extern "C" void DMA1_Channel2_3_IRQHandler() __attribute__ ( (interrupt ("IRQ")));
extern "C" void  TIM16_IRQHandler() __attribute__ ( (interrupt ("IRQ")));
#endif

struct rgb_value{
   constexpr rgb_value(): red{0U},green{0U},blue{0U}{}
   constexpr rgb_value(uint8_t red_in,uint8_t green_in, uint8_t blue_in) : red{red_in},green{green_in},blue{blue_in}{}
   uint8_t red;
   uint8_t green;
   uint8_t blue;
};

struct neopixel{

   static constexpr uint32_t num_leds = 8U;
   static_assert (num_leds %2 == 0 ,"num leds must be even");
   static constexpr uint32_t bytes_per_led = 3U;
   // put colour v to index index
   static bool put(uint32_t index, rgb_value const & v);
   // do a transfer to all the leds
   static void send();
   // return number of bytes left to transfer
   // if zero then no transfer in progress
   static uint32_t transfer_bytes_left();
private:
   static void putbit(uint32_t bit_idx, bool val);
   static void copy_user_to_dma();
   static inline void refill(uint32_t dma_buf_id, uint32_t data_idx);
   friend void ::setup();
// ok used for both timer options
   friend void ::DMA1_Channel2_3_IRQHandler();
   static void initialise();
   static rgb_value user_led_data[num_leds];  
   static rgb_value dma_led_data[num_leds];
   static uint8_t dma_buffer[ 8U * bytes_per_led * 2U];
};

#endif // SUBHUB_NEOPIXEL_HPP_INCLUDED
