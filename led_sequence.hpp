#ifndef SUBHUB_TEST_LED_SEQUENCE_HPP_INCLUDED
#define SUBHUB_TEST_LED_SEQUENCE_HPP_INCLUDED

extern "C" void setup();
extern "C" void SPI2_IRQHandler();

struct rgb_value{
   rgb_value(uint8_t red_in,uint8_t green_in, uint8_t blue_in) : red{red_in},green{green_in},blue{blue_in}{}
   uint8_t red;
   uint8_t green;
   uint8_t blue;
};

struct led_sequence{

   static constexpr uint32_t num_leds = 8U;
   static constexpr uint32_t preamble = 40U;
   static constexpr uint32_t bits_per_colorbit = 3U;
   static constexpr uint32_t bytes_per_neopixel = 3U;
   static constexpr uint32_t zerobits = 1U;
   // put colour v to index index
   static bool put(uint32_t index, rgb_value const & v);
   // do a transfer to all the leds
   static void send();
   // return number of bytes left to transfer
   // if zero then no transfer in progress
   static uint32_t transfer_bytes_left();
private:
   static void putbit(uint32_t bit_idx, bool val);
   friend void ::setup();
   friend void SPI2_IRQHandler();
   static void initialise();
   static constexpr uint32_t led_data_size = bytes_per_neopixel * bits_per_colorbit * num_leds;
   static uint8_t led_data [led_data_size + preamble];   
};

#endif // SUBHUB_TEST_LED_SEQUENCE_HPP_INCLUDED
