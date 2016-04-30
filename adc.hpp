#ifndef QUAN_SUBHUB_ADC_HPP_INCLUDED
#define QUAN_SUBHUB_ADC_HPP_INCLUDED

extern "C" void setup();

struct adc{

   static uint16_t read( uint8_t channel);

private:
   friend void ::setup();
   static void init();
};

#endif // QUAN_SUBHUB_ADC_HPP_INCLUDED
