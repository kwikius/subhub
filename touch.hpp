#ifndef QUAN_SUBHUB_ADC_HPP_INCLUDED
#define QUAN_SUBHUB_ADC_HPP_INCLUDED

extern "C" void setup();

struct touch{

   static bool start_conversion();
   static bool conversion_complete();
   static bool conversion_good();
   static uint32_t get_count();
   static bool timeout();

private:

   static void initialise();
};

#endif // QUAN_SUBHUB_ADC_HPP_INCLUDED
