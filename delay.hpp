#ifndef SUBHUB_DELAY_HPP_INCLUDED
#define SUBHUB_DELAY_HPP_INCLUDED

#include <quan/stm32/millis.hpp>

inline void delay(quan::time_<uint32_t>::ms const & t)
{
   auto const now = quan::stm32::millis();
   while ( (quan::stm32::millis() - now ) < t){
     asm volatile ("nop":::);
   }
}

#endif // SUBHUB_DELAY_HPP_INCLUDED
