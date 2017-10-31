#ifndef LIGHT_SHOW_EXAMPLES_HPP_INCLUDED
#define LIGHT_SHOW_EXAMPLES_HPP_INCLUDED

#include <quan/time.hpp>

void walking_led(
      rgb_value const & background_colour, 
      rgb_value const & walk_colour,
      quan::time_<uint32_t>::ms const & delay_duration,
      quan::time_<uint32_t>::ms const & demo_duration);

void blend(quan::time_<uint32_t>::ms const & demo_duration);

void pulse(quan::time_<uint32_t>::ms const & demo_duration);

#endif // LIGHT_SHOW_EXAMPLES_HPP_INCLUDED
