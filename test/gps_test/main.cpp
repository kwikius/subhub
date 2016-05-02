#include <apm/gps.hpp>
#include <quan/stm32/millis.hpp>

int main()
{
    apm::gps_t gps;
}

volatile uint32_t quan::stm32::detail::systick_tick::current = 0;