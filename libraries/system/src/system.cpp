
#include <cstddef>
#include <quan/malloc_free.hpp>
#include <quan/stm32/millis.hpp>

extern "C" void __cxa_pure_virtual() { while (1); }

void *__dso_handle;


void * operator new(size_t size) throw()
{
  return quan::malloc(size);
}

void operator delete (void*p)
{
   quan::free(p);
}

volatile uint32_t quan::stm32::detail::systick_tick::current = 0;