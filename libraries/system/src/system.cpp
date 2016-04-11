
#include <cstddef>
#include <quan/malloc_free.hpp>

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