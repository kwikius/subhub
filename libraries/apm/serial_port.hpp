#ifndef QUAN_SUBHUB_SERIALPORT_HPP_INCLUDED
#define QUAN_SUBHUB_SERIALPORT_HPP_INCLUDED

#include <cstdint>
#include <cstddef>

namespace apm{

   /* The minimum requirements for apm::GPS serial port */
   struct abc_serial_port {
      virtual ~abc_serial_port(){}
      virtual void      begin(uint32_t baud)=0;  
      virtual int16_t   available()const=0;
      virtual int16_t   read()=0;         
      virtual int16_t   txspace()const=0;   
      virtual size_t    write(uint8_t c)=0;
      virtual size_t    write(const uint8_t *buffer, size_t size)=0;
   };

}// apm

#endif // QUAN_SUBHUB_SERIALPORT_HPP_INCLUDED

