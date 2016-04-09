#ifndef QUAN_SUBHUB_SERIALPORT_HPP_INCLUDED
#define QUAN_SUBHUB_SERIALPORT_HPP_INCLUDED

#include <cstdarg>
#include <cstdint>
#include <cstddef>
#include <ap_common/ap_common.hpp>

namespace apm{

   struct SerialPort {
      void begin(uint32_t baud);
      size_t available(void);
      size_t read();
      size_t txspace();
      size_t write(uint8_t c);
      size_t write(const uint8_t *buffer, size_t size);
      static constexpr bool FLOW_CONTROL_DISABLE = false;
      static constexpr bool FLOW_CONTROL_ENABLE = true;
      void set_flow_control(bool) {};
   };

}// apm

#endif // QUAN_SUBHUB_SERIALPORT_HPP_INCLUDED

