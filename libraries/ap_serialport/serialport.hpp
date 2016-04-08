#ifndef QUAN_SUBHUB_SERIALPORT_HPP_INCLUDED
#define QUAN_SUBHUB_SERIALPORT_HPP_INCLUDED

#include <cstdarg>
#include <cstdint>
#include <cstddef>
#include <ap_common/ap_common.hpp>

namespace apm{

   struct SerialPort {
      enum {
         BASE_DEFAULT = 0,
         BASE_BIN     = 2,
         BASE_OCT     = 8,
         BASE_DEC     = 10,
         BASE_HEX     = 16
      };
      void begin(uint32_t b, uint16_t rxbufsize, uint16_t txbufsize) ;
      void begin(uint32_t b);
      void end();
      void flush();
      bool is_initialized();
      void set_blocking_writes(bool blocking);
      int16_t available(void);
      bool tx_pending();
      int16_t read();
      int16_t txspace();
      size_t write(uint8_t c);
      size_t write(const uint8_t *buffer, size_t size);
      size_t write(const char *str);
      size_t print(const char[]);
      size_t print(char);
      size_t print(unsigned char, int = BASE_DEC);
      size_t print(int, int = BASE_DEC);
      size_t print(unsigned int, int = BASE_DEC);
      size_t print(long, int = BASE_DEC);
      size_t print(unsigned long, int = BASE_DEC);
      size_t print(float , int = 2);
      size_t print(double , int = 2);
      size_t println(const char[]);
      size_t println(char);
      size_t println(unsigned char, int = BASE_DEC);
      size_t println(int, int = BASE_DEC);
      size_t println(unsigned int, int = BASE_DEC);
      size_t println(long, int = BASE_DEC);
      size_t println(unsigned long, int = BASE_DEC);
      size_t println(float , int = 2);
      size_t println(double , int = 2);
      size_t println(void);
      void printf(const char *s, ...) FORMAT(2, 3);
      void vprintf(const char *s, va_list ap);
      enum flow_control {
        FLOW_CONTROL_DISABLE=0, FLOW_CONTROL_ENABLE=1, FLOW_CONTROL_AUTO=2
      };
      void set_flow_control(enum flow_control flow_control_setting) {};
      enum flow_control get_flow_control(void) { return FLOW_CONTROL_DISABLE; };
      private:
      size_t printNumber(unsigned long, uint8_t);
      size_t printFloat(float, uint8_t);
   };

}// apm

#endif // QUAN_SUBHUB_SERIALPORT_HPP_INCLUDED

