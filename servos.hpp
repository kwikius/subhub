#ifndef SERVOS_HPP_INCLUDED
#define SERVOS_HPP_INCLUDED

   struct servo_t{

      constexpr servo_t(uint8_t chan)
      :m_chan{chan}{}
      void enable();
      void set(uint16_t pos);
      uint16_t get()const;
      private:
         uint8_t const m_chan;
      static uint16_t min();
      static uint16_t max();

   };



#endif // SERVOS_HPP_INCLUDED
