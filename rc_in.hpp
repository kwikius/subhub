#ifndef QUAN_SUBHUB_RC_IN_HPP_INCLUDED
#define QUAN_SUBHUB_RC_IN_HPP_INCLUDED

extern "C" void setup();

   struct rc_inputs {

      static bool     have_new_input() ;
      static uint8_t  get_num_channels() ;
      static uint16_t get_channel(uint8_t ch);
   private:
      static void     init();
      friend void     ::setup();
      rc_inputs() = delete;
      rc_inputs (rc_inputs const &) = delete;
      rc_inputs& operator = (rc_inputs const &) = delete;
   };

#endif // QUAN_SUBHUB_RC_IN_HPP_INCLUDED
