#ifndef STM32_EEPROM_TEST_LED_HPP_INCLUDED
#define STM32_EEPROM_TEST_LED_HPP_INCLUDED

struct led{

   static void on();
   static void off();
   static void complement();
   static void initialise();

};

#endif // STM32_EEPROM_TEST_LED_HPP_INCLUDED
