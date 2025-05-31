#ifndef pinSetup_h
#define pinSetup_h

#include "Arduino.h"

class pinSetup
{
  public:
    pinSetup();
    static void configurePins();

    const static uint8_t temp[];
    const static uint8_t heater[];
    const static uint8_t sensor = 3;
    const static uint8_t output = 4;
    const static uint8_t SDA = 6;
    const static uint8_t SCL = 7;
    const static uint8_t UPDI = 11;

    
  private:
    static void init(); //Initialize reference variables

};
#endif   
