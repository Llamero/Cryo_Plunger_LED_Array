/*
  Default configuration of teensy pins
*/
#ifndef photogate_h
#define photogate_h

#include "Arduino.h"
#include <elapsedMillis.h>

class photogate{
  public:
    photogate(uint8_t val);// Parameterized constructor
    void init();
    bool startPhotogate();
  
  private:
    elapsedMillis timer;
    bool photogate_triggered;
    uint8_t photogate_state; //0=standby, 1=calibrating, 2=ready;
    uint8_t output_pin;
    bool prev_pin_state;
    void pulseTimer();
    void sendSync();
    bool waitForAck();
    bool waitForPhotogate();
    unsigned long start_time;
    unsigned long pulse_duration;
    byte pulse_state; //State of interrupt pulse 0=wait for start (falling), 1=wait for end (rising), 2=pulse complete
    const uint16_t ACK_TIMEOUT[2] = {100, 5000};
    const uint16_t PHOTOGATE_TIMEOUT = 10000; 
};
#endif