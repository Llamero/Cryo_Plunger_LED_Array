#include "Arduino.h"
#include "pinSetup.h"
#include "Wire.h"

const uint8_t pinSetup::temp[] = {8, 9, 10};
const uint8_t pinSetup::heater[] = {5, 1, 0};
bool pinSetup::heater_state[] = {false, false, false};

pinSetup::pinSetup(){
}

void pinSetup::init(){
}

void pinSetup::configurePins(){
  for(uint8_t a=0; a<3; a++){
    pinMode(temp[a], INPUT);
    pinMode(heater[a], OUTPUT);
    digitalWrite(heater[a], LOW);
  }
  pinMode(sensor, INPUT);

  // ADC0.CTRLC = ADC_PRESC_DIV4_gc; // Set ADC clock prescaler (e.g., DIV4 for faster clock)
  // ADC1.CTRLC = ADC_PRESC_DIV4_gc; // Set ADC clock prescaler (e.g., DIV4 for faster clock)
}
