// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this

// Created 29 March 2006

// This example code is in the public domain.
const int SERIES_RESISTOR = 4700; //Value of series resistor to the thermistor on the PCB
const int PCB_THERMISTOR_NOMINAL = 4700; //Value of thermistor resistor on PCB at nominal temp (25°C)
const int PCB_B_COEFFICIENT = 3545; //Beta value for the PCB thermistor

#include <Wire.h>
union BYTE16UNION
{
 uint16_t bytes_var;
 uint8_t bytes[2];
}uint16Union;

void setup() {
  Wire.begin(0x54);                // join I2C bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(115200);           // start serial for output
}

void loop(){}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  delay(10);
  while (0 < Wire.available()) { // loop through all but the last
    for(uint8_t a=0; a<3; a++){
      uint16Union.bytes[0] = Wire.read();
      uint16Union.bytes[1] = Wire.read();
      float temp = adcToTemp(uint16Union.bytes_var);
      Serial.print(temp);
      Serial.print("°C, ");
    }
    for(uint8_t a=0; a<3; a++){
      uint8_t power = Wire.read();
      Serial.print(power);
      Serial.print(", ");
    }
    Serial.println();
  }
}

float adcToTemp(uint16_t adc){
  float steinhart;
  float raw = (float) adc;
  raw = 1024 / raw - 1;
  raw = SERIES_RESISTOR / raw;
  steinhart = raw / PCB_THERMISTOR_NOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= PCB_B_COEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (25 + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;   
  return steinhart;
}
