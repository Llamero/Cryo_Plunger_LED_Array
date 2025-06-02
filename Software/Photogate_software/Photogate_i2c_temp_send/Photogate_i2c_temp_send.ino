#include <Wire.h>
#include "pinSetup.h"
#include <elapsedMillis.h>

pinSetup pin;
elapsedMillis timer;
uint32_t elapsed_timer;
uint8_t power[3];

const int SERIES_RESISTOR = 4700; //Value of series resistor to the thermistor on the PCB
const int PCB_THERMISTOR_NOMINAL = 4700; //Value of thermistor resistor on PCB at nominal temp (25°C)
const int PCB_B_COEFFICIENT = 3545; //Beta value for the PCB thermistor
const float set_temp = 50;

uint8_t i2c_buffer[20];
uint8_t buffer_len = 0;
uint16_t adc_temp;

union BYTE16UNION
{
 uint16_t bytes_var;
 uint8_t bytes[2];
}uint16Union;

void setup() {
  pin.configurePins();
  Wire.begin();
  adc_temp = tempToAdc(set_temp);
}

void loop() {
  timer = 0;
  elapsed_timer = 0;
  while(timer<30000){
    for(uint8_t a=0; a<3; a++){
      if(timer >= 2000 && timer <= 5000 && power[a] != 255){
        power[a] = 255;
        analogWrite(pin.heater[a], power[a]);
      } 
      else if((timer < 2000 || timer > 5000) && power[a] != 0){
        power[a] = 0;
        analogWrite(pin.heater[a], power[a]);  
      }
      uint16Union.bytes_var = analogRead(pin.temp[a]);
      buffer_len = sizeof(uint16Union.bytes);
      memcpy(i2c_buffer + sizeof(uint16Union.bytes)*a, uint16Union.bytes, buffer_len);
      if(uint16Union.bytes_var < adc_temp && pin.heater_state[a]){ 
        digitalWrite(pin.heater[a], LOW);
        pin.heater_state[a] = false;
        power[a] = 0;
      }
    }
    i2c_buffer[8] = power[0];
    buffer_len = 9;
 

    if(timer-elapsed_timer >= 50){
      uint16Union.bytes_var = (uint16_t) timer;
      memcpy(i2c_buffer + 6, uint16Union.bytes, sizeof(uint16Union.bytes));
      sendDataWire();
      elapsed_timer += 50;
    }
  }
}

void sendDataWire() {
  Wire.beginTransmission(0x54);     // prepare transmission to slave with address 0x54
  for (uint8_t i = 0; i < buffer_len; i++) {
    Wire.write(i2c_buffer[i]);           // Write the received data to the bus buffer
  }              // add new line and carriage return for the Serial monitor
  Wire.endTransmission();           // finish transmission
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

uint16_t tempToAdc(float temperature){
  float steinhart = temperature;
  float raw;
  steinhart += 273.15;  
  steinhart = 1.0 / steinhart;  
  steinhart -= 1.0 / (25 + 273.15); // + (1/To); 
  steinhart *= PCB_B_COEFFICIENT;
  steinhart = exp(steinhart);
  raw = steinhart * PCB_THERMISTOR_NOMINAL; 
  raw = SERIES_RESISTOR/raw;
  raw = 1024/(raw+1); 
  return (int) round(raw);
}