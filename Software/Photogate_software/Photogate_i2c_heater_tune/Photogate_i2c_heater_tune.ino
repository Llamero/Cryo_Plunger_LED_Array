#include <Wire.h>
#include "pinSetup.h"
#include <elapsedMillis.h>


pinSetup pin;
elapsedMillis timer;

uint32_t elapsed_message;
uint8_t power[3] = {255, 255, 255};
const uint8_t ARDUINO_ADDRESS = 0x54;
const uint8_t DIGIPOT_ADDRESS = 0x2c;
const int SERIES_RESISTOR = 4700; //Value of series resistor to the thermistor on the PCB
const int PCB_THERMISTOR_NOMINAL = 4700; //Value of thermistor resistor on PCB at nominal temp (25°C)
const int PCB_B_COEFFICIENT = 3545; //Beta value for the PCB thermistor
const uint16_t ADC_MAX = 1024;
const float HEATER_SET_TEMP = 40; //Temperature of the board heaters

uint8_t i2c_buffer[20];

uint8_t buffer_len = 0;

uint16_t photodiode_baseline; //Adc value of reference voltage used to bias photodiode
uint16_t photodiode_max; //Photodiode adc value with LED on 
uint8_t photodiode_gain; //8-bit resitor value used to set photodiode gain
uint8_t led_intensity; //8-bit resistor value used to set LED current


union BYTE16UNION
{
 uint16_t bytes_var;
 uint8_t bytes[2];
}uint16Union;

void setup() {
  pin.configurePins();
  Wire.begin();
  i2c_buffer[0] = 0;
  sendDataWire(DIGIPOT_ADDRESS);
  i2c_buffer[0] = 128;
  sendDataWire(DIGIPOT_ADDRESS);
  runCalibration();
}

void loop() {
  checkHeater();
  if(timer-elapsed_message >= 200){
    sendStatus();
  }
}

void sendStatus(){
    buffer_len = sizeof(uint16Union.bytes)*3;
    memcpy(i2c_buffer + buffer_len, power, sizeof(power));
    buffer_len += sizeof(power);
    uint16Union.bytes_var = photodiode_baseline;
    memcpy(i2c_buffer + buffer_len, uint16Union.bytes, sizeof(uint16Union.bytes));
    buffer_len += sizeof(uint16Union.bytes);
    uint16Union.bytes_var = photodiode_max;
    memcpy(i2c_buffer + buffer_len, uint16Union.bytes, sizeof(uint16Union.bytes));
    buffer_len += sizeof(uint16Union.bytes);
    i2c_buffer[buffer_len] = photodiode_gain;
    buffer_len++;
    i2c_buffer[buffer_len] = led_intensity;
    buffer_len++;
    uint16Union.bytes_var = analogRead(pin.sensor);
    memcpy(i2c_buffer + buffer_len, uint16Union.bytes, sizeof(uint16Union.bytes));
    buffer_len += sizeof(uint16Union.bytes);
    sendDataWire(ARDUINO_ADDRESS);
    i2c_buffer[0] = B00000000;
    elapsed_message += 200;
}

void checkHeater(){
  const uint16_t INCREMENT_INTERVAL = 1000;
  const uint8_t TEMP_ARRAY_SIZE = 64;
  const static uint16_t ADC_TEMP = tempToAdc(HEATER_SET_TEMP);
  static uint32_t elapsed_heater[3];
  static uint16_t temp_array[3][TEMP_ARRAY_SIZE];
  static uint16_t temp_array_index[3];
  static uint16_t min_adc[3] = {65535, 65535, 65535};
  static uint16_t max_adc[3] = {0, 0, 0};
  uint16_t heater_reduction;
  static uint8_t heater_index;
  uint8_t b;

  heater_index++;
  if(heater_index >= 3) heater_index = 0;
  temp_array[heater_index][temp_array_index[heater_index]] = analogRead(pin.temp[heater_index]); //Record value in circular buffer
  temp_array_index[heater_index]++;
  if(temp_array_index[heater_index] >= TEMP_ARRAY_SIZE) temp_array_index[heater_index] = 0; //Roll over index
  uint16Union.bytes_var = 0;
  for(b=0; b<TEMP_ARRAY_SIZE; b++) uint16Union.bytes_var += temp_array[heater_index][b];
  uint16Union.bytes_var >>= 6;
  memcpy(i2c_buffer + sizeof(uint16Union.bytes)*heater_index, uint16Union.bytes, sizeof(uint16Union.bytes));
  if(uint16Union.bytes_var < ADC_TEMP){ //If heater is over set temperature
    if(pin.heater_state[heater_index]){ //Turn off heater if it is on
      analogWrite(pin.heater[heater_index], 0);
      pin.heater_state[heater_index] = false;
      min_adc[heater_index] = 65535; //initialize max temp
      max_adc[heater_index] = 0; //Reset min temp
    }
    if(uint16Union.bytes_var <= min_adc[heater_index]) min_adc[heater_index] = uint16Union.bytes_var; //Record max temp 
  }
  else if(uint16Union.bytes_var > ADC_TEMP){ //If below cut-off temperature
    if(!pin.heater_state[heater_index]){
      heater_reduction = (ADC_TEMP - min_adc[heater_index]) << 1;
      if(heater_reduction > 100) heater_reduction = 100;
      if(power[heater_index] >= heater_reduction) power[heater_index] -= heater_reduction; //Reduce heater by overshoot
      if(power[heater_index]) power[heater_index]--;
      analogWrite(pin.heater[heater_index], power[heater_index]); //Turn on heater
      pin.heater_state[heater_index] = true; 
      elapsed_heater[heater_index] = timer; //Reset timer
      min_adc[heater_index] = 65535; //Reset max temp
      max_adc[heater_index] = 0; //initialize min temp
    }
    else if(timer-elapsed_heater[heater_index] >= INCREMENT_INTERVAL && uint16Union.bytes_var >= max_adc[heater_index]){
      max_adc[heater_index] = uint16Union.bytes_var;
      if(power[heater_index] < 255) power[heater_index]++; //If temp is falling, turn up heater
      elapsed_heater[heater_index] += INCREMENT_INTERVAL;
      analogWrite(pin.heater[heater_index], power[heater_index]);
    }
  }
}

void runCalibration(){
  uint8_t bit_mask;
  uint16_t bias_voltage;
  uint8_t index;
  setLedIntensity(0); //Turn off IR LED
  setPhotodiodeGain(0); //Turn down photodiode gain
  delay(100);
  bias_voltage = analogRead(pin.sensor); //Measure photodiode baseline voltage
  for(index = 0; index < 8; index++){ //Calibrate gain one bit at a time
    bit_mask = B10000000 >> index;
    photodiode_gain += bit_mask;
    setPhotodiodeGain(photodiode_gain);
    delay(10);
    photodiode_baseline = analogRead(pin.sensor);
    if(photodiode_baseline > bias_voltage + 1) photodiode_gain -= bit_mask;
  }
  delay(100);
  photodiode_baseline = analogRead(pin.sensor);

  for(index = 0; index < 8; index++){ //Calibrate gain one bit at a time
    bit_mask = B10000000 >> index;
    led_intensity += bit_mask;
    setLedIntensity(led_intensity);
    delay(10);
    photodiode_max = analogRead(pin.sensor);
    if(photodiode_max > 800) led_intensity -= bit_mask;
  }
  delay(100);
  photodiode_max = 0;
  index = 64;
  while(index--){
    photodiode_max += analogRead(pin.sensor);
    delay(10);
  }
  photodiode_max >>= 6;
}

void setLedIntensity(uint8_t intensity){
    i2c_buffer[0] = B10000000;
    i2c_buffer[1] = intensity;
    buffer_len = 2;
    sendDataWire(DIGIPOT_ADDRESS);
}

void setPhotodiodeGain(uint8_t gain){
    i2c_buffer[0] = B00000000;
    i2c_buffer[1] = gain;
    buffer_len = 2;
    sendDataWire(DIGIPOT_ADDRESS);
}

void sendDataWire(uint8_t address) {
  Wire.beginTransmission(address);     // prepare transmission to slave with address 0x54
  for (uint8_t i = 0; i < buffer_len; i++) {
    Wire.write(i2c_buffer[i]);           // Write the received data to the bus buffer
  }              // add new line and carriage return for the Serial monitor
  Wire.endTransmission();           // finish transmission
}

float adcToTemp(uint16_t adc){
  float steinhart;
  float raw = (float) adc;
  raw = ADC_MAX / raw - 1;
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
  raw = ADC_MAX/(raw+1); 
  return (int) round(raw);
}