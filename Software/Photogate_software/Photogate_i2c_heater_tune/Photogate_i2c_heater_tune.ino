#include <Wire.h>
#include "pinSetup.h"
#include <elapsedMillis.h>
#include <Comparator.h>

pinSetup pin;
elapsedMillis timer;  //General ms timer for messages and timeouts
elapsedMicros interrupt_timer; //Fast timer for timing interrupt durations


const uint8_t LED_INTENSITY = 20; //IR LED intensity from 0-255
uint32_t elapsed_message; //Timestamp when last status message was send
uint8_t power[3] = {255, 255, 255};
const uint8_t ARDUINO_ADDRESS = 0x54;
const uint8_t DIGIPOT_ADDRESS = 0x2c;
const int SERIES_RESISTOR = 4700; //Value of series resistor to the thermistor on the PCB
const int PCB_THERMISTOR_NOMINAL = 4700; //Value of thermistor resistor on PCB at nominal temp (25°C)
const int PCB_B_COEFFICIENT = 3545; //Beta value for the PCB thermistor
const uint16_t ADC_MAX = 1024;
const float HEATER_SET_TEMP = 40; //Temperature of the board heaters

uint8_t serial_buffer[20];
uint8_t buffer_len = 0;
uint16_t STATUS_MESSAGE_INTERVAL = 500; //How often in ms to send status update.  0 = off

uint16_t photodiode_baseline; //Adc value of reference voltage used to bias photodiode
uint16_t photodiode_max; //Photodiode adc value with LED on 
uint8_t photodiode_gain; //8-bit resitor value used to set photodiode gain
uint8_t led_intensity; //8-bit resistor value used to set LED current
uint8_t photodiode_state = 0; //0 = inactive; 1 = beam intact; 2 = beam broken
uint32_t elapsed_photogate = 0; //Time in ms photodiode has been waiting for trigger
const uint16_t PHOTOGATE_TIMEOUT = 9000; //Time in ms to wait for photogate to be tripped
volatile uint32_t interrupt_duration; //Duration that output pin was pulled low

union BYTE16UNION
{
 uint16_t bytes_var;
 uint8_t bytes[2];
}uint16Union;

void ac_interrupt()
{
  if (AC1.STATUS & AC_STATE_bm){
    photodiode_state = 1;
    interrupt_duration = interrupt_timer;
  }
  else{ 
    photodiode_state = 2;
    interrupt_timer = 0;
  }
}

ISR(PORTB_PORT_vect) {
  if (PORTB.INTFLAGS & PIN3_bm) {
    bool current_state = (PORTB.IN & PIN3_bm);

    if (current_state) interrupt_duration = interrupt_timer;
    else interrupt_timer = 0; //Reset interrupt timer
    
    PORTB.INTFLAGS = PIN3_bm; // Clear the interrupt flag for PB3
  }
}

void setup() {
  pin.configurePins();
  Wire.begin();
  turnOffPhotogate();
  startInterrupt();
}

void loop() {
  checkHeater();
  if(interrupt_duration) messageRouter();
  if(photodiode_state){
    if(timer-elapsed_photogate > PHOTOGATE_TIMEOUT || (photodiode_state == 2 && interrupt_timer > 1000)){
      stopComparator();
      startInterrupt();
      turnOffPhotogate();
    }
  }
  if(timer-elapsed_message >= STATUS_MESSAGE_INTERVAL && STATUS_MESSAGE_INTERVAL){
    sendStatus();
  }
}

void startInterrupt() {
  pinMode(pin.output, INPUT_PULLUP);
  PORTB.DIRCLR = PIN3_bm; // Set PB3 as input
  PORTB.PIN3CTRL = PORT_PULLUPEN_bm; // Optional: Enable pull-up resistor
  PORTB.PIN3CTRL |= PORT_ISC_BOTHEDGES_gc; // Enable interrupt on both edges
  sei(); // Enable global interrupts
}

void stopInterrupt(){
  PORTB.PIN3CTRL &= ~PORT_ISC_gm;           // Clear existing ISC bits
  PORTB.PIN3CTRL |= PORT_ISC_INTDISABLE_gc; // Disable interrupt
  PORTB.INTFLAGS = PIN3_bm; // Clear any pending interrupt flag for PB3
}

void messageRouter(){
  if(interrupt_duration > 100 && interrupt_duration < 500){ //If a valid 300 µs pulse was received, send an ACK
    stopInterrupt();
    delay(1);
    pinMode(pin.output, OUTPUT);
    digitalWriteFast(pin.output, LOW);
    delayMicroseconds(300);
    pinMode(pin.output, INPUT_PULLUP);
    if(runCalibration()){ //If calibration successful, send ack
      pinMode(pin.output, OUTPUT);
      digitalWriteFast(pin.output, LOW);
      delayMicroseconds(800);
      pinMode(pin.output, INPUT_PULLUP);
      delay(10);
      startComparator();
    }
    else{ //If calibration failed turn off photogate and wait for next sync command
      stopComparator();
      turnOffPhotogate();
      startInterrupt();
    }
  }
  interrupt_duration = 0; //Reset interrupt
}

//https://github.com/SpenceKonde/megaTinyCore/tree/master/megaavr/libraries/Comparator
//https://github.com/grughuhler/attiny/blob/main/attiny_ac/attiny_ac.ino
void startComparator(){
  pinMode(pin.output, OUTPUT);
  Comparator1.input_p = comparator::in_p::in0;       // pos input PA7.  See datasheet
  Comparator1.input_n = comparator::in_n::dacref;    // neg pin to the DACREF voltage
  Comparator1.reference = comparator::ref::vref_4v3; // Set the DACREF voltage
  Comparator1.dacref = 127;                          // (dacref/256)*VREF

  Comparator1.hysteresis = comparator::hyst::large;  // Use 50mV hysteresis
  Comparator1.output = comparator::out::enable;      // Enable output PB3
  Comparator1.output_initval = comparator::out::init_high; // Output pin high after initialization

  Comparator1.init();
  Comparator1.attachInterrupt(ac_interrupt, CHANGE);
  Comparator1.start();
  elapsed_photogate = timer; //Start photogate timer
  photodiode_state = 1;  //Set state to active
}

void stopComparator(){
  Comparator1.detachInterrupt();
  Comparator1.stop(true); // Stop comparator. Digital input on the pins that this comparator was using will be re-enabled.
  photodiode_state = 0; //Set state to standby
}

void turnOffPhotogate(){
  setLedIntensity(0);
  setPhotodiodeGain(0);
}

void sendStatus(){
    buffer_len = sizeof(uint16Union.bytes)*3;
    memcpy(serial_buffer + buffer_len, power, sizeof(power));
    buffer_len += sizeof(power);
    uint16Union.bytes_var = photodiode_baseline;
    memcpy(serial_buffer + buffer_len, uint16Union.bytes, sizeof(uint16Union.bytes));
    buffer_len += sizeof(uint16Union.bytes);
    uint16Union.bytes_var = photodiode_max;
    memcpy(serial_buffer + buffer_len, uint16Union.bytes, sizeof(uint16Union.bytes));
    buffer_len += sizeof(uint16Union.bytes);
    serial_buffer[buffer_len] = photodiode_gain;
    buffer_len++;
    serial_buffer[buffer_len] = LED_INTENSITY;
    buffer_len++;
    serial_buffer[buffer_len] = photodiode_state;
    buffer_len++;
    sendDataWire(ARDUINO_ADDRESS);
    serial_buffer[0] = B00000000;
    elapsed_message += STATUS_MESSAGE_INTERVAL;
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
  memcpy(serial_buffer + sizeof(uint16Union.bytes)*heater_index, uint16Union.bytes, sizeof(uint16Union.bytes));
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

bool runCalibration(){
  uint8_t bit_mask;
  uint16_t bias_voltage;
  uint8_t index;
  stopInterrupt();
  stopComparator();
  turnOffPhotogate();
  analogRead(pin.sensor);
  bias_voltage = analogRead(pin.sensor);
  photodiode_baseline = 0;
  while(bias_voltage != photodiode_baseline){ //Wait for the bias voltage to stabilize
    delay(200);
    photodiode_baseline = bias_voltage;
    bias_voltage = analogRead(pin.sensor); 
  }
  setLedIntensity(LED_INTENSITY); //Turn on LED and calibrate photodiode gain
  for(index = 0; index < 8; index++){ //Calibrate gain one bit at a time
    bit_mask = B10000000 >> index;
    photodiode_gain += bit_mask;
    setPhotodiodeGain(photodiode_gain);
    delay(100);
    photodiode_max = analogRead(pin.sensor);
    if(photodiode_max > 800) photodiode_gain -= bit_mask;
  }
  delay(100);
  photodiode_max = analogRead(pin.sensor);
  setLedIntensity(0); //Turn off LED
  delay(100);
  photodiode_baseline = analogRead(pin.sensor);
  setLedIntensity(LED_INTENSITY);
  delay(100);
  startInterrupt();
  if(photodiode_gain < 200 && photodiode_max > 700 && photodiode_max < 900) return true;
  else return false;
}

void setLedIntensity(uint8_t intensity){
    serial_buffer[0] = B10000000;
    serial_buffer[1] = intensity;
    buffer_len = 2;
    sendDataWire(DIGIPOT_ADDRESS);
}

void setPhotodiodeGain(uint8_t gain){
    serial_buffer[0] = B00000000;
    serial_buffer[1] = gain;
    buffer_len = 2;
    sendDataWire(DIGIPOT_ADDRESS);
}

void sendDataWire(uint8_t address) {
  Wire.beginTransmission(address);     // prepare transmission to slave with address 0x54
  for (uint8_t i = 0; i < buffer_len; i++) {
    Wire.write(serial_buffer[i]);           // Write the received data to the bus buffer
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