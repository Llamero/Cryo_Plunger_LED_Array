// Wire Slave Receiver
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Receives data as an I2C/TWI slave device
// Refer to the "Wire Master Writer" example for use with this

// Created 29 March 2006
#include<digitalWriteFast.h>
#include <elapsedMillis.h>

elapsedMillis timer;

// This example code is in the public domain.
const int SERIES_RESISTOR = 4700; //Value of series resistor to the thermistor on the PCB
const int PCB_THERMISTOR_NOMINAL = 4700; //Value of thermistor resistor on PCB at nominal temp (25°C)
const int PCB_B_COEFFICIENT = 3545; //Beta value for the PCB thermistor
const uint16_t ADC_MAX = 1024;
const uint8_t output_pin = 2;

uint8_t serial_buffer[20];
uint8_t buffer_len = 0;

volatile unsigned long start_time = 0;
volatile unsigned long pulse_duration = 0;
volatile byte pulse_state = 0; //State of interrupt pulse 0=wait for start (falling), 1=wait for end (rising), 2=pulse complete
const uint16_t ACK_TIMEOUT[2] = {100, 5000};
const uint16_t PHOTOGATE_TIMEOUT = 10000; 


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
  pinMode(output_pin, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWriteFast(LED_BUILTIN, LOW);
}

void loop(){
  if(timer > 500){
    timer = 0;
    Serial.println("Send sync");
    sendSync();
    if(waitForAck()) waitForPhotogate();
    else Serial.println("Ack FAILED!");
    delay(2000);
  }
}

void interruptTimer(){
  if (digitalRead(output_pin) == LOW) {
    start_time = micros(); // Start timing
    pulse_state = 1;
  } else if(pulse_state){
    pulse_duration = micros() - start_time; // Stop timing
    pulse_state = 2;                 // Flag for loop
  }
}

void sendSync(){
  detachInterrupt(digitalPinToInterrupt(output_pin));
  pinMode(output_pin, OUTPUT);
  digitalWriteFast(output_pin, LOW);
  delayMicroseconds(300);
  pinMode(output_pin, INPUT_PULLUP);
}

bool waitForAck(){ //First short pulse = starting calibration, second long pulse = comparator ready
  for(uint8_t a=0; a<2; a++){
    timer = 0;
    pulse_state = 0; //Reset pulse state
    pinMode(output_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(output_pin), interruptTimer, CHANGE);
    while(timer < ACK_TIMEOUT[a]){
      if (pulse_state == 2) {
        detachInterrupt(digitalPinToInterrupt(output_pin));
        pulse_state = 0; //Reset pulse state
        Serial.print("Pulse LOW Duration: ");
        Serial.print(pulse_duration);
        Serial.println(" us");
        break;
      }
    }
    if(timer > ACK_TIMEOUT[a]){ //If timed out, return false
      Serial.print("Timeout waiting for ACK #"); 
      Serial.println(a+1);
      return false; 
    } 
    else if(!a && !(pulse_duration > 200 && pulse_duration < 500)){ //If first pulse isn't 300 µs return false
      Serial.print("Invalid duration of ACK #1, expected 300 µs and received ");
      Serial.print(pulse_duration);
      return false; 
    }
    else if(a && !(pulse_duration > 700 && pulse_duration < 1000)){ //If second pulse isn't 800 µs return false  
      Serial.print("Invalid duration of ACK #2, expected 800 µs and received ");
      Serial.println(pulse_duration);
      Serial.println("Check to ensure the beam path is not blocked.");
      return false; 
    }   
  }
  delay(100);
  Serial.println("Sync SUCCESS!");
  return true;
}

void waitForPhotogate(){
  timer = 0;
  pulse_state = 0;
  pinMode(output_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(output_pin), interruptTimer, CHANGE);
  while(timer < PHOTOGATE_TIMEOUT && (micros() - start_time > 1000 || pulse_state != 1)); //Wait for photogate
  if(timer < PHOTOGATE_TIMEOUT){
    detachInterrupt(digitalPinToInterrupt(output_pin));
    Serial.println("PHOTOGATE!");
    while(!digitalRead(output_pin));
  } 
  else Serial.println("Photogate timeout");
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  uint8_t value;
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
      value = Wire.read();
      Serial.print("Heater #");
      Serial.print(a+1);
      Serial.print(": ");
      Serial.print(value);
      Serial.print(", ");
    }
    for(uint8_t a=0; a<2; a++){
      if(a) Serial.print("Max: ");
      else Serial.print("Baseline: ");
      uint16Union.bytes[0] = Wire.read();
      uint16Union.bytes[1] = Wire.read();
      Serial.print(uint16Union.bytes_var);
      Serial.print(", ");
    }
    Serial.print("Gain: ");
    value = Wire.read();
    Serial.print(value);
    Serial.print(", ");
    Serial.print("Current: ");
    value = Wire.read();
    Serial.print(value);

    Serial.print(", Sensor: ");
    value = Wire.read();
    Serial.print(value);
    Serial.println();
  }
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
