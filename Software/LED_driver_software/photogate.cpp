#include "Arduino.h"
#include "photogate.h"
#include <digitalWriteFast.h>
#include <elapsedMillis.h>

#define bit9600Delay 104  
#define halfBit9600Delay 52
#define rx_pin 4
#define tx_pin 0
#define debug false

photogate::photogate(uint8_t val){
  output_pin = val;
}

void photogate::init(){
}

bool photogate::startPhotogate(){
  pinMode(output_pin, INPUT_PULLUP);
  photogate_triggered = false; //Reset photogate flag
  Serial.println("Sending initialize signal to photogate...");
  sendSync();
  if(waitForAck()) return true;
  else Serial.println("Ack FAILED!");
  return false;
}

void photogate::pulseTimer(){
  if (digitalReadFast(output_pin) == LOW) {
    if(prev_pin_state){
      start_time = micros(); // Start timing
      pulse_state = 1;
      prev_pin_state = false;
    }
  } else if(pulse_state){
    pulse_duration = micros() - start_time; // Stop timing
    pulse_state = 2; // Flag for loop
    prev_pin_state = true;
  }
}

void photogate::sendSync(){
  Serial.println(output_pin);
  pinMode(output_pin, OUTPUT);
  digitalWriteFast(output_pin, LOW);
  delayMicroseconds(300);
  pinMode(output_pin, INPUT_PULLUP);
}

bool photogate::waitForAck(){ //First short pulse = starting calibration, second long pulse = comparator ready
  bool photogate_calibrating = false;
  while(true){
    timer = 0;
    pulse_state = 0; //Reset pulse state
    prev_pin_state = true; //Reset pin state
    pinMode(output_pin, INPUT_PULLUP);
    while(timer < ACK_TIMEOUT[photogate_calibrating]){
      pulseTimer();
      if (pulse_state == 2) {
        pulse_state = 0; //Reset pulse state
        Serial.print("Pulse LOW Duration: ");
        Serial.print(pulse_duration);
        Serial.println(" us");
        break;
      }
    }
    if(timer > ACK_TIMEOUT[photogate_calibrating]){ //If timed out, return false
      Serial.print("Timeout waiting for ACK #"); 
      Serial.println(photogate_calibrating+1);
      return false; 
    } 
    else if(pulse_duration > 250 && pulse_duration < 400){ //If first pulse is 300 µs photogate is calibrating
      photogate_calibrating = true;
    }
    else if(photogate_calibrating && (pulse_duration > 750 && pulse_duration < 900)){ //If second pulse is 800 µs calibration complete
      break;
    }
    else{
      Serial.print("Invalid duration of ACK #");
      Serial.print(photogate_calibrating+1);
      if(photogate_calibrating) Serial.print(", expected 800 µs and received ");
      else Serial.print(", expected 300 µs and received ");
      Serial.println(pulse_duration);
      Serial.println("Check to ensure the beam path is not blocked.");
      return false; 
    }   
  }
  delay(100);
  Serial.println("Sync SUCCESS!");
  return true;
}
