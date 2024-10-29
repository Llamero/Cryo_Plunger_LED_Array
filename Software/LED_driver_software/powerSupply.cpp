#include "Arduino.h"
#include "powerSupply.h"
#include <SoftwareSerial.h>
#include <digitalWriteFast.h>

#define bit9600Delay 104  
#define halfBit9600Delay 52
#define rx_pin 4
#define tx_pin 0

char powerSupply::command[13];
const char powerSupply::connect_commands[][14] = {"<09100000000>", "<08000000000>", "<01000000000>", "<03000000000>"}; //Sequence for starting  connection to power supply
const char powerSupply::disconnect_command[] = {"<09200000000>"}; //Sequence for closing  connection to power supply
const char powerSupply::output_commands[][14] = {"<08000000000>", "<07000000000>"}; //Disable and enable power supply output

powerSupply::powerSupply(){
}

void powerSupply::init(){
}

bool powerSupply::connect(){
  digitalWrite(tx_pin, HIGH); //Force clean start bit on init
  delay(1);
  for(uint8_t i=0; i<sizeof(connect_commands)/sizeof(connect_commands[0]); i++){
    rs232print(connect_commands[i]);
    if (!rs232read() && i != 1){ return false;} //Toggle output (second command) does not have a reply
    else{
       Serial.begin(250000);
       while(!Serial);
       Serial.println(command);
    }
  }
  return true;
}

bool powerSupply::disconnect(){
  for(uint8_t i=0; i<sizeof(command); i++) rs232print(disconnect_command[i]); //Load command into string buffer
  if (!rs232read()) return false;
  return true;
}

bool powerSupply::toggleOutput(bool out_on){
  for(uint8_t i=0; i<sizeof(command); i++) rs232print(output_commands[(uint8_t) out_on]); //Load command into string buffer
  if (!rs232read()); //Toggle output does not have a reply
  return true;
}

bool powerSupply::setVoltage(float volt){
  if(volt > voltage_limit) volt = voltage_limit;
  if(volt < 0) volt = 0;
  formatCommand('0', '1', volt);
  if (!rs232read()) return false;
  Serial.begin(250000);
  while(!Serial);
  Serial.println(command);
  return true;
}

bool powerSupply::setCurrent(float amps){
  if(amps > current_limit) amps = current_limit;
  if(amps < 0) amps = 0;
  formatCommand('0', '3', amps);
  if (!rs232read()) return false;
  Serial.begin(250000);
  while(!Serial);
  Serial.println(command);
  return true;
}

bool powerSupply::disable(){
  if(!toggleOutput(0)) return false;
  if(!setCurrent(0)) return false;
  if(!setVoltage(0)) return false;
  return true;
}

void powerSupply::formatCommand(char com1, char com2, float value){
  uint8_t i;
  uint8_t offset = 3;
  char temp_command[7];
  value *= 100; //Remove extra precision
  value = round(value);
  value /= 100;
  for(i=0; i<sizeof(command); i++) command[i] = '0'; //Clear out command with '0' 
  dtostrf(value, 6, 2, temp_command);
  command[0] = '<';
  command[1] = com1;
  command[2] = com2;
  command[sizeof(command)-1] = '>';
  for(i=0; i<sizeof(temp_command); i++){
    if(temp_command[i] == ' ' || temp_command[i] == 0) command[i+offset] = '0';
    else if (temp_command[i] == '.'){
      i++;
      offset -= 1;
      command[i+offset] = temp_command[i];
    }
    else command[i+offset] = temp_command[i];
  }
  rs232print(command);
}


void powerSupply::rs232print(char* data){
  for(uint8_t i=0; i<sizeof(command); i++) SWprint(data[i]);
}

void powerSupply::SWprint(char data)
{
  byte mask;
  //startbit
  digitalWrite(tx_pin,LOW);
  delayMicroseconds(bit9600Delay);
  for (mask = 0x01; mask>0; mask <<= 1) {
    if (data & mask){ // choose bit
     digitalWrite(tx_pin,HIGH); // send 1
    }
    else{
     digitalWrite(tx_pin,LOW); // send 0
    }
    delayMicroseconds(bit9600Delay);
  }
  //stop bit
  digitalWrite(tx_pin, HIGH);
  delayMicroseconds(bit9600Delay);
  delayMicroseconds(bit9600Delay);
}

bool powerSupply::rs232read(){
  bool valid_reply;
  for(uint8_t i=0; i<sizeof(command); i++){
    command[i] = SWread();
    if(!command[i]) valid_reply = false; //Null terminator means incomplete message received
  }
  delay(1); //pad time between commands 
  valid_reply = true;
  return valid_reply;
}

char powerSupply::SWread(){
  char val = 0;
  uint16_t timeout = 1;
  if(!digitalReadFast(rx_pin)) return false; //If pin is down at start, then there was a sync error
  while (timeout){
     if(!digitalReadFast(rx_pin)) break;
     delayMicroseconds(1);
     timeout++;
  }
  if(!timeout) return 0; //If timed out, return null
  
  //wait for start bit
  if (digitalRead(rx_pin) == LOW) {
    //digitalWrite(LED_BUILTIN, HIGH);
    delayMicroseconds(halfBit9600Delay);
    for (int offset = 0; offset < 8; offset++) {
     delayMicroseconds(bit9600Delay);
     val |= digitalRead(rx_pin) << offset;
    }
    //wait for stop bit + extra
    delayMicroseconds(halfBit9600Delay);
    return val;
  }
}