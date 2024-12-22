#include "Arduino.h"
#include "powerSupply.h"
#include <digitalWriteFast.h>
#include <elapsedMillis.h>

#define bit9600Delay 104  
#define halfBit9600Delay 52
#define rx_pin 4
#define tx_pin 0
#define debug false

char powerSupply::command[14];
const char powerSupply::connect_commands[][14] = {"<09100000000>", "<08000000000>", "<01000000000>", "<03000000000>"}; //Sequence for starting  connection to power supply
const char powerSupply::disconnect_command[] = {"<09200000000>"}; //Sequence for closing  connection to power supply
const char powerSupply::output_commands[][14] = {"<08000000000>", "<07000000000>"}; //Disable and enable power supply output
const char powerSupply::get_voltage_command[] = {"<02012200000>"};
const char powerSupply::get_current_command[] = {"<04003300000>"};
static elapsedMicros powerSupply::com_timer;

powerSupply::powerSupply(){
}

void powerSupply::init(){
}

bool powerSupply::connect(){
  bool com_success = true;
  digitalWrite(tx_pin, HIGH); //Force clean start bit on init
  delay(1);
  for(uint8_t i=0; i<sizeof(connect_commands)/sizeof(connect_commands[0]); i++){
    retry_counter = retry_limit;
    while(retry_counter--){ 
      rs232print(connect_commands[i]);
      if (rs232read() || i == 1) break; //Toggle output (second command) does not have a reply
      delay(retry_delay);
      if(!retry_counter) com_success = false;
    }
  }
  return com_success;
}

bool powerSupply::disconnect(){
  retry_counter = retry_limit;
  while(retry_counter--){ 
    rs232print(disconnect_command); //Load command into string buffer
    if (rs232read()) break; //Toggle output (second command) does not have a reply
    delay(retry_delay);
    if(!retry_counter) return false;
  }
  return true;
}

bool powerSupply::toggleOutput(bool out_on){
  rs232print(output_commands[(uint8_t) out_on]); //Load command into string buffer
  if (!rs232read()); //Toggle output does not have a reply
  return true;
}

float powerSupply::getVoltage(){
  retry_counter = retry_limit;
  while(retry_counter--){ 
    rs232print(get_voltage_command);
    if (rs232read()) break; //Toggle output (second command) does not have a reply
    delay(retry_delay);
    if(!retry_counter) return -1;
  }
  return formatReply();
}

float powerSupply::getCurrent(){
  retry_counter = retry_limit;
  while(retry_counter--){
    rs232print(get_current_command);
    if (rs232read()) break; //Toggle output (second command) does not have a reply
    delay(retry_delay);
    if(!retry_counter) return -1;
  }
  return formatReply();
}

void powerSupply::getCommand(char* out_array){
  for(uint8_t i = 0; i<sizeof(out_array) && i < sizeof(command)-1; i++) out_array[i] = command[i];
}

bool powerSupply::setVoltage(float volt){
  retry_counter = retry_limit;
  if(volt > voltage_limit) volt = voltage_limit;
  if(volt < 0) volt = 0;
  while(retry_counter--){
    formatCommand('0', '1', volt);
    if (rs232read()) break; //Toggle output (second command) does not have a reply
    delay(retry_delay);
    if(!retry_counter) return false;
  }
  return true;
}

bool powerSupply::setCurrent(float amps){
  retry_counter = retry_limit;
  if(amps > current_limit) amps = current_limit;
  if(amps < 0) amps = 0;
  while(retry_counter--){
    formatCommand('0', '3', amps);
    if (rs232read()) break; //Toggle output (second command) does not have a reply
    delay(retry_delay);
    if(!retry_counter) return false;
  }
  return true;
}

bool powerSupply::disable(){
  if(!toggleOutput(0)) return false;
  if(!setCurrent(0)) return false;
  if(!setVoltage(0)) return false;
  return true;
}

float powerSupply::formatReply(){
  float value = 0;
  float factor = 100;
  uint8_t integer;
  for(uint8_t i = 0; i<7; i++){
    integer = command[i+3] - '0';
    if(command[i+3] < '0' || command[i+3] > '9') return -1;
    value += integer * factor;
    factor /= 10;
  }
  return value;
}

void powerSupply::formatCommand(char com1, char com2, float value){
  uint8_t i;
  uint8_t offset = 3;
  char temp_command[7];
  value *= 100; //Remove extra precision
  value = round(value);
  value /= 100;
  for(i=0; i<sizeof(command)-1; i++) command[i] = '0'; //Clear out command with '0' 
  dtostrf(value, 6, 2, temp_command);
  command[0] = '<';
  command[1] = com1;
  command[2] = com2;
  command[sizeof(command)-2] = '>';
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
  if(Serial && debug){Serial.print("tx->: "); Serial.println(data);}
  for(uint8_t i=0; i<sizeof(command)-1; i++) SWprint(data[i]);
}

void powerSupply::SWprint(char data)
{
  byte mask;
  //startbit
  cur_time = com_timer;
  digitalWrite(tx_pin,LOW);
  while(com_timer - cur_time < bit9600Delay);
  cur_time += bit9600Delay;
  for (mask = 0x01; mask>0; mask <<= 1) {
    if (data & mask){ // choose bit
     digitalWrite(tx_pin,HIGH); // send 1
    }
    else{
     digitalWrite(tx_pin,LOW); // send 0
    }
    while(com_timer - cur_time < bit9600Delay);
    cur_time += bit9600Delay;
  }
  //stop bit
  digitalWrite(tx_pin, HIGH);
  while(com_timer - cur_time < 2*bit9600Delay);
  cur_time += 2*bit9600Delay;
}

bool powerSupply::rs232read(){
  for(uint8_t i=0; i<sizeof(command)-1; i++){
    command[i] = SWread();
    if(command[i] < '0' || command[i] > 'O'){ //Check that character is valid
       if(Serial && debug){Serial.print("<-Bad reply. Retries remaining: "); Serial.println(retry_counter);}
       return false;
    } 
  }
  delay(1); //pad time between commands 
  if(Serial && debug){Serial.print("<-rx: "); Serial.println(command);}
  return true;
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
    cur_time = com_timer;
    while(com_timer - cur_time < halfBit9600Delay);
    cur_time += halfBit9600Delay;
    for (int offset = 0; offset < 8; offset++) {
      while(com_timer - cur_time < bit9600Delay);
      cur_time += bit9600Delay;
      val |= digitalRead(rx_pin) << offset;
    }
    //wait for stop bit + extra
    delayMicroseconds(halfBit9600Delay);
    return val;
  }
}