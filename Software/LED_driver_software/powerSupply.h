/*
  Default configuration of teensy pins
*/
#ifndef powerSupply_h
#define powerSupply_h

#include "Arduino.h"
#include <SoftwareSerial.h>
class powerSupply{
  public:
    powerSupply();
    void init();
    bool connect();
    bool disconnect();
    bool setVoltage(float);
    bool setCurrent(float);
    bool toggleOutput(bool);
    bool disable();

  private:
    void psTx();
    void psRx();
    void formatCommand(char, char, float);
    void rs232print(char*);
    bool rs232read();
    void SWprint(char);
    char SWread();
    
    const static char powerSupply::connect_commands[][14]; //Sequence for starting  connection to power supply
    const static char powerSupply::disconnect_command[]; //Sequence for closing  connection to power supply
    const static char powerSupply::output_commands[][14]; //Disable and enable power supply output
    static char command[13]; //Array for RS-232 commands sent
    
    const static uint8_t com_delay = 50;
    const static uint8_t current_limit = 1;
    const static uint8_t voltage_limit = 200;
    float active_voltage;
    float active_current;
    float active_output;
    uint8_t tx_pin;
    uint8_t rx_pin;
};
#endif