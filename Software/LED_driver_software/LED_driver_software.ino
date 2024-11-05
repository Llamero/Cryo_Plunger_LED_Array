#include <digitalWriteFast.h>
#include <elapsedMillis.h>
#include "powerSupply.h"
#define USE_TIMER_1     true
#define USE_TIMER_3     true
#warning Using Timer1, Timer3
#include "TimerInterrupt_Generic.h"

constexpr struct pulseStruct{
  uint8_t duration = 100; //Total duration of LED pulse: 0-255 ms
  float current = 3; //Peak output current in amps
  uint16_t  delay = 500; //Delay from trigger event to LED on: 0-65535 ms
  uint16_t cooldown_period = 5000; //Cooldown delay after LED turns off: 0-65535 ms 
} pulse;

//Button LED 
#define LED_PWM 1
#define LED_FLASH_INTERVAL 500
#define off 0
#define red 10 //Device armed for LED trigger
#define flashing_red 11 //Cooling down LED and discharging cap
#define green 20 //Driver in standby
#define flashing_green 21 //Driver in standby due to error condition
#define yellow 40 //Calibrated waiting for press to arm trigger
#define flashing_yellow 41 //Calibrating

//System parameters
#define series_resistance 31 //Total series resistance of the LED driver
#define max_safe_voltage 30 //Maximum voltage before performing high voltage checks
#define max_ps_voltage 200 //Maximum voltage the power supply can output
#define max_ps_current 1 //Maximum current the power supply can output
#define debounce 100 //Debounce delay (ms)

const struct inputPinsStruct{
  uint8_t rx = 4;
  uint8_t door = 20;
  uint8_t pushbutton = 12;
} in;
const uint8_t in_pins[sizeof(in)] = {in.rx, in.door, in.pushbutton};

const struct outputPinsStruct{
  uint8_t led[2] = {10, 11};
  uint8_t speaker[2] = {5, 9};
  uint8_t led_trigger = 3;
  uint8_t tx = 0;
} out;
const uint8_t out_pins[sizeof(out)] = {out.led[0], out.led[1], out.speaker[0], out.speaker[1], out.led_trigger, out.tx};

const struct analogPinsStruct{
  uint8_t vsense = 8;
  uint8_t isense = 18;
  uint8_t audio = 21;
} ana;
const uint8_t ana_pins[sizeof(ana)] = {ana.vsense, ana.isense, ana.audio};

constexpr static struct stateStruct{
  uint8_t standby = 0;
  uint8_t calibrating_led = 1;
  uint8_t waiting_trapdoor = 2;
  uint8_t waiting_plunger = 3;
  uint8_t led_on = 5;
  uint8_t led_cooldown = 6;
  uint8_t capacitor_discharging = 7;
  
  uint8_t door_open = 101;
  uint8_t com_failure = 102;
  uint8_t bulb_out = 103;
  uint8_t driver_under_current = 104;
  uint8_t driver_under_voltage = 105;
  uint8_t driver_over_current = 106;
  uint8_t driver_over_voltage = 107;
  uint8_t ps_under_current = 108;
  uint8_t ps_under_voltage = 109;
  uint8_t ps_over_current = 110;
  uint8_t ps_over_voltage = 111;
  uint8_t no_mic = 113;
  uint8_t ps_voltage_limit = 114;
  uint8_t ps_unstable = 115;
  uint8_t reboot = 116;
} state;

const struct defaultStatusStruct{
  float driver_voltage = 0;
  float driver_current = 0;
  float ps_voltage = 0;
  float ps_current = 0;
  uint16_t audio = 0;
  bool door_interlock = false;
  uint8_t state = 0; //0 = good, 1 = failsafe, 2 = minor error, 3 = critical error
} default_status;

struct statusStruct{
  float driver_voltage = 0;
  float driver_current = 0;
  float ps_voltage = 0;
  float ps_current = 0;
  uint16_t audio = 0;
  bool door_interlock = false;
  bool button_led[2] = {0, 0};
  uint8_t button_color = 0; //0=off, 10 = solid red, 11 = flashing red, 20 = solid yellow, 21 = flashing yellow, 31 = solid green, 32 = flashing green
  uint8_t state = 0;
} status;

const uint16_t flash_interval = 500;
const float vref = 4.2; //ADC Vref
const float vfactor = 0.01960784313; //Voltage divider on Vsense
uint8_t status_index;
bool flash; //Tracks whether led is on or off when flashing

powerSupply ps;
elapsedMillis driver_timer;

void setColor(uint8_t led, bool flashing = false); //Declare prototype with default arguments
void(* resetFunc) (void) = 0;//declare reset function at address 0

void setup() {
  //Sertup serial
  Serial.begin(250000);
}

void loop() {
  //Setup device
  initialize();
 
  //Wait for pushbutton to be pressed and released
  while(digitalReadFast(in.pushbutton)){checkStatus(); delay(50);}
  playStatusTone(); 
  delay(debounce);
  while(!digitalReadFast(in.pushbutton)){checkStatus(); delay(50);}
  delay(debounce);
  
  //Start LED calibration
  calibrate();

  //Wait for pushbutton to be pressed and released
  while(digitalReadFast(in.pushbutton)){checkStatus(); delay(50);}
  playStatusTone(); 
  delay(debounce);
  while(!digitalReadFast(in.pushbutton)){checkStatus(); delay(50);}
  delay(debounce);

  //Start LED pulse sequence
  ps.toggleOutput(true);
  delay(3000);
  digitalWriteFast(out.led_trigger, HIGH);
  delay(50);
  digitalWriteFast(out.led_trigger, LOW);
  ps.toggleOutput(false);
  dischargeCapacitor();


  //Wait for pushbutton to be pressed and released
  while(digitalReadFast(in.pushbutton)){checkStatus(); delay(50);}
  playStatusTone(); 
  delay(debounce);
  while(!digitalReadFast(in.pushbutton)){checkStatus(); delay(50);}
  delay(debounce);
}

void initialize(){
  //Set pin states
  uint8_t i;
  for(i=0; i<sizeof(in_pins); i++) pinMode(in_pins[i], INPUT_PULLUP);
  for(i=0; i<sizeof(out_pins); i++) pinMode(out_pins[i], OUTPUT);
  for(i=0; i<sizeof(ana_pins); i++) analogRead(ana_pins[i]);
  digitalWriteFast(out.led_trigger, LOW); //Ensure LED output is disabled
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ana.isense+1, OUTPUT);
  //configrue adc
  //analogReference(EXTERNAL); //External ADC is causing noise issues

  //Connect to powersupply
  if(ps.connect()) status.state = state.standby;
  else status.state = state.com_failure;
  
  //Verify that led is not powered
  checkPowerStatus();
  
  //Make sure boot was without error
  checkStatus();
  setColor(green); //update button color
  playStatusTone();
}

void calibrate(){
  float set_voltage = pulse.current * series_resistance; //Calculate the starting test current
  uint8_t i;
  uint32_t timer[3];
  bool ps_stable;
  float prev_voltage;
  float led_voltage;


  auto checkDriver = [&] (){ //Wait for the trigger event to reset - used to initially sync the LED driver to the trigger input
    status.ps_voltage = ps.getVoltage();
    if(status.ps_voltage < 0) status.state = state.com_failure;
    delay(50);
    if(status.ps_voltage > max_safe_voltage){ //If DC voltage above safe level, check that bulb is discharching the capacitor
      status.ps_current = ps.getCurrent(); 
      if(status.ps_current == 0) status.state = state.bulb_out;
      else if (status.ps_current < 0) status.state = state.com_failure;
    }
    delay(50);
    checkStatus();
  };

  status.state = state.calibrating_led; //Update state
  setColor(red);
  if(Serial) Serial.println("Starting LED current calibration...");
  delay(100);
  ps.toggleOutput(true); //Turn on output
  delay(200);
  pinMode(in.rx, INPUT_PULLUP);
  if(!ps.setVoltage(set_voltage));// status.state = state.com_failure; //Set voltage to 1 before turning on output
  if(!ps.setCurrent(pulse.current));// status.state = state.com_failure;//Set driver current limit to pulse current
  status.driver_current = 0; //Reset driver current
  
  while (status.driver_current < pulse.current){
    if(set_voltage > max_ps_voltage){
      status.state = state.ps_voltage_limit;
      break;
    }
    timer[0] = driver_timer;
    ps_stable = false;
    while(driver_timer - timer[0] < 10000 && !ps_stable){ //Increment PS voltage and wait for it to stabilize
      //Increment power supply to new voltage and perform safety checks
      digitalWriteFast(out.led_trigger, LOW);
      ps.setVoltage(set_voltage);
      delay(100);
      timer[1] = driver_timer;
      while(driver_timer - timer[1] < 5000 && !ps_stable){
        checkDriver();
        timer[2] = driver_timer;
        while(status.ps_voltage == set_voltage && driver_timer - timer[2] < 500 && !ps_stable){
          checkDriver();
        }
        if(driver_timer - timer[2] >= 500) ps_stable = true;
      }
    }
    if(ps_stable){
      prev_voltage = 0;
      while(prev_voltage < status.driver_voltage*0.999 || prev_voltage > status.driver_voltage*1.001){ //Wait for driver voltage to stabilize
        prev_voltage = status.driver_voltage;
        checkVoltage();
        checkStatus();
        delay(200);
      }

      //Briefly pulse LED to measure its current
      digitalWriteFast(out.led_trigger, HIGH); //Turn LED on to check current
      delayMicroseconds(50); //Wait for current to stabilize
      checkCurrent(); //Measure LED current
      digitalWriteFast(out.led_trigger, LOW); //Turn off LED

      //Calculate the voltage drop across the LEDs
      led_voltage = status.ps_voltage - (status.driver_current * series_resistance);

      if(Serial){
        Serial.print("Calibrating... Volt: ");
        Serial.print(status.ps_voltage);
        Serial.print(", Current: ");
        Serial.print(status.driver_current);
        Serial.print(", LED Vf: ");
        Serial.println(led_voltage);

      }

      //Increment the set voltage by 1
      set_voltage = floor((pulse.current * series_resistance) + led_voltage);
      if(set_voltage <= status.ps_voltage) set_voltage = round(status.ps_voltage + 1);
    }
    else{
      status.state = state.ps_unstable;
      checkStatus();
    } 
  }
  ps.toggleOutput(false); //Disable the power supply
  delay(500);
  status.ps_current = ps.getCurrent();
  if(status.ps_current > 0) status.state = state.ps_over_current;
  checkStatus();
  if(Serial){
    if(status.driver_current < pulse.current) Serial.println("Calibration stopped - power supply maximum voltage reached.");
    else Serial.println("Success, LED current calibration complete!");
  }
  dischargeCapacitor();
  status.state = state.waiting_trapdoor;
}

void dischargeCapacitor(){
  float prev_voltage;
  uint32_t timer;
  uint8_t prev_status_state = status.state;

  status.state = state.capacitor_discharging;
  Serial.println("Waiting for capacitor to discharge to a safe voltage...");
  delay(500);
  checkVoltage();
  prev_voltage = status.driver_voltage;
  timer = driver_timer;
  while(status.driver_voltage > max_safe_voltage){ //Wait for cap to discharge
    if(driver_timer - timer > 500){
      if(status.driver_voltage >= prev_voltage) status.state = state.bulb_out;
      prev_voltage = status.driver_voltage;
      timer = driver_timer;
    }
    checkStatus();
    if(driver_timer & 256) {setColor(off);}
    else setColor(red);
  }
  Serial.println("Capacitor is safe, device is ready.");
  setColor(green);
  status.state = prev_status_state;
}

void checkStatus(){
  status_index++;
  checkError(); //Always check error codes
  switch (status_index) {
  case 1: //Check door
    if(!digitalReadFast(in.door)){
      if(status.state){
        status.state = state.door_open;
      }
    }
    break;
  case 2: //Check driver current
    checkCurrent();
    if(status.driver_current > 0 && status.state != state.led_on) status.state = state.driver_over_current;
    break;
  case 3: //Check driver voltage
    checkVoltage();
    if(status.driver_voltage > max_safe_voltage && (status.state >= 100 || !status.state)){
      status.state = state.driver_over_voltage;
    } 
    else if(status.driver_voltage == 0 && status.ps_voltage > 0) status.state = state.driver_under_voltage;
    break;
  case 4: //Check PS voltage
    status.ps_voltage = ps.getVoltage();
    if(status.ps_voltage > 0 && (status.state >= 100 || !status.state)){status.state = state.ps_over_voltage;} 
    else if(status.ps_voltage < 0) status.state = state.com_failure;
    break;
  case 5: //Check PS current
    status.ps_current = ps.getCurrent();
    if(status.ps_voltage > max_safe_voltage && status.ps_current == 0){
       status.ps_voltage = ps.getVoltage(); //Double check voltage in case output was just disabled
       if(status.ps_voltage > max_safe_voltage) status.state = state.bulb_out;
    }
    else if(status.ps_voltage < 0) status.state = state.com_failure;
    break;
  case 6: //Check pushbutton
    if(!digitalReadFast(in.pushbutton)){
      playStatusTone();
      delay(debounce);
      if(!(status.state == state.standby || status.state == state.waiting_trapdoor)) status.state = state.reboot; //Initialize if press isn't to cycle to next driver step
      while(!digitalReadFast(in.pushbutton)); // Wait for button release
      delay(debounce);
    }
    break;
  case 7:
    if(status.state >= state.led_cooldown){
      if(status.ps_current == 0 && status.ps_voltage == 0 && status.driver_current == 0 && status.driver_voltage == 0){ //If LED driver is fully safed and is cooling down or in error, restart driver
        initialize();
      }
    }
    break;
  default:
    status_index = 0;
    break;
  }
  checkError(); //Always check error codes
}

void checkError(){
  if(status.state < 100) return; //If there are no errors, then return
  else{
    setColor(red);
    failSafe();
    playAlarmTone();
    switch (status.state) {
      case state.door_open: //No active errors
        if(Serial) Serial.println("Error: Chamber door opened while driver was armed.");
        break;
      case state.com_failure: 
        if(Serial) Serial.println("Error: No communication with power supply.");
        break;
      case state.bulb_out: 
        if(Serial) Serial.println("Error: No current through light bulb.");
        break;
      case state.driver_under_current: 
        if(Serial) Serial.println("Error: Insufficient LED current.  Check LED connection and fuses.");
        break;
      case state.driver_under_voltage: 
        if(Serial) Serial.println("Error: No voltage on LED driver.  Check power supply connection and fuses.");
        break;
      case state.driver_over_current: 
        if(Serial) Serial.println("Error: Excessive LED current.");
        break;
      case state.driver_over_voltage: 
        if(Serial) Serial.println("Error: Driver over-voltage.");
        break;
      case state.ps_under_current: 
        if(Serial) Serial.println("Error: Insufficient power supply current.  Check LED connection and fuses.");
        break;
      case state.ps_under_voltage: 
        if(Serial) Serial.println("Error: Power supply under voltage.");
        break;
      case state.ps_over_current: 
        if(Serial) Serial.println("Error: Power supply over current.");
        break;
      case state.ps_over_voltage: 
        if(Serial) Serial.println("Error: Power supply over voltage.");
        break;
      case state.no_mic: //No active errors
        if(Serial) Serial.println("Error: Mic not connected.");
        break;
      case state.ps_voltage_limit:
        if(Serial) Serial.println("Error: Power supply voltage limit reached.");
        break;
      case state.ps_unstable:
        if(Serial) Serial.println("Error: Power supply voltage failed to stabilize.");
        break;
      case state.reboot:
        if(Serial) Serial.println("Rebooting LED driver, please wait...");
      default:
        // statements
        break;
    }
    delay(100);
    pinMode(7, OUTPUT); //Reboot driver
    digitalWrite(7, LOW);
  }
}

void failSafe(){
  uint32_t cur_time = driver_timer;
  pinMode(out.led_trigger, OUTPUT);
  digitalWriteFast(out.led_trigger, LOW);
  ps.connect();
}

//0=off, 10 = solid red, 11 = flashing red, 20 = solid green, 21 = flashing green, 40 = solid yellow, 41 = flashing yellow,
void setColor(uint8_t led, bool flashing){
  //Set timers if needed
  if(!flashing){ //If color flashes and not called from the flash handler
    // if(led & 1) ITimer3.resumeTimer(); //Start flashing
    // else ITimer3.pauseTimer(); //Stop flashing if not called from flashing handler
    // if(led & 32) ITimer1.resumeTimer(); //Start yellow
    // else ITimer1.pauseTimer(); //Stop yellow
    status.button_color = led;
  }
 
  if(!led){
    status.button_led[0] = 0;
    status.button_led[1] = 0;
  }
  else if(led < 16){
    status.button_led[0] = 1;
    status.button_led[1] = 0;
  }
  else{
    status.button_led[0] = 0;
    status.button_led[1] = 1;
  }
  digitalWriteFast(out.led[0], status.button_led[0]);
  digitalWriteFast(out.led[1], status.button_led[1]);
}

void checkPowerStatus(){
  if(ps.getCurrent() > status.ps_current) status.state = state.ps_over_current;
  if(ps.getVoltage() > status.ps_voltage) status.state = state.ps_over_voltage;
  dischargeCapacitor(); //Ensure capacitor is at a safe voltage
}

void checkCurrent(){
  status.driver_current = analogRead(ana.isense);
  status.driver_current *= vref / 1023.0;
}

float checkVoltage(){
  status.driver_voltage = analogRead(ana.vsense);
  status.driver_voltage *= vref / (1023.0 * vfactor);
}

void playStatusTone(){
  uint8_t i = 255;
  while(i--){
    digitalWriteFast(out.speaker[0], LOW);
    digitalWriteFast(out.speaker[1], HIGH);
    delayMicroseconds(127);
    digitalWriteFast(out.speaker[0], HIGH);
    digitalWriteFast(out.speaker[1], LOW);
    delayMicroseconds(127);
  }
  digitalWriteFast(out.speaker[0], LOW);
  digitalWriteFast(out.speaker[1], LOW);
}

void playAlarmTone(){
  uint16_t i = 16000;
  while(i--){
    digitalWriteFast(out.speaker[0], HIGH);
    digitalWriteFast(out.speaker[1], LOW);
    delayMicroseconds(127);
    digitalWriteFast(out.speaker[0], LOW);
    digitalWriteFast(out.speaker[1], HIGH);
    delayMicroseconds(128);
  }
  digitalWriteFast(out.speaker[0], LOW);
  digitalWriteFast(out.speaker[1], LOW);
}
