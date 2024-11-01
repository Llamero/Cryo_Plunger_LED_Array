#include <digitalWriteFast.h>
#include <elapsedMillis.h>
#include "powerSupply.h"
#define USE_TIMER_1     true
#define USE_TIMER_3     true
#warning Using Timer1, Timer3
#include "TimerInterrupt_Generic.h"

const struct pulseStruct{
  uint8_t pulse_duration = 100; //Total duration of LED pulse: 0-255 ms
  float pulse_current = 3; //Peak output current in amps
  uint16_t  pulse_delay = 500; //Delay from trigger event to LED on: 0-65535 ms
  uint16_t cooldown_period = 5000; //Cooldown delay after LED turns off: 0-65535 ms 
} pulse;

#define LED_PWM 1
#define LED_FLASH_INTERVAL 500
#define off 0
#define red 10
#define flashing_red 11
#define green 20
#define flashing_green 21
#define yellow 40
#define flashing_yellow 41

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
  uint8_t led_on = 4;
  uint8_t led_cooldown = 5;
  
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
} state;

const struct defaultStatusStruct{
  float driver_volt = 0;
  float driver_current = 0;
  float ps_volt = 0;
  float ps_current = 0;
  uint16_t audio = 0;
  bool door_interlock = false;
  uint8_t state = 0; //0 = good, 1 = failsafe, 2 = minor error, 3 = critical error
} default_status;

struct statusStruct{
  float driver_voltage = 0;
  float driver_current = 0;
  float ps_volt = 0;
  float ps_current = 0;
  uint16_t audio = 0;
  bool door_interlock = false;
  bool button_led[2] = {0, 0};
  uint8_t button_color = 0; //0=off, 10 = solid red, 11 = flashing red, 20 = solid yellow, 21 = flashing yellow, 31 = solid green, 32 = flashing green
  uint8_t state = 0;
} status;

const int8_t tone_volume = 128; 
const uint16_t flash_interval = 500;
const float vref = 4.351; //ADC Vref
const float vfactor = 0.01960784313; //Voltage divider on Vsense
uint8_t status_index;
bool flash; //Tracks whether led is on or off when flashing

powerSupply ps;
elapsedMillis driver_timer;

void yellowLedHandler(){
  static uint8_t i;
  i++;
  if(i<8){ //Yellow balance LED by adding more red
    digitalWriteFast(out.led[0], 1);
    digitalWriteFast(out.led[1], 0);
  }
  else if(i<10){
    digitalWriteFast(out.led[0], 0);
    digitalWriteFast(out.led[1], 1);
  }
  else{
    i=255;
  }

}

void flashLedHandler(){
  flash = !flash;
  if(flash) setColor(status.button_color, true);
  else setColor(0, true);
}

void setColor(uint8_t led, bool flashing = false); //Declare prototype with default arguments

void setup() {
  //Sertup serial
  Serial.begin(250000);
  while(!Serial);

  //Setup device
  initialize();


ps.setVoltage(31);
ps.setCurrent(0.1);
ps.toggleOutput(true);
ps.disconnect();
digitalWriteFast(out.led_trigger, HIGH);
  //Initialize button
  while(true){
    float value;
    setColor(red);
    value = checkCurrent() * 1000;
    Serial.print(value);
    delay(LED_FLASH_INTERVAL);
    setColor(yellow);
    Serial.print(" ");
    delay(LED_FLASH_INTERVAL);
    setColor(green);
    value = checkVoltage();
    Serial.println(value);
    delay(LED_FLASH_INTERVAL);
  }
}

void loop() {
  digitalWriteFast(out.led[0], LOW);
  digitalWriteFast(out.led[1], HIGH);
  while(digitalReadFast(in.pushbutton)) delay(10);
  digitalWriteFast(out.led[0], HIGH);
  digitalWriteFast(out.led[1], LOW);
  digitalWriteFast(out.led_trigger, HIGH);
  delay(100);
  digitalWriteFast(out.led_trigger, LOW);
  delay(3000);
  while(!digitalReadFast(in.pushbutton)) delay(10);
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

  //Initialize timer interrupts
  ITimer1.init();
  ITimer1.attachInterruptInterval(LED_PWM, yellowLedHandler);
  ITimer1.pauseTimer();
  ITimer3.init();
  ITimer3.attachInterruptInterval(LED_FLASH_INTERVAL, flashLedHandler);
  ITimer3.pauseTimer();

  //Connect to powersupply
  if(!ps.connect()) status.state = state.com_failure;
  
  //Verify that led is not powered
  checkPowerStatus();
  
  //Make sure boot was without error
  checkError();
  playStatusTone();
}



void checkStatus(){
  status_index++;
  switch (status_index) {
  case state.com_failure: //Check door interlock
    status.door_interlock = digitalReadFast(in.door);
    if(!status.door_interlock) status.state = state.com_failure;
    break;
  case 1:
    //Com failure talking to power supply
    break;
  }
}

void checkError(){
  if(status.state < 100) return; //If there are no errors, then return
  else{
    switch (status.state) {
      case state.standby: //No active errors
        break;
      case 1:
        //Com failure talking to power supply
        break;
      default:
        // statements
        break;
    }
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
    if(led & 1) ITimer3.resumeTimer(); //Start flashing
    else ITimer3.pauseTimer(); //Stop flashing if not called from flashing handler
    if(led & 32) ITimer1.resumeTimer(); //Start yellow
    else ITimer1.pauseTimer(); //Stop yellow
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
  uint8_t i;
  if(ps.getCurrent() > status.ps_current) status.state = state.ps_over_current;
  if(ps.getVoltage() > status.ps_volt) status.state = state.ps_over_voltage;
  if(checkCurrent() > status.driver_current + 0.01) status.state = state.driver_over_current;
  if(checkVoltage() > status.driver_voltage + 1) status.state = state.driver_over_voltage;
}

float checkCurrent(){
  float current = 0;
  uint16_t adc = 0;
  analogRead(ana.isense);
  for(uint8_t i = 0; i < 64; i++){
    adc += analogRead(ana.isense);
    delayMicroseconds(100);
  }
  current = ((float) adc * vref) / 65535.0;
  return current;
}

float checkVoltage(){
  float voltage = 0;
  uint16_t adc = 0;
  analogRead(ana.vsense);
  for(uint8_t i = 0; i < 64; i++){
    adc += analogRead(ana.vsense);
    delayMicroseconds(100);
  }
  voltage = ((float) adc * vref) / (65535 * vfactor);
  return voltage;
}

void playStatusTone(){
  uint8_t i = 255;
  while(i--){
    digitalWriteFast(out.speaker[0], HIGH);
    digitalWriteFast(out.speaker[1], LOW);
    delayMicroseconds(tone_volume);
    digitalWriteFast(out.speaker[0], LOW);
    digitalWriteFast(out.speaker[1], HIGH);
    delayMicroseconds(255-tone_volume);
  }
  digitalWriteFast(out.speaker[0], LOW);
  digitalWriteFast(out.speaker[1], LOW);
}

void playAlarmTone(){
  uint16_t i = 16000;
  while(flash && i--){
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
