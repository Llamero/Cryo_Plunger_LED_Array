#include <digitalWriteFast.h>
#include <elapsedMillis.h>
#include "powerSupply.h"
#include "photogate.h"
//#define USE_TIMER_1     true
//#define USE_TIMER_3     true
//#warning Using Timer1, Timer3
//#include "TimerInterrupt_Generic.h"

constexpr struct pulseStruct{
  uint32_t duration = 1000; //Total duration of LED pulse in µs
  float current = 0.1; //Peak output current in amps
  uint16_t plunger_delay = 6000; //How long in ms before the plunger starts to fall
  uint16_t  led_delay = 500; //Delay from trigger event to LED on: 0-65535 µs
  uint16_t cooldown_period = 5000; //Cooldown delay after LED turns off: 0-65535 ms
  float voltage = 41; //Power supply voltage for the pulse.  -1 = autocalibrate the voltage
  bool timing_test = false; //Whether to strobe the light in 1/10 second intervals for delay timing 
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
#define max_led_current 3 //Maximum current for the LED
#define max_ps_current 5 //Maximum output current for the power supply
#define series_resistance 31 //Total series resistance of the LED driver
#define max_safe_voltage 30 //Maximum voltage before performing high voltage checks
#define min_bulb_voltage 105 //Minimum voltage needed to see current across the bulb
#define max_ps_voltage 200 //Maximum voltage the power supply can output
#define max_ps_current 1 //Maximum current the power supply can output
#define debounce 100 //Debounce delay (ms)
#define driver_timeout 600000 //Revert to standby if there hasn't been a button press in x ms

const struct inputPinsStruct{
  uint8_t rx = 4;
  uint8_t photogate = 22; //Pin 21 was damaged, so bypassed to pin 22
  uint8_t pushbutton = 12;
  uint8_t pedal = 20;
} in;
const uint8_t in_pins[sizeof(in)] = {in.rx, in.photogate, in.pushbutton, in.pedal};

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
} ana;
const uint8_t ana_pins[sizeof(ana)] = {ana.vsense, ana.isense};

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
  uint8_t timeout = 117;
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

const uint8_t TIMING_PULSE_INTERVAL = 100;
const uint8_t TIMING_PULSE_DURATION = 50;
const uint16_t FLASH_INTERVAL = 500;
const float VREF = 4.2; //ADC Vref
const float VFACTOR = 0.01960784313; //Voltage divider on Vsense
const uint32_t MAX_DURATION = 200000; //Maximum duration of LED pulse- prevents LEDs from overheating
uint8_t status_index;
bool flash; //Tracks whether led is on or off when flashing
float pulse_voltage;

powerSupply ps;
photogate pg(in.photogate);
elapsedMillis driver_timer;
elapsedMicros pulse_timer;

void setColor(uint8_t led, bool flashing = false); //Declare prototype with default arguments
void(* resetFunc) (void) = 0;//declare reset function at address 0

void setup() {
  //Sertup serial
  Serial.begin(250000);
  //pinMode(in.photogate, OUTPUT);
  //pinMode(in.photogate, INPUT);
  // while(true){
  //   pinMode(in.photogate, INPUT_PULLUP);
  //   //digitalWriteFast(in.photogate, HIGH);
  //   delay(10);
  //   pinMode(in.photogate, OUTPUT);
  //   digitalWriteFast(in.photogate, LOW);
  //   delay(10);
  // }
}

void loop() {
  //Setup device
  initialize();
  waitForButton();
  calibrate();
  while(true){
    waitForButton();
    ledPulse();
  }
}

void waitForButton(){
  uint32_t timer[3];
  uint32_t start_countdown = 10240;
  bool button_state = true;
  bool connect_sent;

  auto checkButton = [&] (){ //Wait for the trigger event to reset - used to initially sync the LED driver to the trigger input
    //Wait for pushbutton to be pressed and released
    timer[0] = driver_timer;
    timer[1] = timer[0];
    timer[2] = timer[0];
    while(digitalReadFast(in.pushbutton) == button_state && (driver_timer - timer[0] <= driver_timeout || status.state == state.standby)){
      if(!button_state && (driver_timer - timer[2] > 3000)) status.state = state.reboot; //If button is held reboot.
      checkStatus(); 
      if(status.state != state.standby){ //Flash LED if not in stndby
        if(driver_timer - timer[0]> driver_timeout-start_countdown){ //Give a warning beep each second during final timeout countdown
          playStatusTone();
          start_countdown -= 1024;
        }
        if(driver_timer & 512){
          setColor(green);
          if(!connect_sent){
            if(!ps.connect()) status.state = state.com_failure; //send a connection request once a second to disarm PS
            else connect_sent = true;
          }
        }
        else{
          setColor(off);
          connect_sent = false;
        } 
      }
    }
    if(driver_timer - timer[0]> driver_timeout && status.state != state.standby) status.state = state.timeout;
    checkStatus();
    if(button_state) playStatusTone();
    delay(debounce);
    button_state = !button_state;
  };

  checkButton();
  checkButton();
}

void ledPulse(){
  uint32_t pedal_time;
  uint32_t led_time;
  uint32_t timer[3];
  uint32_t measured_plunger_delay;
  float start_current;
  float avg_current;
  uint32_t n_samples;
  bool ps_stable;
  float prev_voltage;
  uint32_t capped_pulse_duration; //Duration of pulse, capped at MAX_DURATION for safety
  uint32_t measured_pulse_delay; //Measured duration in delay between trigger and LED pulse

  auto checkDriver = [&] (){ //Wait for the trigger event to reset - used to initially sync the LED driver to the trigger input
    status.ps_voltage = ps.getVoltage();
    if(status.ps_voltage < 0) status.state = state.com_failure;
    delay(50);
    if(status.ps_voltage > min_bulb_voltage){ //If DC voltage above safe level, check that bulb is discharching the capacitor
      status.ps_current = ps.getCurrent(); 
      if(status.ps_current == 0) status.state = state.bulb_out;
      else if (status.ps_current < 0) status.state = state.com_failure;
    }
    delay(50);
    checkStatus();
  };

  //Start LED pulse sequence
  Serial.println("Arming power supply...");
  status.state = state.waiting_plunger;
  setColor(red);
  ps.toggleOutput(true);
  delay(100);
  while (status.ps_voltage < pulse_voltage){
    timer[0] = driver_timer;
    ps_stable = false;
    while(driver_timer - timer[0] < 10000 && !ps_stable){ //Increment PS voltage and wait for it to stabilize
      //Increment power supply to new voltage and perform safety checks
      digitalWriteFast(out.led_trigger, LOW);
      if(!ps.setVoltage(pulse_voltage)) status.state = state.com_failure; //Set voltage to 1 before turning on output
      if(!ps.setCurrent(pulse.current)) status.state = state.com_failure;//Set driver current limit to pulse current
      ps.toggleOutput(true); //Turn on output
      delay(500);
      timer[1] = driver_timer;
      while(driver_timer - timer[1] < 5000 && !ps_stable){
        checkDriver();
        timer[2] = driver_timer;
        while(status.ps_voltage == pulse_voltage && driver_timer - timer[2] < 500 && !ps_stable){
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
    }
    else{
      status.state = state.ps_unstable;
      checkStatus();
    } 
  }

  //Turn on photogate
  Serial.println("Power supply stable, Initializing photogate...");
  if(pg.startPhotogate()) Serial.println("Photogate ready, sending trigger...");
  else{
    Serial.println("Photogate failed to initialize, resetting driver...");
    failSafe();
    return;
  }

  //Press pedal
  timer[2] = driver_timer;
  pedal_time = timer[2];
  pinMode(in.pedal, OUTPUT);
  digitalWrite(in.pedal, LOW);
  delay(50);
  pinMode(in.pedal, INPUT_PULLUP);  

  //Trigger LED
  if(pulse.timing_test){
    timer[1] = driver_timer;
    while(driver_timer - timer[2] < 10000){
      if(driver_timer - timer[1] > TIMING_PULSE_INTERVAL){
        digitalWriteFast(out.led_trigger, HIGH);
        timer[1] += 100;
        while(driver_timer - timer[1] > TIMING_PULSE_DURATION);
        digitalWriteFast(out.led_trigger, LOW);
      }
    }
  }
  else{
    pinMode(in.photogate, INPUT_PULLUP);
    while(driver_timer - timer[2] < pulse.plunger_delay){ //Wait for plunger to start falling
      checkStatus();
      if(!digitalReadFast(in.photogate)){
        Serial.print("Error: photogate triggered early at: ");
        Serial.print(driver_timer - timer[2]);
        Serial.println("ms.");
      }
    }
    timer[1] = pulse_timer;
    while(digitalReadFast(in.photogate) && pulse_timer - timer[1] < 1000000); //Wait up to one second for the photogate to trip
    timer[0] = pulse_timer;
    status.state = state.led_on;
    measured_plunger_delay = driver_timer - timer[2];
    if(pulse_timer - timer[1] > 1000000){
      Serial.println("Error: Photogate failed to trigger.");
      failSafe();
      return;
    }
    while(pulse_timer - timer[0] < pulse.led_delay); //Wait for pulse timer
    digitalWriteFast(out.led_trigger, HIGH);
    measured_pulse_delay = pulse_timer - timer[0];
    timer[0] += pulse.led_delay; //Increment timer for pulse
    capped_pulse_duration = pulse.duration;
    if(pulse.duration > MAX_DURATION) capped_pulse_duration = MAX_DURATION;
    delayMicroseconds(10); //Wait for LED to get to full current
    checkCurrent();
    start_current = status.driver_current;
    avg_current += start_current;
    n_samples++;
    while(pulse_timer - timer[0] < capped_pulse_duration){
      checkCurrent();
      avg_current += status.driver_current;
      n_samples++;
      if(status.driver_current > max_led_current){
        digitalWriteFast(out.led_trigger, LOW);
        status.state = state.driver_over_current;
        checkStatus();
      }
    }
    digitalWriteFast(out.led_trigger, LOW);
    timer[1] = pulse_timer;
    ps.toggleOutput(false);
    Serial.print("Pulse complete! Start: ");
    Serial.print(start_current);
    Serial.print(" amps, end: ");
    Serial.print(status.driver_current);
    Serial.print(" amps, average: ");
    Serial.print(avg_current / n_samples);
    Serial.print(" amps, duration: ");
    Serial.print(timer[1] - timer[0]);
    Serial.print(" µs, # of samples: ");
    Serial.print(n_samples);
    Serial.print(", LED delay: ");
    Serial.print(measured_pulse_delay);
    Serial.print(" µs, Plunger delay: ");
    Serial.print(measured_plunger_delay);
    Serial.println(" ms.");
    dischargeCapacitor();
    status.state = state.waiting_trapdoor;
  }
}

void initialize(){
  Serial.println("Initializing LED driver...");
  
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
  
  //Check that the pulse current and voltage are valid
  if(pulse.current > max_led_current) status.state = state.driver_over_current;
  if(pulse.current <= 0) status.state = state.driver_under_current;
  if(pulse.voltage > max_ps_voltage) status.state = state.ps_over_voltage;
  if(pulse.voltage == 0) status.state = state.ps_under_voltage;

  //Make sure boot was without error
  checkStatus();
  setColor(green); //update button color
  Serial.println("Driver initalized. Press button to start calibration.");
  playStatusTone();
}

void calibrate(){
  float set_voltage = round(pulse.current * series_resistance); //Calculate the starting test current
  uint8_t i;
  uint32_t timer[3];
  bool ps_stable;
  float prev_voltage;
  float led_voltage;

  if(pulse.voltage > 0){ //If voltage is already set, do not calibrate
    pulse_voltage = pulse.voltage;
    status.state = state.waiting_trapdoor;
    return;
  }

  auto checkDriver = [&] (){ //Wait for the trigger event to reset - used to initially sync the LED driver to the trigger input
    status.ps_voltage = ps.getVoltage();
    if(status.ps_voltage < 0) status.state = state.com_failure;
    delay(50);
    if(status.ps_voltage > min_bulb_voltage){ //If DC voltage above safe level, check that bulb is discharching the capacitor
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
      if(!ps.setVoltage(set_voltage)) status.state = state.com_failure; //Set voltage to 1 before turning on output
      if(!ps.setCurrent(pulse.current)) status.state = state.com_failure;//Set driver current limit to pulse current
      ps.toggleOutput(true); //Turn on output
      delay(200);
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
      while(prev_voltage < status.driver_voltage*0.99 || prev_voltage > status.driver_voltage*1.01){ //Wait for driver voltage to stabilize
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
      pulse_voltage = set_voltage - 1; //Prevent overcurrent by setting the pulse 1 volt lower

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
  if(Serial){
    if(status.driver_current < pulse.current) Serial.println("Calibration stopped - power supply maximum voltage reached.");
    else Serial.println("Success, LED current calibration complete!");
  }
  checkStatus();
  dischargeCapacitor();
  status.state = state.waiting_trapdoor;
}

void dischargeCapacitor(){
  float prev_voltage;
  uint32_t timer;
  uint8_t prev_status_state = status.state;

  status.state = state.capacitor_discharging;
  Serial.println("Waiting for capacitor to discharge to a safe voltage...");
  if(!ps.connect()) status.state = state.com_failure;
  timer = driver_timer;
  while(driver_timer - timer > 500) checkStatus();
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
//    if(!digitalReadFast(in.photogate)){
//      if(!(status.state == state.standby || status.state == state.waiting_trapdoor)) status.state = state.door_open;
//    }
    break;
  case 2: //Check driver current
    checkCurrent();
    if(status.driver_current > 0 && status.state != state.led_on) status.state = state.driver_over_current;
    break;
  case 3: //Check driver voltage
    checkVoltage();
    if(status.driver_voltage > max_safe_voltage && (status.state >= 100 || status.state == state.standby || status.state == state.waiting_trapdoor)){
      status.state = state.driver_over_voltage;
    } 
    else if(status.driver_voltage == 0 && status.ps_voltage > 0) status.state = state.driver_under_voltage;
    break;
  case 4: //Check PS voltage
    status.ps_voltage = ps.getVoltage();
    if(status.ps_voltage > 0 && (status.state >= 100 || status.state == state.standby || status.state == state.waiting_trapdoor)){status.state = state.ps_over_voltage;} 
    else if(status.ps_voltage < 0) status.state = state.com_failure;
    break;
  case 5: //Check PS current
    status.ps_current = ps.getCurrent();
    if(status.ps_current < 0) status.state = state.com_failure;
    else if(status.ps_voltage > min_bulb_voltage && status.ps_current == 0){
       status.ps_voltage = ps.getVoltage(); //Double check voltage in case output was just disabled
       if(status.ps_voltage > min_bulb_voltage) status.state = state.bulb_out;
    }
    else if(status.ps_voltage < 0) status.state = state.com_failure;
    break;
  case 6: //Check pushbutton
    if(!(status.state == state.standby || status.state == state.waiting_trapdoor)){
      if(!digitalReadFast(in.pushbutton)){
        delay(debounce);
        playStatusTone();
        status.state = state.reboot; //Initialize if press isn't to cycle to next driver step
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
      case state.timeout:
        if(Serial) Serial.println("Driver timed out waiting for button press, rebooting...");
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
  status.driver_current *= VREF / 1023.0;
}

float checkVoltage(){
  status.driver_voltage = analogRead(ana.vsense);
  status.driver_voltage *= VREF / (1023.0 * VFACTOR);
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
