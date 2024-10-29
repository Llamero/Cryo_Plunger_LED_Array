#include <digitalWriteFast.h>
#include "powerSupply.h"

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

const float vref = 4.096; //ADC Vref
const float vfactor = 0.01960784313; //Voltage divider on Vsense
uint8_t i;
uint8_t j;

powerSupply ps;

uint32_t l;
uint32_t ana_sum;
float result;

void setup() {
  //Set pin states
  for(i=0; i<sizeof(in_pins); i++) pinMode(in_pins[i], INPUT_PULLUP);
  for(i=0; i<sizeof(out_pins); i++) pinMode(out_pins[i], OUTPUT);
  for(i=0; i<sizeof(ana_pins); i++) analogRead(ana_pins[i]);
  pinMode(LED_BUILTIN, OUTPUT);

  //configrue adc
  analogReference(EXTERNAL);

  //Connect to powersupply
  if(!ps.connect());
}

void loop() {
  i++;
  //ps.setVoltage(i);
  //ps.setCurrent((float)i/100);
  uint16_t n_samples = 5e3;
  digitalWriteFast(out.led[0], LOW);
  digitalWriteFast(out.led[1], HIGH);
  ana_sum = 0;
  for(l=0; l<n_samples; l++) ana_sum += analogRead(ana.vsense);
  delay(1000);
  ps.toggleOutput(i%2);
  ana_sum /= n_samples;
  result = (float) ana_sum * vref / (vfactor * 1024);
  digitalWriteFast(out.led[0], HIGH);
  digitalWriteFast(out.led[1], LOW);
  ana_sum = 0;
  for(l=0; l<n_samples; l++) ana_sum += analogRead(ana.isense);
  result = (float) ana_sum / (float) n_samples;
  result = result * vref / 1024;
}

