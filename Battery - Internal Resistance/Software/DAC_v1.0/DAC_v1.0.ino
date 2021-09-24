#include <Arduino.h>
#include <SPI.h>
#include "LTC2630.h"

LTC2630 dac;

uint8_t DAC_input = 100;  // choose DAC Voltage Output

void setup() {
  
  Serial.begin(9600);

  //dac power down and then set Vref = Vcc = 5V
  dac.init();

}

void loop() {
  
  //Write and Update Command and then set Vout = DAC_voltage
  dac.backlight_set(DAC_input);  


  delay(3000);


  //Vout = 0 (maybe not needed)
  dac.backlight_set(0);
  //DAC Power Off
  dac.cmd_send(DAC_POWER_DOWN, 0);
  
}

