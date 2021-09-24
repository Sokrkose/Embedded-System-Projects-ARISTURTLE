#include <Arduino.h>
#include <SPI.h>
#include "ADCCONVERT.h"

ADCCONVERT adc;

uint16_t V_batt;
uint16_t V_load;
int CS2 = 44;
float Da_Real_V_batt = 0.0;

void setup() {
  Serial.begin(9600);
  SPI.begin();
  
  pinMode(44, OUTPUT);
}  


void loop() { 


  V_batt = adc.adcConvert(1, CS2);
  Serial.print("Battery Voltage: ");
  Da_Real_V_batt = ((float) V_batt / 4096) * 5;
  Serial.print(Da_Real_V_batt,3);
  Serial.print("     ");
  Serial.println(Da_Real_V_batt-4.204,3);
  delay(2);
  

  delay(1);
}
