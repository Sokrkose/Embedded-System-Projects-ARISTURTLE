#include <Arduino.h>
#include <SPI.h>
#include <LiquidCrystal.h>
#include "LTC2630.h"

LTC2630 dac;

const int rs = 35, en = 33, d4 = 32, d5 = 31, d6 = 30, d7 = 15;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

uint8_t V_control = 61;  // choose DAC Voltage Output
int V_ctrl = (int ) V_control;
float a = (float ) V_ctrl/255;
float V = a*4.97;
float Id = V/0.150;

void setup() {
  Serial.begin(9600);
  pinMode(53, OUTPUT);
  
  //dac power down and then set Vref = Vcc = 5V
  dac.init();

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
}

void loop() {

  //Write and Update Command and then set Vout = DAC_voltage
  dac.backlight_set(V_control);
  lcd.setCursor(0, 0);
  lcd.print("V=");
  lcd.setCursor(2, 0);
  lcd.print(V);
  lcd.setCursor(7, 0);
  lcd.print("I=");
  lcd.setCursor(9,0);
  lcd.print(Id);

  
  delay(3000);

  //Vout = 0 (maybe not needed)
  dac.backlight_set(BACKLIGHT_FULL_OFF);
  //DAC Power Off
  dac.cmd_send(DAC_POWER_DOWN, 0);
 
  delay(1000);
}
