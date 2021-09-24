#include <Arduino.h>
#include <LiquidCrystal.h>
#include <SPI.h>
#include "LTC2630.h"
#include "ADCCONVERT.h"

LTC2630 dac;
ADCCONVERT adc;

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

uint8_t DAC_input = 100;
int CS2 = 10;
int V_load;
int V_OC;
int button_pin;

void setup() {
  
  Serial.begin(9600);
  Serial.println("Setup Begin");

  lcd.begin(16, 2);
  lcd.print("Waiting");

  V_OC = adc.adcConvert(0x00, CS2);
  
  dac.init();

  Serial.println("Setup End");

}

void loop() {
  
  if(digitalRead(button_pin) == 0){
        int V_batt[500];                          //1000 samples
        dac.backlight_set(DAC_input);             //Sets DAC output voltage... V_control
        V_load = adc.adcConvert(0x01, CS2);       //Monitor Load Voltage... V_load

        lcd.setCursor(0,0);
        lcd.write("I = ");
        lcd.print(V_load/150);
        
        if(V_load - (DAC_input/4096)*5 <= 0.05  || V_load - (DAC_input/4096)*5 >= 0.05 ){    // if (V_load == V_Control)
              for(int index = 0; index < 500; index++){
                V_batt[index] = adc.adcConvert(0x00, CS2);
                Serial.println(V_batt[index]);
                delay(1);
              }
              Serial.print("\n");
        }

        //Vout = 0 (maybe not needed)
        dac.backlight_set(0);
        //DAC Power Off
        dac.cmd_send(DAC_POWER_DOWN, 0);

        int min_voltage = V_batt[0];
        for(int index = 1; index < 500; index++){
          if(V_batt[index] < min_voltage){
            min_voltage = V_batt[index];
          }
        }
        Serial.print(min_voltage);
        
        int res = (V_OC - min_voltage)/(V_load/150);

        lcd.setCursor(0,1);
        lcd.write("Int Res = ");
        lcd.print(res);
  
  }  

}

