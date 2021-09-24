#include "ADCCONVERT.h"

int CS2 = 10;
ADCCONVERT adc;

int V_batt;
int V_load;

void setup() {

}  


void loop() {
  V_batt = adc.adcConvert(1, CS2);
  V_load = adc.adcConvert(2, CS2);
}
