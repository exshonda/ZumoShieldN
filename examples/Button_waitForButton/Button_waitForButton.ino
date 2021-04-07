#include <Wire.h>
#include <ZumoShield.h>

void setup() {
  
}

void loop() {
  button.waitForPress();
  led.on();
  delay(1000);
  led.off();  
  delay(1000);
  led.on();
  delay(1000);
  led.off();    
  button.waitForRelease();
}
