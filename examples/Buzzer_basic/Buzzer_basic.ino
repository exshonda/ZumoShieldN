#include <Wire.h>
#include <ZumoShield.h>

void setup() {
   buzzer.playOn();
}

void loop() {
  button.waitForPress();
  buzzer.playNum(2);
  button.waitForRelease();
}
