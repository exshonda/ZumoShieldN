#include <Wire.h>
#include <ZumoShield.h>

void setup() {
  
}

void loop() {
  if(button.isPressed()) {
    led.on();
  }else{
    led.off();
  }
}
