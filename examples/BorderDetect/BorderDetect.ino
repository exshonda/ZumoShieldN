#include <Wire.h>
#include <ZumoShield.h>

// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  600

// these might need to be tuned for different motor types
#define REVERSE_SPEED     200 // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define FORWARD_SPEED     200
#define REVERSE_DURATION  200 // ms
#define TURN_DURATION     300 // ms

void setup()
{
  buzzer.playOn();

  Serial.begin(9600);
  Serial.println("Zumo sample Start!");

  led.on();
  button.waitForButton();
  led.off();
  buzzer.playNum(3);
  delay(1000);
  buzzer.playStart();
  delay(1000);
}

void loop()
{
  if (button.isPressed()) {
    motors.setSpeeds(0, 0);
    button.waitForRelease();

    led.on();
    button.waitForButton();
    led.off();
    buzzer.playNum(3);
    delay(1000);
    buzzer.playStart();
    delay(1000);
  }

  reflectances.update();

  if (reflectances.value(1) > QTR_THRESHOLD) {
    // if leftmost sensor detects line, reverse and turn to the right
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  } else if (reflectances.value(6)  > QTR_THRESHOLD) {
    // if rightmost sensor detects line, reverse and turn to the left
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  } else {
    // otherwise, go straight
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
}
