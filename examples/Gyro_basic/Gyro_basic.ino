#include <Wire.h>
#include <ZumoShield.h>

void setup()
{  
  buzzer.playOn();  
  
  gyro.turnSensorSetup();
  delay(500);
  gyro.turnSensorReset();
  
  buzzer.playStart();
  Serial.begin(9600);
}

void loop()
{  
  gyro.turnSensorUpdate();
  Serial.println(gyro.turnAngleDegree);
}

