/* This example reads the raw values from the accelerometer,
magnetometer, and gyro on the Zumo Shield, and prints those raw
values to the serial monitor.

The accelerometer readings can be converted to units of g using
the sensitivity specified in the LSM303 or LSM6DS33 datasheet.
The default full-scale (FS) setting is +/- 2 g, so the conversion
factor is 0.061 mg/LSB (least-significant bit).  A raw reading of
16384 would correspond to 1 g.

The gyro readings can be converted to degrees per second (dps)
using the sensitivity specified in the L3GD20H or LSM6DS33
datasheet.  The default sensitivity is 8.75 mdps/LSB (least-
significant bit).  A raw reading of 10285 would correspond to
90 dps.  (Earlier versions of the Zumo Shield before v1.2 do
not have a gyro; with those boards, the gyro readings are
displayed as 0.)

The magnetometer readings are more difficult to interpret and
will usually require calibration. */

#include <Wire.h>
#include <ZumoShieldN.h>

char report[120];

void setup()
{
  Serial.begin(9600);

  imu.begin();

  imu.enableDefault();
}

void loop()
{
  imu.read();

  snprintf_P(report, sizeof(report),
    PSTR("A: %6d %6d %6d    M: %6d %6d %6d    G: %6d %6d %6d"),
    imu.a.x, imu.a.y, imu.a.z,
    imu.m.x, imu.m.y, imu.m.z,
    imu.g.x, imu.g.y, imu.g.z);
  Serial.println(report);

  delay(100);
}
