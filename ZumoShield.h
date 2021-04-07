#pragma once

#include <LSM303.h>
#include <L3G.h>
#include <Pushbutton.h>
#include <QTRSensors.h>
#include <ZumoBuzzer.h>
#include <ZumoIMU.h>
#include <ZumoMotors.h>
#include <ZumoReflectanceSensorArray.h>

extern ZumoMotors  motors;

class ZumoLED
{
  public:
	ZumoLED(){pinMode(13, OUTPUT);};
	void on(){digitalWrite(13, HIGH);};
	void off(){digitalWrite(13, LOW);};
	void set(int i){
		digitalWrite(13, (i == 1)? HIGH : LOW);
	};
};

class ZumoBuzzerN : public ZumoBuzzer
{
  public:
	ZumoBuzzer2(){};
	void playOn(void) {play(">g32>>c32");};
	void playStart(void) {playNote(NOTE_G(4), 500, 15);}
	void playNum(int cnt) {
		for (int i = 0; i < cnt; i++){
			delay(1000);
			playNote(NOTE_G(3), 50, 12);
		}  
	};
};

class ZumoReflectanceSensorArrayN : public ZumoReflectanceSensorArray {
  public:
	unsigned int values[6];
	ZumoReflectanceSensorArrayN() : ZumoReflectanceSensorArray(QTR_NO_EMITTER_PIN){
		
	};
	void update(void){
		read(values);
	}
	unsigned int value(int i){
		if((i <= 6) && (i > 0)) {
			return values[i-1];
		}
		return 0;
	}
};

// Converts x and y components of a vector to a heading in degrees.
// This calculation assumes that the Zumo is always level.
template <typename T> float heading(ZumoIMU::vector<T> v, ZumoIMU::vector<int16_t> m_max, ZumoIMU::vector<int16_t> m_min)
{
  float x_scaled =  2.0*(float)(v.x - m_min.x) / (m_max.x - m_min.x) - 1.0;
  float y_scaled =  2.0*(float)(v.y - m_min.y) / (m_max.y - m_min.y) - 1.0;

  float angle = atan2(y_scaled, x_scaled)*180 / M_PI;
  if (angle < 0)
    angle += 360;
  return angle;
}

class ZumoIMUN : public ZumoIMU
{
  public:
	ZumoIMU::vector<int16_t> m_max; // maximum magnetometer values, used for calibration
	ZumoIMU::vector<int16_t> m_min; // minimum magnetometer values, used for calibration

	ZumoIMUN() {};
	void begin(void) {
		// Initialize the Wire library and join the I2C bus as a master
		Wire.begin();

		// Initialize IMU
		init();

		// Enables accelerometer and magnetometer
		enableDefault();

		configureForCompassHeading();
	};

	void doCalibration(void) {
		// The highest possible magnetic value to read in any direction is 32767
		// The lowest possible magnetic value to read in any direction is -32767
		ZumoIMU::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32767, -32767, -32767};
		unsigned char index;

		// To calibrate the magnetometer, the Zumo spins to find the max/min
		// magnetic vectors. This information is used to correct for offsets
		// in the magnetometer data.
		motors.setLeftSpeed(200);
		motors.setRightSpeed(-200);

		for(index = 0; index < 70; index ++)
		  {
			  // Take a reading of the magnetic vector and store it in compass.m
			  readMag();

			  running_min.x = min(running_min.x, m.x);
			  running_min.y = min(running_min.y, m.y);

			  running_max.x = max(running_max.x, m.x);
			  running_max.y = max(running_max.y, m.y);

			  Serial.println(index);
			  
			  delay(50);
		  }

		motors.setLeftSpeed(0);
		motors.setRightSpeed(0);

		Serial.print("max.x   ");
		Serial.print(running_max.x);
		Serial.println();
		Serial.print("max.y   ");
		Serial.print(running_max.y);
		Serial.println();
		Serial.print("min.x   ");
		Serial.print(running_min.x);
		Serial.println();
		Serial.print("min.y   ");
		Serial.print(running_min.y);
		Serial.println();

		// Store calibrated values in m_max and m_min
		m_max.x = running_max.x;
		m_max.y = running_max.y;
		m_min.x = running_min.x;
		m_min.y = running_min.y;
	};

	void setCalibration(int m_max_x, int m_max_y, int m_min_x, int m_min_y) {
		m_max.x = m_max_x;
		m_max.y = m_max_y;
		m_min.x = m_min_x;
		m_min.y = m_min_y;
	};

	// Average 10 vectors to get a better measurement and help smooth out
	// the motors' magnetic interference.
	float averageHeading() {
		ZumoIMU::vector<int32_t> avg = {0, 0, 0};

		for(int i = 0; i < 10; i ++) {
			readMag();
			avg.x += m.x;
			avg.y += m.y;
		}
		avg.x /= 10.0;
		avg.y /= 10.0;

		// avg is the average measure of the magnetic vector.
		return heading(avg, m_max, m_min);
	};
};



ZumoLED     led;
Pushbutton  button(ZUMO_BUTTON);
ZumoBuzzerN buzzer;
ZumoMotors  motors;
ZumoReflectanceSensorArrayN reflectances;
ZumoIMUN imu;
