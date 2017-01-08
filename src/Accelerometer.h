/*
 * IMUControl.h
 *
 *  Created on: Jan 8, 2017
 *      Author: grantdeshazer
 */

#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <I2CDev.h>
#include <MPU6050_6Axis_MotionApps20.h>

namespace std {

class Accelerometer {
public:
	Accelerometer();
	virtual ~Accelerometer();

	float findAdjustedTilt(int);
	void calibrate();
	bool tooFar();

private:

	float kalman(float, float, int);
	int arctan2(int, int);
	void mean(int);

	// -- Kalman Filter variables --/
	float Q_angle; //0.001
	float Q_gyro ; //0.003
	float R_angle; //0.03

	float x_angle; // = 0;
	float x_bias;  // = 0;
	float P_00;    // = 0
	float P_01;    // = 0
	float P_10;    // = 0
	float P_11;    // = 0
	float dt, y, S;
	float K_0, K_1;


	MPU6050 _imu;
	int16_t _ax, _ay, _az, _gx, _gy, _gz;
	float _mean[6];
	int _angle;

};

} /* namespace std */

#endif /* ACCELEROMETER_H_ */
