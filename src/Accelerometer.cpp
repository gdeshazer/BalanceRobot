/*
 * IMUControl.cpp
 *
 *  Created on: Jan 8, 2017
 *      Author: grantdeshazer
 */

#include "Accelerometer.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <I2CDev.h>
#include <MPU6050_6Axis_MotionApps20.h>

namespace std {

Accelerometer::Accelerometer() {

	Q_angle  = 0.001; //0.001
	Q_gyro   = 0.003; //0.003
	R_angle  = 0.03; //0.03

	x_angle = 0;  // = 0;
	x_bias  = 0;  // = 0;
	P_00    = 0;  // = 0
	P_01    = 0;  // = 0
	P_10    = 0;  // = 0
	P_11    = 0;  // = 0
	dt      = 0;
	y       = 0;
	S       = 0;
	K_0     = 0;
	K_1     = 0;


	_ax = 0;
	_ay = 0;
	_az = 0;
	_gx = 0;
	_gy = 0;
	_gz = 0;

	_mean[0] = 0;
	_mean[1] = 0;
	_mean[2] = 0;
	_mean[3] = 0;
	_mean[4] = 0;
	_mean[5] = 0;

	_angle = 0;

	Wire.begin();
	TWBR = 24;
	_imu.initialize();

	//default offset values
	_imu.setXAccelOffset(1501);
	_imu.setZAccelOffset(1262);
	_imu.setXGyroOffset(-4);
	_imu.setYGyroOffset(4);
	_imu.setZGyroOffset(4);

}


Accelerometer::~Accelerometer() {
	delete this;
}


float Accelerometer::findAdjustedTilt(int time){

	_imu.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);

	_gx = _gx * 250.0/32768.0;
	_angle = this->arctan2(-_az,-_ay) - 36;


	//	// ----- DEADBAND ----- //
	//
	//	if(_angle <= 5 && _angle > 0){
	//		_angle = 20;
	//	} else if (_angle >= -5 && _angle < 0){
	//		_angle = -20;
	//	}
	//
	//	// -------------------- //


	return kalman(_angle, _gx, time);
}

void Accelerometer::calibrate(){
	int offset[6] = {0};
	this ->mean(500);

	offset[0] = -_mean[0]/8;
	offset[1] = -_mean[1]/8;
	offset[2] = (16384-_mean[2])/4;

	offset[3] = -_mean[3]/4;
	offset[4] = -_mean[4]/4;
	offset[5] = -_mean[5]/4;

	_imu.setXAccelOffset(offset[0]);
	_imu.setYAccelOffset(offset[1]);
	_imu.setZAccelOffset(offset[2]);
	_imu.setXGyroOffset(offset[3]);
	_imu.setYGyroOffset(offset[4]);
	_imu.setZGyroOffset(offset[5]);

}


bool Accelerometer::tooFar(){
	if(_angle >= 150 || _angle <= -150 ){
		return true;
	} else {
		return false;
	}

}


float Accelerometer::kalman(float newAngle, float newRate, int time){

	dt = float(time)/1000;
	x_angle += dt * (newRate - x_bias);
	P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
	P_01 +=  - dt * P_11;
	P_10 +=  - dt * P_11;
	P_11 +=  + Q_gyro * dt;

	y = newAngle - x_angle;
	S = P_00 + R_angle;
	K_0 = P_00 / S;
	K_1 = P_10 / S;

	x_angle +=  K_0 * y;
	x_bias  +=  K_1 * y;
	P_00 -= K_0 * P_00;
	P_01 -= K_0 * P_01;
	P_10 -= K_1 * P_00;
	P_11 -= K_1 * P_01;

	return x_angle;
}

void Accelerometer::mean(int size){
	int toss = 0.1 * size;
	int i = 0;

	long buff[6] = {0};

	while(i < (size + (toss + 1))){
		_imu.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);

		if(i < toss && i<=size + toss){
			buff[0] = _ax;
			buff[1] = _ay;
			buff[2] = _az;
			buff[3] = _gx;
			buff[4] = _gy;
			buff[5] = _gz;
		}

		if (i==(size+toss)){
			for(int x = 0; x < 6; x++){
				_mean[x] = buff[x]/size;
			}
		}
	}
}

int Accelerometer::arctan2(int y, int x) {          // http://www.dspguru.com/comp.dsp/tricks/alg/fxdatan2.htm
	int coeff_1 = 128;
	int coeff_2 = 3*coeff_1;

	float abs_y = abs(y)+1e-10;
	float r, angle;

	if (x >= 0) {
		r = (x - abs_y) / (x + abs_y);
		angle = coeff_1 - coeff_1 * r;
	}  else {
		r = (x + abs_y) / (abs_y - x);
		angle = coeff_2 - coeff_1 * r;
	}

	if (y < 0){
		return int(-angle);
	} else {
		return int(angle);
	}
}

} /* namespace std */
