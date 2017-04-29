/*
 * Balance_Bot.cpp
 *
 *  Created on: Jan 7, 2017
 *      Author: grantdeshazer
 */

#include "Balance_Bot.h"

#include <Arduino.h>
#include <math.h>




#include "MotorController.h"
#include "PidController.h"


using namespace std;

#define LED 13

MotorController m;
PidController pid;

MPU6050 mpu;


/* Accelerometer Class
 *
 * Objects of this class are responsible for retrieving, filtering,
 * and otherwise dealing with the accelerometer and gyroscope.  This
 * class assumes that the MPU class which handles the basic communications
 * with the IMU is initiated correctly and communication is working with
 * the MPU 6050.
 *
 */
class Accelerometer{
public:

	//Returns filtered and combined values from IMU
	//Returned value is the combination of an angle calculated
	//by raw accelerometer data, and the gyroscope.  Filtering is
	//handled by a Kalman filter and a basic low pass filter.
	//Time constant passed in for loop duration.  Units are in Seconds.
	float findAdjustedTilt(float time){

		mpu.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);

		_gx = _gx * 250.0/32768.0;
		_angle = (this->arctan2(-_az,-_ay)) - 36; //The minus 36 is a calibrated offset found through testing

		Serial.print("  / "); Serial.print(" "); Serial.print(_angle); Serial.print(" /   ");


		float lpfBeta = 0.05;

		float raw = kalman2Value(_angle, _gx, time);

		Serial.print(raw); Serial.print(", ");

		_smoothed -= lpfBeta * (_smoothed - raw);

		return _smoothed;
	}

	//Calibration function used for basic calibration of IMU offsets.
	//In current configuration this function is not being used but has
	//been retained for reference.
	void calibrate(){
		int offset[6] = {0};
		this ->mean(500);

		offset[0] = -_mean[0]/8;
		offset[1] = -_mean[1]/8;
		offset[2] = (16384-_mean[2])/4;

		offset[3] = -_mean[3]/4;
		offset[4] = -_mean[4]/4;
		offset[5] = -_mean[5]/4;

		mpu.setXAccelOffset(offset[0]);
		mpu.setYAccelOffset(offset[1]);
		mpu.setZAccelOffset(offset[2]);
		mpu.setXGyroOffset(offset[3]);
		mpu.setYGyroOffset(offset[4]);
		mpu.setZGyroOffset(offset[5]);

	}

	//This function returns a true or false value depending on
	//the degree of tilt.  It will return true if the robot has
	//remained tilted for more than 15 calls to this function.
	bool tooFar(){
		if(_angle >= 150 || _angle <= -150 ){
			_counter ++;
		} else {
			_counter = 0;
		}


		return (_counter > 15);
	}



private:

	float Q_angle  = 0.04; //0.001 //adjusts how much raw value of accelerometer is trusted
	float Q_bias   = 0.001; //0.003 //0.035 //adjusts how much raw value of gyro is trusted
	float R_measure  = 0.07; //0.03 //0.01  //adjust amount of noise rejection

	float angle = 0;  // = 0;
	float bias  = 0;  // = 0;
	float rate  = 0;

	float P[2][2] = {{0,0},{0,0}};
	float K[2] = {0,0};
	float y       = 0;
	float S       = 0;

	float _smoothed = 0;

	int _counter = 0;


	int16_t _ax = 0;
	int16_t _ay = 0;
	int16_t _az = 0;
	int16_t _gx = 0;
	int16_t _gy = 0;
	int16_t _gz = 0;

	float _mean[6] = {0};

	float _angle = 0;

	float kalman2Value(float newAngle, float newRate, float dt){

		rate = newRate - bias;
		angle += dt * rate;

		P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
		P[0][1] -= dt * P[1][1];
		P[1][0] -= dt * P[1][1];
		P[1][1] += Q_bias * dt;

		S = P[0][0] + R_measure;

		K[0] = P[0][0] / S;
		K[1] = P[1][0] / S;

		y = newAngle - angle;

		angle += K[0] * y;
		bias += K[1] * y;

		P[0][0] -= K[0] * P[0][0];
		P[0][1] -= K[0] * P[0][1];
		P[1][0] -= K[1] * P[0][0];
		P[1][1] -= K[1] * P[0][1];

		return angle;
	}

	void mean(int size){
		int toss = 0.1 * size;
		int i = 0;

		long buff[6] = {0};

		while(i < (size + (toss + 1))){
			mpu.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);

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

	int arctan2(int y, int x) {          // http://www.dspguru.com/comp.dsp/tricks/alg/fxdatan2.htm
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


}imu;



void setup(){
	Serial.begin(9600);
	Serial.println("Hello World");

	pinMode(LED, OUTPUT);

	Wire.begin();
	mpu.initialize();

	//default offset values
	mpu.setXAccelOffset(1501);
	mpu.setZAccelOffset(1262);
	mpu.setXGyroOffset(-4);
	mpu.setYGyroOffset(4);
	mpu.setZGyroOffset(4);


	for( int i = 0; i < 10; i++){
		digitalWrite(LED, LOW);
		delay(100);
		digitalWrite(LED, HIGH);
		delay(100);
		digitalWrite(LED, LOW);
	}

}


void loop(){
	float tilt = imu.findAdjustedTilt(0.1);

	double kval[3];
	pid.getPid(kval); //Return PID values

	//  Serial.print(kval[0]); Serial.print(", ");
	//  Serial.print(kval[1]*100); Serial.print(", ");
	//  Serial.print(kval[2]*100); Serial.print(", ");
	Serial.print(tilt);
	Serial.println(",  ");

	//If bot is tilted too far turn motors off
	if(!imu.tooFar()){
		double pwm = pid.calculate(0,tilt, 1);

		//    Serial.println(-pwm);
		m.motor('l', pwm);
		m.motor('r', pwm);

	} else {
		m.motor('l', 0);
		m.motor('r', 0);
	}

	delay(5);

}
