/*
 * myCalibration.cpp
 *
 *  Created on: Apr 18, 2015
 *      Author: grantdeshazer
 */

#include <Arduino.h>

#include "Wire.h"
#include"MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"

struct IMUControl{
	int16_t ax, ay, az;
	int16_t gx, gy, gz;

	int offsetA[3];
	int offsetG[3];

	long meanA[3];
	long meanG[3];

	bool ready = false;
	uint8_t mpuIntStat;
	uint8_t devStat;
	uint8_t packetSize;
	uint8_t fifoCount;
	uint8_t fifobuffer[64];

}control;

MPU6050 imu;

void getMean(){
	int buffer = 0, i = 0, size =100;
	int toss = size *.1;
	long bufA[3] = {0};
	long bufG[3] = {0};

	while(i < (size + (toss+1))){
		imu.getMotion6(&control.ax, &control.ay, &control.az, &control.gx,
				&control.gy, &control.gz);
		//		imu.getAcceleration(&control.ax, &control.ay, &control.az);
		//		imu.getRotation(&control.gx, &control.gy, &control.gz);

		if(i > (toss) && i<= size+toss){
			bufA[0] += control.ax;
			bufA[1] += control.ay;
			bufA[2] += control.az;

			bufG[0] += control.gx;
			bufG[1] += control.gy;
			bufG[2] += control.gz;
		}

		if(i ==(size+toss)){
			for(int x = 0; x < 3; x ++){
				control.meanA[x] = bufA[x]/size;
				control.meanG[x] = bufG[x]/size;
			}
		}
		i++;
		delay(2);
	}

}

void calibrate(){
	const int deadZone = 10;
	const int gdeadZone = 1;
	int ready = 0;

	control.offsetA[0] = -control.meanA[0]/8;
	control.offsetA[1] = -control.meanA[1]/8;
	control.offsetA[2] = (16384-control.meanA[2])/8;

	control.offsetG[0] = -control.meanG[0]/4;
	control.offsetG[1] = -control.meanG[1]/4;
	control.offsetG[2] = -control.meanG[2]/4;

	while(1){
		imu.setXAccelOffset(control.offsetA[0]);
		imu.setYAccelOffset(control.offsetA[1]);
		imu.setZAccelOffset(control.offsetA[2]);
		imu.setXGyroOffset(control.offsetG[0]);
		imu.setYGyroOffset(control.offsetG[1]);
		imu.setZGyroOffset(control.offsetG[2]);

		getMean();

		if(abs(control.meanA[0]) <= deadZone){
			ready++;
		} else {
			control.offsetA[0] = control.offsetA[0] - control.meanA[0]/8;
		}

		if(abs(control.meanA[1]) <= deadZone){
			ready++;
		} else {
			control.offsetA[1] = control.offsetA[1] - control.meanA[1]/8;
		}

		if(abs(16348-control.meanA[2]) <= deadZone){
			ready++;
		} else {
			control.offsetA[2] = control.offsetA[2] + (16384 - control.meanA[2])/8;
		}

		for(int x = 0; x < 3; x ++){
			if(abs(control.meanG[x]) <= gdeadZone){
				ready++;
			} else {
				control.offsetG[x] = control.offsetG[x]-control.meanG[x]/(gdeadZone+1);
			}
		}
		Serial.println("...");
		if(ready==6) break;
	}
}


void setup(){
	Wire.begin();
	Serial.begin(115200);

	while (Serial.available() && Serial.read()); // empty buffer
	Serial.println(F("Send any character to start sketch.\n"));
	while (!Serial.available()){
		delay(1500);
	}
	while (Serial.available() && Serial.read());

	Serial.println("Initializing...");
	imu.initialize();

	imu.setXAccelOffset(0);
	imu.setYAccelOffset(0);
	imu.setZAccelOffset(0);
	imu.setXGyroOffset(0);
	imu.setYGyroOffset(0);
	imu.setZGyroOffset(0);

	calibrate();

	Serial.println("Init and calibration successful");

}

void loop(){
	getMean();

	Serial.print("Your values are:\n");
	Serial.print("a\t");
	for(int x = 0; x < 3; x ++){
		Serial.print(control.meanA[x]); Serial.print("\t");
	}
	Serial.print("\nypr\t");
	for(int x = 0; x < 3; x ++){
		Serial.print(control.meanG[x]); Serial.print("\t");
	}
	Serial.println();
}
