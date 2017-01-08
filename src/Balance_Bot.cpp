/*
 * Balance_Bot.cpp
 *
 *  Created on: Jan 7, 2017
 *      Author: grantdeshazer
 */

#include "Balance_Bot.h"

#include <Arduino.h>
#include <math.h>
#include <Wire.h>

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

const int LOOP_TIME = 9;
int lastLoopT = LOOP_TIME;
int lastLoopTUSE = LOOP_TIME;
unsigned long loopStartT = 0;

#define leftM 11
#define leftDir 13
#define rightM 3
#define rightDir 12

MPU6050 imu;


//IDEA: include in cnstructor methods to set up the chip for reading
class IMUControl{
public:
	bool dmpReady;
	uint16_t packetSize;
	uint16_t fifoCount;
	uint8_t intStat;
	uint8_t fifoBuffer[64];

	Quaternion q;
	VectorInt16 aa;
	VectorInt16 aaReal;
	VectorFloat gravity;

	float euler[3];
	float ypr[3];


	volatile bool data;

	void getMeasure(){
		this->data = false;
		this->intStat = imu.getIntStatus();
		this->fifoCount = imu.getFIFOCount();
		if((this->intStat & 0x10) || this->fifoCount == 1024){
			imu.resetFIFO();
			//overflow
		} else if(con.intStat & 0x02){
			while(this->fifoCount < this->packetSize) this->fifoCount = imu.getFIFOCount();

			imu.getFIFOBytes(this->fifoBuffer, this->packetSize);

			this->fifoCount -= this->packetSize;

			//---yaw pitch roll ---
			imu.dmpGetQuaternion(&q, fifoBuffer);
			imu.dmpGetGravity(&gravity, &q);
			imu.dmpGetYawPitchRoll(ypr, &q, &gravity);

			//---accel compensated for gravity --
            imu.dmpGetQuaternion(&q, fifoBuffer);
            imu.dmpGetAccel(&aa, fifoBuffer);
            imu.dmpGetGravity(&gravity, &q);
            imu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

		}

	}

	int filter(){
		this->getMeasure();
		return kalmanCalculate(gx, ax, LOOP_TIME);
	}

private:
	int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

	float Q_angle  =  0.001; //0.001
	float Q_gyro   =  0.003;  //0.003
	float R_angle  =  0.03;  //0.03

	float x_angle = 0;
	float x_bias = 0;
	float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;
	float dt, y, S;
	float K_0, K_1;

	float kalmanCalculate(float newAngle, float newRate,int looptime) {
		dt = float(looptime)/1000;
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

}con;

class MotorControl{
public:

	void motorForward(int speedA, int speedB){
		this->motor('L', pid(0, con.filter()));
		this->motor('R', pid(0, con.filter()));

	}

	void reverse(int t){
		this->motor('L',-130);
		this->motor('R',-130);

		delay(t);

		this->stop('B','B');

	}


	/*
	 * Given a particular previous state passed to this function
	 * in chars the Stop will momentarily reverse the direction
	 * of the motors in order to prevent drifting from motor
	 * over spin.
	 *
	 * The chars passed to this function represent the previous
	 * state of the motor.
	 *   -- F ~> forward motor direction i.e. dir pin set HIGH
	 *   -- B ~> backward motor direction i.e. dir pin set LOW
	 *
	 * While these values for each char is hard coded in at the
	 * moment, it could be feasible to design the software to
	 * set the chars itself.
	 */
	void stop(char l, char r){
		int rt = 50;

		//both motors were previously set to forward
		if(l == 'F' && r == 'F'){
			//reverse direction
			digitalWrite(leftDir, LOW);
			digitalWrite(rightDir, LOW);

			analogWrite(leftM, 255);
			analogWrite(rightM, 255);

			delay(rt);

			//motors off
			analogWrite(leftM, 0);
			analogWrite(rightM, 0);

		} else if (l == 'F' && r == 'B'){
			digitalWrite(leftDir, LOW);
			digitalWrite(rightDir, HIGH);

			analogWrite(leftM, 255);
			analogWrite(rightM, 255);

			delay(rt);

			analogWrite(leftM, 0);
			analogWrite(rightM, 0);

		} else if( l == 'B' && r == 'F'){
			digitalWrite(leftDir, HIGH);
			digitalWrite(rightDir, LOW);

			analogWrite(leftM, 255);
			analogWrite(rightM, 255);

			delay(rt);

			analogWrite(leftM, 0);
			analogWrite(rightM, 0);

		}
	}

private:
	const int _kp = 1;
	const int _ki = 1;
	const int _kd = 1;
	const int _k = 1;

	int _error = 0;
	int _lastE = 0;

	//Motor A is the right motor
	//Motor B is the left motor
	void motor(char m, int pwm){
		//forward motor direction defined as positive power level
		//for reverse direction pass in a negative value
		if(m == 'L'){
			if(pwm > 0){
				digitalWrite(leftDir, HIGH);
				analogWrite(leftM, pwm);

			} else if(pwm < 0){
				digitalWrite(leftDir, LOW);
				analogWrite(leftM, -pwm);

			} else{
				analogWrite(leftM, 0);
			}
		}  // end motor A

		if(m == 'R'){
			if(pwm > 0){  //see motor A above
				digitalWrite(rightDir, HIGH);
				analogWrite(rightM, pwm);

			} else if(pwm < 0){
				digitalWrite(rightDir, LOW);
				analogWrite(rightM, -pwm);

			} else {
				analogWrite(rightM, 0);
			}
		}  // end motor B
	}

	int pid(int setPoint, int current){
		const int guard = 15;
		_error = setPoint - current;
		int p =_kp * _error;
		int intError = intError + _error;
		int i = _ki * constrain(intError, -guard, guard);
		int d = _kd * (_error - _lastE);
		_lastE = _error;

		return -constrain(_k*(p + i + d), -255, 255);
	}

}m;


void dataReady(){
	con.data = true;
}

void setup(){
	//set interrupt data pin
	//maybe calibrate?

	attachInterrupt(0, dataReady, RISING);

	Wire.begin();
	imu.initialize();

	uint8_t devStat = imu.dmpInitialize();

	imu.setXAccelOffset(1501);
	imu.setYAccelOffset(1251);
	imu.setZAccelOffset(1262);
	imu.setXGyroOffset(-4);
	imu.setYGyroOffset(4);
	imu.setZGyroOffset(4);

	if(devStat == 0){
		imu.setDMPEnabled(true);
		con.dmpReady = true;

		con.packetSize = imu.dmpGetFIFOPacketSize();
	} else {
		//something has failed
		//flash some lights or something
	}

}

void loop(){

	if(!con.dmpReady) return;

	while(!con.data && con.fifoCount < con.packetSize);
	//motor control ~> this will call for sensor reading?
	//or does sensor reading need to happen here?

	lastLoopTUSE = millis() - loopStartT;
	if(lastLoopTUSE < LOOP_TIME) delay(LOOP_TIME - lastLoopTUSE);
	lastLoopT = millis() - loopStartT;
	loopStartT = millis();
}
