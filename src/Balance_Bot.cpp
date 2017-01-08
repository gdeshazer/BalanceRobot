/*
 * Balance_Bot.cpp
 *
 *  Created on: Jan 7, 2017
 *      Author: grantdeshazer
 */

#include "Balance_Bot.h"

#include <Arduino.h>

#include "Accelerometer.h"
#include "MotorController.h"
#include "PidController.h"
#include "Timer.h"

using namespace std;

MotorController m;
Accelerometer imu;
PidController pid;
Timer t;


void setup(){
	Serial.begin(115200);

}

void loop(){
	float tilt = imu.findAdjustedTilt(t.getDelta());

	if(!imu.tooFar()){
		double pwm = pid.calculate(0,tilt,(int) t.getDelta());
		m.motor('l', pwm);
		m.motor('r', pwm);

	} else {
		m.motor('l', 0);
		m.motor('r', 0);
	}

	t.setStart();
}
