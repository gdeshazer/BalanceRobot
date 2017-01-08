/*
 * MotorController.cpp
 *
 *  Created on: Jan 8, 2017
 *      Author: grantdeshazer
 */

#include "MotorController.h"
#include <Arduino.h>

namespace std {


//#define leftM 11
//#define leftDir 13
//#define rightM 3
//#define rightDir 12


/* Default Constructor:
 *
 * This constructor assumes which pins are default for motor control.
 * These valuse may not be correct for different boards.
 *
 * Initializes pinMode for each pin and sets default member variable values.
 *
 */
MotorController::MotorController() {

	_leftDirrection   = 7;
	_leftMotor        = 9;
	_rightDirrection  = 8;
	_rightMotor       = 10;
	_enable           = 4;
	_fault            = 12;

	_currentLeftDirrection   = 'F';
	_currentRightDirrection  = 'F';

	pinMode(_leftDirrection,  OUTPUT);
	pinMode(_leftMotor,       OUTPUT);
	pinMode(_rightDirrection, OUTPUT);
	pinMode(_rightMotor,      OUTPUT);
	pinMode(_enable,          OUTPUT);
	pinMode(_fault,           INPUT);

}


/* Custom Constructor:
 *
 * Accepts custom values for pin locations.
 *
 * Initializes pinMode for each pin and sets default member variables.
 *
 */
MotorController::MotorController(int leftD, int leftM, int rightD, int rightM, int enable, int fault ) {

	_leftDirrection   = leftD;
	_leftMotor        = leftM;
	_rightMotor       = rightD;
	_rightDirrection  = rightM;
	_enable           = enable;
	_fault            = fault;


	_currentLeftDirrection   = 'F';
	_currentRightDirrection  = 'F';


	pinMode(_leftDirrection,   OUTPUT);
	pinMode(_leftMotor,        OUTPUT);
	pinMode(_rightDirrection,  OUTPUT);
	pinMode(_rightMotor,       OUTPUT);
	pinMode(_enable,           OUTPUT);
	pinMode(_fault,            INPUT);

}

MotorController::~MotorController() {
	delete this;
}


/* Function motor:
 *   char motor ~> which motor to turn on
 *   			   Left motor is indicated by LOWERCASE 'l'
 *   			   right motor is inidicated by LOWERCASE 'r'
 *
 *   int pwm ~> speed to turn motor on at
 *   			accepts range of values from -255 to 255
 *   			where negative values are reverese and
 *   			positive valuse are forward
 *
 *   currentDirrection chars:
 *   	F ~> motor is rotating forward and dirrection pin
 *   		 is set to HIGH
 *   	R ~> motor is rotating in reverse and dirrection pin
 *   	     is set to LOW
 *
 */
void MotorController::motor(char motor, int pwm){
	//forward motor direction defined as positive power level
	//for reverse direction pass in a negative value

	if(digitalRead(_fault) == 0){
		while(true);
	} else {
		digitalWrite(_enable, HIGH);
	}

	if(motor == 'l'){
		if(pwm > 0){
			digitalWrite(_leftDirrection, HIGH);
			analogWrite(_leftMotor, pwm);
			_currentLeftDirrection = 'F';

		} else if(pwm < 0){
			digitalWrite(_leftDirrection, LOW);
			analogWrite(_leftMotor, -pwm);
			_currentLeftDirrection = 'R';

		} else{
			analogWrite(_leftMotor, 0);
		}
	}

	if(motor == 'r'){
		if(pwm > 0){
			digitalWrite(_rightDirrection, HIGH);
			analogWrite(_rightMotor, pwm);
			_currentRightDirrection = 'F';

		} else if(pwm < 0){
			digitalWrite(_rightDirrection, LOW);
			analogWrite(_rightMotor, -pwm);
			_currentRightDirrection = 'R';

		} else {
			analogWrite(_rightMotor, 0);
		}
	}  // end motor B
}


void MotorController::stop(){
	int rt = 50;

	if(digitalRead(_fault) == 0){
			while(true);
		} else {
			digitalWrite(_enable, HIGH);
		}

	//both motors were previously set to forward
	if(_currentLeftDirrection == 'F' && _currentRightDirrection == 'F'){
		//reverse direction
		digitalWrite(_leftDirrection, LOW);
		digitalWrite(_rightDirrection, LOW);

		analogWrite(_leftMotor, 255);
		analogWrite(_rightMotor, 255);

		delay(rt);

		//motors off
		analogWrite(_leftMotor, 0);
		analogWrite(_rightMotor, 0);

	} else if (_currentLeftDirrection == 'R' && _currentRightDirrection == 'R'){
		//reverse direction
		digitalWrite(_leftDirrection, LOW);
		digitalWrite(_rightDirrection, LOW);

		analogWrite(_leftMotor, 255);
		analogWrite(_rightMotor, 255);

		delay(rt);

		//motors off
		analogWrite(_leftMotor, 0);
		analogWrite(_rightMotor, 0);

	}else if (_currentLeftDirrection == 'F' && _currentRightDirrection == 'R'){
		digitalWrite(_leftDirrection, LOW);
		digitalWrite(_rightDirrection, HIGH);

		analogWrite(_leftMotor, 255);
		analogWrite(_rightMotor, 255);

		delay(rt);

		analogWrite(_leftMotor, 0);
		analogWrite(_rightMotor, 0);

	} else if( _currentLeftDirrection == 'F' && _currentRightDirrection == 'R'){
		digitalWrite(_leftDirrection, HIGH);
		digitalWrite(_rightDirrection, LOW);

		analogWrite(_leftMotor, 255);
		analogWrite(_rightMotor, 255);

		delay(rt);

		analogWrite(_leftMotor, 0);
		analogWrite(_rightMotor, 0);

	}

}



} /* namespace std */
