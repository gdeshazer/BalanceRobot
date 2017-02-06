/*
 * MotorController.h
 *
 *  Created on: Jan 8, 2017
 *      Author: grantdeshazer
 *
 *  See the cpp file for implementation notes
 */

#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include <Arduino.h>

namespace std {

class MotorController {
public:
	MotorController();
	MotorController(int, int, int, int, int, int);


	void motor(char, int);
	void stop();

private:

	int _leftMotor;
	int _leftDirrection;
	int _rightMotor;
	int _rightDirrection;
	int _enable;
	int _fault;

	char _currentLeftDirrection;
	char _currentRightDirrection;
};

} /* namespace std */

#endif /* MOTORCONTROLLER_H_ */
