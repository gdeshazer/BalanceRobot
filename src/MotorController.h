/*
 * MotorController.h
 *
 *  Created on: Jan 8, 2017
 *      Author: grantdeshazer
 */

#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

namespace std {

class MotorController {
public:
	MotorController();
	virtual ~MotorController();

	void forward(int, int);
	void reverse(int, int);
	void stop(char, char);

private:
	void motor(char, int);

	int _leftMotor;
	int _leftDirrection;
	int _rightMotor;
	int _rightDirrection;
};

} /* namespace std */

#endif /* MOTORCONTROLLER_H_ */
