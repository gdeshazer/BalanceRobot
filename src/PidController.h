/*
 * PidController.h
 *
 *  Created on: Jan 8, 2017
 *      Author: grantdeshazer
 */

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include <Potentiometer.h>

namespace std {

class PidController {
public:
	PidController();
	virtual ~PidController();

	double calculate(int, float, int);

private:
	Potentiometer kp;
	Potentiometer ki;
	//Potentiometer kd;


	double _kp, _ki, _kd;
	double _error, _preError, _proportional, _derivative, _integral, _integralError, _out;

};

} /* namespace std */

#endif /* PIDCONTROLLER_H_ */
