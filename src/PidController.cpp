/*
 * PidController.cpp
 *
 *  Created on: Jan 8, 2017
 *      Author: grantdeshazer
 */

#include "PidController.h"

namespace std {

PidController::PidController() {

	kp = new Potentiometer(2);
	ki = new Potentiometer(3);
	//kd = new Potentiometer(2);


	_kp = 1, _ki = .1, _kd = .5;
	_error = 0, _preError = 0, _proportional = 0, _derivative = 0, _integral = 0, _integralError = 0;
	_out = 0;

}

PidController::~PidController() {
	delete kp, ki;
	delete this;
}


double PidController::calculate(int setPoint, float pv, int time){

	_error = setPoint - pv;

	_proportional = _kp * _error;

	_integralError  += _error * time;
	_integral = _ki * _integralError;

	double deriv = (_error - _preError) / time;
	_derivative = _kd * deriv;

	_out = _proportional + _derivative + _integral;

	if(_out > 255){
		_out = 255;
	} else if (_out < -255){
		_out = -255;
	}

	_preError = _error;


	return _out;

}

} /* namespace std */
