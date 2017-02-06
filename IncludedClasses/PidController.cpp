/*
 * PidController.cpp
 *
 *  Created on: Jan 8, 2017
 *      Author: grantdeshazer
 */

#include "PidController.h"

namespace std {

PidController::PidController() :  kpPot(2), kiPot(3){

//	Serial.println("Building PID Controller");

	_kp = 1, _ki = .1, _kd = 0;
	_error = 0, _preError = 0, _proportional = 0, _derivative = 0, _integral = 0, _integralError = 0;
	_out = 0;

}

PidController::~PidController() {
	delete &kpPot;
	delete &kiPot;
	delete this;
}

/* PID calculation
 *
 * This PID controller uses potentiometers to set P I and D values so as
 * to allow for on the fly adjustments of the PID controllers behavior.
 *
 * Setpoint is the point around which the controller wants to be.
 *
 * pv or process variable is the variable the algorithm is attempting to
 * adjust for.
 *
 * Time is the time that elapses between calculations.
 */
double PidController::calculate(float setPoint, float pv, float time){


	_kp = kpPot.getReading(0,10); //2.77
	_ki = 0;
	_kd = kiPot.getReading(0,5);

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

void PidController::getPid(double kval[]){

	kval[0] = _kp;
	kval[1] = _ki;
	kval[2] = _kd;


}

} /* namespace std */
