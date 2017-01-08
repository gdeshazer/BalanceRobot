/*
 * Timer.cpp
 *
 *  Created on: Jan 8, 2017
 *      Author: grantdeshazer
 */

#include "Timer.h"

#include <Arduino.h>

namespace std {

Timer::Timer() {
	_startTime = 0;
}

Timer::~Timer() {
	delete this;
}

void Timer::setStart(){
	_startTime = millis();
}

unsigned long Timer::getDelta(){
	return millis() - _startTime;
}

} /* namespace std */
