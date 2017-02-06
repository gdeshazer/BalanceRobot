/*
 * Timer.cpp
 *
 * UNUSED CLASS
 *
 *  Created on: Jan 8, 2017
 *      Author: grantdeshazer
 */

#include "SetTimer.h"

#include <Arduino.h>

namespace std {

SetTimer::SetTimer() {
	_startTime = 0;
	Serial.println("Building timer");
}


void SetTimer::setStart(){
	_startTime = millis();
}

unsigned long SetTimer::getDelta(){
	return millis() - _startTime;
}

} /* namespace std */
