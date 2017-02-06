/*
 * Potentiometer.cpp
 *
 *    This class takes care of the initialization and
 *     data retrieval of a potentiometer attached to one
 *     of the analog input pins.
 *
 *     It provides the functionality to both return just
 *     the raw value, but also it will return a mapped value
 *     that takes the value read from the potentiometer and
 *     maps it to a range that is more suitable for use.
 *
 *  Created on: Jun 21, 2015
 *      Author: grantdeshazer
 */

#include "Potentiometer.h"
#include <Arduino.h>

Potentiometer::Potentiometer(int pin) {
		pinMode(pin, INPUT);
		_pot = pin;
		_reading = 0;
}

int Potentiometer::getRaw(){
	_reading = analogRead(_pot);
	return _reading;

}

float Potentiometer::getReading(float min, float max){
	return this->getCompensated((float) this ->getRaw(), min, max);
}

float Potentiometer::getCompensated(float in, float outMin, float outMax){
	float inMin = 0, inMax = 1024;
	return outMin + (( (in - inMin) * (outMax-outMin) )/(inMax-inMin));
}
