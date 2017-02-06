/*
 * Potentiometer.h
 *
 *     This class takes care of the initialization and
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

#include <Arduino.h>

#ifndef POTENTIOMETER_H_
#define POTENTIOMETER_H_

class Potentiometer {
public:
	Potentiometer(int);
	int getRaw();
	float getReading(float, float);

private:
	int _pot;
	int _reading;

	float getCompensated(float, float, float);
};

#endif /* POTENTIOMETER1_H_ */
