/*
 * Timer.h
 *
 *  Created on: Jan 8, 2017
 *      Author: grantdeshazer
 */

#ifndef SETTIMER_H_
#define SETTIMER_H_

#include <Arduino.h>

namespace std {

class SetTimer {
public:
	SetTimer();

	void setStart();
	unsigned long getDelta();

private:
	unsigned long _startTime;
};

} /* namespace std */

#endif /* SETTIMER_H_ */
