/*
 * Timer.h
 *
 *  Created on: Jan 8, 2017
 *      Author: grantdeshazer
 */

#ifndef TIMER_H_
#define TIMER_H_

#include <Arduino.h>

namespace std {

class Timer {
public:
	Timer();
	virtual ~Timer();

	void setStart();
	unsigned long getDelta();

private:
	unsigned long _startTime;
};

} /* namespace std */

#endif /* TIMER_H_ */
