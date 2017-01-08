/*
 * Balance_Bot.h
 *
 *  Created on: Jan 7, 2017
 *      Author: grantdeshazer
 */

#ifndef BALANCE_BOT_H_
#define BALANCE_BOT_H_

#include <Arduino.h>
#include <math.h>
#include <Wire.h>

#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>


//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

class IMUControl;
class MotorControl;


#endif /* BALANCE_BOT_H_ */
