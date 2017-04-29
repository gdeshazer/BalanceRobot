# Arduino Balance Bot
An Arduino balance bot using the MPU6050 and 3D printed frame.

## Overview
This is the functioning balancing robot code for an Arduino and an MPU6050.  At the moment there are two potentiometers wired to the analog input pins.  These potentionmeters control two values out of the proportional, integral, and derivative feedback control to allow for on the fly adjustments to the robots behavior.

Included is a simple calibration script which can be uploaded onto the Arduino.  Running it will help to narrow down specific offest values that can be programmed into the MPU6050.  However, within the Accelerometer class in Balance_Bot.cpp, there is a parameter used to adjust the value returned from the MPU6050 to zero (seen below):
 
 ```
 float findAdjustedTilt(float time){		
	
	mpu.getMotion6(&_ax, &_ay, &_az, &_gx, &_gy, &_gz);
	
	_gx = _gx * 250.0/32768.0;
	_angle = (this->arctan2(-_az,-_ay)) - 36; //The 36 here is the adjustment value
						  //Idealy, when upright the returned angle should be zero

	Serial.print("  / "); Serial.print(" "); Serial.print(_angle); Serial.print(" /   ");

	float lpfBeta = 0.05;

	float raw = kalman2Value(_angle, _gx, time);

	Serial.print(raw); Serial.print(", ");

	_smoothed -= lpfBeta * (_smoothed - raw);

	return _smoothed;
}
```

## Using this code
For IDE's like Ecplise, download the repository, compile the code, and uploade the binary via an avr tool to the Arduino.  Pin connections for the MPU6050 are specific to the MPU6050 library.  Potentiometers are wired to analog pins 2 & 3.  To modify what they control, go to IncludedClasses/PidController.cpp.

For the Arduino IDE, start a new Arduino project and paste in the code from Balance_Bot.cpp.  Then, depending on your OS, navigate to where Arduino stores its libraries.  
* For Mac the path should be ~/Documents/Arduino/libraries.
* For Windows the path should be My Documents\Arduino\libraries\

Create a new folder, named BalanceControl, and drop in all of the files in the IncludedClasses dirrectory except for Wire.
Compile the program as usual from the main Arduino project window.

## References
Kas's project on the Arduino form has a considerable amount of insight into the construction of a balance bot.  I wrote much of this project based on what Kas has posted.  You can find his form post [here](http://forum.arduino.cc/index.php?topic=8871.30).

The MPU6050 library and the I2Cdev library are both [here](https://github.com/jrowberg/i2cdevlib).

Further documentation for the MPU6050 and the I2Cdevlib can be found [here](http://www.i2cdevlib.com/forums/forum/2-mpu-6050-6-axis-accelerometergyroscope-invensense/).
