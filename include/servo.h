/*
 * servo.h
 *
 *  Created on: Mar 9, 2019
 *      Author: crash
 */

#ifndef SERVO_H_
#define SERVO_H_

#include "../include/servoController.h"
/*
 *
 */
class servo {
public:
	servo( servoController*, int );
	virtual ~servo();
	void setPosition( int );
	void setAngle( float );
private:
	int file_i2c;
	int servo_channel;
};

#endif /* SERVO_H_ */
