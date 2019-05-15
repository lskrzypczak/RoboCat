/*
 * servoController.h
 *
 *  Created on: Mar 9, 2019
 *      Author: crash
 */

#ifndef SERVOCONTROLLER_H_
#define SERVOCONTROLLER_H_

/*
 *
 */
class servoController {
public:
	servoController();
	virtual ~servoController();
	int getFile();

private:
	int file_i2c = -1;
};

#endif /* SERVOCONTROLLER_H_ */
