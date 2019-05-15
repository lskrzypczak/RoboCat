/*
 * servo.cpp
 *
 *  Created on: Mar 9, 2019
 *      Author: crash
 */

#include "../include/servo.h"
#include <unistd.h> // required for I2C device access and usleep()
#include <fcntl.h>  // required for I2C device configuration
#include <sys/ioctl.h> // required for I2C device usage
#ifndef __APPLE__
#include <linux/i2c-dev.h> // required for constant definitions
#endif
#include <stdio.h>  // required for printf statements
#include <stdint.h>
#include <iostream>
#include <cmath>
using namespace std;

/*
 *  servo	-180	0		180
 *  		Up		Mid		Down
 *  0		490		320		190
 *  1		490		320		210
 *  2		490		340		200
 *  3		480		340		220
 *  		Back	Mid		Front
 *  4		460		340		230
 *  5		220		340		460
 *  6		200		340		460
 *  7		460		340		230
 *  		Back	Mid		Front
 *  8		460		280		110
 *  9		500		320		150
 *  10		180		320		470
 *  11		120		320		460
 *  		Up		Mid		Down
 *  12		190		320		400		(13)
 *  		Left	Mid		Right
 *  13		450		290		180
 *  14
 *  15
 */
typedef struct servoBoundary_s {
	int minimum;
	int middle;
	int maximum;
}servoBoundary_t;

servoBoundary_t servoBoundaries[16] = {
		{ 190, 320, 490 },	//0
		{ 210, 320, 490 },	//1
		{ 200, 340, 490 },	//2
		{ 220, 340, 480 },	//3

		{ 230, 340, 460 },	//4
		{ 220, 340, 460 },	//5
		{ 200, 340, 460 },	//6
		{ 230, 340, 460 },	//7

		{ 110, 280, 460 },	//8
		{ 150, 320, 500 },	//9
		{ 180, 320, 470 },	//10
		{ 120, 320, 460 },	//11

		{ 180, 290, 450 },	//12
		{ 190, 320, 400 },	//13

		{ 190, 320, 490 },	//14
		{ 190, 320, 490 },	//15
};

servo::servo(servoController *controller, int channel) {
	// TODO Auto-generated constructor stub
	if (channel <= 15)
		servo_channel = channel;
	else {
		cout << "Servo channel must be <= 15." << endl;
		servo_channel = -1;
	}
	file_i2c = controller->getFile();
	if (file_i2c < 0)
		cout << "Wrong I2C file descriptor" << endl;
}

servo::~servo() {
	// TODO Auto-generated destructor stub
}

void servo::setPosition( int value ) {
	if (file_i2c >= 0) {
		uint8_t buffer[3];
		if (servo_channel < 0)
			return;

		/* servo angle must be limited according to the calibration table */
		if (value < servoBoundaries[servo_channel].minimum) value = servoBoundaries[servo_channel].minimum;
		if (value > servoBoundaries[servo_channel].maximum) value = servoBoundaries[servo_channel].maximum;
		if (value == -1) value = servoBoundaries[servo_channel].middle;	/* -1 is an universal alternative value for 0 position */

		buffer[0] = 0x08 + (4 * servo_channel);
		buffer[1] = (uint8_t)(value & 0xFF);
		buffer[2] = (uint8_t)((value >> 8) & 0xFF);
		write(file_i2c, buffer, 3);

		printf("Written %i to servo %i [0x%02X]\n", value, servo_channel, 0x08 + (4 * servo_channel));
	}
}

void servo::setAngle( float angle ) {
	float fval;
	if (angle < -M_PI / 2) angle = -M_PI / 2;
	if (angle > M_PI / 2) angle = M_PI / 2;

	/* This is section linear function */
	if (angle < 0) /* section x < 0 */
		fval = (((float)servoBoundaries[servo_channel].middle - servoBoundaries[servo_channel].minimum) / (M_PI / 2)) * (float)angle * -1.0 + servoBoundaries[servo_channel].minimum;
	if (angle > 0) /* section x > 0 */
		fval = (((float)servoBoundaries[servo_channel].maximum - servoBoundaries[servo_channel].middle) / (M_PI / 2)) * (float)angle + servoBoundaries[servo_channel].middle;
	else /* x = 0 */
		fval = (float)servoBoundaries[servo_channel].middle;
	setPosition( (int)round(fval) );
}
