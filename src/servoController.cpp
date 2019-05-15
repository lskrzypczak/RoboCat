/*
 * servoController.cpp
 *
 *  Created on: Mar 9, 2019
 *      Author: crash
 */

#include "../include/servoController.h"
#include <unistd.h> // required for I2C device access and usleep()
#include <fcntl.h>  // required for I2C device configuration
#include <sys/ioctl.h> // required for I2C device usage
#ifndef __APPLE__
#include <linux/i2c-dev.h> // required for constant definitions
#endif
#include <stdio.h>  // required for printf statements
#include <stdint.h>
#include <iostream>
using namespace std;

#define ADDR	0x40

servoController::servoController() {
	// TODO Auto-generated constructor stub
#ifndef __APPLE__
	char *filename = (char*)"/dev/i2c-1"; // Define the filename

	file_i2c = open(filename, O_RDWR); // open file for R/W
	if (file_i2c < 0)
		cout << "Can't open " << filename << endl;

	ioctl(file_i2c, I2C_SLAVE, ADDR); // Set the I2C address for upcoming
	//  transactions

	char buffer[5];   // Create a buffer for transferring data to the I2C device


	// First we need to enable the chip. We do this by writing 0x20 to register
	//  0. buffer[0] is always the register address, and subsequent bytes are
	//  written out in order.
	buffer[0] = 0;    // target register
	buffer[1] = 0x20; // desired value
	int length = 2;       // number of bytes, including address
	write(file_i2c, buffer, length); // initiate write
	usleep(250*1000);

	buffer[0] = 0x0;
	buffer[1] = 0x10;
	write(file_i2c, buffer, length);
	usleep(250*1000);

	// Enable multi-byte writing.

	buffer[0] = 0xfe;
	buffer[1] = 0x79;
	write(file_i2c, buffer, length);

	buffer[0] = 0x0;
	buffer[1] = 0x20;
	write(file_i2c, buffer, length);

	usleep(500*1000);

	for (int i = 0; i < 15; i++) {
		//start time = 0
		buffer[0] = 0x06 + (4 * i);
		buffer[1] = 0;
		buffer[2] = 0;
		write(file_i2c, buffer, 3);

		printf("Written start time 0 to servo %i [0x%02X]\n", i, 0x06 + (4 * i));
	}
#endif
}

servoController::~servoController() {
	// TODO Auto-generated destructor stub
#ifndef __APPLE__
	close(file_i2c);
#endif
}

int servoController::getFile() {
	return file_i2c;
}
