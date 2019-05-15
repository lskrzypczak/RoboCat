/*
 * Simulation.h
 *
 *  Created on: Apr 27, 2019
 *      Author: crash
 */

#ifndef SIMULATION_H_
#define SIMULATION_H_

#include <iostream>
#include <cstddef>
#include <vector>
#include "Leg.h"

/*
 *
 */
class Simulation {
private:
	std::vector<Leg> *legs;
public:
	Simulation();
	virtual ~Simulation();

	void AddLeg();
};

#endif /* SIMULATION_H_ */
