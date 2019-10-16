/*
 * Leg.h
 *
 *  Created on: Apr 27, 2019
 *      Author: crash
 */

#ifndef LEG_H_
#define LEG_H_

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <unistd.h>
#include <eigen3/Eigen/Core>
#include <cairo.h>
#include <gtk/gtk.h>
#include <unistd.h>
#include <cmath>
#include <time.h>
#include <stdlib.h>
#include <stdint.h>

using namespace std;
using namespace KDL;

/*
 *
 */
class Leg {

private:
	vector<Frame> *p_out;
	vector<Frame> *initial_p_out;
	KDL::JntArray jointpositions;
	KDL::JntArray jointposOut;
	ChainFkSolverPos_recursive *fksolver;
	ChainIkSolverPos_NR *iksolver;
	KDL::Chain chain;
	KDL::Frame cartpos, startCartPos;
	ChainIkSolverVel_wdls *viksolver;
	int iterations = 0;
	bool forth = true;
	bool forthMove = false;

public:
	Leg();
	virtual ~Leg();

	vector<Frame> *getOutFrameVector() { return p_out; }
	vector<Frame> *getInitialOutFrameVector() { return initial_p_out; }
	KDL::Frame getCartPos() { return cartpos; }
	KDL::Frame getStartCartPos() { return startCartPos; }

	gboolean timeout(gpointer data);
	void setStartIterations(int it) {iterations = it; }
	void setForth(bool fo) { forth = fo; }
};

#endif /* LEG_H_ */
