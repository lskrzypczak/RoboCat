/*
 * Leg.cpp
 *
 *  Created on: Apr 27, 2019
 *      Author: crash
 */

#include "../include/Leg.h"

pthread_mutex_t p_mutex = PTHREAD_MUTEX_INITIALIZER;
int trajectoryStep = 0;

void *worker( void *ptr ) {
	double x, y, z;

	pthread_mutex_lock( &p_mutex );

	x = startCartPos.p.x() + 1.0 - ((float)iterations / 100.0);
	y = startCartPos.p.y();
	z = startCartPos.p.z();

	Frame F_dest = Frame(Vector(x, y, z));
	cartpos = F_dest;

	if (iksolver == nullptr) {
		cout << "iksolver NULL" << endl;
		return false;
	}

	int error = iksolver->CartToJnt(jointpositions, F_dest, jointposOut);
	if (error < 0)
		cout << "Error: " << iksolver->strError(error) << endl;
	else
		fksolver->JntToCart(jointposOut, *p_out);
	jointpositions = jointposOut;

	usleep(1000);	//1ms time granulation

	pthread_mutex_unlock( &p_mutex );
}

Leg::Leg() {
	// TODO Auto-generated constructor stub
	int threadRet;
	//Definition of a kinematic chain & add segments to the chain
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0, 1.0, 0.0))));
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0, 0.75, 0.0))));

	// Create joint array
	unsigned int nj = chain.getNrOfJoints();
	jointpositions = JntArray(nj);
	jointpositions(0) = -M_PI / 9.0;
	jointpositions(1) = M_PI / 3.0;
	jointposOut = JntArray(nj);

	// Create the frame that will contain the results
	p_out = new vector<Frame>(chain.getNrOfSegments());
	initial_p_out = new vector<Frame>(chain.getNrOfSegments());

	// Create solver based on kinematic chain
	fksolver = new ChainFkSolverPos_recursive(chain);
	viksolver = new ChainIkSolverVel_wdls(chain, 100.0, 150);
	Eigen::MatrixXd matrix_Mx = Eigen::MatrixXd::Identity(6,6);
	matrix_Mx(3,3) = 0; matrix_Mx(4,4) = 0; matrix_Mx(5,5) = 0;
	viksolver->setWeightTS(matrix_Mx);
	iksolver = new ChainIkSolverPos_NR(chain, *fksolver, *viksolver, 100, 1.0);
	fksolver->JntToCart(jointpositions, startCartPos);
	fksolver->JntToCart(jointpositions, *initial_p_out);

	threadRet = pthread_create( &p_thread, NULL, worker, NULL );
}

Leg::~Leg() {
	// TODO Auto-generated destructor stub
	pthread_cancel(p_thread);

	delete iksolver;
	delete fksolver;
	delete viksolver;
}

void Leg::go(std::vector<KDL::Frame> trajectory, MoveSpeed speed ) {
	startCartPos = p_out->back();	//get last position of a cart (end of leg) & assignit as a start position for the next move
	p_trajectory = trajectory;
	p_speed = speed;
	p_moving = true;
}

bool Leg::timeout()
{
	/*
	const float step = 180.0 / 100;
	double x, y, z;

	//Set destination frame
	if (forth) {
		float rad = ((float)iterations) * step * M_PI / 180;
		x = startCartPos.p.x() + ((float)iterations / 100.0);
		y = startCartPos.p.y() - (0.25 * sin(rad));
		z = startCartPos.p.z();
	}
	else {
		x = startCartPos.p.x() + 1.0 - ((float)iterations / 100.0);
		y = startCartPos.p.y();
		z = startCartPos.p.z();
	}
	Frame F_dest = Frame(Vector(x, y, z));
	cartpos = F_dest;

	if (iksolver == nullptr) {
		cout << "iksolver NULL" << endl;
		return false;
	}

	int error = iksolver->CartToJnt(jointpositions, F_dest, jointposOut);
	if (error < 0)
		cout << "Error: " << iksolver->strError(error) << endl;
	else
		fksolver->JntToCart(jointposOut, *p_out);
	jointpositions = jointposOut;


	if ( forth ? iterations+=2 : iterations++ == 100) {
		iterations = 0;
//		jointpositions(0) = -M_PI / 9.0;
//		jointpositions(1) = M_PI / 3.0;
		forth = !forth;
	}
*/
    return TRUE;
}


