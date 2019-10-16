/*
 * Leg.cpp
 *
 *  Created on: Apr 27, 2019
 *      Author: crash
 */

#include "../include/Leg.h"

Leg::Leg() {
	// TODO Auto-generated constructor stub
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
	//ChainFkSolverPos_recursive fksolver_s = ChainFkSolverPos_recursive(chain);
	fksolver = new ChainFkSolverPos_recursive(chain);
	//ChainIkSolverVel_wdls viksolver = ChainIkSolverVel_wdls(chain, 100.0, 150);
	viksolver = new ChainIkSolverVel_wdls(chain, 100.0, 150);
	Eigen::MatrixXd matrix_Mx = Eigen::MatrixXd::Identity(6,6);
	matrix_Mx(3,3) = 0; matrix_Mx(4,4) = 0; matrix_Mx(5,5) = 0;
	viksolver->setWeightTS(matrix_Mx);
//	static ChainIkSolverPos_NR iksolver_s = ChainIkSolverPos_NR(chain, fksolver_s, viksolver, 100, 1.0);
//	fksolver = &fksolver_s;
//	iksolver = &iksolver_s;
	iksolver = new ChainIkSolverPos_NR(chain, *fksolver, *viksolver, 100, 1.0);
	fksolver->JntToCart(jointpositions, startCartPos);
	fksolver->JntToCart(jointpositions, *initial_p_out);
}

Leg::~Leg() {
	// TODO Auto-generated destructor stub
	delete iksolver;
	delete fksolver;
	delete viksolver;
}

gboolean Leg::timeout(gpointer data)
{
	const float step = 180.0 / 100;
	double x, y, z;

	//Set destination frame
	if ((forth) && (forthMove)) {
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


	if (iterations++ == 100) {
		iterations = 0;
//		jointpositions(0) = -M_PI / 9.0;
//		jointpositions(1) = M_PI / 3.0;
		if ((forth) && (iterations > 25) && (iterations < 75))
			forthMove = true;
		else
			forthMove = false;
		forth = !forth;
	}

    GtkWidget *widget = GTK_WIDGET(data);
    if (!widget->window) return TRUE;
    gtk_widget_queue_draw(widget);
    return TRUE;
}
