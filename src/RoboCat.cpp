//============================================================================
// Name        : RoboCat.cpp
// Author      : Lukasz Skrzypczak
// Version     :
// Copyright   : slk@data.pl
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include "../include/servo.h"
#include "../include/servoController.h"

#include "version.h"
#include <cmath>
#include <time.h>
#include <stdlib.h>
#include <stdint.h>
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

using namespace std;
using namespace KDL;

#define _STRINGIFY(s) #s
#define STRINGIFY(s) _STRINGIFY(s)

static void do_drawing(cairo_t *);

vector<Frame> *p_out;
vector<Frame> *initial_p_out;
KDL::JntArray jointpositions;
KDL::JntArray jointposOut;
ChainFkSolverPos_recursive *fksolver;
ChainIkSolverPos_NR *iksolver;
cairo_t *cr = NULL;
KDL::Frame cartpos, startCartPos;
int iterations = 0;
bool forth = true;

static gboolean window_expose_event (GtkWidget      *widget,
        GdkEventExpose *event)
{
	cr = gdk_cairo_create (widget->window);
	do_drawing(cr);

	return FALSE;
}

#define CENTER_X	200
#define CENTER_Y	200

static void do_drawing(cairo_t *cr)
{
	int i;

	cairo_set_source_rgb(cr, 0, 1, 0);
	cairo_set_line_width(cr, 2);

	//draw initial leg position
	cairo_move_to(cr, CENTER_X, CENTER_Y);
	for (i = 0; i < initial_p_out->size(); i++) {
		KDL::Frame frame = static_cast<KDL::Frame>(initial_p_out->at(i));
		int x1 = frame.p.x()*100;
		int y1 = frame.p.y()*100;

		cairo_line_to(cr, CENTER_X + x1, CENTER_Y + y1);
	}
	cairo_stroke(cr);

	cairo_set_source_rgb(cr, 1, 0, 0);

	//draw current leg position
	cairo_move_to(cr, CENTER_X, CENTER_Y);
	for (i = 0; i < p_out->size(); i++) {
		KDL::Frame frame = p_out->at(i);
		int x1 = frame.p.x()*100;
		int y1 = frame.p.y()*100;

		cairo_line_to(cr, CENTER_X + x1, CENTER_Y + y1);
	}
	cairo_stroke(cr);

	//draw circle indicating starting cart position
	cairo_set_source_rgb(cr, 0, 1, 1);
	cairo_arc(cr, CENTER_X + startCartPos.p.x() * 100, CENTER_Y + startCartPos.p.y() * 100, 5, 0, 2 * M_PI);

	cairo_stroke(cr);

	//draw circle indicating current cart position
	cairo_set_source_rgb(cr, 0, 0, 1);
	cairo_arc(cr, CENTER_X + cartpos.p.x() * 100, CENTER_Y + cartpos.p.y() * 100, 5, 0, 2 * M_PI);

	cairo_stroke(cr);

}

gboolean timeout(gpointer data)
{
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
		forth = !forth;
	}

    GtkWidget *widget = GTK_WIDGET(data);
    if (!widget->window) return TRUE;
    gtk_widget_queue_draw(widget);
    return TRUE;
}


int main( int   argc,
		char *argv[] ) {
	GtkWidget *window;
	GtkWidget *darea;

	gtk_init(&argc, &argv);

	window = gtk_window_new(GTK_WINDOW_TOPLEVEL);

	darea = gtk_drawing_area_new();
	gtk_container_add(GTK_CONTAINER(window), darea);

	g_signal_connect(G_OBJECT(darea), "expose-event",
			G_CALLBACK(window_expose_event), NULL);

	g_signal_connect(window, "destroy",
			G_CALLBACK(gtk_main_quit), NULL);

	gtk_window_set_position(GTK_WINDOW(window), GTK_WIN_POS_CENTER);
	gtk_window_set_default_size(GTK_WINDOW(window), 400, 400);
	gtk_window_set_title(GTK_WINDOW(window), "RoboCat");

	cout << "Hello RoboCat v" << STRINGIFY(ROBOCAT_VERSION_MAJOR) << "." << STRINGIFY(ROBOCAT_VERSION_MINOR) << endl;

	cout << "Initializing servo controller" << endl;

	servoController *ctrl = new servoController();
	servo *servos[16];
	for (int i = 0; i < 16; i ++) {
		servos[i] = new servo(ctrl, i);
		servos[i]->setAngle(0.0);
		delete servos[i];
	}

	delete ctrl;

	//Definition of a kinematic chain & add segments to the chain
	KDL::Chain chain;
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
	ChainFkSolverPos_recursive fksolver_s = ChainFkSolverPos_recursive(chain);
	ChainIkSolverVel_wdls viksolver = ChainIkSolverVel_wdls(chain, 100.0, 150);
	Eigen::MatrixXd matrix_Mx = Eigen::MatrixXd::Identity(6,6);
	matrix_Mx(3,3) = 0; matrix_Mx(4,4) = 0; matrix_Mx(5,5) = 0;
	viksolver.setWeightTS(matrix_Mx);
	ChainIkSolverPos_NR iksolver_s = ChainIkSolverPos_NR(chain, fksolver_s, viksolver, 100, 1.0);
	fksolver = &fksolver_s;
	iksolver = &iksolver_s;
	fksolver_s.JntToCart(jointpositions, startCartPos);
	fksolver_s.JntToCart(jointpositions, *initial_p_out);

	gtk_widget_show(darea);
	g_timeout_add(50, timeout, window);

	gtk_widget_show_all(window);

	gtk_main();


	return 0;
}
