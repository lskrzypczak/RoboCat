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
#include "Leg.h"

using namespace std;
using namespace KDL;

#define _STRINGIFY(s) #s
#define STRINGIFY(s) _STRINGIFY(s)

static void do_drawing(cairo_t *);

cairo_t *cr = NULL;
Leg *l1, *l2, *l3, *l4;

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
//	cairo_move_to(cr, CENTER_X, CENTER_Y);
//	for (i = 0; i < l1->getInitialOutFrameVector()->size(); i++) {
//		KDL::Frame frame = static_cast<KDL::Frame>(l1->getInitialOutFrameVector()->at(i));
//		int x1 = frame.p.x()*100;
//		int y1 = frame.p.y()*100;
//
//		cairo_line_to(cr, CENTER_X + x1, CENTER_Y + y1);
//	}
//	cairo_stroke(cr);

	cairo_set_source_rgb(cr, 1, 0, 0);

	//draw current leg position
	cairo_move_to(cr, CENTER_X, CENTER_Y);
	for (i = 0; i < l1->getOutFrameVector()->size(); i++) {
		KDL::Frame frame = l1->getOutFrameVector()->at(i);
		int x1 = frame.p.x()*100;
		int y1 = frame.p.y()*100;

		cairo_line_to(cr, CENTER_X + x1, CENTER_Y + y1);
	}
	cairo_stroke(cr);

	//draw circle indicating starting cart position
//	cairo_set_source_rgb(cr, 0, 1, 1);
//	cairo_arc(cr, CENTER_X + l1->getStartCartPos().p.x() * 100, CENTER_Y + l1->getStartCartPos().p.y() * 100, 5, 0, 2 * M_PI);
//
//	cairo_stroke(cr);

	//draw circle indicating current cart position
//	cairo_set_source_rgb(cr, 0, 0, 1);
//	cairo_arc(cr, CENTER_X + l1->getCartPos().p.x() * 100, CENTER_Y + l1->getCartPos().p.y() * 100, 5, 0, 2 * M_PI);
//
//	cairo_stroke(cr);

	//draw current leg position
	cairo_move_to(cr, CENTER_X + 100, CENTER_Y);
	for (i = 0; i < l2->getOutFrameVector()->size(); i++) {
		KDL::Frame frame = l2->getOutFrameVector()->at(i);
		int x1 = frame.p.x()*100;
		int y1 = frame.p.y()*100;

		cairo_line_to(cr, CENTER_X + 100 + x1, CENTER_Y + y1);
	}
	cairo_stroke(cr);

	//draw current leg position
	cairo_move_to(cr, CENTER_X, CENTER_Y - 200);
	for (i = 0; i < l3->getOutFrameVector()->size(); i++) {
		KDL::Frame frame = l3->getOutFrameVector()->at(i);
		int x1 = frame.p.x()*100;
		int y1 = frame.p.y()*100;

		cairo_line_to(cr, CENTER_X + x1, CENTER_Y - 200 + y1);
	}
	cairo_stroke(cr);

	//draw current leg position
	cairo_move_to(cr, CENTER_X + 100, CENTER_Y - 200);
	for (i = 0; i < l4->getOutFrameVector()->size(); i++) {
		KDL::Frame frame = l4->getOutFrameVector()->at(i);
		int x1 = frame.p.x()*100;
		int y1 = frame.p.y()*100;

		cairo_line_to(cr, CENTER_X + 100 + x1, CENTER_Y - 200 + y1);
	}
	cairo_stroke(cr);

}

gboolean timeout(gpointer data) {
	l1->timeout(data);
	l2->timeout(data);
	l3->timeout(data);
	l4->timeout(data);
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

	l1 = new Leg();
	l2 = new Leg();
	l2->setStartIterations(50);
	l3 = new Leg();
	l3->setForth(false);
	l4 = new Leg();
	l4->setStartIterations(50);
	l4->setForth(false);

	gtk_widget_show(darea);
	g_timeout_add(50, timeout, window);

	gtk_widget_show_all(window);

	gtk_main();

	delete l1;
	delete l2;
	delete l3;
	delete l4;

	return 0;
}
