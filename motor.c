#include <stdio.h>
#include "daq.h"
#include "motor.h"


double y_now = 0;

void motor_init (double init_pos) {
	y_now = (y_now == 0) ? init_pos : y_now;
}

double move (double control_value, int dt) {
	double dt_sec = ((double)dt)/1000000;
	double xn = (double) control_value;
	double y_proportional = y_now;
	double y_derivative = (dt_sec)*(90.645*xn - y_now);
	y_derivative = y_derivative / 0.029;
	double y_next = y_proportional + y_derivative;

	/* rollover (servo motor position is absolute not relative) */
	while (y_next > MAX_IN_VALUE) y_next -= MAX_IN_VALUE;
//	if (y_next > MAX_IN_VALUE) y_next = MAX_IN_VALUE;
	while (y_next < 0) y_next += MAX_IN_VALUE;

//	printf ("dt_sec: %f, y_prop: %f, y_deriv: %f, y_next: %f\n",dt_sec,y_proportional, y_derivative, y_next);
	y_now = y_next;
	return y_now;
}
