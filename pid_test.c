#include <stdio.h>
#include <stdlib.h>
#include "daq.h"
#include "pid.h"
#include <time.h>

#define TIME_INTERVAL_NS 9000
#define MOTOR_SIMULATION_FACTOR 10
#define MOTOR_START_POSITION 0	/* degrees */
#define NEW_SETPOINT 100
#define MAX_SIM_SIZE 9000000

int degreesFromRange (int position);
int simulate_time (int i);

int debug = 0;
/* Not needed */
struct time_slice {
	int slice_num;
	int pid_out;
	int pid_ovfl [2];
	int set_point;
	int control_saturation;
	double motor_pos;
};
struct time_slice ** sim_data;
/** **/
int old_position;
int ctrl_out_saturate = 0;
int set;
int degrees;
int max_value;
int print_interval;
int SET_POINT;
int overshoot = 0;
int period;

int main (int argc, char ** argv) {
/** Change **/
	int SIMULATION_SIZE = 30;
	SET_POINT = NEW_SETPOINT;
	printf ("\n\nStarting Simulation...\n");
/** **/
	period = TIME_INTERVAL_NS / 1000;
	if (argc > 1) {
		int i;
		int new_size = 0;
		int new_set = 0;
		for (i=1;i<argc;i++) {
			if (!strcmp (argv[i],"debug")) debug = 1;
			else {
				if (new_size == 0) {
					new_size = atoi(argv[i]);
					if (new_size >= SIMULATION_SIZE && new_size <= MAX_SIM_SIZE) {
						SIMULATION_SIZE = new_size;
					} else {
						printf("Input out of range: %i, use simulation size [%i - %i]\n",new_size,SIMULATION_SIZE,MAX_SIM_SIZE);
						return -1;
					}
					continue;
				}
				if (new_set == 0) {
					int new_set = atoi(argv[i]);
					if (new_set < 0) {
						printf("Input out of range: %i, set point cannot be negative\n",new_set);
						return -1;
					}
					if (new_set <= MOTOR_RANGE_DEGREES) {
						new_set = rangeFromDegrees (new_set);
						printf ("interpreting input as degrees. Control will use %i deg, value %i\n",degreesFromRange(new_set),new_set);
					} else {
						printf ("interpreting input as raw value. Control will use %i (%i degrees)\n",new_set,degreesFromRange(new_set));
					}
					SET_POINT = new_set;
					continue;
				}
			}
		}
	}
	if (TIME_INTERVAL_NS / 1000 == 0) {
		printf ("Value out of range: Simulation time slice is %i ns, must be greater than 1000 ns\n",TIME_INTERVAL_NS);
		return -1;
	}
	printf ("Using a simulation size of: %i with period %i ms. Total duration ~ %i ms\n", SIMULATION_SIZE, period, SIMULATION_SIZE*period);
	set = rangeFromDegrees (MOTOR_START_POSITION);

//	int set = 0;
	int i;
	old_position = (rangeFromDegrees (MOTOR_START_POSITION));
//	int old_position = 0;
	motor_init ((double) old_position);
	old_position *= PID_ACCURACY_FACTOR;
	printf ("Motor initial position %i\n", old_position);
	print_interval = (SIMULATION_SIZE > 30) ? (SIMULATION_SIZE / 30) : 1;
	if (SET_POINT > MAX_OUT_VALUE) {
		printf ("Set Point out of range: %i valid range is [%i - %i]\n",SET_POINT,0,MAX_OUT_VALUE);
		return -1;
	}
	max_value = 0;
	degrees = 0;
	ctrl_out_saturate = 0;
	sim_data = (struct time_slice **) malloc (SIMULATION_SIZE * sizeof(struct time_slice));
	for (i=0;i<SIMULATION_SIZE;i++) {
		sim_data [i] = (struct time_slice *) malloc (sizeof(struct time_slice));
	}
	for (i=0;i<SIMULATION_SIZE;i++) {
		simulate_time(i);
	}
	for (i=0;i<SIMULATION_SIZE;i++) {
		if (i % print_interval == 0) {
			struct time_slice slice = *(sim_data [i]);
			printf ("Time Slice %6i ms : set_point: %10i,\t PID_output: %15i ,\t motor_position: %15f,\t deg: %i",(slice.slice_num*period),slice.set_point,slice.pid_out,slice.motor_pos,degreesFromRange (slice.motor_pos));
			printf ("\t%s %s %s\n", ((slice.pid_ovfl [0]) ? "p" : " ") , ((slice.pid_ovfl [1]) ? "i" : " "),((slice.control_saturation) ? "sat" : "   "));
		}
	}
	if (ctrl_out_saturate) printf ("Control output saturated %i times\n",ctrl_out_saturate);
//	int overshoot = (max_value >degreesFromRange( SET_POINT)) ? (max_value - degreesFromRange(SET_POINT)) : 0;
	printf ("Simulation Completed: Angle Desired: %i , Final angle: %i, Overshoot of %i degrees, max angle: %i\n",degreesFromRange(SET_POINT),degrees,degreesFromRange(overshoot),max_value);
	return 0;
}
int simulate_time (int i) {
	int j;
	int old_pid_overflow [2] = {pid_overflow[0] , pid_overflow[1]};
	int iteration_contrl_saturation = 0;
	if (i==6) set = (int) SET_POINT;

	int motion_direction = 0;
	int motor_period = (int) period / MOTOR_SIMULATION_FACTOR;
	if (motor_period <= 0) motor_period = 1;
	int control_output = process_value (old_position, set * PID_ACCURACY_FACTOR, period,  debug);// / PID_ACCURACY_FACTOR ; // * PID_ACCURACY_FACTOR;
	if (control_output >= MAX_OUT_VALUE) {
		ctrl_out_saturate ++;
		if (debug) printf ("Control output too large: %i valid range [0 - %i], will cut off\n",control_output,MAX_OUT_VALUE);
		control_output = MAX_OUT_VALUE;
		iteration_contrl_saturation = 1;
	} else if (control_output < 0) {
		if (debug) printf ("Control output too small: %i valid range [0 - %i], will cut off\n",control_output,0);
		control_output = 0;
		iteration_contrl_saturation = -1;
	}
	double motor_position = 0;
	for (j=0;j<MOTOR_SIMULATION_FACTOR;j++) {
		put_value (control_output, motor_period);
		motor_position = get_value ();
	}
	int direction_of_motion = ((int) (motor_position * PID_ACCURACY_FACTOR)) - old_position; // if > 0 ,clockwise else counter clockwise
	if (direction_of_motion < 2*PID_ACCURACY_FACTOR) direction_of_motion = 0;

	if (direction_of_motion < 0 && control_output < ((MAX_OUT_VALUE/2)-PID_ACCURACY_FACTOR)) {
		overshoot = max(overshoot,abs(set - motor_position));
		overshoot = 11;
	} else if (direction_of_motion > 0 && control_output > ((MAX_OUT_VALUE/2) + PID_ACCURACY_FACTOR)) {
		//overshoot = max(overshoot,(set - motor_position));
	}
	old_position = (int) (motor_position * PID_ACCURACY_FACTOR);
	degrees = degreesFromRange (motor_position);
	max_value = max(max_value,degrees);
	int prop_ovfl = pid_overflow [0] > old_pid_overflow [0];
	int intg_ovfl = pid_overflow [1] > old_pid_overflow [1];
	if (i % print_interval == 0) {
		if (debug) {
			printf ("Time Slice i=%5i : set_point: %10i,\t PID_output: %15i ,\t motor_position: %15f,\t deg: %i",i,set,control_output,motor_position,degrees);
			printf ("\t%s %s\n", ((prop_ovfl) ? "p ovfl" : " ") , ((intg_ovfl) ? "i ovfl" : " "));
		}
	}
	struct time_slice * slice = sim_data [i];
	slice->slice_num = i;
	slice->pid_ovfl [0] = prop_ovfl;
	slice->pid_ovfl [1] = intg_ovfl;
	slice->pid_out = control_output;
	slice->set_point = set;
	slice->motor_pos = motor_position;
	slice->control_saturation = iteration_contrl_saturation;

	struct timespec requested={0,TIME_INTERVAL_NS}, remaining={0,0};
	nanosleep(&requested,&remaining);
}
int degreesFromRange (int position) {
	return (position*MOTOR_RANGE_DEGREES) / MAX_OUT_VALUE;
}
int rangeFromDegrees (int degrees) {
	// Looks crazy, but is necessary to be in the middle of desired degree
	if (degrees == 0) return 0;
	return (degrees * MAX_OUT_VALUE)/ ((MOTOR_RANGE_DEGREES)) + ((MAX_OUT_VALUE/MOTOR_RANGE_DEGREES)/2);
}
/*void motor_init (void) {
	int i;
	for (i=0;i<3;i++)
		move (0,9);
}*/
