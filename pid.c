#include <stdio.h>
#include "pid.h"
#include "daq.h"

int integral (void);
int derivative (int period);
int calculate_error (void);

int SCALED_FACTOR = 10000;
int Kp_scaled = 3000;
int Ki_scaled = 1;
int Kd_scaled = 9000;

int integral_component = 0;
value_buffer_t e;


int process_value (int in_from_pot, int set_point, int period, int print_message) {
	// Shift the buffer, current element at index 0
	int i;
	for (i=VALUE_BUFFER_LENGTH-1;i>0;i--) {
		e.values [i] = e.values[i-1];
	}
	int max    = MAX_IN_VALUE * PID_ACCURACY_FACTOR;
	int error2 = set_point - in_from_pot;
	int error1 = set_point - in_from_pot - max;
	int error3 = max - in_from_pot + set_point;

	e.values[0] = error2;
	if (abs(error1) < abs(e.values[0])) e.values[0] = error1;
	if (abs(error3) < abs(e.values[0])) e.values[0] = error3;

	e.length++;

	if (e.length > VALUE_BUFFER_LENGTH) e.length = VALUE_BUFFER_LENGTH;

	/** proportional **/
	int p_scaled = Kp_scaled * e.values[0];
	int ovflw_prop = 0;
	if (p_scaled / Kp_scaled != e.values[0]) {
		ovflw_prop = 1;
		if (print_message) printf ("Overflow occured in the proportional term! %i",p_scaled);
		p_scaled = Kp_scaled * (e.values[0]/SCALED_FACTOR);
		if (print_message) printf (" , fixed: %i\n",p_scaled);
		pid_overflow [0] ++;
	}

	/** integral **/
	int i_scaled = (Ki_scaled * period * e.values[0]);
	if (i_scaled / (Ki_scaled*period) != e.values[0]) {
		if (print_message) printf ("Overflow occured in the integral term! %i",i_scaled);
		i_scaled = ((period*e.values[0])/SCALED_FACTOR) * Ki_scaled;
		if (print_message) printf (" , fixed: %i\n",i_scaled);
		integral_component += i_scaled;
		pid_overflow [1] ++;
	}
	else	integral_component += i_scaled / SCALED_FACTOR;

	/** derivative **/
	int d_scaled = derivative (period) / 100;


	if (print_message) printf ("error: %i, period: %i,  int_from_pot: %i, setpoint: %i,  p_scaled: %i, i_unscaled: %i, d_scaled: %i\n",e.values[0],period,in_from_pot,set_point,p_scaled,integral_component,d_scaled);
	int ret_val = 0;
	if (!ovflw_prop) {
		ret_val = p_scaled + d_scaled;
		ret_val = ret_val / SCALED_FACTOR;
	}
	else ret_val = p_scaled + (d_scaled / SCALED_FACTOR);
	ret_val += integral_component;

	ret_val += (MAX_OUT_VALUE / 2);		// scaled since "0 out" is actually half of the range of the channel and neg vals are [0 - (1/2)range]
	return ret_val;
}
int calculate_error (void) {
}
int derivative (int period) {
//	if (e.length < 2) return 0;
	int numerator = e.values [0] - e.values [1];
	numerator = numerator * 100 * Kd_scaled;
	int ret_val = numerator / (period);
	return ret_val;
}
