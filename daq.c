#include "motor.h"
#include "daq.h"
#include <stdio.h>

double position = 0;

void put_value (int value, int dt) {
	position = move ((double) (value-(MAX_OUT_VALUE/2)), dt);
}
double get_value (void) {
	return position;
}
