int process_value (int in_from_pot, int set_point, int period, int print_message);
void task_code (int value);

#define VALUE_BUFFER_LENGTH 1000
#define PID_ACCURACY_FACTOR 100

extern int SCALED_FACTOR;

extern int Kp_scaled;   // 0.3   scaled up by 10 000
extern int Ki_scaled;   // 0.015 scaled up by 10 000
extern int Kd_scaled;   // 0.015 scaled up by 10 000

#define max(a,b) \
	({ __typeof__ (a) _a = (a); \
	__typeof__ (b) _b = (b); \
	_a > _b ? _a : _b; })

#define min(a,b) \
	({ __typeof__ (a) _a = (a); \
	__typeof__ (b) _b = (b); \
	_a < _b ? _a : _b; })

#define abs(a) \
	({ __typeof__ (a) _a = (a); \
	_a < 0 ? ((-1)*_a) : _a; })

int debug;
int pid_overflow [2];

typedef struct {
        int values [VALUE_BUFFER_LENGTH];
        int length;
} value_buffer_t;

