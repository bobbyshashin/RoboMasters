#include "stdint.h"
#ifndef PID
#define PID
/*This is the struct that record the cumulated error, current error
and last error to record the 

*/

/*
Note: 
1, the "int16_t" type is "signed 16bits integer type", this is included in stdint.h
Usage:
1, declare a structure in type in pid_control_state
outside the main function, this is used to keep the record
of pid process
2, declare two integers to store the reference and feedback 
3, use the pid_process in the main loop, it will get the 
output of pid and adjust the data in pid_control_states from
error
4, the function below are for the increment pid control, almost the same
*/



//you have to initailize the value in this structure, 
struct pid_control_states {
	int32_t cummulated_error;
	int32_t current_error;
	int32_t last_error;
};

struct fpid_control_states {
	float cummulated_error;
	float current_error;
	float last_error;
};

extern int16_t pid_process(struct pid_control_states* states, int32_t* setpoint, int32_t* feedback, int32_t kp, int32_t ki, int32_t kd);

extern float fpid_process(struct fpid_control_states* states, int32_t* setpoint, float* feedback, float kp, float ki, float kd ); 

extern int16_t pid_process_gai1(struct pid_control_states* states, int32_t* setpoint, int32_t* feedback, int32_t kp, int32_t ki, int32_t kd);






struct inc_pid_states {
	signed int setpoint;
	float kp;
	float ki;
	float kd;
	int sum_error;
	//e[-1]
	int last_error;
	//e[-2]
	int prev_error;
};

extern void incPIDinit(struct inc_pid_states * state_ptr );

extern void incPIDset(struct inc_pid_states * states_ptr, float kp, float ki, float kd);

extern float incPIDcalc (struct inc_pid_states * state_ptr, signed int nextpoint);

extern void incPIDsetpoint (struct inc_pid_states * state_ptr, signed int setvalue);

extern void incPIDClearError(struct inc_pid_states * state_ptr);

#endif