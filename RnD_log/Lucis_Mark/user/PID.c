#include "PID.h"
int16_t pid_process( struct pid_control_states* states, int32_t* setpoint, int32_t* feedback, int32_t kp, int32_t ki, int32_t kd){

		//first update current state
	int32_t ceLimit = 200000;
	states->last_error	= states->current_error;
	states->current_error = *setpoint - *feedback;
	states->cummulated_error += states->current_error;
	if (states->cummulated_error>ceLimit) states->cummulated_error =ceLimit;
	else if(states->cummulated_error<-ceLimit) states->cummulated_error=-ceLimit;
	
		//then return the output value
	int32_t output = kp* 	states->current_error 
									+ki* states->cummulated_error 
									+kd* (states->current_error-states->last_error);
		
	return output;
	
}	

float fpid_process( struct fpid_control_states* states, int32_t* setpoint, float* feedback, float kp, float ki, float kd){

		//first update current state 
	states->last_error	= states->current_error;
	states->current_error = *setpoint - *feedback;
	states->cummulated_error += states->current_error;
	
	int32_t ceLimit = 2000;
	if (states->cummulated_error>ceLimit) states->cummulated_error =ceLimit;
	else if(states->cummulated_error<-ceLimit) states->cummulated_error=-ceLimit;
	
	
		//then return the output value
	float output = kp* 	states->current_error 
									+ki* states->cummulated_error 
									+kd* (states->current_error-states->last_error);
		
	return output;
	
}	




int16_t pid_process_gai1( struct pid_control_states* states, int32_t* setpoint, int32_t* feedback, int32_t kp, int32_t ki, int32_t kd ) {
	states->last_error	= states->current_error;
	states->current_error = *setpoint - *feedback;
	states->cummulated_error *= 0.90;
	states->cummulated_error += states->current_error;
	
	int32_t output = kp* 	states->current_error 
									+ki* states->cummulated_error 
									+kd* (states->current_error-states->last_error);

	return output;
}

void incPIDinit(struct inc_pid_states * state_ptr ) {
	state_ptr->sum_error = 0;
	state_ptr->last_error = 0;
	state_ptr->prev_error = 0;
	state_ptr->kp = 0;
	state_ptr->ki = 0;
	state_ptr->kd = 0;
	state_ptr->setpoint = 0;
}


void incPIDset(struct inc_pid_states * states_ptr, float kp, float ki, float kd) {
	states_ptr -> kp = kp;
	states_ptr -> ki = ki;
	states_ptr -> kd = kd;
}


float incPIDcalc (struct inc_pid_states * state_ptr, signed int nextpoint) {
	int iError;
	float iincpid;
	iError = state_ptr->setpoint - nextpoint;
	//calculate the result 
	iincpid = 
	state_ptr->kp * (iError - state_ptr->last_error) +
	state_ptr->ki * iError +
	state_ptr->kd * (iError-2*state_ptr->last_error+state_ptr->prev_error);
	//save the data for the next calculation
	state_ptr->prev_error = state_ptr->last_error;
	state_ptr->last_error = iError;
	return iincpid;
}

void incPIDsetpoint (struct inc_pid_states * state_ptr, signed int setvalue) {
		state_ptr->setpoint = setvalue;
}


void incPIDClearError(struct inc_pid_states * state_ptr) {
	state_ptr->sum_error = 0;
	state_ptr->last_error = 0;
	state_ptr->prev_error = 0;
}









