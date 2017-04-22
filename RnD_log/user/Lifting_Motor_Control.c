#include "Lifting_Motor_Control.h"

void LiftingMotorInit(){
	GO_ON_STAGE_ONE_KEY=false;		//become false when the process is finished, or by having received the break command
	GO_DOWN_STAGE_ONE_KEY=false;		//become false when the process is finished, or by having received the break command
	FRICTION_WHEEL_STATE=0;
	GO_ON_STAGE_ONE_KEY_PREV=false;
	GO_DOWN_STAGE_ONE_KEY_PREV=false;
	BREAK=false;
//	ticks_msimg_on_prev=0, ticks_msimg_down_prev=0;
	for(uint8_t i=0;i<4;i++){
		LiftingMotorPositionState[i].cummulated_error=0;
		LiftingMotorPositionState[i].current_error=0;
		LiftingMotorPositionState[i].last_error=0;
	}
	LMpos_kp = 0.3;
	LMpos_ki = 0.0005;
	LMpos_kd = 20;
	friction_wheel_setpoint=0;

//PID controls

//The control of filter rate of wheels
// Structure to strore PID data
	for(uint8_t i=0;i<4;i++){
		LiftingMotorState[i].cummulated_error=0;
		LiftingMotorState[i].current_error=0;
		LiftingMotorState[i].last_error=0;
	}


	for(uint8_t i=0;i<4;i++){
		LiftingMotorOutput[i]=0;
	}
	kp = 80;
	ki = 4;
	kd = 1;
}