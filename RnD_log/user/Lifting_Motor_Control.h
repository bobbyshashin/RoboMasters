#include "main.h"
#include "function_list.h"
#define POWER_BUFFER_LENGTH 20
#define ANGLE_PID_LIMIT 500
#define MOVING_BOUND_1 200
#define MOVING_BOUND_2 450
#define SPEED_SETPOINT_LIMIT 800
#define UP_SETPOINT 255000						//determined by the height of the pneumatic, where pneumatice can be put on the stage precisely
#define DOWN_SETPOINT 1000//determined by the relative height between the pneumatic and the wheels, whe wheels should be put on the stage precisely
#define MID_SETPOINT 164000
#define TOTALLY_DOWN_SETPOINT 1000


extern int32_t LiftingMotorSpeedFeedback[4];
extern float LiftingMotorPositionFeedback[4];

extern int32_t LiftingMotorSpeedSetpoint[4];
extern int32_t LiftingMotorSpeedSetpointBuffered[4];


extern volatile int32_t LiftingMotorPositionSetpoint[4];
extern int32_t LiftingMotorSpeedSetpointBuffered[4];

extern volatile bool ONE_KEY_UP_FRONT;
extern volatile bool ONE_KEY_UP_BACK;
extern volatile bool ONE_KEY_DOWN_FRONT;
extern volatile bool ONE_KEY_DOWN_BACK;
extern volatile bool GO_ON_STAGE_ONE_KEY;		//become false when the process is finished, or by having received the break command
extern volatile bool GO_DOWN_STAGE_ONE_KEY;		//become false when the process is finished, or by having received the break command
extern volatile bool FRICTION_WHEEL_STATE;
extern bool GO_ON_STAGE_ONE_KEY_PREV;
extern bool GO_DOWN_STAGE_ONE_KEY_PREV;
extern volatile bool BREAK;
//extern u32 ticks_msimg_on_prev=0, ticks_msimg_down_prev=0;
extern struct fpid_control_states LiftingMotorPositionState[4];
extern float LMpos_kp;
extern float LMpos_ki;
extern float LMpos_kd;
extern uint16_t friction_wheel_setpoint;

//PID controls

//The control of filter rate of wheels
// Structure to strore PID data
extern struct pid_control_states LiftingMotorState[4];

extern int32_t LiftingMotorOutput[4];
extern int32_t kp, ki, kd;
void LiftingMotorInit();