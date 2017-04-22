#include "Lifting_Motor_Control.h"

void readFeedback();
void speedProcess();
void setSetpoint();

int32_t LiftingMotorSpeedFeedback[4];
float LiftingMotorPositionFeedback[4];
volatile int32_t LiftingMotorPositionSetpoint[4];
int32_t LiftingMotorSpeedSetpoint[4];
int32_t LiftingMotorSpeedSetpointBuffered[4];
volatile bool ONE_KEY_UP_FRONT;
volatile bool ONE_KEY_UP_BACK;
volatile bool ONE_KEY_DOWN_FRONT;
volatile bool ONE_KEY_DOWN_BACK;
volatile bool GO_ON_STAGE_ONE_KEY;		//become false when the process is finished, or by having received the break command
volatile bool GO_DOWN_STAGE_ONE_KEY;		//become false when the process is finished, or by having received the break command
volatile bool FRICTION_WHEEL_STATE;
bool GO_ON_STAGE_ONE_KEY_PREV;
bool GO_DOWN_STAGE_ONE_KEY_PREV;
volatile bool BREAK;
//extern u32 ticks_msimg_on_prev=0, ticks_msimg_down_prev=0;
struct fpid_control_states LiftingMotorPositionState[4];
float LMpos_kp;
float LMpos_ki;
float LMpos_kd;
uint16_t friction_wheel_setpoint;

//PID controls

//The control of filter rate of wheels
// Structure to strore PID data
struct pid_control_states LiftingMotorState[4];

int32_t LiftingMotorOutput[4];
int32_t kp, ki, kd;
