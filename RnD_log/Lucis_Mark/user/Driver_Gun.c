#define GUN_FILE

#include "Dbus.h"
#include "Driver_Encoder.h"
#include "Driver_Gun.h"
#include "canBusProcess.h"
#include "PID.h"
#include <string.h>
#include <stdbool.h>
static PID_Controller PokeSpeedController;
static PID_Controller PokeAngleController;
static struct fpid_control_states gunPositionState = {0,0,0};
float gunpos_kp = 0.3;
float gunpos_ki = 0;
float gunpos_kd = 0;
int32_t gunSpeedKp = 80, gunSpeedKi = 4, gunSpeedKd = 1;
int32_t gunSpeedSetpoint=0;
float gunPositionSetpoint=0;
int32_t targetAngleBuffer=0;
float curr_angle=0;
static struct pid_control_states gunSpeedMoveState={0,0,0};// gimbalSpeedStaticState;
int32_t gunSpeedMoveOutput = 0;
int32_t gunSpeed=0;
bool dir=true;
int32_t curr_speed=0;
int32_t current_cummulated=0;
void dir_switch(bool* b){
	if(*b) *b=false;
	else *b=true;
}




void GUN_BSP_Init(void) {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;

    // Brush Motor
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_0;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);

    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_1;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_1);

    // Friction Wheel
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // TIM1 (friction wheel, 400Hz)
    TIM_TimeBaseInitStructure.TIM_ClockDivision =   TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   =   TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period        =   2500-1;
    TIM_TimeBaseInitStructure.TIM_Prescaler     =   (uint32_t) (((SystemCoreClock / 1) / 1000000)-1); // 1MHz clock
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode       =   TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_Pulse        =   1000;
    TIM_OCInitStructure.TIM_OutputState  =   TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState =   TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity   =   TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity  =   TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState  =   TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState =   TIM_OCNIdleState_Set;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    // TIM2 (brush motor, 1kHz)
    TIM_TimeBaseInitStructure.TIM_ClockDivision =   TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   =   TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period        =   12000-1;
    TIM_TimeBaseInitStructure.TIM_Prescaler     =   (uint32_t) (((SystemCoreClock / 2) / 12000000)-1); // 12MHz clock
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode       =   TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_Pulse        =   0;
    TIM_OCInitStructure.TIM_OutputState  =   TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity   =   TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

void GUN_Init(void) {
    GUN_BSP_Init();

    memset((char*)&GUN_Data, 0, sizeof(GUN_Data));

    PID_Reset(&PokeSpeedController);
    PokeSpeedController.Kp = 100.0f;
    PokeSpeedController.Ki = 20.00f;
    PokeSpeedController.Kd = 0.00f;
    PokeSpeedController.MAX_Pout = 12000;
    PokeSpeedController.MAX_Integral = 100000;
    PokeSpeedController.MAX_PIDout = 12000;
    PokeSpeedController.MIN_PIDout = 0;
    PokeSpeedController.mode = kPositional;

    PID_Reset(&PokeAngleController);
    PokeAngleController.Kp = 0.30f;
    PokeAngleController.Ki = 0.00f;
    PokeAngleController.Kd = 0.00f;
    PokeAngleController.MAX_Pout = 12000;
    PokeAngleController.MAX_Integral = 100000;
    PokeAngleController.MAX_PIDout = 80;
    PokeAngleController.MIN_PIDout = 0;
    PokeAngleController.mode = kPositional;
		
		//incPIDset(&gunSpeedMoveState, 50,3.7,1);
		
		
}

extern volatile u32 ticks_msimg;
void GUN_SetMotion(void) {
    static char shoot = 0;
    static char jumpPress = 0, jumpRelease = 0;
    static int32_t lastTick = 0;
    static int32_t pressCount = 0;

    // friction wheel
    if (DBUS_ReceiveData.rc.switch_right != 1) {
        FRIC_SET_THRUST_L(800);
        FRIC_SET_THRUST_R(800);
    }
    else {
        FRIC_SET_THRUST_L(0);
        FRIC_SET_THRUST_R(0);
    }

    // poke motor
    jumpPress = DBUS_ReceiveData.mouse.press_left &&
        !LASTDBUS_ReceiveData.mouse.press_left;
    jumpRelease = !DBUS_ReceiveData.mouse.press_left &&
        LASTDBUS_ReceiveData.mouse.press_left;
    if (jumpRelease) pressCount = 0;
    if (DBUS_ReceiveData.mouse.press_left) {
        ++pressCount;
    }

    shoot = jumpPress || (((pressCount & 0x000FU) == 0)&&pressCount);
    shoot = shoot && (DBUS_ReceiveData.rc.switch_right != 1);
    shoot = shoot && (ticks_msimg - lastTick > 220);
    if (shoot) {
        GUN_ShootOne();
        lastTick = ticks_msimg;
    }
}

void GUN_ShootOne(void) {
	
	
	if(dir)
    GUN_Data.pokeTargetAngle += 1368;
	else 
		GUN_Data.pokeTargetAngle -= 1368;
	if(GMballfeedEncoder.ecd_angle>16777151){
		GMballfeedEncoder.ecd_angle=GUN_Data.pokeTargetAngle=0;
	}
	else if(GMballfeedEncoder.ecd_angle<-16777151){
		GMballfeedEncoder.ecd_angle=GUN_Data.pokeTargetAngle=0;
	}
	//gunSpeedMoveState.cummulated_error=0;
	current_cummulated=0;
}

void GUN_PokeControl(void) {
			GUN_SetMotion();
			GUN_PokeSpeedControl();
			
}

void GUN_PokeSpeedControl(void) {
		current_cummulated *= 0.92;
		if(abs(gunSpeed)>4000)
			current_cummulated+=gunSpeed;
			
    if((int32_t)current_cummulated>200000) {
		
			gunPositionState.cummulated_error = 0;
			//gunPositionState.current_error = 0;
			//gunPositionState.last_error = 0;
		
			current_cummulated=0;
			gunSpeedMoveState.cummulated_error = 0;
			if(dir){
			targetAngleBuffer=GUN_Data.pokeTargetAngle;
			GUN_Data.pokeTargetAngle -= 1368;
			dir_switch(&dir);
			}
		}else if((int32_t)current_cummulated<-200000){
			gunPositionState.cummulated_error = 0;
			current_cummulated=0;
			gunSpeedMoveState.cummulated_error = 0;
			if(!dir){
			targetAngleBuffer=GUN_Data.pokeTargetAngle;
			GUN_Data.pokeTargetAngle += 1368;
			dir_switch(&dir);
			}
		}
		/*
		if(dir && targetAngleBuffer<GUN_Data.pokeTargetAngle){
			targetAngleBuffer+=30;
		}
		else if( !dir && targetAngleBuffer>GUN_Data.pokeTargetAngle){
			targetAngleBuffer-=30;
		}
		*/
		curr_angle=GMballfeedEncoder.ecd_angle;
		gunPositionSetpoint=GUN_Data.pokeTargetAngle;
		
		gunSpeedSetpoint = (int32_t)fpid_process(&gunPositionState,&gunPositionSetpoint, &curr_angle,gunpos_kp,gunpos_ki,gunpos_kd );
		gunPositionState.cummulated_error = 0;
		//gunSpeedSetpoint=300;
		//Limit the output
		if (gunSpeedSetpoint > 300) gunSpeedSetpoint = 300;
		else if (gunSpeedSetpoint < -300) gunSpeedSetpoint = -300;
		curr_speed=GMballfeedEncoder.filter_rate;
		spSp = current_cummulated;
		
		Cerror = targetAngleBuffer;
		pidLimitI(&gunSpeedMoveState,200000);
		gunSpeed=pid_process(&gunSpeedMoveState,&gunSpeedSetpoint,&curr_speed,gunSpeedKp,gunSpeedKi,gunSpeedKd);
	
		error=(int32_t)gunPositionState.cummulated_error;
		}
/*
void GUN_SetFree(void) {
    PID_Reset(&PokeSpeedController);
    PID_Reset(&PokeAngleController);

    GUN_Data.pokeOutput = 0;
    GUN_Data.pokeTargetSpeed = 0;
    GUN_Data.pokeTargetAngle = 0;
}

#include <string.h>
*/
/*
    Trim val to [-lim, lim]
*/
/*
static float PID_Trim(float val, float lim) {
    if (lim < 0.0f) return val;
    if (val < -lim) val = -lim;
    if (val > lim) val = lim;
    return val;
}
*/
/*
    Restore to initial state
*/

void PID_Reset(PID_Controller *pid) {
    memset(pid->set, 0, sizeof(pid->set));
    memset(pid->real, 0, sizeof(pid->real));
    memset(pid->err, 0, sizeof(pid->err));
    pid->errIntegral = 0.0f;
    pid->output = 0.0f;
}


/*
    Update PID controller and return output
*/
/*
float PID_Update(PID_Controller *pid, float target, float measure) {
    // update error
    pid->set[kNOW] = target;
    pid->real[kNOW] = measure;
    pid->err[kNOW] = target - measure;

    float Pout, Iout, Dout;
    if (pid->mode == kIncremental) {
        Pout = pid->Kp * (pid->err[kNOW] - pid->err[kLAST]);
        Iout = pid->Ki * pid->err[kNOW];
        Dout = pid->Kd * (pid->err[kNOW] - 2*pid->err[kLAST] + pid->err[kLLAST]);

        Iout = PID_Trim(Iout, pid->MAX_Integral * pid->Ki);
        pid->output += (Pout + Iout + Dout);
        pid->output = PID_Trim(pid->output, pid->MAX_PIDout);
    }
    else if (pid->mode == kPositional) {
        pid->errIntegral += pid->err[kNOW];
        pid->errIntegral = PID_Trim(pid->errIntegral, pid->MAX_Integral);

        Pout = pid->Kp * pid->err[kNOW];
        Iout = pid->Ki * pid->errIntegral;
        Dout = pid->Kd * (pid->err[kNOW] - pid->err[kLAST]);

        Pout = PID_Trim(Pout, pid->MAX_Pout);
        pid->output = Pout + Iout + Dout;
        pid->output = PID_Trim(pid->output, pid->MAX_PIDout);
    }
    else if (pid->mode == kIntegralDecay) {
        pid->errIntegral = pid->errIntegral * pid->IDecayFactor + pid->err[kNOW];
        pid->errIntegral = PID_Trim(pid->errIntegral, pid->MAX_Integral);

        Pout = pid->Kp * pid->err[kNOW];
        Iout = pid->Ki * pid->errIntegral;
        Dout = pid->Kd * (pid->err[kNOW] - pid->err[kLAST]);

        Pout = PID_Trim(Pout, pid->MAX_Pout);
        pid->output = Pout + Iout + Dout;
        pid->output = PID_Trim(pid->output, pid->MAX_PIDout);
    }

    pid->set[kLLAST] = pid->set[kLAST];
    pid->set[kLAST] = pid->set[kNOW];
    pid->real[kLLAST] = pid->real[kLAST];
    pid->real[kLAST] = pid->real[kNOW];
    pid->err[kLLAST] = pid->err[kLAST];
    pid->err[kLAST] = pid->err[kNOW];

    float ret = pid->output;
    float abs_ret = ret;
    if (abs_ret < 0.0f) abs_ret = -abs_ret;
    if (abs_ret < pid->MIN_PIDout) ret = 0.0f;
    return ret;
}
*/
