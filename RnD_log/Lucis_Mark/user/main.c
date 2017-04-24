#include "main.h"
#include "function_list.h"
#include "Lifting_Motor_Control.h"


#define POWER_BUFFER_LENGTH 20
#define ANGLE_PID_LIMIT 500
#define MOVING_BOUND_1 200
#define MOVING_BOUND_2 450
#define SPEED_SETPOINT_LIMIT 800
#define UP_SETPOINT 255000						//determined by the height of the pneumatic, where pneumatice can be put on the stage precisely
#define DOWN_SETPOINT 1000//determined by the relative height between the pneumatic and the wheels, whe wheels should be put on the stage precisely
#define MID_SETPOINT 164000
#define TOTALLY_DOWN_SETPOINT 1000
static u32 ticks_msimg = (u32)-1;

void init(){
//	InfantryJudge.LastBlood = 1500;
	SysTick_Init();  
	//Dbus_init();//usart1
	//buzzer_init();	 //initialization of buzzer
	tft_init(2,WHITE,BLACK,BLACK);
	LED_master_init();
	DataMonitor_Init();
	gyro_init();
	ADC1_init();
	//judging_system_init(); //usart3
	Bilateral_Init();
	gyro_init();
	pneumatic_init();
	CAN1_Configuration();
	CAN2_Configuration();
	gyro_init();
	gyro_cal();
	//Friction_wheel_init();
	GUN_Init();
	TIM5_Int_Init(24,13124);// 256hz //3.9xx ms for gyro usage
	LiftingMotorInit();
}









//PID controls

//The control of filter rate of wheels 
// Structure to strore PID data 

int main(void)
{	
	init();
//	DBUS_ReceiveData.mouse.ytotal=0;
	
	
	
	
	while (1)  
	{	


		if (ticks_msimg != get_ms_ticks()) 
		{
			ticks_msimg = get_ms_ticks();  //maximum 1000000	
			LED_blink(LED1);
			readFeedback();
			
			setSetpoint();
			
			speedProcess();
			
			
			
			if(ticks_msimg>10000){
				if(!FRICTION_WHEEL_STATE)
					friction_wheel_setpoint=0;
				else {
					if(friction_wheel_setpoint<300)
						friction_wheel_setpoint+=1;
				}
				FRIC_SET_THRUST_L(friction_wheel_setpoint);
				FRIC_SET_THRUST_R(friction_wheel_setpoint);
			}
			
			
			
			
		}//main loop with ticks	
	}
	
	
}	//main

