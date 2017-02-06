#include "main.h"
#include "function_list.h"
typedef unsigned short u16_t;
static u32 ticks_msimg = (u32)-1;
const float coeff=4.5;
//const double rot_coeff=0.3;
const double K=2.8;
float judge_coeff=1;
float max=0;
volatile u8 i=0;
volatile int current[20]={0};
volatile int voltage[20]={0};
int sum=0;
volatile	float aver_current=0;
volatile float aver_voltage=0;
volatile float aver_power=0;
u8 KB=0;
u8 sl,sr,sr_prev;
int sp_prev[4]={0};
int sp[4]={0};
int offset=0;
int16_t speed[4]={0};
float interval[4]={0};
typedef struct{
	signed int setpoint;
	int sum_error;
	float proportion;
	float integral;
	float derivative;
	int last_error;        //e[-1];
	int prev_error;        //e[-2]
}PID_t;

PID_t PID[4];
const int MAX=3000;


void incPIDinit(void)
{
//front LEFT
	for(int i=0;i<4;i++){
 PID[i].sum_error=0;
 PID[i].last_error=0;
 PID[i].prev_error=0;
 PID[i].proportion=0;//constants
 PID[i].integral=0;	 //constants
 PID[i].derivative=0;//constants
 PID[i].setpoint=0;
	}
////front RIGHT
// PID2.sum_error=0;
// PID2.last_error=0;
// PID2.prev_error=0;
// PID2.proportion=0;
// PID2.integral=0;
// PID2.derivative=0;
// PID2.setpoint=0;
//	
////backward RIGHT
// PID3.sum_error=0;
// PID3.last_error=0;
// PID3.prev_error=0;
// PID3.proportion=0;
// PID3.integral=0;
// PID3.derivative=0;
// PID3.setpoint=0;
// 
////backward LEFT
// PID4.sum_error=0;
// PID4.last_error=0;
// PID4.prev_error=0;
// PID4.proportion=0;
// PID4.integral=0;
// PID4.derivative=0;
// PID4.setpoint=0;


}
void init(){
	SysTick_Init();  
	Dbus_init();//usart1
	//buzzer_init();	 //initialization of buzzer
	tft_init(2,WHITE,BLACK,BLACK);
	LED_master_init();
	gyro_init();
	ADC1_init();
	judging_system_init(); //usart3
	gyro_init();
	pneumatic_init();
	CAN1_Configuration();
	CAN2_Configuration();
	gyro_init();
	gyro_cal();
	Quad_Encoder_Configuration();
	Encoder_Start1();
	Encoder_Start2();
	Friction_wheel_init();
	TIM5_Int_Init(24,13124);// 256hz //3.9xx ms for gyro usage
	incPIDinit();
}

void PID_set()        //pp,ii,dd maybe those constants I've set
{
  PID[0].proportion=9;
	PID[0].integral=4;
	PID[0].derivative=0; 
	PID[1].proportion=10;
	PID[1].integral=4;
	PID[1].derivative=0;
	PID[2].proportion=10;
	PID[2].integral=4;
	PID[2].derivative=0.1;
	PID[3].proportion=9;
	PID[3].integral=4;
	PID[3].derivative=1;
 
}

float incPIDcalc(PID_t* PIDx,signed int nextpoint)
{
	int iError;
	float iincpid;
	iError=PIDx->setpoint-nextpoint;    //current error
	/*                                        //fisrt way to calculate increment
	iincpid = PIDx->proportion * iError       //e[k]
	- PIDx->integral * PIDx->last_error       //e[k-1]
	+ PIDx->derivative * PIDx->prev_error;
	
	*/
	
	iincpid=																	//second way to calculate increment
	PIDx->proportion * (iError-PIDx->last_error)
	+PIDx->integral*iError
	+PIDx->derivative * (iError-2 * PIDx->last_error + PIDx->prev_error);
	
	PIDx->prev_error = PIDx->last_error;			//save the error to be used in the next calculation
	PIDx->last_error = iError;
	return iincpid;
	
	
}
 void PID_setpoint(PID_t * PIDx,signed int setvalue)
 {
  PIDx->setpoint=setvalue; 
 }
 
 
float judge_Kp = 0.004;
float judge_Ki = 0.001;
float judge_Kd = 0;
float judge_temp_derivative = 0;
float judge_temp_integral = 0;
float judge_pre_error= 0;

void judge_pid_cal(float target_power, float current_power,float * power_pid_output){
	
	float error = target_power - current_power;
	
	float Kout = error * judge_Kp;
	
	judge_temp_integral += error;
	
	float Iout = judge_temp_integral * judge_Ki;
	
	judge_temp_derivative = error - judge_pre_error;
	
	float Dout = judge_temp_derivative * judge_Kd;
	
	*power_pid_output = Dout + Iout + Kout; 

}

 
 
 
 

void set_speed(int16_t vx,int16_t vy,const float ome,const int32_t enc1,const int32_t enc2,const int32_t enc3,const int32_t enc4){
	
	if(sr==2){
		vx=0;vy=0;
	if(DBUS_CheckPush(KEY_W)) vx+=3000;
	if(DBUS_CheckPush(KEY_A)) vy+=2500;
	if(DBUS_CheckPush(KEY_S)) vx-=3000;
	if(DBUS_CheckPush(KEY_D)) vy-=2500;
	}
	sp[0]=vx-vy-K*ome;
	sp[1]=vx+vy-K*ome;
	sp[2]=vx-vy+K*ome;
	sp[3]=vx+vy+K*ome;
	if(aver_power>max) max=aver_power;
	if(aver_power>150) judge_coeff=0.6;
	else if(aver_power<60) judge_coeff=1;
	else judge_pid_cal(79,aver_power,&judge_coeff);
	if(judge_coeff>1.1) judge_coeff=1.1;
	if(judge_coeff<0.8)judge_coeff=0.7;
	for(int i=0;i<4;i++){
	if(sp_prev[i]>300) PID_setpoint(&PID[i],0.4*sp_prev[i]*judge_coeff-115);
	else if(sp_prev[i]<-300) PID_setpoint(&PID[i],0.4*sp_prev[i]*judge_coeff+115);
	else PID_setpoint(&PID[i],0);
	}
	speed[0]+=incPIDcalc(&PID[0],enc1);
	speed[1]+=incPIDcalc(&PID[1],enc2);
	speed[2]+=incPIDcalc(&PID[2],enc3);
	speed[3]+=incPIDcalc(&PID[3],enc4);
	for(int i=0;i<4;i++){
	if(speed[i]>MAX) speed[i]=MAX;
	if(speed[i]<-MAX) speed[i]=-MAX;
		interval[i]=(sp[i]-sp_prev[i])*1.0/20;
		sp_prev[i]=sp[i];
	}
	
}


int main(void)
{	
	int gyro_target=0;
	float gyro_output=0;
	signed int gyro_cur=0;
	init();
	int k=0;
	//u8 i=0;
	PID_set();
	sr_prev=sr=DBUS_ReceiveData.rc.switch_right;
	
	//buzzer_play_song(START_UP, 125, 0);
		
	
	while (1)  {	
		if (ticks_msimg != get_ms_ticks()) 
		{
		
		
			ticks_msimg = get_ms_ticks();  //maximum 1000000	
			buzzer_check();			
			//**********************************************
			
			if(ticks_msimg%20==0){
				KB=DBUS_ReceiveData.keyBoard.key_code;
				sum=0;
				for(int j=0;j<i;j++)
				sum+=voltage[j];
				if(i!=0) aver_voltage=sum*1.0/i;
				sum=0;
				for(int j=0;j<i;j++)
				sum+=current[j];
				if(i!=0) aver_current=sum*1.0/i;
				aver_power=aver_voltage*aver_current;
				for(int j=0;j<20;j++) {current[j]=0;voltage[j]=0;}
				k=i;
				i=0;
				
				gyro_cur=real_angle;
				gyro_cur-=offset;
				if(gyro_cur<0) gyro_cur+=3600;
				if(gyro_cur>1800&&gyro_cur<3600) gyro_cur-=3600;        //[-1799,1800]
				sr=DBUS_ReceiveData.rc.switch_right;
				if(sr_prev==3&&sr==1) {
					offset=real_angle;
				//gyro_init();
				//set_angle(gyro_target);
				gyro_target=0;
				gyro_output=0;
				}
				else if(sr==1&&sr_prev==1) yaw_axis_pid_cal(gyro_target, gyro_cur,&gyro_output);
				else{
				gyro_target=gyro_cur;
				gyro_output=0;
				}
				sr_prev=sr;
			set_speed(coeff*DBUS_ReceiveData.rc.ch1,-coeff*DBUS_ReceiveData.rc.ch0,DBUS_ReceiveData.rc.ch2+gyro_output
			,-CM3Encoder.filter_rate,-CM2Encoder.filter_rate,CM1Encoder.filter_rate,CM4Encoder.filter_rate);
				
				
				
				
			Set_CM_Speed(CAN2,speed[0],-speed[1],-speed[2],speed[3]);
			
			for (int j=2;j<12;j++) tft_clear_line(j);
			tft_prints(1,2,"setpoint1=%d",PID[2].setpoint);
			tft_prints(1,3,"%d",speed[0]);
			tft_prints(1,4,"setpoint2=%d",PID[1].setpoint);
			tft_prints(1,5,"%d",speed[1]);
			tft_prints(1,6,"setpoint3=%d",PID[0].setpoint);
			tft_prints(1,7,"%d",speed[2]);
			tft_prints(1,8,"setpoint4=%d",PID[3].setpoint);
			tft_prints(1,9,"%d",speed[3]);
			tft_prints(1,10,"%d",k);
			tft_prints(1,11,"%d",gyro_target);
			tft_update();		
			

			}
			//*********************************************
			voltage[i++]=InfantryJudge.RealVoltage;
			current[i-1]=InfantryJudge.RealCurrent;
		
		}
		for(int i=0;i<4;i++){
			sp_prev[i]+=interval[i];
	if(sp[i]>300) PID_setpoint(&PID[i],0.4*sp_prev[i]*judge_coeff-115);
	else if(sp[i]<-300) PID_setpoint(&PID[i],0.4*sp_prev[i]*judge_coeff+115);
	else PID_setpoint(&PID[i],0);
	}
		
	}
	

	


}	

	



