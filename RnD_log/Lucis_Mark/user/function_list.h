#include "pwm.h"
#include "led.h"
#include "1.8 tft_display.h"
#include "yaw_gyro.h"
#include "encoder.h"
#include "pneumatic.h"
#include "canBusProcess.h"
#include "Dbus.h"
#include "judge.h"
#include "PID.h"
#include "helper_functions.h"
#include "general_control.h"

//Brush Motor Functions
void Brush_motor_pwm_dir_init(void);
void Brush_motor_control1(u8 dir, u16 speed);
void Brush_motor_control2(u8 dir, u16 speed);

//Friction Wheel Functions
void Friction_wheel_init(void);
void Friction_wheel_control1(u16 speed);
void Friction_wheel_control2(u16 speed);

//LED functions
/*
param : const GPIO *gpio : LED1
*/
void LED_master_init();
void LED_control(const GPIO *gpio, u8 state);
void LED_blink(const GPIO *gpio);

//1.8 TFT_display
void tft_update();
void tft_clear();
void tft_prints(u8 x, u8 y, const char * pstr, ...); 
void tft_put_pixel(u8 x, u8 y, u16 color);

//Gyro yaw
void gyro_init(void);	
void gyro_cal(void);
void gyro_update(void);
s16 get_angle(void);

//Encoder
void Quad_Encoder_Configuration(void);

void Encoder_Start1(void);
void Encoder_Start2(void);

int32_t GetQuadEncoderDiff1(void);
int32_t GetQuadEncoderDiff2(void);

//Pneumatic
void pneumatic_init();
void pneumatic_control(u8 port, u8 state);

//CAN bus
void CAN1_Configuration();
void CAN2_Configuration();
	//control usage
void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq,int16_t gimbal_x_iq);
void Set_CM_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
/*
if wanna get encoder feedback value of each motor, all by default from CAN2
CM1Encoder...
CM2Encoder...
CM3Encoder...
CM4Encoder...
GMYawEncoder...
GMPitchEncoder...
GMxEncoder...

//important notes for 3510
1 2 3 4  id:201
h l l h  CM1			

1 2 3 4	 id:202
l h l h  CM2

1 2 3 4  id:203
h h l h  CM3

1 2 3 4  id:204
l l h h  CM4

1 2 3 4	 id:205
h l h h  yaw

1 2 3 4  id:206
l h h h  pitch

1 2 3 4  id:207
h h h h  x


raw_rate
etc
*/


//Dbus//haha dont get confused. This is the Remote Controller
void Dbus_init(void);
/*
get the information from here, you can dot dot dot
DBUS_ReceiveData...
LASTDBUS_ReceiveData...
*/

//judge system

void judging_system_init(void);
/*
get the information from here you can dot dot dot
InfantryJudge...
*/

//generic pid control















