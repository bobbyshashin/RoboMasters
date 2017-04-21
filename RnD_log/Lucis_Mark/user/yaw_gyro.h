#ifndef __gyro_h
#define __gyro_h


#include "ticks.h"

#include "gpio.h"

extern s32 get_to_turning_angle;


//NSS //pc12
#define gyro_NSS_pin GPIO_Pin_2
#define gyro_NSS_port GPIOD
#define NSS &PD2

#define gyro_reset_pin GPIO_Pin_15
#define gyro_reset_port GPIOA
#define reset_gyro &PA15
//reset Pa15



// SPI MOSI CLK
#define gyro_SPI					SPI3
#define gyro_GPIO_CLOCK  RCC_AHB1Periph_GPIOC
#define gyro_SPI_CLOCK   RCC_APB1Periph_SPI3

#define gyro_MOSI        GPIO_Pin_12
#define gyro_MOSI_SOURCE GPIO_PinSource12

#define gyro_MISO_SOURCE GPIO_PinSource11
#define gyro_MISO				 GPIO_Pin_11

#define gyro_CLK         GPIO_Pin_10
#define gyro_CLK_SOURCE  GPIO_PinSource10

#define gyro_GPIO        GPIOC
#define gyro_AF          GPIO_AF_SPI3




#define GYRO_ANG_VEL_TH 		10
#define GYRO_SCALE 				350.4661806	//		1 / (0.07326 * 3.908ms ) / 10 =349.2838703

#define GYRO_FLASH				0x01
#define GYRO_POWER				0x03
#define GYRO_VEL				0x05
#define GYRO_ADC				0x0B
#define GYRO_TEMP				0x0D
#define GYRO_ANGL				0x0F
#define GYRO_OFF				0x15   
#define GYRO_COMD				0x3F
#define GYRO_SENS				0x39
#define GYRO_SMPL				0x37
	

#define X 0
#define Y 1

extern s16 prev_ang_vel;
extern s16 curr_ang_vel;
extern s32 gyro_angle;
extern s32 sim_now;
extern s32 sim_angle;
extern s32 gyro_comb;
extern s32 gyro_temp;
extern s16 real_angle;
extern u8 gyro_state;
//private functions
void gyro_spi_init(void);			   	//init spi communication channel
u16 spi_frame(u16 data);	   			//transmit 16-bit data via spi

void adis_write(u8 addr, u16 data);		//write data to gyro
u16 adis_read(u8 addr );				//read data from gyro

void gyro_chip_select( void );			//select chip
void gyro_chip_deselect( void );		//deselect chip


//public functions
void gyro_init(void);					//init gyro
void gyro_cal(void);					//cal gyro, only for first calibration
void gyro_cal_short(void);				//cal gyro in shorter time, for re-calibration 	 
void gyro_update_tim_init(void); 		//init timer for updating gyro				
void gyro_update(void);				//carry out the calculation		
														   	 
u8 gyro_get_state(void);				//read status for gyro
s16 gyro_get_vel(void);				//read angular velocity from gyro
u16 gyro_get_angle(void);				//read angle from gyro

s16 gyro_get_off(void);				//read offset(result of callibration) for angular velocity 
u16 gyro_get_flash(void);				//read number of flash for the rom un gyro
u16 gyro_get_power(void);	 			//return milli-volt
u16 gyro_get_adc(void);		  		//return milli-volt
u16 gyro_get_temp(void); 		   		//return milli-degree
void set_angle( s16 angle );
s16 get_angle(void);
extern s32 output_angle ;

void TIM5_Int_Init(u16 period,u16 psc);
extern float yaw_pid_output_angle;
extern s32 require_angle;


void polar_coordinate(s16 x, s16 y,s32 current_angle);

extern float polar_angle ;
extern float polar_distance;

extern float Vx_by_polar;
extern float Vy_by_polar;

extern s32 angle_in_3600;

extern s32 angle_updated;


#endif
