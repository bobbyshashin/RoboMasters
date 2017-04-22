#include "yaw_gyro.h"
#include "1.8 tft_display.h"
#include "buzzer_song.h"
#include <math.h>
#include "Dbus.h"

/**
  * @brief  Initialization of SPI for TFT
  * @param  None
  * @retval None
  */

void gyro_spi_init(void)
{
   SPI_InitTypeDef   	SPI_InitStructure;
   GPIO_InitTypeDef 	GPIO_InitStructure;

 /* Configure TFT_SPI Pin: CS,RST,DC */

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(gyro_reset_port, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_Pin_15,GPIO_AF_SWJ);
	
	GPIO_SetBits(GPIOA,GPIO_Pin_15);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = gyro_NSS_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(gyro_NSS_port, &GPIO_InitStructure);
	GPIO_SetBits(gyro_NSS_port,gyro_NSS_pin);


  RCC_AHB1PeriphClockCmd(gyro_GPIO_CLOCK, ENABLE);

	RCC_APB1PeriphClockCmd(gyro_SPI_CLOCK, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin =  gyro_CLK  |  gyro_MOSI | gyro_MISO ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
     GPIO_Init(gyro_GPIO, &GPIO_InitStructure);
		
   
	GPIO_PinAFConfig(gyro_GPIO,gyro_CLK_SOURCE,gyro_AF);
	GPIO_PinAFConfig(gyro_GPIO,gyro_MOSI_SOURCE,gyro_AF);
  GPIO_PinAFConfig(gyro_GPIO,gyro_MISO_SOURCE,gyro_AF);
	  

   SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
   SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
   SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
   SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
   SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
   SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
   SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
   SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; 
	 
	 	SPI_I2S_DeInit(gyro_SPI); 
	SPI_Cmd(gyro_SPI, DISABLE);  
	
   SPI_Init(gyro_SPI, &SPI_InitStructure);
   /* Enable TFT_SPI   */
   SPI_Cmd(gyro_SPI, ENABLE);
   SPI_CalculateCRC( gyro_SPI , DISABLE );		// Disable the CRC checking
   SPI_SSOutputCmd( gyro_SPI , DISABLE );

}




 s16 curr_ang_vel = 0;
 s16 prev_ang_vel = 0;
 s32 gyro_angle = 0;
 s16 curr_real_angle = 0;
 s16 prev_real_angle = 0;	
s16 gyro_cal_result = 0;
u8 gyro_state = 0;

 s32 sim1_angle = 0;
 s32 sim2_angle = 0;
 u8 sim_factor = 0;
 s32 sim_before = 0;
 s32 sim_now = 0;
 s32 sim_angle = 0;
 s16 real_angle =0;
s16 sim_angle_off  =  0;

u16 spi_frame(u16 data )
{
	u16 temp;

	gyro_chip_select();	 	

	while (SPI_I2S_GetFlagStatus(gyro_SPI, SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData( gyro_SPI , data );

	while (SPI_I2S_GetFlagStatus(gyro_SPI, SPI_I2S_FLAG_RXNE) == RESET );	  	
	temp = SPI_I2S_ReceiveData(gyro_SPI);  

	gyro_chip_deselect();


for(int i = 0; i<400;i++){ // for delay timer usage
}
//	_delay_us(10);	
    return temp;	
}

void adis_write(u8 addr, u16 data)
{		   	
	u16 cmd1, cmd2;	
	u8 address = ( ( addr & 0x3F ) | 0x80 ); 
	cmd1 = ( address << 8 ) | ( data >> 8 );
	spi_frame( cmd1 );

	cmd2 = ( (address-1) << 8 ) | ( data & 0x00FF);
	spi_frame( cmd2 );
}

u16 adis_read(u8 addr )
{				     
	u16 address = (0x3F & addr);
	address = address << 8;
	
    spi_frame(address); 
	address = spi_frame( address );
	return address;
}

void gyro_init()
{
 	gyro_spi_init();
	
    //Reset.....
    adis_write(GYRO_COMD,0x0080);		// sofware reset
	delay_nms(50);

    //Factory Cal...
    adis_write(GYRO_COMD, 0x0002); 

    //Set Filter...
    adis_write(GYRO_SENS,0x0404);		// setting the Dynamic Range 320/sec
    adis_write(GYRO_SMPL,0x0001);		// set Internal Sample Rate 1.953 * ( 0x0001 & 0x2F + 1 ) = 3.906ms
    adis_write(GYRO_COMD,0x0008 );		// Auxiliary DAC data latch
	delay_nms(100);
	
	gyro_state = 1;
}

void gyro_cal(void)
{
    s32 gyro_zero32 = 0;
    s16 gyro_zero = 0;
    s16 tmp = 0 , i =0;
    u16 buf = 0;
	
	if (gyro_state == 0)
		return;
	
    for (i = 0; i < 1024; i++) {
        if (i >= 512) {
            tmp = gyro_get_vel();
            gyro_zero32 += tmp;

        }
				delay_nms(4);
    }

    gyro_zero32 = gyro_zero32 / 128;
    gyro_zero32 = -gyro_zero32;
    gyro_zero = (s16)gyro_zero32;
	
    buf |= gyro_zero;
	gyro_cal_result = buf;
	
    adis_write(GYRO_OFF, buf );
    adis_write(GYRO_COMD, 0x0008 );
	delay_nms(100); //nessasary. otherwise will fly
	
		
		
		
	gyro_state = 2;
}

s32 Abs(s32 M) {
	return M < 0 ? -M : M;
}
void gyro_cal_short(void)
{
    s32 gyro_zero32 = 0;
    s16 gyro_zero = 0;
    s16 tmp = 0 , i =0;
    u16 buf = 0;
	s16 prev_zero = 0;
	
	if (gyro_state == 0)
		return;
	
    for (i = 0; i < 256; i++) {
        tmp = gyro_get_vel();
        if (Abs(tmp) > 12)
            return;
        gyro_zero32 += tmp;
				delay_nms(4);
    }

    gyro_zero32 /= 64;
    gyro_zero32 = -gyro_zero32;
    gyro_zero = (s16)gyro_zero32;

    if (Abs(gyro_zero) > 1) {
        prev_zero = gyro_get_off();
        gyro_zero += prev_zero;
        buf |= gyro_zero;
        adis_write(GYRO_OFF, buf);
        adis_write(GYRO_COMD,0x0008);
				delay_nms(100);
    }
	
	gyro_state = 2;
}

s16 gyro_get_off(void)
{					
    u16 buf = 0;
    s16 off = 0;
    buf = adis_read(GYRO_OFF);	
 
    if (buf & 0x0800 ) // 0b0000100000000000
        buf |= 0xF000; // 0b1111000000000000
    else
        buf &= 0x0FFF; // 0b0000111111111111;	
    off |=buf;
    return off;
}

s16 gyro_get_vel(void)
{		
    u16 buf = 0;
	s16 vel = 0;
    buf = adis_read(GYRO_VEL);
    if (buf & 0x2000 ) // 0b0010000000000000)
        buf |= 0xC000;  //0b1100000000000000
    else
        buf &= 0x3FFF; //0b0011111111111111;
    vel |=buf;

    return vel;
}

u16 gyro_get_angle(void)
{		  	 	
    u16 angle = 0;
    angle = adis_read(GYRO_ANGL) & 0x3FFF;   //0b0011111111111111;
    return angle;
}		  




void gyro_update(void){
	
	curr_ang_vel = gyro_get_vel();

	if ( curr_ang_vel < GYRO_ANG_VEL_TH && curr_ang_vel > -GYRO_ANG_VEL_TH) {
	    curr_ang_vel = 0;
	    if (sim_factor) {
			sim_factor = 0;
	    }
	}

	if (curr_ang_vel) {
		if (!sim_factor) {		// 1st time
			sim_before = sim_now;
			sim1_angle = curr_ang_vel;
			sim2_angle = 0;
			sim_factor = 3;
		} else {
			sim1_angle += (sim_factor*prev_ang_vel+curr_ang_vel)/3;
			sim_factor = 4-sim_factor;
			sim2_angle += (sim_factor*prev_ang_vel+curr_ang_vel)/3;
		}
	
		sim_now = sim_before+(sim_factor == 3 ? sim1_angle : sim2_angle);
		sim_angle = sim_now/GYRO_SCALE ;
		
		sim_angle = sim_angle % 3600;
		if (sim_angle < 0)
			sim_angle +=3600;
		real_angle = sim_angle = (sim_angle!=0)?3600-sim_angle:0;
		real_angle = (real_angle + sim_angle_off ) % 3600;
		if( real_angle < 0 )
			real_angle += 3600;
	}

//		gyro_angle += curr_ang_vel;
//		curr_real_angle = gyro_angle/GYRO_SCALE; //divide the time constant

    prev_ang_vel = curr_ang_vel;
	
}  


s16 get_angle(void){
	return real_angle;
}


u16 gyro_get_flash(void)
{	
	adis_read(GYRO_FLASH);   
	return adis_read(GYRO_FLASH);
}

u16 gyro_get_power(void)
{ 	
	adis_read(GYRO_POWER);
	return 1832*(adis_read(GYRO_POWER) & 0x0FFF);
}

u16 gyro_get_adc(void)				 
{	
	adis_read(GYRO_ADC);
	return 6105*(adis_read(GYRO_ADC) & 0x0FFF);
}

u16 gyro_get_temp(void)
{	
	adis_read(GYRO_TEMP);
	return 145*(adis_read(GYRO_TEMP) & 0x0FFF);
}

void gyro_chip_select( void ){
	GPIO_ResetBits( gyro_NSS_port , gyro_NSS_pin );	
}

void gyro_chip_deselect( void ){
	GPIO_SetBits( gyro_NSS_port , gyro_NSS_pin );
}





void set_angle( s16 angle ){
	
	//printf( "angle:%d, real_angle:%d , sim_angle:%d, sim_angle_off:%d " , angle , real_angle , sim_angle , sim_angle_off );
	sim_angle_off = angle - sim_angle;
	real_angle = angle ;
	//printf( "new_off:%d \r\n" , sim_angle_off);
}



void TIM5_Int_Init(u16 period,u16 psc)//make timer interrupt by 2ms interrupt one time 
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5 , ENABLE); 
	 
  TIM_TimeBaseStructure.TIM_Period = period;      
  TIM_TimeBaseStructure.TIM_Prescaler =psc;	    
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	
  TIM_TimeBaseStructure.TIM_CounterMode =TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

  //TIM_SelectOutputTrigger(TIM3,TIM_TRGOSource_Update);
	/*选择update event作为TRGO,利用TIM3触发ADC通道 */
	//每个定时周期结束后触发一次
	TIM_ClearFlag(TIM5, TIM_FLAG_Update);
	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM5, ENABLE);                 
	
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;	  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5 , ENABLE); 
	/*先关闭等待使用*/ 
	
}



float yaw_Kp = 1.5;
float yaw_Ki = 0;
float yaw_Kd = 0;

float yaw_temp_derivative = 0;
float yaw_temp_integral = 0;
float yaw_pre_error= 0;

float yaw_pid_output_angle = 0;
s32 angle_updated = 0;


void yaw_axis_pid_cal(s32 target_angle, s32 current_angle){
	
	float error = target_angle - current_angle;
	
	float Kout = error * yaw_Kp;
	
	yaw_temp_integral += error;
	
	float Iout = yaw_temp_integral * yaw_Ki;
	
	yaw_temp_derivative = error - yaw_pre_error;
	
	float Dout = yaw_temp_derivative * yaw_Kd;
	
	yaw_pid_output_angle = Dout + Iout + Kout; 

}


s32 old_yaw = 0;
s16 yaw_turn_time = 0;
s32 output_angle = 0;


void TIM5_IRQHandler(void)
{
	
	if(TIM_GetITStatus(TIM5,TIM_IT_Update)!=RESET)
	{
		gyro_update();
		
    if(old_yaw < 200 && real_angle >3400){
      yaw_turn_time --;
    }
    if(old_yaw>3400 && real_angle < 200){
      yaw_turn_time ++;
    }
    old_yaw = real_angle;
    
    if(yaw_turn_time != 0){
      output_angle = yaw_turn_time *3600 + real_angle;
    }
    else if (yaw_turn_time == 0){
      output_angle = real_angle;
    }
		
	}

	TIM_ClearITPendingBit(TIM5,TIM_IT_Update);

}




