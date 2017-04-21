#include "temp_gyro.h"

u8 next_byte;
u8 byte_count = 0;
u8 higher_byte;
u8 lower_byte;
s16 angle;

void read_from_robocon_gyro(){

	if(TM_USART_BufferEmpty(USART3) == 0){
		next_byte = TM_USART_Getc(USART3);
		
	}
	else return;
	
	if(byte_count == 0){
		higher_byte = next_byte;
		byte_count ++;
	}
	else if(byte_count == 1){
		lower_byte = next_byte;
		byte_count = 0;
		RF_temp_gyro();//send signal out
	}
	
	angle = (s16)higher_byte<<8 | (s16)lower_byte;
	
}

u8 yaw_array_temp[6];
void RF_temp_gyro(){

	yaw_array_temp[1] = yaw_ID;
	yaw_array_temp[2] = lower_byte;
	yaw_array_temp[3] = higher_byte;
	
	RF_send(yaw_array_temp);
}

void robocon_gyro_init(){
	
	TM_USART_Init(USART3, TM_USART_PinsPack_2, 115200);
	
}