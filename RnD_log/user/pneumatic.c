
#include "pneumatic.h"

void pneumatic_init(){

	LED_init(&PC6);
	LED_init(&PC7);
	LED_init(&PC8);
	LED_init(&PC9);
	
}


void pneumatic_control(u8 port, u8 state){
	switch(port){
		case 1: {GPIO_WriteBit(GPIOC, GPIO_Pin_6, state);break;}
		case 2:	{GPIO_WriteBit(GPIOC, GPIO_Pin_7, state);break;}
		case 3:	{GPIO_WriteBit(GPIOC, GPIO_Pin_8, state);break;}
		case 4:	{GPIO_WriteBit(GPIOC, GPIO_Pin_9, state);break;}
	}
}