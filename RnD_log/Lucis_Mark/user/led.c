#include "led.h"



void LED_master_init(){
	
	LED_init(LED1);

	
}



void LED_control(const GPIO *gpio, u8 state){
	
	if (state == 1){
	GPIO_SetBits(gpio->gpio, gpio->gpio_pin);
		
	}
	if (state == 0){
	GPIO_ResetBits(gpio->gpio, gpio->gpio_pin);
	}

}

static u8 state=0;
void LED_blink(const GPIO *gpio)
{

	if (state == 1){
	GPIO_SetBits(gpio->gpio, gpio->gpio_pin);
		state=0;
	}
	else if (state == 0){
	GPIO_ResetBits(gpio->gpio, gpio->gpio_pin);
		state=1;
	}

}	

