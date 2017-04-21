#include "pwm.h"
#include "gpio.h"
#include "timer.h"

void Brush_motor_pwm_dir_init(void){

	timer_gpio_init(&PA0);
	timer_gpio_init(&PA2);
	
	AF_config(&PA0,GPIO_AF_TIM2);  // to be put in void easier
	AF_config(&PA2,GPIO_AF_TIM2);  // to be put in void easier
	
	timer_init(TIM2, 83, TIM_CounterMode_Up, 999, TIM_CKD_DIV1);//1khz
  
	pwm_timer_init(TIM_Channel_1,TIM2, TIM_OCMode_PWM1, TIM_OutputState_Enable, 0, TIM_OCPolarity_High);//0 is pulse to be tune
	pwm_timer_init(TIM_Channel_3,TIM2, TIM_OCMode_PWM1, TIM_OutputState_Enable,0, TIM_OCPolarity_High);//0 is pulse to be tune
	
	LED_init(&PA1);
	LED_init(&PA3);
}

void Brush_motor_control1(u8 dir, u16 speed){
	
	if(dir == 0){
		GPIO_ResetBits(GPIOA, GPIO_Pin_1);
	}
	else if(dir == 1){
		GPIO_SetBits(GPIOA, GPIO_Pin_1);
	}
	
	TIM_SetCompare1(TIM2,speed);
}

void Brush_motor_control2(u8 dir, u16 speed){
	
	if(dir == 0){
		GPIO_ResetBits(GPIOA, GPIO_Pin_3);
	}
	else if(dir == 1){
		GPIO_SetBits(GPIOA, GPIO_Pin_3);
	}
	
	TIM_SetCompare3(TIM2,speed);
}


void Friction_wheel_init(){
	timer_gpio_init(&PA8);
	timer_gpio_init(&PA9);
	timer_gpio_init(&PA10);
	
	AF_config(&PA8,GPIO_AF_TIM1);  // to be put in void easier
	AF_config(&PA9,GPIO_AF_TIM1);  // to be put in void easier
	AF_config(&PA10,GPIO_AF_TIM1);  // to be put in void easier

	timer_init(TIM1, 20, TIM_CounterMode_Up, 9999, TIM_CKD_DIV1);// 400hz
  
	pwm_timer_init(TIM_Channel_1,TIM1, TIM_OCMode_PWM1, TIM_OutputState_Enable, 0, TIM_OCPolarity_High);//0 is pulse to be tune
	pwm_timer_init(TIM_Channel_2,TIM1, TIM_OCMode_PWM1, TIM_OutputState_Enable, 0, TIM_OCPolarity_High);//0 is pulse to be tune
	pwm_timer_init(TIM_Channel_3,TIM1, TIM_OCMode_PWM1, TIM_OutputState_Enable, 0, TIM_OCPolarity_High);//0 is pulse to be tune

}

void Friction_wheel_control1(u16 speed){
	
	TIM_SetCompare1(TIM1,speed);
}

void Friction_wheel_control2(u16 speed){
	
	TIM_SetCompare2(TIM1,speed);
}

void Friction_wheel_control3(u16 speed){
	
	TIM_SetCompare3(TIM1,speed);
}




