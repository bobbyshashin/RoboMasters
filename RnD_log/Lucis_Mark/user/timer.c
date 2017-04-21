#include "timer.h"

	
	
	

	

void timer_init(TIM_TypeDef* TIMx, u16 Prescaler, u16 CounterMode, u16 Period, u16 ClockDivision)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	timer_rcc_init(TIMx);
	TIM_TimeBaseStructure.TIM_Period = Period;
	TIM_TimeBaseStructure.TIM_ClockDivision =  ClockDivision;
	TIM_TimeBaseStructure.TIM_Prescaler = Prescaler;
	TIM_TimeBaseStructure.TIM_CounterMode = CounterMode;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

}

void timer_rcc_init(TIM_TypeDef* TIMx)
{
	if (TIMx == TIM1) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	} else if (TIMx == TIM2) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	} else if (TIMx == TIM3) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	} else if (TIMx == TIM4) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	} else if (TIMx == TIM5) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	} else if (TIMx == TIM6) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	} else if (TIMx == TIM7) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	} else if (TIMx == TIM8) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	} else if (TIMx == TIM9) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	} else if (TIMx == TIM10) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
	} else if (TIMx == TIM11) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
	} else if (TIMx == TIM12) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
	} else if (TIMx == TIM13) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
	} else if (TIMx == TIM14) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
	}
}

void pwm_timer_init(u16 OC,TIM_TypeDef *TIMER, u16 OCMode, u16 OutputState, u16 Pulse, u16 OCPolarity)
{
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	TIM_OCInitStructure.TIM_OCPolarity = OCPolarity;   		// set "high" to be effective output
	TIM_OCInitStructure.TIM_OCMode = OCMode;	           		// produce output when counter < CCR
	TIM_OCInitStructure.TIM_OutputState = OutputState;  	// this part enable the output
	TIM_OCInitStructure.TIM_Pulse = Pulse;
	
	if (TIMER == TIM1 || TIMER == TIM8) {
		TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;   		
		TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;		
		TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;	
		TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; 
	}
	

	
	if (OC==TIM_Channel_1){
		TIM_OC1Init(TIMER,&TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIMER,TIM_OCPreload_Enable);
	}
	if (OC==TIM_Channel_2){
		TIM_OC2Init(TIMER,&TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIMER,TIM_OCPreload_Enable);
	}	
	if (OC==TIM_Channel_3){
		TIM_OC3Init(TIMER,&TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIMER,TIM_OCPreload_Enable);
	}	
	if (OC==TIM_Channel_4){
		TIM_OC4Init(TIMER,&TIM_OCInitStructure);
		TIM_OC4PreloadConfig(TIMER,TIM_OCPreload_Enable);
	}

	
	TIM_CtrlPWMOutputs(TIMER, ENABLE);
	TIM_ARRPreloadConfig(TIMER, ENABLE);
    TIM_Cmd(TIMER, ENABLE);	
}

/*


void encoder_timer_init(TIM_TypeDef* TIMx, const u16 initial_value)
{
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI12,						//TIM_EncoderMode_TI12->count on 4 edge per cycle
	                         TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
//	TIM_ICStructInit(&TIM_ICInitStructure);
//	TIM_ICInitStructure.TIM_ICFilter = 8;
//	TIM_ICInit(TIMx, &TIM_ICInitStructure);
//	
//	// Clear all pending interrupts
//	TIM_ClearFlag(TIMx, TIM_FLAG_Update);
//	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
	// Reset counter to initial value
	TIM_SetCounter(TIMx, initial_value);
	// Counter Enable
	TIM_Cmd(TIMx, ENABLE);
}
*/