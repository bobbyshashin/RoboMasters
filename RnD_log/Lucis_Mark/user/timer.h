
#include "stm32f4xx_tim.h"
#include "stm32f4xx.h"
#include "gpio.h"














void timer_rcc_init(TIM_TypeDef* TIMx);
void timer_init(TIM_TypeDef* TIMx, u16 Prescaler, u16 CounterMode, u16 Peroid, u16 ClockDivision);
void pwm_timer_init(u16 OC,TIM_TypeDef *TIMER, u16 OCMode, u16 OutputState, u16 Pulse, u16 OCPolarity);
void encoder_timer_init(TIM_TypeDef* TIMx, const u16 initial_value);

