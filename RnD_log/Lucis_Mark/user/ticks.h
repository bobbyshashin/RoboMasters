
#include "stm32f4xx.h"
 
void SysTick_Init(void); //need to be called in main
void TimeTick_Decrement(void); //need to be called in systick handler
void TimeTick_Increment(void);//need to be called in systick handler
void delay_nus(u32 n);
void delay_1ms(void);
void delay_nms(u32 n);
u32 get_us_ticks(void);
u32 get_ms_ticks(void);
 
