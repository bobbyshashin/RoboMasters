#include "encoder.h"

//encoder1.INPUT_B---PB4(TIM3_CH1)
//encoder1.INPUT_A---PB5(TIM3_CH2)

//encoder2.INPUT_B---PB4(TIM4_CH3)
//encoder2.INPUT_A---PB5(TIM4_CH4)



void Quad_Encoder_Configuration(void)
{
    GPIO_InitTypeDef gpio;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

		gpio.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
		gpio.GPIO_Mode = GPIO_Mode_AF;
		gpio.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOB,&gpio);
		
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_TIM3);
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3);
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
    TIM_Cmd(TIM3,ENABLE);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

		gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
		gpio.GPIO_Mode = GPIO_Mode_AF;
		gpio.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOB,&gpio);
		
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM4);
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_TIM4);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
    TIM_Cmd(TIM4,ENABLE);
	
}

void Encoder_Start1(void)
{
    TIM3->CNT = 0x7fff;		
}

void Encoder_Start2(void)
{
    TIM4->CNT = 0x7fff;
		
}

static char Encoder_Dir1 = 0;
  
int32_t GetQuadEncoderDiff1(void)
{
    int32_t cnt = 0;    
    cnt = (TIM3->CNT)-0x7fff;
    TIM3->CNT = 0x7fff;    
    if(Encoder_Dir1 == 1)
	{
		return cnt;	
	}
	else
	{
		return -cnt;            
	}
}  

static char Encoder_Dir2 = 0;
  
int32_t GetQuadEncoderDiff2(void)
{
    int32_t cnt = 0;    
    cnt = (TIM4->CNT)-0x7fff;
    TIM3->CNT = 0x7fff;    
    if(Encoder_Dir2 == 1)
	{
		return cnt;	
	}
	else
	{
		return -cnt;            
	}
}
