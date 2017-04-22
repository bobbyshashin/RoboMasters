#define ENCODER_FILE

#include "Driver_Encoder.h"

void ENCODER_BSP_Init(void) {
    // Encoder1
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);

    // TIM3 (encoder1)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
    TIM_Cmd(TIM3,ENABLE);
    TIM3->CNT = 0x7FFF;
}

void ENCODER_Init(void) {
    ENCODER_BSP_Init();
    ENCODER_Data = 0;
}

void ENCODER_Update(void) {
    ENCODER_Data = ENCODER_TIM->CNT - 0x7FFF;
    ENCODER_TIM->CNT = 0x7FFF;
    if (ENCODER_DIR)
        ENCODER_Data = -ENCODER_Data;
}
