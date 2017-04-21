
#ifndef	__ADC_H
#define __ADC_H

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"

/*define ??????can??????? change
 *1=ON, 0=OFF
*/

#define ADC_channel_0_enable  0
#define ADC_channel_1_enable  0
#define ADC_channel_2_enable  0
#define ADC_channel_3_enable  0
#define ADC_channel_4_enable  1
#define ADC_channel_5_enable  0
#define ADC_channel_6_enable  0
#define ADC_channel_7_enable  0
#define ADC_channel_8_enable  0
#define ADC_channel_9_enable  0
#define ADC_channel_10_enable  0
#define ADC_channel_11_enable  0
#define ADC_channel_12_enable  0
#define ADC_channel_13_enable  0
#define ADC_channel_14_enable  0
#define ADC_channel_15_enable  0
#define ADC_channel_16_enable  0

/*adc function*/

void ADC1_init(void);//this function must be called if you need adc.. and change the enable define

float get_mcu_temperature();

u16 ADC_raw_data(int channel);//put in the channel number can directly get the ADC data


/*define !!!!!!must not!!!!!!! change*/
#define Total_adc_count  16 //at most 16 adc detect at the same time

#define ADC_channel_0_CLOCK   RCC_AHB1Periph_GPIOA
#define ADC_channel_0_pin     GPIO_Pin_0
#define ADC_channel_0_GPIOx   GPIOA

#define ADC_channel_1_CLOCK   RCC_AHB1Periph_GPIOA
#define ADC_channel_1_pin     GPIO_Pin_1
#define ADC_channel_1_GPIOx   GPIOA

#define ADC_channel_2_CLOCK   RCC_AHB1Periph_GPIOA
#define ADC_channel_2_pin     GPIO_Pin_2
#define ADC_channel_2_GPIOx   GPIOA

#define ADC_channel_3_CLOCK   RCC_AHB1Periph_GPIOA
#define ADC_channel_3_pin     GPIO_Pin_3
#define ADC_channel_3_GPIOx   GPIOA

#define ADC_channel_4_CLOCK   RCC_AHB1Periph_GPIOA
#define ADC_channel_4_pin     GPIO_Pin_4
#define ADC_channel_4_GPIOx   GPIOA

#define ADC_channel_5_CLOCK   RCC_AHB1Periph_GPIOA
#define ADC_channel_5_pin     GPIO_Pin_5
#define ADC_channel_5_GPIOx   GPIOA

#define ADC_channel_6_CLOCK   RCC_AHB1Periph_GPIOA
#define ADC_channel_6_pin     GPIO_Pin_6
#define ADC_channel_6_GPIOx   GPIOA

#define ADC_channel_7_CLOCK   RCC_AHB1Periph_GPIOA
#define ADC_channel_7_pin     GPIO_Pin_7
#define ADC_channel_7_GPIOx   GPIOA

#define ADC_channel_8_CLOCK   RCC_AHB1Periph_GPIOB
#define ADC_channel_8_pin     GPIO_Pin_0
#define ADC_channel_8_GPIOx   GPIOB

#define ADC_channel_9_CLOCK   RCC_AHB1Periph_GPIOB
#define ADC_channel_9_pin     GPIO_Pin_1
#define ADC_channel_9_GPIOx   GPIOB

#define ADC_channel_10_CLOCK   RCC_AHB1Periph_GPIOC
#define ADC_channel_10_pin     GPIO_Pin_0
#define ADC_channel_10_GPIOx   GPIOC

#define ADC_channel_11_CLOCK   RCC_AHB1Periph_GPIOC
#define ADC_channel_11_pin     GPIO_Pin_1
#define ADC_channel_11_GPIOx   GPIOC

#define ADC_channel_12_CLOCK   RCC_AHB1Periph_GPIOC
#define ADC_channel_12_pin     GPIO_Pin_2
#define ADC_channel_12_GPIOx   GPIOC

#define ADC_channel_13_CLOCK   RCC_AHB1Periph_GPIOC
#define ADC_channel_13_pin     GPIO_Pin_3
#define ADC_channel_13_GPIOx   GPIOC

#define ADC_channel_14_CLOCK   RCC_AHB1Periph_GPIOC
#define ADC_channel_14_pin     GPIO_Pin_4
#define ADC_channel_14_GPIOx   GPIOC

#define ADC_channel_15_CLOCK   RCC_AHB1Periph_GPIOC
#define ADC_channel_15_pin     GPIO_Pin_5
#define ADC_channel_15_GPIOx   GPIOC

#endif
