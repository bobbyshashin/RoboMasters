/**
  ******************************************************************************
  * @file    ADC.c
  * @author  Cheung Ngai { JOHN }
  * @version V1.0.0
  * @date    17-AUG-2015
  * @brief   This file provide you a ADC with DMA. It helps you to read ADC signal from any ADC pin

  ******************************************************************************
  * @attention
  *
  * This source is designed for application use. Unless necessary, try NOT to
	* modify the function definition. The constants which are more likely to 
	* vary among different schematics have been placed as pre-defined constant
	* (i.e., "#define") in the header file.
	*
  ******************************************************************************
  */
	

#include "adc.h"
#include "1.8 TFT_display.h"
#include "buzzer_song.h"


u16 ADC1_level[Total_adc_count+1];


void ADC1_init(void)
{/* ADC init sturcture*/
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef       DMA_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;   	
	
/*Enable clock for adc,dma,gpio*/
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	
	/*ADC1 DMA init*/
	DMA_DeInit(DMA2_Stream0);
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC1_level[1];//to make it easier to see, the array start from 1
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = Total_adc_count;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);

	/*ADC init*/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = Total_adc_count;
	ADC_Init(ADC1, &ADC_InitStructure);
	
	ADC_DMACmd(ADC1, ENABLE);
	
	/*ADC channel config*/
	
//	#if   ADC_channel_0_enable==1
//	RCC_AHB1PeriphClockCmd(ADC_channel_0_CLOCK, ENABLE);
//	GPIO_InitStructure.GPIO_Pin = ADC_channel_0_pin;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(ADC_channel_0_GPIOx, &GPIO_InitStructure); 
//	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_480Cycles);
//	#endif
	
	#if   ADC_channel_1_enable==1
	RCC_AHB1PeriphClockCmd(ADC_channel_1_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ADC_channel_1_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ADC_channel_1_GPIOx, &GPIO_InitStructure); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_480Cycles);
	#endif
	
	#if ADC_channel_2_enable==1
	RCC_AHB1PeriphClockCmd(ADC_channel_2_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ADC_channel_2_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ADC_channel_2_GPIOx, &GPIO_InitStructure); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_480Cycles);
	#endif
	
	#if ADC_channel_3_enable==1
	RCC_AHB1PeriphClockCmd(ADC_channel_3_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ADC_channel_3_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ADC_channel_3_GPIOx, &GPIO_InitStructure); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_480Cycles);
	#endif
	
	#if ADC_channel_4_enable==1
	RCC_AHB1PeriphClockCmd(ADC_channel_4_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ADC_channel_4_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ADC_channel_4_GPIOx, &GPIO_InitStructure); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 4, ADC_SampleTime_480Cycles);
	#endif
	
	#if ADC_channel_5_enable==1
	RCC_AHB1PeriphClockCmd(ADC_channel_5_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ADC_channel_5_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ADC_channel_5_GPIOx, &GPIO_InitStructure); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 5, ADC_SampleTime_480Cycles);
	#endif
	
	#if ADC_channel_6_enable==1
	RCC_AHB1PeriphClockCmd(ADC_channel_6_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ADC_channel_6_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ADC_channel_6_GPIOx, &GPIO_InitStructure); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 6, ADC_SampleTime_480Cycles);
	#endif
	
	#if ADC_channel_7_enable==1
	RCC_AHB1PeriphClockCmd(ADC_channel_7_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ADC_channel_7_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ADC_channel_7_GPIOx, &GPIO_InitStructure); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 7, ADC_SampleTime_480Cycles);
	#endif
	
	#if ADC_channel_8_enable==1
	RCC_AHB1PeriphClockCmd(ADC_channel_8_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ADC_channel_8_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ADC_channel_8_GPIOx, &GPIO_InitStructure); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 8, ADC_SampleTime_480Cycles);
	#endif
	
	#if ADC_channel_9_enable==1
	RCC_AHB1PeriphClockCmd(ADC_channel_9_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ADC_channel_9_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ADC_channel_9_GPIOx, &GPIO_InitStructure); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 9, ADC_SampleTime_480Cycles);
	#endif
	
	#if ADC_channel_10_enable==1
	RCC_AHB1PeriphClockCmd(ADC_channel_10_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ADC_channel_10_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ADC_channel_10_GPIOx, &GPIO_InitStructure); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 10, ADC_SampleTime_480Cycles);
	#endif
	
	#if ADC_channel_11_enable==1
	RCC_AHB1PeriphClockCmd(ADC_channel_11_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ADC_channel_11_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ADC_channel_11_GPIOx, &GPIO_InitStructure); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 11, ADC_SampleTime_480Cycles);
	#endif
	
	#if ADC_channel_12_enable==1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 12, ADC_SampleTime_480Cycles);
	RCC_AHB1PeriphClockCmd(ADC_channel_12_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ADC_channel_12_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ADC_channel_12_GPIOx, &GPIO_InitStructure); 
	#endif
	
	#if ADC_channel_13_enable==1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 13, ADC_SampleTime_480Cycles);
	RCC_AHB1PeriphClockCmd(ADC_channel_13_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ADC_channel_13_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ADC_channel_13_GPIOx, &GPIO_InitStructure); 
	#endif
	
	#if ADC_channel_14_enable==1
	RCC_AHB1PeriphClockCmd(ADC_channel_14_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ADC_channel_14_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ADC_channel_14_GPIOx, &GPIO_InitStructure); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 14, ADC_SampleTime_480Cycles);
	#endif
	
	#if ADC_channel_15_enable==1
	RCC_AHB1PeriphClockCmd(ADC_channel_15_CLOCK, ENABLE);
	GPIO_InitStructure.GPIO_Pin = ADC_channel_15_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(ADC_channel_15_GPIOx, &GPIO_InitStructure); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 15, ADC_SampleTime_480Cycles);
	#endif
	
	#if ADC_channel_16_enable==1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 16, ADC_SampleTime_480Cycles);//temperature sensor
	ADC_TempSensorVrefintCmd(ENABLE);	
	#endif
	
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
	ADC_Cmd(ADC1, ENABLE);
    ADC_SoftwareStartConv(ADC1); 
}


u16 ADC_raw_data(int channel){
	return ADC1_level[channel];
}



float get_5v(){
	float voltage_factor=0.0016117216;				//factor = 3300 / 4095 * total resistance / gnd_resistance / 1000
	return ADC_raw_data(4)*voltage_factor;		//3.176039647=3300/4095*67/17
}



float get_mcu_temperature() //return degree celcius
{
	#if ADC_channel_16_enable==1
	return (ADC1_level[16]*0.805861-760)/2.5+25;//0.805860805=(3300/4095/1000-0.76)/0.0025
	#else
	return 0;//return 0 if it is not enable
	#endif
	   //Temperature (in °„C) = {(VSENSE ®C V25) / Avg_Slope} + 25
}

