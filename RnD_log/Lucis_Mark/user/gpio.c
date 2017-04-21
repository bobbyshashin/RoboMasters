#include "gpio.h"

	/**
  * @brief  Changes the mapping of the specified pin.
  * @param  GPIOx: where x can be (A..K) to select the GPIO peripheral for STM32F405xx/407xx and STM32F415xx/417xx devices
  *                      x can be (A..I) to select the GPIO peripheral for STM32F42xxx/43xxx devices.
  *                      x can be (A, B, C, D and H) to select the GPIO peripheral for STM32F401xx devices. 
  * @param  GPIO_PinSource: specifies the pin for the Alternate function.
  *         This parameter can be GPIO_PinSourcex where x can be (0..15).
  * @param  GPIO_AFSelection: selects the pin to used as Alternate function.
  *          This parameter can be one of the following values:
  *            @arg GPIO_AF_RTC_50Hz: Connect RTC_50Hz pin to AF0 (default after reset) 
  *            @arg GPIO_AF_MCO: Connect MCO pin (MCO1 and MCO2) to AF0 (default after reset) 
  *            @arg GPIO_AF_TAMPER: Connect TAMPER pins (TAMPER_1 and TAMPER_2) to AF0 (default after reset) 
  *            @arg GPIO_AF_SWJ: Connect SWJ pins (SWD and JTAG)to AF0 (default after reset) 
  *            @arg GPIO_AF_TRACE: Connect TRACE pins to AF0 (default after reset)
  *            @arg GPIO_AF_TIM1: Connect TIM1 pins to AF1
  *            @arg GPIO_AF_TIM2: Connect TIM2 pins to AF1
  *            @arg GPIO_AF_TIM3: Connect TIM3 pins to AF2
  *            @arg GPIO_AF_TIM4: Connect TIM4 pins to AF2
  *            @arg GPIO_AF_TIM5: Connect TIM5 pins to AF2
  *            @arg GPIO_AF_TIM8: Connect TIM8 pins to AF3
  *            @arg GPIO_AF_TIM9: Connect TIM9 pins to AF3
  *            @arg GPIO_AF_TIM10: Connect TIM10 pins to AF3
  *            @arg GPIO_AF_TIM11: Connect TIM11 pins to AF3
  *            @arg GPIO_AF_I2C1: Connect I2C1 pins to AF4
  *            @arg GPIO_AF_I2C2: Connect I2C2 pins to AF4
  *            @arg GPIO_AF_I2C3: Connect I2C3 pins to AF4
  *            @arg GPIO_AF_SPI1: Connect SPI1 pins to AF5
  *            @arg GPIO_AF_SPI2: Connect SPI2/I2S2 pins to AF5
  *            @arg GPIO_AF_SPI4: Connect SPI4 pins to AF5 
  *            @arg GPIO_AF_SPI5: Connect SPI5 pins to AF5 
  *            @arg GPIO_AF_SPI6: Connect SPI6 pins to AF5
  *            @arg GPIO_AF_SAI1: Connect SAI1 pins to AF6 for STM32F42xxx/43xxx devices.       
  *            @arg GPIO_AF_SPI3: Connect SPI3/I2S3 pins to AF6
  *            @arg GPIO_AF_I2S3ext: Connect I2S3ext pins to AF7
  *            @arg GPIO_AF_USART1: Connect USART1 pins to AF7
  *            @arg GPIO_AF_USART2: Connect USART2 pins to AF7
  *            @arg GPIO_AF_USART3: Connect USART3 pins to AF7
  *            @arg GPIO_AF_UART4: Connect UART4 pins to AF8
  *            @arg GPIO_AF_UART5: Connect UART5 pins to AF8
  *            @arg GPIO_AF_USART6: Connect USART6 pins to AF8
  *            @arg GPIO_AF_UART7: Connect UART7 pins to AF8
  *            @arg GPIO_AF_UART8: Connect UART8 pins to AF8
  *            @arg GPIO_AF_CAN1: Connect CAN1 pins to AF9
  *            @arg GPIO_AF_CAN2: Connect CAN2 pins to AF9
  *            @arg GPIO_AF_TIM12: Connect TIM12 pins to AF9
  *            @arg GPIO_AF_TIM13: Connect TIM13 pins to AF9
  *            @arg GPIO_AF_TIM14: Connect TIM14 pins to AF9
  *            @arg GPIO_AF_OTG_FS: Connect OTG_FS pins to AF10
  *            @arg GPIO_AF_OTG_HS: Connect OTG_HS pins to AF10
  *            @arg GPIO_AF_ETH: Connect ETHERNET pins to AF11
  *            @arg GPIO_AF_FSMC: Connect FSMC pins to AF12 
  *            @arg GPIO_AF_FMC: Connect FMC pins to AF12 for STM32F42xxx/43xxx devices.   
  *            @arg GPIO_AF_OTG_HS_FS: Connect OTG HS (configured in FS) pins to AF12
  *            @arg GPIO_AF_SDIO: Connect SDIO pins to AF12
  *            @arg GPIO_AF_DCMI: Connect DCMI pins to AF13
  *            @arg GPIO_AF_LTDC: Connect LTDC pins to AF14 for STM32F429xx/439xx devices. 
  *            @arg GPIO_AF_EVENTOUT: Connect EVENTOUT pins to AF15
  * @retval None
  */
void AF_config(const GPIO *gpio,u16 AF_function)  //AF_function is the things up,,, the GPIO  just type PE4 sth like that 
{
	if(gpio->gpio_pin==GPIO_Pin_0){
GPIO_PinAFConfig(gpio->gpio,GPIO_PinSource0,AF_function);
	}
	if(gpio->gpio_pin==GPIO_Pin_1){
GPIO_PinAFConfig(gpio->gpio,GPIO_PinSource1,AF_function);
	}
	if(gpio->gpio_pin==GPIO_Pin_2){
GPIO_PinAFConfig(gpio->gpio,GPIO_PinSource2,AF_function);
	}
	if(gpio->gpio_pin==GPIO_Pin_3){
GPIO_PinAFConfig(gpio->gpio,GPIO_PinSource3,AF_function);
	}
	if(gpio->gpio_pin==GPIO_Pin_4){
GPIO_PinAFConfig(gpio->gpio,GPIO_PinSource4,AF_function);
	}
	if(gpio->gpio_pin==GPIO_Pin_5){
GPIO_PinAFConfig(gpio->gpio,GPIO_PinSource5,AF_function);
	}
	if(gpio->gpio_pin==GPIO_Pin_6){
GPIO_PinAFConfig(gpio->gpio,GPIO_PinSource6,AF_function);
	}
	if(gpio->gpio_pin==GPIO_Pin_7){
GPIO_PinAFConfig(gpio->gpio,GPIO_PinSource7,AF_function);
	}
	if(gpio->gpio_pin==GPIO_Pin_8){
GPIO_PinAFConfig(gpio->gpio,GPIO_PinSource8,AF_function);
	}
	if(gpio->gpio_pin==GPIO_Pin_9){
GPIO_PinAFConfig(gpio->gpio,GPIO_PinSource9,AF_function);
	}
	if(gpio->gpio_pin==GPIO_Pin_10){
GPIO_PinAFConfig(gpio->gpio,GPIO_PinSource10,AF_function);
	}
	if(gpio->gpio_pin==GPIO_Pin_11){
GPIO_PinAFConfig(gpio->gpio,GPIO_PinSource11,AF_function);
	}
	if(gpio->gpio_pin==GPIO_Pin_12){
GPIO_PinAFConfig(gpio->gpio,GPIO_PinSource12,AF_function);
	}
	if(gpio->gpio_pin==GPIO_Pin_13){
GPIO_PinAFConfig(gpio->gpio,GPIO_PinSource13,AF_function);
	}
	if(gpio->gpio_pin==GPIO_Pin_14){
GPIO_PinAFConfig(gpio->gpio,GPIO_PinSource14,AF_function);
	}
	if(gpio->gpio_pin==GPIO_Pin_15){
GPIO_PinAFConfig(gpio->gpio,GPIO_PinSource15,AF_function);
	}
}







const GPIO 	
			/*** GPIOA ***/
			PA0 = {GPIOA, GPIO_Pin_0},
			PA1 = {GPIOA, GPIO_Pin_1},
			PA2 = {GPIOA, GPIO_Pin_2},
			PA3 = {GPIOA, GPIO_Pin_3},
			PA4 = {GPIOA, GPIO_Pin_4},
			PA5 = {GPIOA, GPIO_Pin_5},
			PA6 = {GPIOA, GPIO_Pin_6},
			PA7 = {GPIOA, GPIO_Pin_7},
			PA8 = {GPIOA, GPIO_Pin_8},
			PA9 = {GPIOA, GPIO_Pin_9},
			PA10 = {GPIOA, GPIO_Pin_10},
			PA11 = {GPIOA, GPIO_Pin_11},
			PA12 = {GPIOA, GPIO_Pin_12},
			PA13 = {GPIOA, GPIO_Pin_13},
			PA14 = {GPIOA, GPIO_Pin_14},
			PA15 = {GPIOA, GPIO_Pin_15},
		

			/*** GPIOB ***/
			PB0 = {GPIOB, GPIO_Pin_0},
			PB1 = {GPIOB, GPIO_Pin_1},
			PB2 = {GPIOB, GPIO_Pin_2},
			PB3 = {GPIOB, GPIO_Pin_3},
			PB4 = {GPIOB, GPIO_Pin_4},
			PB5 = {GPIOB, GPIO_Pin_5},
			PB6 = {GPIOB, GPIO_Pin_6},
			PB7 = {GPIOB, GPIO_Pin_7},
			PB8 = {GPIOB, GPIO_Pin_8},
			PB9 = {GPIOB, GPIO_Pin_9},
			PB10 = {GPIOB, GPIO_Pin_10},
			PB11 = {GPIOB, GPIO_Pin_11},
			PB12 = {GPIOB, GPIO_Pin_12},
			PB13 = {GPIOB, GPIO_Pin_13},
			PB14 = {GPIOB, GPIO_Pin_14},
			PB15 = {GPIOB, GPIO_Pin_15},

			/*** GPIOC ***/
			PC0 = {GPIOC, GPIO_Pin_0},
			PC1 = {GPIOC, GPIO_Pin_1},
			PC2 = {GPIOC, GPIO_Pin_2},
			PC3 = {GPIOC, GPIO_Pin_3},
			PC4 = {GPIOC, GPIO_Pin_4},
			PC5 = {GPIOC, GPIO_Pin_5},
			PC6 = {GPIOC, GPIO_Pin_6},
			PC7 = {GPIOC, GPIO_Pin_7},
			PC8 = {GPIOC, GPIO_Pin_8},
			PC9 = {GPIOC, GPIO_Pin_9},
			PC10 = {GPIOC, GPIO_Pin_10},
			PC11 = {GPIOC, GPIO_Pin_11},
			PC12 = {GPIOC, GPIO_Pin_12},
			PC13 = {GPIOC, GPIO_Pin_13},
			PC14 = {GPIOC, GPIO_Pin_14},
			PC15 = {GPIOC, GPIO_Pin_15},
			
			/*** GPIOD ***/
			PD0 = {GPIOD, GPIO_Pin_0},
			PD1 = {GPIOD, GPIO_Pin_1},
			PD2 = {GPIOD, GPIO_Pin_2},
			PD3 = {GPIOD, GPIO_Pin_3},
			PD4 = {GPIOD, GPIO_Pin_4},
			PD5 = {GPIOD, GPIO_Pin_5},
			PD6 = {GPIOD, GPIO_Pin_6},
			PD7 = {GPIOD, GPIO_Pin_7},
			PD8 = {GPIOD, GPIO_Pin_8},
			PD9 = {GPIOD, GPIO_Pin_9},
			PD10 = {GPIOD, GPIO_Pin_10},
			PD11 = {GPIOD, GPIO_Pin_11},
			PD12 = {GPIOD, GPIO_Pin_12},
			PD13 = {GPIOD, GPIO_Pin_13},
			PD14 = {GPIOD, GPIO_Pin_14},
			PD15 = {GPIOD, GPIO_Pin_15},

			/*** GPIOE ***/
			PE0 = {GPIOE, GPIO_Pin_0},
			PE1 = {GPIOE, GPIO_Pin_1},
			PE2 = {GPIOE, GPIO_Pin_2},
			PE3 = {GPIOE, GPIO_Pin_3},
			PE4 = {GPIOE, GPIO_Pin_4},
			PE5 = {GPIOE, GPIO_Pin_5},
			PE6 = {GPIOE, GPIO_Pin_6},
			PE7 = {GPIOE, GPIO_Pin_7},
			PE8 = {GPIOE, GPIO_Pin_8},
			PE9 = {GPIOE, GPIO_Pin_9},
			PE10 = {GPIOE, GPIO_Pin_10},
			PE11 = {GPIOE, GPIO_Pin_11},
			PE12 = {GPIOE, GPIO_Pin_12},
			PE13 = {GPIOE, GPIO_Pin_13},
			PE14 = {GPIOE, GPIO_Pin_14},
			PE15 = {GPIOE, GPIO_Pin_15},

			/*** GPIOF ***/
			PF0 = {GPIOF, GPIO_Pin_0},
			PF1 = {GPIOF, GPIO_Pin_1},
			PF2 = {GPIOF, GPIO_Pin_2},
			PF3 = {GPIOF, GPIO_Pin_3},
			PF4 = {GPIOF, GPIO_Pin_4},
			PF5 = {GPIOF, GPIO_Pin_5},
			PF6 = {GPIOF, GPIO_Pin_6},
			PF7 = {GPIOF, GPIO_Pin_7},
			PF8 = {GPIOF, GPIO_Pin_8},
			PF9 = {GPIOF, GPIO_Pin_9},
			PF10 = {GPIOF, GPIO_Pin_10},
			PF11 = {GPIOF, GPIO_Pin_11},
			PF12 = {GPIOF, GPIO_Pin_12},
			PF13 = {GPIOF, GPIO_Pin_13},
			PF14 = {GPIOF, GPIO_Pin_14},
			PF15 = {GPIOF, GPIO_Pin_15},

			/*** GPIOG ***/
			PG0 = {GPIOG, GPIO_Pin_0},
			PG1 = {GPIOG, GPIO_Pin_1},
			PG2 = {GPIOG, GPIO_Pin_2},
			PG3 = {GPIOG, GPIO_Pin_3},
			PG4 = {GPIOG, GPIO_Pin_4},
			PG5 = {GPIOG, GPIO_Pin_5},
			PG6 = {GPIOG, GPIO_Pin_6},
			PG7 = {GPIOG, GPIO_Pin_7},
			PG8 = {GPIOG, GPIO_Pin_8},
			PG9 = {GPIOG, GPIO_Pin_9},
			PG10 = {GPIOG, GPIO_Pin_10},
			PG11 = {GPIOG, GPIO_Pin_11},
			PG12 = {GPIOG, GPIO_Pin_12},
			PG13 = {GPIOG, GPIO_Pin_13},
			PG14 = {GPIOG, GPIO_Pin_14},
			PG15 = {GPIOG, GPIO_Pin_15};
			
			



void gpio_rcc_init(const GPIO* gpio){
	switch((u32)(gpio->gpio)){
		case((u32)(GPIOA)):
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		break;	
		case((u32)(GPIOB)):
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
		break;
		case((u32)(GPIOC)):
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		break;
		case((u32)(GPIOD)):
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
		break;
		case((u32)(GPIOE)):
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
		break;
		case((u32)(GPIOF)):
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
		break;
		case((u32)(GPIOG)):
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
		break;
		case((u32)(GPIOH)):
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
		break;
		case((u32)(GPIOI)):
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
		break;
		case((u32)(GPIOJ)):
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOJ, ENABLE);
		break;
		case((u32)(GPIOK)):
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOK, ENABLE);
		break;
	}
}


void gpio_init(const GPIO *gpio, GPIOMode_TypeDef mode, GPIOOType_TypeDef otype, GPIOSpeed_TypeDef speed, GPIOPuPd_TypeDef pupd)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	gpio_rcc_init(gpio);
	GPIO_InitStructure.GPIO_Pin = gpio->gpio_pin;
	GPIO_InitStructure.GPIO_Mode = mode;
	GPIO_InitStructure.GPIO_OType = otype;
	GPIO_InitStructure.GPIO_Speed = speed;
	GPIO_InitStructure.GPIO_PuPd = pupd;
	GPIO_Init(gpio->gpio, &GPIO_InitStructure);
}



void timer_gpio_init(const GPIO *gpio)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	gpio_rcc_init(gpio);
	GPIO_InitStructure.GPIO_Pin = gpio->gpio_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(gpio->gpio, &GPIO_InitStructure);	
}


void BUTTON_init(const GPIO *gpio)
{   GPIO_InitTypeDef  GPIO_InitStructure;
	gpio_rcc_init(gpio);
	GPIO_InitStructure.GPIO_Pin = gpio->gpio_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(gpio->gpio, &GPIO_InitStructure);
}

void LED_init(const GPIO *gpio){
	GPIO_InitTypeDef  GPIO_InitStructure;
	gpio_rcc_init(gpio);
	GPIO_InitStructure.GPIO_Pin = gpio->gpio_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(gpio->gpio, &GPIO_InitStructure);
	
	if(gpio == &PA15){
		
	GPIO_PinAFConfig(gpio->gpio,gpio->gpio_pin,GPIO_AF_SWJ);
	}
	else if (gpio == &PB3){

	GPIO_PinAFConfig(gpio->gpio,gpio->gpio_pin,GPIO_AF_SWJ);
	}
	
}



/**
	* @brief Return GPIO state
	* @param GPIO pointer
	* @retval GPIO state
	*/
u8 gpio_read_input(const GPIO* gpio) {
	return GPIO_ReadInputDataBit(gpio->gpio, gpio->gpio_pin);
}
	
/**
	* @brief Write GPIO value
	* @param GPIO pointer
	* @retval None
	*/
void gpio_write(const GPIO* gpio, BitAction BitVal) {
	GPIO_WriteBit(gpio->gpio, gpio->gpio_pin, BitVal);
}
