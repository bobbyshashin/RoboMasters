#ifndef	__GPIO_H
#define	__GPIO_H

#include "stm32f4xx_gpio.h"

#include "stm32f4xx_rcc.h"

typedef struct {
   GPIO_TypeDef *gpio;
	uint16_t gpio_pin;
} GPIO;

extern const GPIO 
	PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13, PA14, PA15,
	PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15,
	PC0, PC1, PC2, PC3, PC4, PC5, PC6, PC7, PC8, PC9, PC10, PC11, PC12, PC13, PC14, PC15,
	PD0, PD1, PD2, PD3, PD4, PD5, PD6, PD7, PD8, PD9, PD10, PD11, PD12, PD13, PD14, PD15,
	PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7, PE8, PE9, PE10, PE11, PE12, PE13, PE14, PE15,
	PF0, PF1, PF2, PF3, PF4, PF5, PF6, PF7, PF8, PF9, PF10, PF11, PF12, PF13, PF14, PF15,
	PG0, PG1, PG2, PG3, PG4, PG5, PG6, PG7, PG8, PG9, PG10, PG11, PG12, PG13, PG14, PG15;



void gpio_init(const GPIO *gpio, GPIOMode_TypeDef mode, GPIOOType_TypeDef otype, GPIOSpeed_TypeDef speed, GPIOPuPd_TypeDef pupd);

void BUTTON_init(const GPIO *gpio);
void timer_gpio_init(const GPIO *gpio);
void AF_config(const GPIO *gpio,u16 AF_function);
void gpio_rcc_init(const GPIO* gpio);
u8 gpio_read_input(const GPIO* gpio);
void LED_init(const GPIO *gpio);








#endif /* __GPIO_H */
