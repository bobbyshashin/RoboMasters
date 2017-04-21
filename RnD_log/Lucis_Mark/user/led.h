#include "gpio.h"



#define LED1 &PB3

void LED_master_init();
void LED_control(const GPIO *gpio, u8 state); //1:ON or 0:OFF
void LED_blink(const GPIO *gpio); //blink according to how frequent you call it
