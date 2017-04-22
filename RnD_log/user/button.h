#ifndef __button_h_
#define __button_h_

#include "gpio.h"

#define JOY_RIGHT  &PC15
#define JOY_LEFT   &PC0
#define JOY_MIDDLE &PC14
#define JOY_UP     &PC1
#define JOY_BOTTOM &PC13

void button_init();
void button_check();

#endif