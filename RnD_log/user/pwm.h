#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"

void Brush_motor_pwm_dir_init(void);
void Friction_wheel_init(void);

void Brush_motor_control1(u8 dir, u16 speed);
void Brush_motor_control2(u8 dir, u16 speed);

void Friction_wheel_control1(u16 speed);
void Friction_wheel_control2(u16 speed);