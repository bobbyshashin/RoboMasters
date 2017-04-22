#ifndef __MAIN_H
#define __MAIN_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h> 
#include <math.h>


/*** Essential ***/
#include "yaw_gyro.h"
#include "gpio.h"
#include "ticks.h"
#include "led.h"
#include "pwm.h"
#include "timer.h"
#include "buzzer.h"
#include "buzzer_song.h"
#include "1.8 TFT_display.h"
#include "encoder.h"
#include "data_monitor.h"
#include "BilateralCommunication.h"
#include "adc.h"


#include "stm32f4xx_crc.h"
#include "core_cm4.h"
#include "core_cmFunc.h"
#include "Driver_Gun.h"
#include "Driver_Encoder.h"

#define USE_SIMULATED_JUDGE

#endif /* __MAIN_H */
