//This the file that include all the saparate control of the base, the gimbal Yaw and gimbal Pitch
#ifndef GENERAL_CONTROL
#define GENERAL_CONTROL

#include "stdint.h"

extern void baseControl(int forward, int rightward, int clockwise);
extern void gimbalYawControl(int position);
extern void gimbalPitchControl(int position);









#endif