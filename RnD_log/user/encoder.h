#ifndef __Encoder_H__
#define __Encoder_H__
#include "stm32f4xx.h"

void Quad_Encoder_Configuration(void);

void Encoder_Start1(void);
void Encoder_Start2(void);

int32_t GetQuadEncoderDiff1(void);
int32_t GetQuadEncoderDiff2(void);

#endif
