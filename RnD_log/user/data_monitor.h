#ifndef DATA_MONITOR_H
#define DATA_MONITOR_H

#include "stm32f4xx.h"

// only for uart3 Tx
void DataMonitor_Init(void);

u8 DataMonitor_Send(uint8_t id, int16_t);

#endif // DATA_MONITOR_H
