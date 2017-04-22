#include "data_monitor.h"
//#include "judge.h"
#define bufferLength 3
static DMA_InitTypeDef DataMonitor_DMA_InitStruct;
int8_t DataMonitorBuffer[bufferLength];

void DataMonitor_Init(void) {
    // UART3_TX
    DataMonitor_DMA_InitStruct.DMA_Channel           =   DMA_Channel_4;
    DataMonitor_DMA_InitStruct.DMA_PeripheralBaseAddr=   (uint32_t)(&USART3->DR);
    DataMonitor_DMA_InitStruct.DMA_Memory0BaseAddr   =   (int32_t)(DataMonitorBuffer);
    DataMonitor_DMA_InitStruct.DMA_DIR               =   DMA_DIR_MemoryToPeripheral;
    DataMonitor_DMA_InitStruct.DMA_BufferSize        =   bufferLength;
    DataMonitor_DMA_InitStruct.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable;
    DataMonitor_DMA_InitStruct.DMA_MemoryInc         =   DMA_MemoryInc_Enable;
    DataMonitor_DMA_InitStruct.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte;
    DataMonitor_DMA_InitStruct.DMA_PeripheralDataSize=   DMA_PeripheralDataSize_Byte;
    DataMonitor_DMA_InitStruct.DMA_Mode              =   DMA_Mode_Normal;
    DataMonitor_DMA_InitStruct.DMA_Priority          =   DMA_Priority_Medium;
    DataMonitor_DMA_InitStruct.DMA_FIFOMode          =   DMA_FIFOMode_Disable;
    DataMonitor_DMA_InitStruct.DMA_FIFOThreshold     =   DMA_FIFOThreshold_Full;
    DataMonitor_DMA_InitStruct.DMA_MemoryBurst       =   DMA_MemoryBurst_Single;
    DataMonitor_DMA_InitStruct.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream3, &DataMonitor_DMA_InitStruct);
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
    DMA_Cmd(DMA1_Stream3, DISABLE);
}

u8 DataMonitor_Send(uint8_t id, int16_t setpoint) {
    // Clear transfer complete flag
    DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3);

    // restart DMA
    DMA_Cmd(DMA1_Stream3, DISABLE);
    while(DMA_GetCmdStatus(DMA1_Stream3) != DISABLE);
		DataMonitorBuffer[0]=id;
		DataMonitorBuffer[1]=setpoint>>8;
		if(setpoint-(setpoint>>8)>127){
			DataMonitorBuffer[1]++;
			DataMonitorBuffer[2]=(int16_t)(setpoint)-(int16_t)(DataMonitorBuffer[1]<<8);
		}
		else DataMonitorBuffer[2]=setpoint-((setpoint>>8)<<8);
	
	

    DMA_Init(DMA1_Stream3, &DataMonitor_DMA_InitStruct);
    DMA_Cmd(DMA1_Stream3, ENABLE);

    return 1;
}

