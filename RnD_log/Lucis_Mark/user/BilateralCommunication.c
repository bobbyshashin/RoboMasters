#include "BilateralCommunication.h"
static DMA_InitTypeDef Bilateral_DMA_InitStruct;
#define BilateralCommunication_buffer 3
int8_t BilateralBuffer[BilateralCommunication_buffer];
void Bilateral_Init(void) {
	
	
    DMA_InitTypeDef     DMA_InitStructure;
    NVIC_InitTypeDef	NVIC_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	
    GPIO_InitTypeDef	GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
    
		
		USART_InitTypeDef   USART_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
    USART_InitStructure.USART_BaudRate              =   115200;
    USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                  =   USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_Parity                =   USART_Parity_No;
    USART_InitStructure.USART_StopBits              =   USART_StopBits_1;
    USART_InitStructure.USART_WordLength            =   USART_WordLength_8b;
    USART_Init(USART3, &USART_InitStructure);
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
    USART_Cmd(USART3, ENABLE);
		
		    //USART3
		NVIC_InitStructure.NVIC_IRQChannel						=	USART3_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	8;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	0;
		NVIC_Init(&NVIC_InitStructure);
		USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
			
		  //UART3_RX
    DMA_InitStructure.DMA_Channel           =   DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr=   (uint32_t)(&USART3->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr   =   (uint32_t)(BilateralBuffer);   
    DMA_InitStructure.DMA_DIR               =   DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize        =   BilateralCommunication_buffer;	//now its 3
    DMA_InitStructure.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc         =   DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_PeripheralDataSize=   DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_Mode              =   DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority          =   DMA_Priority_Medium;
    DMA_InitStructure.DMA_FIFOMode          =   DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold     =   DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst       =   DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream1, ENABLE);
		
}

uint8_t getID(){
	return BilateralBuffer[0];
}

int16_t getPositionSetpoint(){
	return (BilateralBuffer[1]<<8)+BilateralBuffer[2];
}

u8 UARTtemp1;
void USART3_IRQHandler(void)
{
    UARTtemp1 = USART3->DR;
    UARTtemp1 = USART3->SR;
    
    DMA_Cmd(DMA1_Stream1, DISABLE);
		ONE_KEY_DOWN_FRONT=false;
		ONE_KEY_UP_FRONT=false;
		ONE_KEY_DOWN_BACK=false;
		ONE_KEY_UP_BACK=false;
		BREAK=false;
		if(getID()==0x05){
			LiftingMotorPositionSetpoint[0]=LiftingMotorPositionSetpoint[1]=LiftingMotorPositionSetpoint[2]=LiftingMotorPositionSetpoint[3]=0;
		}
		else if(getID()==0xFF){
			GO_ON_STAGE_ONE_KEY=true;
		}
		else if(getID()==0xFE){
			GO_DOWN_STAGE_ONE_KEY=true;
		}
		else if(getID()==0xFD){
			BREAK=true;
		}
		else if(getID()==0xFC){
			ONE_KEY_DOWN_FRONT=true;
		}
		else if(getID()==0xFB){
			ONE_KEY_DOWN_BACK=true;
		}
		else if(getID()==0xFA){
			ONE_KEY_UP_FRONT=true;
		}
		else if(getID()==0xF9){
			ONE_KEY_UP_BACK=true;
		}
		else if(getID()==0x00){
			LiftingMotorPositionSetpoint[0]=LiftingMotorPositionSetpoint[1]=8*getPositionSetpoint();
		}
		else if(getID()==0x01){
			LiftingMotorPositionSetpoint[0]=LiftingMotorPositionSetpoint[1]=8*getPositionSetpoint();
		}
		else if(getID()==0x02){
			LiftingMotorPositionSetpoint[2]=LiftingMotorPositionSetpoint[3]=8*getPositionSetpoint();
		}
		else if(getID()==0x03){
			LiftingMotorPositionSetpoint[2]=LiftingMotorPositionSetpoint[3]=8*getPositionSetpoint();
		}
		else if(getID()==16){
			FRICTION_WHEEL_STATE=false;
		}
		else if(getID()==17){
			FRICTION_WHEEL_STATE=false;
		}
		else if(getID()==18){
			FRICTION_WHEEL_STATE=true;
		}
		

		
		
		//????DMA
    DMA_ClearFlag(DMA1_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
    
		while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
    DMA_SetCurrDataCounter(DMA1_Stream1, BilateralCommunication_buffer);
    DMA_Cmd(DMA1_Stream1, ENABLE);

    
    
}
