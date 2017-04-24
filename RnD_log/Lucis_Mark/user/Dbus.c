#include "Dbus.h"
#include "Driver_Gun.h"


DBUSDecoding_Type DBUS_ReceiveData, LASTDBUS_ReceiveData;
uint8_t DBUSBuffer[DBUSLength + DBUSBackLength];	
u8 connected_timer =0;

void Dbus_init(void){
			
	  DMA_InitTypeDef     DMA_InitStructure;
    NVIC_InitTypeDef	NVIC_InitStructure;
		USART_InitTypeDef   USART_InitStructure;
	
		//enable Dbus related clock
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
		//init pin that has been chosen on the RM2017 board
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
    GPIO_InitTypeDef	GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_7;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
		//init Dbus usart config
    USART_InitStructure.USART_BaudRate              =   100000;
    USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                  =   USART_Mode_Rx;
    USART_InitStructure.USART_Parity                =   USART_Parity_Even;
    USART_InitStructure.USART_StopBits              =   USART_StopBits_1;
    USART_InitStructure.USART_WordLength            =   USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);
		USART_Cmd(USART1, ENABLE);	
		
		//init NVIC, interrupt enable
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
		NVIC_InitStructure.NVIC_IRQChannel						=	USART1_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	0;
		NVIC_Init(&NVIC_InitStructure);
		USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
		
		//USART1 RX DMA config
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
		DMA_DeInit(DMA2_Stream5); 
    DMA_InitStructure.DMA_Channel           =   DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr=   (uint32_t)(&USART1->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr   =   (uint32_t)(DBUSBuffer);
    DMA_InitStructure.DMA_DIR               =   DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize        =   DBUSLength + DBUSBackLength;
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
    DMA_Init(DMA2_Stream5, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream5, ENABLE);
			
}

/**
  * @brief  DBUS���  * @param  void
  * @retval void
  */
void DBUS_DataDecoding(void)
{
	
	LASTDBUS_ReceiveData = DBUS_ReceiveData;
	//for the ch x data will be signed int from  -660 to 660
	DBUS_ReceiveData.rc.ch0 = (DBUSBuffer[0] | DBUSBuffer[1]<<8) & 0x07FF;
	DBUS_ReceiveData.rc.ch0 -= 1024;//-1024 to make middle point as 0
	DBUS_ReceiveData.rc.ch1 = (DBUSBuffer[1]>>3 | DBUSBuffer[2]<<5 ) & 0x07FF;
	DBUS_ReceiveData.rc.ch1 -= 1024;
	DBUS_ReceiveData.rc.ch2 = (DBUSBuffer[2]>>6 | DBUSBuffer[3]<<2 | DBUSBuffer[4]<<10) & 0x07FF;
	DBUS_ReceiveData.rc.ch2 -= 1024;
	DBUS_ReceiveData.rc.ch3 = (DBUSBuffer[4]>>1 | DBUSBuffer[5]<<7) & 0x07FF;		
	DBUS_ReceiveData.rc.ch3 -= 1024;
	
	DBUS_ReceiveData.rc.switch_left = ( (DBUSBuffer[5] >> 4)& 0x000C ) >> 2;
	DBUS_ReceiveData.rc.switch_right =  (DBUSBuffer[5] >> 4)& 0x0003 ;
	
	DBUS_ReceiveData.mouse.x = DBUSBuffer[6] | (DBUSBuffer[7] << 8);	//x axis
	DBUS_ReceiveData.mouse.y = DBUSBuffer[8] | (DBUSBuffer[9] << 8);
	DBUS_ReceiveData.mouse.z = DBUSBuffer[10]| (DBUSBuffer[11] << 8);
	
	
	
  DBUS_ReceiveData.mouse.xtotal += DBUS_ReceiveData.mouse.x;  //x axis
  DBUS_ReceiveData.mouse.ytotal += DBUS_ReceiveData.mouse.y;
  DBUS_ReceiveData.mouse.ztotal += DBUS_ReceiveData.mouse.z;
  
  //if(DBUS_ReceiveData.mouse.xtotal>2000) DBUS_ReceiveData.mouse.xtotal=2000;
  //if(DBUS_ReceiveData.mouse.xtotal<-2000) DBUS_ReceiveData.mouse.xtotal=-2000;
	DBUS_ReceiveData.mouse.press_left 	= DBUSBuffer[12];	// is pressed?
	DBUS_ReceiveData.mouse.press_right 	= DBUSBuffer[13];
	
	
	
	
	DBUS_ReceiveData.keyBoard.key_code 	= DBUSBuffer[14] | DBUSBuffer[15] << 8; //key board code
	//if you wanna use keyboard and mouse, you need to run specific sw that provided by RM.
	/**********************************************************************************
   * ����ͨ��:15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
   *          V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
************************************************************************************/
    GUN_SetMotion();
}


/**
  * @brief  �ж�һ�����Ƿ񱻰���
  * @param  Ҫ�����жϵİ������ַ���д
  * @retval 1 ����        0 δ����
	*				1 push 0 not push
  */

uint8_t DBUS_CheckPush(uint16_t Key)
{
    if((DBUS_ReceiveData.keyBoard.key_code & Key) || (LASTDBUS_ReceiveData.keyBoard.key_code & Key))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


//�������մ��ڱ�־λ����ʱ����
uint8_t UARTtemp;

/**
  * @brief  DBUS�����ж�(USART1)
  * @param  void
  * @retval void
  */

void USART1_IRQHandler(void)
{
			UARTtemp = USART1->DR;
			UARTtemp = USART1->SR;
			
			DMA_Cmd(DMA2_Stream5, DISABLE);
			
			if(DMA2_Stream5->NDTR == DBUSBackLength)
			{
					DBUS_DataDecoding();
			}
			
			//����DMA
			DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);
			while(DMA_GetCmdStatus(DMA2_Stream5) != DISABLE);
			DMA_SetCurrDataCounter(DMA2_Stream5, DBUSLength + DBUSBackLength);
			DMA_Cmd(DMA2_Stream5, ENABLE);
		
}