

#include "stm32f4xx.h"


#define KEY_V		0x4000
#define KEY_C		0x2000
#define KEY_X		0x1000
#define KEY_Z		0x0800
#define KEY_G		0x0400
#define KEY_F		0x0200
#define KEY_R		0x0100
#define KEY_E		0x0080
#define KEY_Q		0x0040
#define KEY_CTRL	0x0020
#define KEY_SHIFT	0x0010
#define KEY_D		0x0008
#define KEY_A		0x0004
#define KEY_S		0x0002
#define KEY_W		0x0001


#define DBUSLength		18				//DBUS����֡��
#define DBUSBackLength	1				//����һ���ֽڱ����ȶ�


//ң�ؽ������ݴ洢�ṹ��
typedef struct {
	
	u8 connected;
	
	struct
	{
		int16_t ch0;	//each ch value from -660 -- +660
		int16_t ch1;
		int16_t ch2;
		int16_t ch3;
		
		uint8_t switch_left;	//3 value ( 1,2,3)
		uint8_t switch_right;
	}rc;
	
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		int32_t xtotal;
		int32_t ytotal;
		int32_t ztotal;
	
		uint8_t press_left;
		uint8_t press_right;
    
		int32_t x_position;
		int32_t y_position;
		
	}mouse;
	
	struct 
	{
/**********************************************************************************
   * ����ͨ��:15   14   13   12   11   10   9   8   7   6     5     4   3   2   1
   *          V    C    X	  Z    G    F    R   E   Q  CTRL  SHIFT  D   A   S   W
************************************************************************************/
		uint16_t key_code;              //ԭʼ��ֵ
	}keyBoard;
}DBUSDecoding_Type;



//#######here is the Data being processed.
extern DBUSDecoding_Type DBUS_ReceiveData, LASTDBUS_ReceiveData;

uint8_t DBUS_CheckPush(uint16_t Key);
extern u8 connected_timer;
void Dbus_init(void);
