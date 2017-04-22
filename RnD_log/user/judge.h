#include "stm32f4xx.h"

#define JudgeBufferLength       150
#define JudgeFrameLength_1      46
#define JudgeFrameLength_2      11
#define JudgeFrameLength_3      24

#define JudgeFrameHeader        0xA5        //֡ͷ 

void judging_system_init(void);

//����ϵͳ�ṹ��
typedef struct
{
    float RealVoltage;                  //ʵʱ��ѹ
    float RealCurrent;                  //ʵʱ����
    int16_t LastBlood;                  //ʣ��Ѫ��
    uint8_t LastHartID;                 //�ϴ��յ��˺���װ�װ�ID��
    float LastShotSpeed;                //�ϴ�����ٶ�

}InfantryJudge_Struct;

//��ʽת��������
typedef union
{
    uint8_t U[4];
    float F;
    int I;
}FormatTrans;


//the data will be stored in the following struct
extern InfantryJudge_Struct InfantryJudge;