#ifndef DRIVER_GUN
#define DRIVER_GUN

#include "stm32f4xx.h"
#include "helper_functions.h"

#ifndef GUN_FILE
    #define GUN_EXT extern
#else
    #define GUN_EXT
#endif

#define POKE_DIR_PORT                           GPIOA
#define POKE_DIR_PIN                            GPIO_Pin_1
#define POKE_DIR                                1
#define POKE_SET_PWM(x)                         TIM_SetCompare1(TIM2, (x))
#define FRIC_SET_THRUST_L(x)                    TIM_SetCompare1(TIM1, 1000+(x))
#define FRIC_SET_THRUST_R(x)                    TIM_SetCompare2(TIM1, 1000+(x))

extern int32_t gunSpeed;	
extern int32_t error;		
extern int32_t spSp;
extern int32_t Cerror;
typedef struct {
    int32_t pokeTargetSpeed;
    int32_t pokeTargetAngle;
    int32_t pokeOutput;
    int32_t pokeAngle;
} GUN_DataTypeDef;

GUN_EXT volatile GUN_DataTypeDef GUN_Data;

void GUN_Init(void);
void GUN_SetMotion(void);
void GUN_PokeControl(void);
void GUN_PokeSpeedControl(void);
void GUN_SetFree(void);

void GUN_ShootOne(void);

typedef enum {
    kLLAST = 0,
    kLAST,
    kNOW,

    kIndexCnt
} PID_IndexTypeDef;

typedef enum {
    kIncremental,
    kPositional,
    kIntegralDecay
} PID_ModeTypeDef;

// should be negative
#define PID_NO_LIMIT -1.0f

typedef struct {
    /* set by user */
    float Kp, Ki, Kd;
    float IDecayFactor;
    float MAX_Integral, MAX_Pout, MAX_PIDout, MIN_PIDout;
    PID_ModeTypeDef mode;

    /* updated by calling PID_Update */
    float set[kIndexCnt];
    float real[kIndexCnt];
    float err[kIndexCnt], errIntegral;
    float output;
} PID_Controller;

void PID_Reset(PID_Controller *pid);
float PID_Update(PID_Controller *pid, float target, float measure);


#endif
